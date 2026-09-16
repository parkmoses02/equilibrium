#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>

// 손으로 잡고 하는 업라이트 밸런스 테스트.
// env:upright_balance_test로 빌드/업로드 후 115200 시리얼 모니터를 연다.
// 처음엔 VM 12V로 시작 (36V 금지).
// 사용법:
//   1) Z - 진자를 아래로 늘어뜨린 뒤 0점 설정.
//   2) A - 진자를 세운 채(±10도 이내) 잡고 컨트롤러 arm.
//   3) 손을 진자 옆에 대기; X(또는 스페이스)로 즉시 disarm.
//   4) P - 카트가 반대로 밀면 피드백 극성 반전 (disarm 상태에서만).
//   5) H - 명령어 도움말 다시 출력.
//   6) M - 진자 실효길이 측정. 누른 뒤 진자를 10~20도 정도 살짝 흔들어 둔다.
//   7) S - 진자가 아래로 늘어져 정지한 상태에서 스윙업 -> 자동으로 밸런스 전환.
//          카트를 레일 가운데 두고 누른다. 한쪽(SWING_START_OFFSET)으로 천천히 간 뒤 반대쪽으로
//          훅 가로지르며 올린다. VM 24~36V 필요 (12V 에선 1.2 m/s 에서 탈조).
//   8) T - 스윙 속도/가속도로 0.15 m 훅 갔다가 천천히 복귀. 출발 표시로 안 돌아오면 탈조.
// 각도 22도 초과 또는 카트 350mm 리밋 초과 시 자동 disarm.
// 밸런싱 포기(disarm) 시점까지의 로그는 이 firmware가 아니라
// src/tests/monitor_and_log.py 를 시리얼 모니터 대신 실행해서 저장한다
// (src/tests/logs/ 에 최근 3개 세션만 남도록 자동 정리됨).

// ESP32 WROOM + carrier Rev.J.
constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr uint8_t PIN_TMC_EN = 13; // active LOW
constexpr uint8_t PIN_TMC_STEP = 14;
constexpr uint8_t PIN_TMC_DIR = 27;
constexpr uint8_t PIN_TMC_CS = 5;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_SCK = 18;

constexpr int32_t ENCODER_COUNTS_PER_REV = 2400;
constexpr float RAD_PER_COUNT = 2.0f * PI / ENCODER_COUNTS_PER_REV;

// Verified mechanical/step settings: 200 full steps/rev, 1/16 microstepping,
// and the current project's 40 mm pulley circumference.
constexpr float STEPS_PER_METRE = (200.0f * 16.0f) / 0.040f;
constexpr float MAX_CART_SPEED_MPS = 0.80f;  // 48,000 pulse/s
constexpr float MAX_CART_ACCEL_MPS2 = 12.0f; // raised for a snappier response
constexpr float CART_SOFT_LIMIT_M = 0.35f;   // relative to arm/S position
constexpr float ARM_WINDOW_RAD = 10.0f * PI / 180.0f;
constexpr float TRIP_ANGLE_RAD = 22.0f * PI / 180.0f;
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz
constexpr uint32_t REPORT_PERIOD_MS = 100;

// 업라이트 게인. 네 값 모두 여기서 직접 고치고 다시 업로드한다.
// 출력은 카트의 지령 가속도 [m/s^2] 이고, updateController() 가 그것을 적분해
// commandedVelocity 를 만든 뒤 스텝 주파수로 그대로 내보낸다.
//
// 네 게인은 거의 분리된 두 모드를 담당하므로 따로 튜닝하면 된다.
//   - 진자 루프      (빠름, 수 Hz)       : gKangle, gKrate
//   - 카트 복귀 루프 (느림, 0.1~0.5 Hz) : gKcartPos, gKcartVel
//
// --- 진자 루프: 진자를 세워 둔다 ---
// gKangle   기울어진 각도에 비례해 카트를 '넘어지는 쪽으로' 가속한다. 세우는
//           힘 그 자체다. 안정하려면 반드시 g(9.81)보다 커야 하고 실용값은 3~5배.
//           올리면 진자가 뻣뻣해지고 외란 복원이 빨라지지만, 너무 올리면
//           인코더 양자화(2400 CPR = 0.15deg)와 500 Hz 샘플링 지연 탓에
//           고주파로 떨기 시작한다.
// gKrate    각속도 피드백 = 진자 루프의 댐핑. 낮으면 진자가 수 Hz 로 빠르게
//           떨고, 높이면 차분으로 만든 rawRate 의 노이즈를 증폭한다
//           (angularRate 는 계수 0.25 의 1차 LPF 를 거친 값이다).
//
// --- 카트 복귀 루프: 카트를 arm 지점 근처로 되돌린다 ---
// 이 두 항이 만드는 느린 모드는 선형화 기준으로
//     w_slow ~ sqrt(g * gKcartPos / (gKangle - g)),  zeta 는 gKcartVel 에 거의 비례
// 이고, 진자 실효길이에는 거의 무관하다 (L = 0.15~0.30 m 에서 값이 같다).
// gKcartPos 복귀가 '얼마나 빠른가'. w_slow 를 정한다. 올리면 주기가 짧아지지만,
//           카트를 중앙으로 되돌리려면 먼저 반대쪽으로 밀어야 하므로
//           (비최소위상, 아래 부호 설명 참고) 과도상태의 각도 진폭도 같이 커진다.
//             3 -> w_slow 0.86 rad/s (주기 7.0 s)   6 -> 1.4 rad/s (4.1 s)
// gKcartVel 복귀가 '얼마나 안 흔들리는가'. 그 느린 모드의 감쇠비를 정한다.
//           여기가 낮으면 진자는 멀쩡히 서 있는데 카트만 좌우로 천천히, 오래
//           흔들린다. gKcartPos = 3 기준, 괄호는 정착시간 4/(zeta*w_slow):
//             Kv=2 -> zeta 0.21 (21 s)    Kv=4 -> zeta 0.53 (7.9 s)
//             Kv=5 -> zeta 0.71 (5.6 s)   Kv=6 -> zeta 0.92 (4.1 s)
//           상한은 gKrate/L > gKcartVel (L = 진자 실효길이, 약 0.2 m) 이라
//           여유가 크다. 곱해지는 commandedVelocity 가 적분기 상태라서
//           미분항인데도 노이즈가 실리지 않는다.
// 복귀를 더 빠르게 하려면 zeta 가 유지되도록 두 값을 같이 올린다:
//   (gKcartPos, gKcartVel) = (6, 7) -> 주기 4.1 s,  (12, 10) -> 주기 2.4 s
constexpr float gKangle = 50.0f;
constexpr float gKrate = 8.0f;
constexpr float gKcartPos = 3.0f;
constexpr float gKcartVel = 5.0f;
// 업라이트 기준각 보정 [rad]. 진자 무게중심/인코더 장착 오차로 '수직'이
// 정확히 0 이 아니면 컨트롤러가 약간 기운 자세를 유지하려 하고, 그러려면
// 계속 가속해야 해서 카트가 한쪽으로 끝없이 흘러간다. 로그의 ANG 이 한쪽으로
// 치우친 채 X 가 계속 흐르면, 치우친 그 각도를 여기에 넣는다.
constexpr float angleTrim = 0.0f;
// 카트 위치/속도 피드백의 방향. 아래 부호 설명 참고.
constexpr int8_t cartFeedbackSign = 1;

// --- 스윙업 (S) ---
// 카트 가속도가 진자에 주는 효과는 cos(각도) 에 비례해서, 진자가 바닥을 지날
// 때 가장 크고 수평일 때 0 이다. 그래서 카트 속도를 '바닥 통과 순간에만'
// 계단처럼 바꾸고(+V -> -V -> 0), 수평을 지날 때 멈춘다. 멈춤은 진자 에너지를
// 거의 건드리지 않으므로 진자는 그대로 꼭대기까지 올라가고, 각도가
// CATCH_WINDOW 안에 들어오면 위의 업라이트 컨트롤러로 넘긴다.
// 마지막 계단의 크기는 진자 에너지로 정한다. 그래서 진자 실효길이가 맞아야
// 하고, M 으로 한 번 재 두는 것이 좋다.
//
// 레일(±0.35 m)을 한 번에 가로지르며 올리기 위해 S 는 세 단계로 움직인다.
//   1) PREPOSITION: 카트를 천천히 한쪽 끝(SWING_START_OFFSET)으로 옮긴다.
//      가속 T0, 감속 T0 (T0 = 진자 주기)로 움직이면 진자가 거의 안 흔들린다.
//   2) SETTLE: 진자가 가라앉길 기다린다.
//   3) 반대쪽으로 SWING_SPEED 로 훅 가로지르고, 진자가 바닥을 되지날 때
//      반대로 차면서 멈추고, 수평에서 정지 -> 진자가 올라오면 캐치.
// 시뮬레이션(L = 0.13~0.2 m, 길이 오차 ±5%) 기준 1.2 m/s, 20 m/s^2 에서
// 킥 후 0.6~0.75 s 만에 캐치한다. 가속이 부족하면(16 m/s^2) 한 번 더 민다.
constexpr float SWING_SPEED_MPS = 1.20f; // 96,000 pulse/s, 풀리 1800 rpm
constexpr float SWING_ACCEL_MPS2 = 16.0f;
// 킥 전에 천천히 옮겨 가는 거리 (S 누른 곳 기준). 0.20 은 너무 멀어서 줄였다.
constexpr float SWING_START_OFFSET_M = 0.10f;
// 스윙 전체의 좌우 방향 (+1 / -1). 진자는 좌우 대칭이라 어느 쪽이든 올라간다.
// -1: 먼저 천천히 오른쪽으로 간 뒤 왼쪽으로 킥 (2026-09-16 실기 확인 요청).
constexpr float SWING_DIRECTION = -1.0f;
constexpr uint32_t SWING_SETTLE_TIMEOUT_MS = 3000;
// 목표 에너지 여유. 에너지는 '아래 정지 = -2, 꼭대기 정지 = 0' 으로 정규화했다.
// 진자가 매번 꼭대기 못 미쳐 떨어지면 올리고, 휙 넘어가면 내린다.
constexpr float SWING_ENERGY_MARGIN = 0.0f;
// 꼭대기에 못 미치고 떨어지면, 모자랐던 에너지의 이 비율만큼 다음 목표를 올린다.
constexpr float SWING_RETRY_GAIN = 0.3f;
constexpr float CATCH_WINDOW_RAD = 25.0f * PI / 180.0f;
// 캐치 직후엔 진자가 아직 흔들리므로 이 시간 동안만 트립 각도를 넓힌다.
constexpr float CATCH_TRIP_RAD = 40.0f * PI / 180.0f;
constexpr uint32_t CATCH_SETTLE_MS = 700;
// 캐치 지점은 원점에서 0.2 m 쯤 떨어져 있고 카트도 움직이는 중이라, 위의 느린
// 카트 복귀 게인(3, 5)으로는 리밋까지 흘러간다. 캐치 직후 CATCH_STIFF_MS 동안은
// 빠른 복귀 게인을 쓰고, CATCH_BLEND_MS 동안 원래 게인으로 선형 전환한다.
constexpr float CATCH_KCARTPOS = 20.0f;
constexpr float CATCH_KCARTVEL = 14.0f;
constexpr uint32_t CATCH_STIFF_MS = 1000;
constexpr uint32_t CATCH_BLEND_MS = 1000;
constexpr uint32_t SWING_TIMEOUT_MS = 10000;
constexpr float SWING_START_ANGLE_RAD = 5.0f * PI / 180.0f;
constexpr float SWING_START_RATE = 0.5f; // rad/s
constexpr float GRAVITY = 9.81f;

// --- 진자 실효길이 측정 (M) ---
// 작은 진폭 주기 T 에서 L = g * (T / 2pi)^2. 이 L 이 곧 카트 가속도 a 에 대한
// 진자 운동방정식 thetaDD = (g sin(theta) - a cos(theta)) / L 의 L 이다.
constexpr uint8_t MEASURE_HALF_PERIODS = 10;
constexpr uint32_t MEASURE_TIMEOUT_MS = 30000;

constexpr uint8_t REG_GCONF = 0x00;
constexpr uint8_t REG_IOIN = 0x04;
constexpr uint8_t REG_GLOBALSCALER = 0x0B;
constexpr uint8_t REG_IHOLD_IRUN = 0x10;
constexpr uint8_t REG_TPOWERDOWN = 0x11;
constexpr uint8_t REG_CHOPCONF = 0x6C;
constexpr uint8_t WRITE_FLAG = 0x80;

volatile int32_t encoderCount = 0;
volatile uint8_t previousAB = 0;
constexpr int8_t QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

SPISettings tmcSpi(1000000, MSBFIRST, SPI_MODE3);

// 펄스 생성은 TMCStepDir(LEDC 하드웨어 PWM)에 맡긴다. 예전 구현은 loop()
// 한 바퀴에 펄스 하나를 digitalWrite 로 찍어서, Serial.printf 가 나갈 때마다
// (115200보에서 한 줄당 약 8ms) 펄스가 끊기고 실제 속도가 지령의 몇 분의
// 일에 그쳤다. 카트가 기울기 방향으로 느리게 밀려가다 리밋에 닿던 원인.
TMCStepDir motor(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_TMC_CS,
                 PIN_TMC_EN, PIN_TMC_STEP, PIN_TMC_DIR);
constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;
uint32_t spsToUnit(float sps) { return (uint32_t)(sps / VEL_UNIT_TO_SPS); }

enum Mode : uint8_t
{
    MODE_SAFE,
    MODE_SWING,
    MODE_BALANCE,
    MODE_MOTION_TEST
};
enum SwingPhase : uint8_t
{
    PHASE_PREPOSITION, // 시작 끝으로 천천히 이동
    PHASE_SETTLE,      // 진자 가라앉길 대기
    PHASE_PUMP,        // 바닥 통과마다 속도 계단
    PHASE_COAST,       // 목표 에너지 도달, 수평까지 그대로
    PHASE_RISE         // 카트 정지, 캐치 창 대기
};

bool calibrated = false;
Mode mode = MODE_SAFE;
bool armed = false;            // mode != MODE_SAFE
float pendulumLengthM = 0.20f; // M 으로 측정하면 갱신된다
SwingPhase swingPhase = PHASE_PUMP;
float swingTargetVelocity = 0.0f;
float swingEnergyBias = 0.0f;
float previousSwingRate = 0.0f;
bool swingKickPending = false;
bool pushedThisPass = false;
bool apexHandled = false;
uint32_t swingStartMs = 0;
uint32_t phaseStartMs = 0;
uint32_t relaxedTripUntilMs = 0;
uint32_t catchMs = 0;

bool measuring = false;
int8_t measureSide = 0;
uint8_t measureCrossings = 0;
uint32_t measureFirstUs = 0;
uint32_t measureLastUs = 0;
uint32_t measureStartMs = 0;
float measureAmplitude = 0.0f;
// Verified on the assembled mechanism: the original sign accelerated the cart
// in the falling direction, so use the opposite feedback polarity by default.
int8_t controlPolarity = -1;
float angularRate = 0.0f;
float commandedVelocity = 0.0f;
float cartPosition = 0.0f;
int32_t cartStepCount = 0;
uint32_t nextStepUs = 0;
uint32_t lastControlUs = 0;

void IRAM_ATTR updateEncoder()
{
    const uint8_t currentAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    encoderCount += QUADRATURE_TABLE[(previousAB << 2) | currentAB];
    previousAB = currentAB;
}

int32_t readEncoderCount()
{
    noInterrupts();
    const int32_t value = encoderCount;
    interrupts();
    return value;
}

void zeroEncoderDownward()
{
    noInterrupts();
    encoderCount = 0;
    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    interrupts();
    calibrated = true;
}

float wrapPi(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle <= -PI)
        angle += 2.0f * PI;
    return angle;
}

// Downward is calibrated to zero count; upright is half a revolution away.
float uprightAngleFromCount(int32_t count)
{
    return wrapPi(count * RAD_PER_COUNT - PI);
}

void writeRegister(uint8_t address, uint32_t value)
{
    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address | WRITE_FLAG);
    SPI.transfer(static_cast<uint8_t>(value >> 24));
    SPI.transfer(static_cast<uint8_t>(value >> 16));
    SPI.transfer(static_cast<uint8_t>(value >> 8));
    SPI.transfer(static_cast<uint8_t>(value));
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();
}

uint32_t readRegister(uint8_t address)
{
    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address & 0x7F);
    for (uint8_t i = 0; i < 4; ++i)
        SPI.transfer(0);
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();

    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address & 0x7F);
    uint32_t value = static_cast<uint32_t>(SPI.transfer(0)) << 24;
    value |= static_cast<uint32_t>(SPI.transfer(0)) << 16;
    value |= static_cast<uint32_t>(SPI.transfer(0)) << 8;
    value |= static_cast<uint32_t>(SPI.transfer(0));
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();
    return value;
}

void configureDriver()
{
    // mStep_=4 -> CHOPCONF.MRES=4 = 1/16 마이크로스텝. STEPS_PER_METRE 와 일치.
    // iHold/iRun 0.4/1.0 은 TMC::setCurrent() 에서 IHOLD=6 / IRUN=15 로 떨어져
    // 기존 설정과 동일하다.
    motor.init(0.4f, 1.0f, 4, 192);

    // TMC::init() 은 en_pwm_mode=1(stealthChop) + TPWMTHRS=500 을 쓴다. 그러면
    // 저속 구간이 stealthChop 이 되는데, 밸런싱은 영속도 부근에서 토크가 가장
    // 필요하므로 여기서는 spreadCycle 로 되돌린다 (원래 이 테스트의 설정).
    writeRegister(REG_GCONF, 0x00000000);
    writeRegister(0x13, 0); // TPWMTHRS = 0

    // 스윙이 더 빠르므로 하드웨어 상한은 스윙 속도로 둔다. 밸런스는 자체적으로 MAX_CART_SPEED 로 자른다.
    motor.setSpeed(spsToUnit(max(MAX_CART_SPEED_MPS, SWING_SPEED_MPS) * STEPS_PER_METRE));
    // 직결 속도 지령을 쓰므로 내부 램프 가속도는 상한 역할만 한다.
    motor.setAcceleration(0xFFFF);
}

void setMode(Mode next)
{
    mode = next;
    armed = next != MODE_SAFE;
}

// 정규화 에너지: 아래 정지 = -2, 꼭대기 정지 = 0.
float pendulumEnergy(float angle, float rate)
{
    return 0.5f * rate * rate * pendulumLengthM / GRAVITY + cos(angle) - 1.0f;
}

void disarm(const char *reason)
{
    setMode(MODE_SAFE);
    commandedVelocity = 0.0f;
    motor.setVelocityDirect(0.0f); // 펄스 정지
    digitalWrite(PIN_TMC_EN, HIGH);
    Serial.print("DISARMED: ");
    Serial.println(reason);
}

void armController()
{
    if (!calibrated)
    {
        Serial.println("ARM REFUSED: calibrate downward with Z first.");
        return;
    }
    const float angle = uprightAngleFromCount(readEncoderCount());
    if (fabsf(angle) > ARM_WINDOW_RAD)
    {
        Serial.printf("ARM REFUSED: hold upright within 10 deg (now %.2f deg).\n",
                      angle * 180.0f / PI);
        return;
    }
    motor.actualPosition(0); // PCNT 위치 원점을 arm 지점으로
    cartStepCount = 0;
    cartPosition = 0.0f;
    commandedVelocity = 0.0f;
    angularRate = 0.0f;
    lastControlUs = micros();
    nextStepUs = lastControlUs;
    relaxedTripUntilMs = millis();
    catchMs = millis() - CATCH_STIFF_MS - CATCH_BLEND_MS; // 캐치 게인 안 씀
    digitalWrite(PIN_TMC_EN, LOW);
    setMode(MODE_BALANCE);
    Serial.println("ARMED: keep one hand near the pendulum and X ready to stop.");
}

void startSwingUp()
{
    if (!calibrated)
    {
        Serial.println("SWING REFUSED: calibrate downward with Z first.");
        return;
    }
    if (armed || measuring)
    {
        Serial.println("SWING REFUSED: stop (X) first.");
        return;
    }
    const float psi = wrapPi(uprightAngleFromCount(readEncoderCount()) - PI);
    if (fabsf(psi) > SWING_START_ANGLE_RAD || fabsf(angularRate) > SWING_START_RATE)
    {
        Serial.printf("SWING REFUSED: let it hang still (now %.1f deg, %.2f rad/s).\n",
                      psi * 180.0f / PI, angularRate);
        return;
    }
    motor.actualPosition(0); // S 누른 지점이 스윙과 밸런스 모두의 원점
    cartPosition = 0.0f;
    commandedVelocity = 0.0f;
    swingTargetVelocity = 0.0f;
    swingEnergyBias = 0.0f;
    swingPhase = PHASE_PREPOSITION;
    swingKickPending = true;
    pushedThisPass = false;
    apexHandled = false;
    previousSwingRate = angularRate;
    lastControlUs = micros();
    swingStartMs = millis();
    phaseStartMs = swingStartMs;
    digitalWrite(PIN_TMC_EN, LOW);
    setMode(MODE_SWING);
    Serial.printf("ARMED: swing-up (L=%.3f m), moving to start. X to stop.\n",
                  pendulumLengthM);
}

// 킥은 controlPolarity 방향으로 나가므로 그 반대쪽 끝에서 출발한다.
float swingStartX() { return -controlPolarity * SWING_DIRECTION * SWING_START_OFFSET_M; }

// PREPOSITION: +a 로 T0, -a 로 T0. 각 구간이 진자 주기 T0 와 같으면 선형 근사에서
// 잔류 흔들림이 0 이다. 끝나면 속도를 0 으로 맞추고 돌려준 값은 무시된다.
float updatePreposition(float dt)
{
    const float period = 2.0f * PI * sqrt(pendulumLengthM / GRAVITY);
    const float t = (millis() - phaseStartMs) * 1.0e-3f;
    const float a = swingStartX() / (period * period);
    if (t < period)
        return a;
    if (t < 2.0f * period)
        return -a;
    commandedVelocity = 0.0f;
    swingPhase = PHASE_SETTLE;
    phaseStartMs = millis();
    return 0.0f;
}

// 스윙업 한 스텝. 카트 지령 가속도를 돌려준다. 캐치하면 모드를 바꾼다.
float updateSwingUp(float angle, float rate, float dt)
{
    const float L = pendulumLengthM;
    const float w2 = GRAVITY / L;
    const float psi = wrapPi(angle - PI); // 바닥 기준 각도
    const float energy = pendulumEnergy(angle, rate);
    const float v = commandedVelocity;
    const bool towardBottom = psi * rate < 0.0f;
    const bool upperHalf = fabsf(angle) < 0.5f * PI;

    if (swingPhase == PHASE_PREPOSITION)
        return updatePreposition(dt);
    if (swingPhase == PHASE_SETTLE)
    {
        const bool still = fabsf(psi) < SWING_START_ANGLE_RAD && fabsf(rate) < SWING_START_RATE;
        if (!still && millis() - phaseStartMs < SWING_SETTLE_TIMEOUT_MS)
            return 0.0f;
        swingPhase = PHASE_PUMP;
        swingStartMs = millis(); // 타임아웃과 CATCH 시간은 킥부터 잰다
        previousSwingRate = rate;
        Serial.printf("KICK: from X=%+.3f m\n", cartPosition);
    }

    // 윗반원 꼭짓점(각속도 부호 반전)인데 캐치 창 밖이면 에너지가 모자랐던 것.
    // 모자란 만큼 일부를 목표에 더하고 다시 펌핑한다.
    if (!upperHalf)
        apexHandled = false;
    if (upperHalf && !apexHandled && previousSwingRate * rate < 0.0f &&
        fabsf(angle) >= CATCH_WINDOW_RAD)
    {
        apexHandled = true;
        swingEnergyBias += SWING_RETRY_GAIN * (1.0f - cos(angle));
        swingPhase = PHASE_PUMP;
        pushedThisPass = false;
    }
    // COAST(한 번 더 차면 충분하다고 본 상태)인데 수평에도 못 가고 아랫반원에서
    // 되돌아오면 계산만큼 에너지가 안 들어간 것. 다시 펌핑한다.
    // (이게 없어서 카트가 끝에 선 채 타임아웃까지 멈춰 있었다.)
    if (swingPhase == PHASE_COAST && !upperHalf && previousSwingRate * rate < 0.0f)
    {
        swingPhase = PHASE_PUMP;
        pushedThisPass = false;
    }
    previousSwingRate = rate;

    // 킥 이후엔 단계와 무관하게 캐치 창에 들어오면 잡는다. RISE 에서만 잡으면,
    // 레일 끝 제동처럼 계산에 없던 에너지로 PUMP 상태인 채 꼭대기를 지날 때
    // 그냥 넘어가서 한 바퀴를 돈다.
    if (fabsf(angle) < CATCH_WINDOW_RAD)
    {
        // commandedVelocity 는 그대로 이어받아 업라이트 컨트롤러가 적분을 계속한다.
        setMode(MODE_BALANCE);
        relaxedTripUntilMs = millis() + CATCH_SETTLE_MS;
        catchMs = millis();
        Serial.printf("CATCH: %.1f deg, %.2f rad/s, X=%+.3f m, V=%+.2f m/s, phase=%d, "
                      "E=%+.2f after %lu ms\n",
                      angle * 180.0f / PI, rate, cartPosition, commandedVelocity,
                      static_cast<int>(swingPhase), energy,
                      static_cast<unsigned long>(millis() - swingStartMs));
        return 0.0f;
    }
    if (swingPhase == PHASE_RISE && !upperHalf && towardBottom)
    {
        swingPhase = PHASE_PUMP; // 못 올라가고 떨어짐
        pushedThisPass = false;
    }

    if (swingPhase != PHASE_RISE)
    {
        if (!towardBottom)
            pushedThisPass = false;

        if (swingPhase == PHASE_PUMP && (swingKickPending || !pushedThisPass))
        {
            // 바닥에서 카트 속도가 dv 바뀌면 진자 각속도는 controlPolarity*dv/L 바뀐다
            // (upright 에서 검증된 극성: thetaDD = (g sin - pol * a cos) / L).
            const float bottomRate = sqrt(max(0.0f, 2.0f * w2 * (energy + 2.0f)));
            const float targetRate =
                sqrt(2.0f * w2 * (SWING_ENERGY_MARGIN + swingEnergyBias + 2.0f));
            const float dvMagnitude = max(0.0f, targetRate - bottomRate) * L;
            const float direction = swingKickPending ? SWING_DIRECTION : (rate >= 0.0f ? 1.0f : -1.0f);
            const float desired = v + controlPolarity * direction * dvMagnitude;
            const float next = constrain(desired, -SWING_SPEED_MPS, SWING_SPEED_MPS);
            // 가속 구간의 가운데가 바닥에 오도록 미리 시작한다.
            const float timeToBottom = fabsf(psi) / max(fabsf(rate), 1e-3f);
            const float leadTime = fabsf(next - v) / (2.0f * SWING_ACCEL_MPS2) + dt;
            if (swingKickPending || (towardBottom && timeToBottom < leadTime))
            {
                swingTargetVelocity = next;
                swingKickPending = false;
                pushedThisPass = true;
                if (fabsf(desired - next) < 1e-4f)
                    swingPhase = PHASE_COAST; // 이 계단으로 목표 에너지 도달
            }
        }

        // 수평 통과: cos ~ 0 이라 여기서 멈추면 진자 에너지가 거의 안 변한다.
        const float brakeLead = fabsf(rate) * (fabsf(v) / SWING_ACCEL_MPS2) * 0.5f;
        if (!towardBottom && fabsf(angle) < 0.5f * PI + brakeLead)
        {
            swingTargetVelocity = 0.0f;
            if (swingPhase == PHASE_COAST)
                swingPhase = PHASE_RISE;
        }
    }

    // 지금 멈춰도 리밋을 넘을 상황이면 무조건 멈춘다. 시작 위치/속도 조합에서는
    // 카트가 반대편 끝에 닿는 시점이 진자의 바닥 복귀와 거의 겹쳐서, 이 제동도
    // 진자를 차올리는 역할을 한다.
    const float stopX = cartPosition + v * fabsf(v) / (2.0f * SWING_ACCEL_MPS2);
    // 리밋 쪽으로 가는 목표만 막는다. 반대 방향 킥까지 막으면 끝에 붙은 채 멈춘다.
    if (fabsf(stopX) > CART_SOFT_LIMIT_M - 0.03f && swingTargetVelocity * stopX > 0.0f)
        swingTargetVelocity = 0.0f;

    return constrain((swingTargetVelocity - v) / dt,
                     -SWING_ACCEL_MPS2, SWING_ACCEL_MPS2);
}

// --- 모터 추종 테스트 (T) ---
// 스윙과 같은 SWING_SPEED / SWING_ACCEL 로 MOTION_TEST_DISTANCE 만큼 훅 갔다가,
// 잠깐 서고, 느리게(탈조 없는 속도로) 원래 자리로 돌아온다. 지령은 정확히
// 제자리이므로, 카트가 출발 표시 자리로 안 돌아오면 빠른 구간에서 스텝을
// 잃은 것이다. X 는 보낸 펄스 수라서 탈조를 보여주지 못한다.
constexpr float MOTION_TEST_DISTANCE_M = 0.15f;
constexpr float MOTION_TEST_RETURN_SPEED_MPS = 0.10f;
constexpr float MOTION_TEST_RETURN_ACCEL_MPS2 = 0.5f;
uint8_t motionTestStage = 0;
uint32_t motionTestStageMs = 0;

void startMotionTest()
{
    if (armed || measuring)
    {
        Serial.println("TEST REFUSED: stop (X) first.");
        return;
    }
    motor.actualPosition(0);
    cartPosition = 0.0f;
    commandedVelocity = 0.0f;
    motionTestStage = 0;
    lastControlUs = micros();
    digitalWrite(PIN_TMC_EN, LOW);
    setMode(MODE_MOTION_TEST);
    Serial.printf("ARMED: motion test %.2f m at %.2f m/s, %.1f m/s^2. "
                  "Mark the cart position now.\n",
                  MOTION_TEST_DISTANCE_M, SWING_SPEED_MPS, SWING_ACCEL_MPS2);
}

// target 까지 speed/accel 한계 안에서 정지 가능한 속도로 간다. 도착하면 true.
bool driveTo(float target, float speed, float accel, float dt)
{
    const float remaining = target - cartPosition;
    if (fabsf(remaining) < 0.0005f)
    {
        commandedVelocity = 0.0f;
        return true;
    }
    const float direction = remaining > 0.0f ? 1.0f : -1.0f;
    float magnitude = min(speed, sqrt(2.0f * accel * fabsf(remaining)));
    magnitude = min(magnitude, fabsf(commandedVelocity) + accel * dt);
    commandedVelocity = direction * magnitude;
    return false;
}

void updateMotionTest(float dt)
{
    switch (motionTestStage)
    {
    case 0: // 빠르게 나감
        if (driveTo(controlPolarity * SWING_DIRECTION * MOTION_TEST_DISTANCE_M,
                    SWING_SPEED_MPS, SWING_ACCEL_MPS2, dt))
        {
            motionTestStage = 1;
            motionTestStageMs = millis();
        }
        break;
    case 1: // 잠깐 정지
        if (millis() - motionTestStageMs > 500)
            motionTestStage = 2;
        break;
    default: // 천천히 복귀
        if (driveTo(0.0f, MOTION_TEST_RETURN_SPEED_MPS, MOTION_TEST_RETURN_ACCEL_MPS2, dt))
        {
            disarm("motion test done. Cart should be back on the mark; "
                   "if not, steps were lost on the fast move.");
            // 테스트 후 원점을 다시 잡을 필요는 없다. S/A 가 각자 원점을 잡는다.
        }
        break;
    }
}

void startLengthMeasurement()
{
    if (!calibrated || armed)
    {
        Serial.println("MEASURE REFUSED: needs Z first and disarmed.");
        return;
    }
    measuring = true;
    measureSide = 0;
    measureCrossings = 0;
    measureAmplitude = 0.0f;
    measureStartMs = millis();
    Serial.println("MEASURE: give the hanging pendulum a small (10-20 deg) swing.");
}

void updateLengthMeasurement(float angle, float rate, uint32_t nowUs)
{
    const float psi = wrapPi(angle - PI);
    if (millis() - measureStartMs > MEASURE_TIMEOUT_MS)
    {
        measuring = false;
        Serial.println("MEASURE TIMEOUT: no steady swing seen.");
        return;
    }
    measureAmplitude = max(measureAmplitude, fabsf(psi));
    const int8_t side = psi > 0.0f ? 1 : (psi < 0.0f ? -1 : measureSide);
    // 바닥 통과(부호 변화)만 센다. 너무 느린 통과는 노이즈라 버린다.
    if (measureSide != 0 && side != measureSide && fabsf(rate) > 0.3f)
    {
        if (measureCrossings == 0)
        {
            measureFirstUs = nowUs;
            measureAmplitude = fabsf(psi);
        }
        measureLastUs = nowUs;
        ++measureCrossings;
        if (measureCrossings > MEASURE_HALF_PERIODS)
        {
            measuring = false;
            const float period =
                2.0f * (measureLastUs - measureFirstUs) * 1.0e-6f / MEASURE_HALF_PERIODS;
            // 진폭에 따른 주기 증가 보정 T = T0 (1 + A^2/16)
            const float period0 =
                period / (1.0f + measureAmplitude * measureAmplitude / 16.0f);
            const float length = GRAVITY * sq(period0 / (2.0f * PI));
            if (length < 0.03f || length > 1.0f)
            {
                Serial.printf("MEASURE FAILED: T=%.3f s gives L=%.3f m.\n", period, length);
                return;
            }
            pendulumLengthM = length;
            Serial.printf("MEASURE: T=%.3f s amp=%.1f deg -> L=%.3f m "
                          "(RAM only; put it in pendulumLengthM to keep it)\n",
                          period, measureAmplitude * 180.0f / PI, length);
        }
    }
    measureSide = side;
}

void serviceStepper()
{
    if (!armed)
        return;
    // 컨트롤러가 이미 가속도를 적분해 속도를 만들었으므로 그대로 직결한다.
    // 펄스 간격은 LEDC 하드웨어가 만들므로 이 함수의 호출 주기나 Serial
    // 출력과 무관하게 정확하다.
    motor.setVelocityDirect(commandedVelocity * STEPS_PER_METRE);
}

void updateController()
{
    const uint32_t now = micros();
    if (now - lastControlUs < CONTROL_PERIOD_US)
        return;
    const float dt = (now - lastControlUs) * 1.0e-6f;
    lastControlUs = now;

    static int32_t previousEncoderCount = 0;
    const int32_t count = readEncoderCount();
    const float angle = uprightAngleFromCount(count);
    const float rawRate = (count - previousEncoderCount) * RAD_PER_COUNT / dt;
    previousEncoderCount = count;
    angularRate += 0.25f * (rawRate - angularRate);
    cartPosition = motor.getSPIPosition() / STEPS_PER_METRE;

    if (measuring)
        updateLengthMeasurement(angle, angularRate, now);
    if (!armed)
        return;
    if (fabsf(cartPosition) > CART_SOFT_LIMIT_M)
    {
        disarm("cart exceeded the 350 mm software limit");
        return;
    }
    if (mode == MODE_MOTION_TEST)
    {
        updateMotionTest(dt);
        return;
    }
    if (mode == MODE_SWING)
    {
        if (millis() - swingStartMs > SWING_TIMEOUT_MS)
        {
            disarm("swing-up timed out");
            return;
        }
        const float swingAcceleration = updateSwingUp(angle, angularRate, dt);
        if (mode == MODE_SWING)
        {
            commandedVelocity += swingAcceleration * dt;
            commandedVelocity = constrain(commandedVelocity,
                                          -SWING_SPEED_MPS, SWING_SPEED_MPS);
            return;
        }
        // 캐치됨: 같은 틱에서 바로 업라이트 제어로 넘어간다.
    }
    const float tripAngle =
        millis() < relaxedTripUntilMs ? CATCH_TRIP_RAD : TRIP_ANGLE_RAD;
    if (fabsf(angle) > tripAngle)
    {
        disarm("pendulum exceeded the trip angle");
        return;
    }

    // 카트 항의 부호가 핵심이다. 역진자는 비최소위상계라서, +X 에 있는 카트를
    // 중앙으로 되돌리려면 먼저 카트를 +X 로 가속해야 한다. 그래야 진자가 -X 로
    // 기울고, 각도 루프가 그 기울기를 따라가며 카트를 -X 로 끌고 온다.
    // 즉 위치/속도 항은 각도 항과 '같은' 방향(+)으로 들어가야 한다.
    // 기존 코드는 - 였고, 그래서 카트 위치가 규제되지 않고 한쪽으로 밀려나
    // 리밋에 닿았다. (진자는 잘 세운 채로 쭉 가던 증상)
    const uint32_t sinceCatchMs = millis() - catchMs;
    const float blend =
        sinceCatchMs < CATCH_STIFF_MS
            ? 0.0f
            : min(1.0f, (sinceCatchMs - CATCH_STIFF_MS) / static_cast<float>(CATCH_BLEND_MS));
    const float kCartPos = CATCH_KCARTPOS + (gKcartPos - CATCH_KCARTPOS) * blend;
    const float kCartVel = CATCH_KCARTVEL + (gKcartVel - CATCH_KCARTVEL) * blend;
    float acceleration =
        controlPolarity * (gKangle * (angle - angleTrim) + gKrate * angularRate) +
        cartFeedbackSign * (kCartPos * cartPosition + kCartVel * commandedVelocity);
    acceleration = constrain(acceleration,
                             -MAX_CART_ACCEL_MPS2,
                             MAX_CART_ACCEL_MPS2);
    commandedVelocity += acceleration * dt;
    commandedVelocity = constrain(commandedVelocity,
                                  -MAX_CART_SPEED_MPS,
                                  MAX_CART_SPEED_MPS);
}

void printGains()
{
    Serial.printf("GAINS angle=%.2f rate=%.2f cartPos=%.2f cartVel=%.2f "
                  "trim=%+.2fdeg pol=%d cartSign=%d L=%.3fm\n",
                  gKangle, gKrate, gKcartPos, gKcartVel,
                  angleTrim * 180.0f / PI, controlPolarity, cartFeedbackSign,
                  pendulumLengthM);
}

void printHelp()
{
    Serial.println("Commands: Z=zero while hanging DOWN, A=arm while held UP,");
    Serial.println("          S=swing up from hanging still, M=measure pendulum length,");
    Serial.println("          T=fast motion test (cart must return to its start mark),");
    Serial.println("          X=disarm, P=flip angle polarity (disarmed only), H=help");
    Serial.println("Gains are compile-time constants: edit the top of this file,");
    Serial.println("then rebuild and reflash. Current build:");
    printGains();
}

void handleSerial()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 'z' || command == 'Z')
        {
            if (armed)
                disarm("zero requested");
            zeroEncoderDownward();
            Serial.println("DOWNWARD ZERO SET. Upright should read near 0 deg at +/-1200 counts.");
        }
        else if (command == 'a' || command == 'A')
        {
            armController();
        }
        else if (command == 't' || command == 'T')
        {
            startMotionTest();
        }
        else if (command == 's' || command == 'S')
        {
            startSwingUp();
        }
        else if (command == 'm' || command == 'M')
        {
            startLengthMeasurement();
        }
        else if (command == 'x' || command == 'X' || command == ' ')
        {
            if (measuring)
            {
                measuring = false;
                Serial.println("MEASURE CANCELLED.");
            }
            if (armed)
                disarm("operator stop");
        }
        else if (command == 'p' || command == 'P')
        {
            if (armed)
            {
                Serial.println("DISARM before changing polarity.");
            }
            else
            {
                controlPolarity = -controlPolarity;
                Serial.printf("Feedback polarity is now %d.\n", controlPolarity);
            }
        }
        else if (command == 'h' || command == 'H')
        {
            printHelp();
        }
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_ENCODER_A, INPUT); // GPIO34/35 require external pull-ups.
    pinMode(PIN_ENCODER_B, INPUT);
    pinMode(PIN_TMC_EN, OUTPUT);
    pinMode(PIN_TMC_STEP, OUTPUT);
    pinMode(PIN_TMC_DIR, OUTPUT);
    pinMode(PIN_TMC_CS, OUTPUT);
    digitalWrite(PIN_TMC_EN, HIGH);
    digitalWrite(PIN_TMC_STEP, LOW);
    digitalWrite(PIN_TMC_DIR, LOW);
    digitalWrite(PIN_TMC_CS, HIGH);

    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TMC_CS);
    delay(300);
    Serial.println("\n=== HAND-HELD UPRIGHT BALANCE TEST ===");
    const uint32_t ioin = readRegister(REG_IOIN);
    const uint8_t version = static_cast<uint8_t>(ioin >> 24);
    Serial.printf("TMC IOIN=0x%08lX VERSION=0x%02X\n", ioin, version);
    if (version != 0x30)
    {
        Serial.println("TMC SPI ERROR: driver will remain disabled.");
        while (true)
            delay(1000);
    }
    configureDriver();
    // motor.init() 이 끝에서 EN 을 LOW 로 내려 드라이버를 켠다. 이 테스트는
    // arm 전까지 출력이 꺼져 있어야 (Z 로 0점 잡을 때 손으로 움직일 수 있게)
    // 하므로 다시 비활성으로 돌린다. armController() 가 LOW 로 내린다.
    digitalWrite(PIN_TMC_EN, HIGH);
    printHelp();
    Serial.println("Begin with VM=12 V. Do not use 36 V for the first balance test.");
}

void loop()
{
    handleSerial();
    updateController();
    serviceStepper();

    static uint32_t lastReportMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastReportMs >= REPORT_PERIOD_MS)
    {
        const int32_t count = readEncoderCount();
        const float angle = uprightAngleFromCount(count);
        Serial.printf("%s C=%ld ANG=%+.2fdeg RATE=%+.2f X=%+.3fm V=%+.3fm/s POL=%d E=%+.2f\n",
                      mode == MODE_SWING ? "SWING" : (mode == MODE_MOTION_TEST ? "TEST" : (armed ? "ARM" : "SAFE")),
                      static_cast<long>(count),
                      angle * 180.0f / PI,
                      angularRate,
                      cartPosition,
                      commandedVelocity,
                      controlPolarity,
                      pendulumEnergy(angle, angularRate));
        lastReportMs = nowMs;
    }
}
