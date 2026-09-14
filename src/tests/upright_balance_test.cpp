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
constexpr float CART_SOFT_LIMIT_M = 0.30f;   // relative to arm position
constexpr float ARM_WINDOW_RAD = 10.0f * PI / 180.0f;
constexpr float TRIP_ANGLE_RAD = 22.0f * PI / 180.0f;
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz
constexpr uint32_t REPORT_PERIOD_MS = 100;

// Initial upright gains. These are intentionally moderate and must be tuned on
// the actual mechanism. Output is commanded cart acceleration in m/s^2.
// 런타임에 시리얼로 조정한다. 재컴파일 없이 튜닝하기 위함.
float gKangle = 50.0f;
float gKrate = 8.0f;
// 카트 항은 부호를 바로잡으면서 보수적으로 낮춰 잡았다. 아래 주석 참고.
float gKcartPos = 3.0f;
float gKcartVel = 2.0f;
// 업라이트 기준각 보정 [rad]. 진자 무게중심/인코더 장착 오차로 '수직'이
// 정확히 0 이 아니면 컨트롤러가 약간 기운 자세를 유지하려 하고, 그러려면
// 계속 가속해야 해서 카트가 한쪽으로 끝없이 흘러간다.
float angleTrim = 0.0f;
// 카트 위치/속도 피드백의 방향. 아래 부호 설명 참고. C 로 뒤집어 확인 가능.
int8_t cartFeedbackSign = 1;

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

bool calibrated = false;
bool armed = false;
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

    motor.setSpeed(spsToUnit(MAX_CART_SPEED_MPS * STEPS_PER_METRE));
    // 직결 속도 지령을 쓰므로 내부 램프 가속도는 상한 역할만 한다.
    motor.setAcceleration(0xFFFF);
}

void disarm(const char *reason)
{
    armed = false;
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
    digitalWrite(PIN_TMC_EN, LOW);
    armed = true;
    Serial.println("ARMED: keep one hand near the pendulum and X ready to stop.");
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

    if (!armed)
        return;
    if (fabsf(angle) > TRIP_ANGLE_RAD)
    {
        disarm("pendulum exceeded 22 degrees");
        return;
    }
    if (fabsf(cartPosition) > CART_SOFT_LIMIT_M)
    {
        disarm("cart exceeded the 350 mm software limit");
        return;
    }

    // 카트 항의 부호가 핵심이다. 역진자는 비최소위상계라서, +X 에 있는 카트를
    // 중앙으로 되돌리려면 먼저 카트를 +X 로 가속해야 한다. 그래야 진자가 -X 로
    // 기울고, 각도 루프가 그 기울기를 따라가며 카트를 -X 로 끌고 온다.
    // 즉 위치/속도 항은 각도 항과 '같은' 방향(+)으로 들어가야 한다.
    // 기존 코드는 - 였고, 그래서 카트 위치가 규제되지 않고 한쪽으로 밀려나
    // 리밋에 닿았다. (진자는 잘 세운 채로 쭉 가던 증상)
    float acceleration =
        controlPolarity * (gKangle * (angle - angleTrim) + gKrate * angularRate) +
        cartFeedbackSign * (gKcartPos * cartPosition + gKcartVel * commandedVelocity);
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
                  "trim=%+.2fdeg pol=%d cartSign=%d\n",
                  gKangle, gKrate, gKcartPos, gKcartVel,
                  angleTrim * 180.0f / PI, controlPolarity, cartFeedbackSign);
}

void printHelp()
{
    Serial.println("Commands: Z=zero while hanging DOWN, A=arm while held UP,");
    Serial.println("          X=disarm, P=flip angle polarity (disarmed only), H=help");
    Serial.println("Tuning (live, works while armed):");
    Serial.println("  [ ] = angle trim -/+ 0.2deg   <- use this to stop steady drift");
    Serial.println("  1/2 = K_angle -/+10%    3/4 = K_rate -/+10%");
    Serial.println("  5/6 = K_cartPos -/+10%  7/8 = K_cartVel -/+10%");
    Serial.println("  C = flip cart feedback sign   G = show gains");
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
        else if (command == 'x' || command == 'X' || command == ' ')
        {
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
        else if (command == 'c' || command == 'C')
        {
            cartFeedbackSign = -cartFeedbackSign;
            Serial.printf("Cart feedback sign is now %d.\n", cartFeedbackSign);
        }
        else if (command == '[')
        {
            angleTrim -= 0.2f * PI / 180.0f;
            Serial.printf("angleTrim = %+.2f deg\n", angleTrim * 180.0f / PI);
        }
        else if (command == ']')
        {
            angleTrim += 0.2f * PI / 180.0f;
            Serial.printf("angleTrim = %+.2f deg\n", angleTrim * 180.0f / PI);
        }
        else if (command == '1') { gKangle *= 0.9f; printGains(); }
        else if (command == '2') { gKangle *= 1.1f; printGains(); }
        else if (command == '3') { gKrate *= 0.9f; printGains(); }
        else if (command == '4') { gKrate *= 1.1f; printGains(); }
        else if (command == '5') { gKcartPos *= 0.9f; printGains(); }
        else if (command == '6') { gKcartPos *= 1.1f; printGains(); }
        else if (command == '7') { gKcartVel *= 0.9f; printGains(); }
        else if (command == '8') { gKcartVel *= 1.1f; printGains(); }
        else if (command == 'g' || command == 'G')
        {
            printGains();
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
        Serial.printf("%s C=%ld ANG=%+.2fdeg RATE=%+.2f X=%+.3fm V=%+.3fm/s POL=%d\n",
                      armed ? "ARM" : "SAFE",
                      static_cast<long>(count),
                      angle * 180.0f / PI,
                      angularRate,
                      cartPosition,
                      commandedVelocity,
                      controlPolarity);
        lastReportMs = nowMs;
    }
}
