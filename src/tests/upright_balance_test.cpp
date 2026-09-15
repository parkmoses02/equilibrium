#include <Arduino.h>
#include <SPI.h>
#include <math.h>

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
constexpr float K_ANGLE = 50.0f;
constexpr float K_ANGULAR_RATE = 8.0f; // raised with K_ANGLE to keep damping
constexpr float K_CART_POSITION = 8.0f;
constexpr float K_CART_VELOCITY = 5.0f;

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
    writeRegister(REG_GCONF, 0x00000000);
    // Last known-working current setting from the oscillation test.
    writeRegister(REG_GLOBALSCALER, 192);
    writeRegister(REG_IHOLD_IRUN, (6UL << 16) | (15UL << 8) | 6UL);
    writeRegister(REG_TPOWERDOWN, 10);
    // SpreadCycle, 1/16 microstep, TOFF=3, HSTRT=4, HEND=1, TBL=2.
    const uint32_t chopconf =
        (4UL << 24) | (2UL << 15) | (1UL << 7) | (4UL << 4) | 3UL;
    writeRegister(REG_CHOPCONF, chopconf);
}

void disarm(const char *reason)
{
    armed = false;
    commandedVelocity = 0.0f;
    digitalWrite(PIN_TMC_STEP, LOW);
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

    const float pulseRate = fabsf(commandedVelocity) * STEPS_PER_METRE;
    if (pulseRate < 1.0f)
    {
        digitalWrite(PIN_TMC_STEP, LOW);
        return;
    }

    const bool positive = commandedVelocity > 0.0f;
    digitalWrite(PIN_TMC_DIR, positive ? HIGH : LOW);
    const uint32_t periodUs = static_cast<uint32_t>(1000000.0f / pulseRate);
    const uint32_t now = micros();
    if (static_cast<int32_t>(now - nextStepUs) >= 0)
    {
        digitalWrite(PIN_TMC_STEP, HIGH);
        delayMicroseconds(2);
        digitalWrite(PIN_TMC_STEP, LOW);
        cartStepCount += positive ? 1 : -1;
        nextStepUs += periodUs;
        if (static_cast<int32_t>(now - nextStepUs) > static_cast<int32_t>(periodUs))
        {
            nextStepUs = now + periodUs;
        }
    }
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
    cartPosition = cartStepCount / STEPS_PER_METRE;

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

    float acceleration =
        controlPolarity * (K_ANGLE * angle + K_ANGULAR_RATE * angularRate) - K_CART_POSITION * cartPosition - K_CART_VELOCITY * commandedVelocity;
    acceleration = constrain(acceleration,
                             -MAX_CART_ACCEL_MPS2,
                             MAX_CART_ACCEL_MPS2);
    commandedVelocity += acceleration * dt;
    commandedVelocity = constrain(commandedVelocity,
                                  -MAX_CART_SPEED_MPS,
                                  MAX_CART_SPEED_MPS);
}

void printHelp()
{
    Serial.println("Commands: Z=zero while hanging DOWN, A=arm while held UP,");
    Serial.println("          X=disarm, P=flip feedback polarity (only while disarmed), H=help");
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
