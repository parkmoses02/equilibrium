// Slow, exact-distance cart move, to compare the COUNTED distance (STEP pulses / STEPS_PER_METRE)
// with the REAL distance measured with a ruler. No encoder, no camera.
//
// Keys in the serial monitor (115200). Put the cart near the rail CENTRE first.
//   a   move  +step     d   move  -step     (+ = the direction in which the STEP count increases)
//   1/2/3  step = 0.05 / 0.10 / 0.15 counted m (default 0.05)
//   o   define the current position as 0 (the +-0.20 limit is measured from it)
//   x or any other key   stop at once and release the driver (also while moving)
// Safety: speed 0.10 counted m/s, acceleration 0.5 counted m/s^2, a move that would end beyond
//   +-0.20 counted m from the zero is refused, hard stop at +-0.21, 10 s timeout per move.
//   0.20 counted m is about 0.30 m real (the rail allows +-0.35 m real).
// After a move the driver keeps holding the cart. Measure with the ruler, then press x to release.
// Driver setup / pins are copied from pipeline_energy_lqr.cpp.

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>

constexpr uint8_t PIN_TMC_EN   = 13; // active LOW
constexpr uint8_t PIN_TMC_STEP = 14;
constexpr uint8_t PIN_TMC_DIR  = 27;
constexpr uint8_t PIN_TMC_CS   = 5;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_SCK  = 18;

constexpr float STEPS_PER_METRE = (200.0f * 16.0f) / 0.040f; // 80,000 steps/m (counted, not real)
constexpr float V_CRUISE = 0.10f;      // counted m/s
constexpr float ACCEL = 0.5f;          // counted m/s^2
constexpr float LIMIT_M = 0.20f;       // refuse moves ending beyond this (counted m)
constexpr float TRIP_M = 0.21f;
constexpr uint32_t MOVE_TIMEOUT_MS = 10000;
constexpr uint32_t TICK_US = 2000;

constexpr uint8_t REG_GCONF = 0x00;
constexpr uint8_t REG_IOIN  = 0x04;
constexpr uint8_t WRITE_FLAG = 0x80;
SPISettings tmcSpi(1000000, MSBFIRST, SPI_MODE3);
TMCStepDir motor(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_TMC_CS,
                 PIN_TMC_EN, PIN_TMC_STEP, PIN_TMC_DIR);
constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;
uint32_t spsToUnit(float sps) { return (uint32_t)(sps / VEL_UNIT_TO_SPS); }

static void writeRegister(uint8_t address, uint32_t value) {
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
static uint32_t readRegister(uint8_t address) {
    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address & 0x7F);
    for (uint8_t i = 0; i < 4; ++i) SPI.transfer(0);
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

static bool driverOk = false;
static bool detectDriver() {
    const uint32_t ioin = readRegister(REG_IOIN);
    Serial.printf("TMC5160 IOIN=0x%08lX (version byte 0x%02X, expected 0x30)\n",
                  (unsigned long)ioin, (unsigned)(ioin >> 24));
    driverOk = (static_cast<uint8_t>(ioin >> 24) == 0x30);
    if (driverOk) {
        motor.init(0.4f, 1.0f, 4, 192); // 1/16 microsteps, spreadCycle
        writeRegister(REG_GCONF, 0x00000000);
        writeRegister(0x13, 0); // TPWMTHRS = 0
        motor.setSpeed(spsToUnit(V_CRUISE * 2.0f * STEPS_PER_METRE));
        motor.setAcceleration(0xFFFF);
    }
    digitalWrite(PIN_TMC_EN, HIGH);
    return driverOk;
}

static bool moving = false;
static bool holding = false;
static float stepM = 0.05f;
static float targetM = 0.0f;       // counted m, relative to the zero
static float startM = 0.0f;
static float cmdVelocity = 0.0f;
static uint32_t moveStartMs = 0, lastTickUs = 0;
static int32_t zeroSteps = 0;

static int32_t rawSteps() { return (int32_t)motor.getSPIPosition(); }
static float positionM() { return (rawSteps() - zeroSteps) / STEPS_PER_METRE; }

static void release(const char* reason) {
    moving = false;
    holding = false;
    cmdVelocity = 0.0f;
    motor.setVelocityDirect(0.0f);
    digitalWrite(PIN_TMC_EN, HIGH);
    Serial.printf("RELEASED: %s (position %.4f counted m)\n", reason, positionM());
}

static void startMove(float delta) {
    if (!driverOk && !detectDriver()) { Serial.println("REFUSED: TMC5160 not detected."); return; }
    const float pos = positionM();
    if (fabsf(pos + delta) > LIMIT_M) {
        Serial.printf("REFUSED: %+.3f from %.4f would end beyond +-%.2f counted m. Move the cart back and press o.\n",
                      delta, pos, LIMIT_M);
        return;
    }
    startM = pos;
    targetM = pos + delta;
    cmdVelocity = 0.0f;
    moveStartMs = millis();
    lastTickUs = micros();
    digitalWrite(PIN_TMC_EN, LOW);
    moving = true;
    holding = true;
    Serial.printf("MOVING %+.4f counted m (from %.4f to %.4f) at <= %.2f m/s ...\n", delta, startM, targetM, V_CRUISE);
}

static void finishMove() {
    moving = false;
    cmdVelocity = 0.0f;
    motor.setVelocityDirect(0.0f);
    const float pos = positionM();
    Serial.printf("DONE: moved %+.4f counted m (steps %+ld); position now %.4f counted m. "
                  "Measure the real distance with a ruler; x releases the driver.\n",
                  pos - startM, (long)lroundf((pos - startM) * STEPS_PER_METRE), pos);
}

static void updateMove(float dt) {
    const float pos = positionM();
    if (fabsf(pos) > TRIP_M) { release("position limit"); return; }
    if (millis() - moveStartMs > MOVE_TIMEOUT_MS) { release("move timeout"); return; }
    const float rem = targetM - pos;
    if (fabsf(rem) < 3.0f / STEPS_PER_METRE) { finishMove(); return; }
    const float dir = rem > 0.0f ? 1.0f : -1.0f;
    float speed = fminf(V_CRUISE, sqrtf(2.0f * ACCEL * fabsf(rem)));
    speed = fmaxf(speed, 0.01f);                 // keep creeping until the target is reached
    const float desired = dir * speed;
    cmdVelocity += constrain(desired - cmdVelocity, -ACCEL * dt, ACCEL * dt);
    motor.setVelocityDirect(cmdVelocity * STEPS_PER_METRE);
}

static void handleSerial() {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') continue;
        if (moving) { release("key pressed while moving"); continue; }
        switch (c) {
            case 'a': startMove(+stepM); break;
            case 'd': startMove(-stepM); break;
            case '1': stepM = 0.05f; Serial.println("step = 0.05 counted m"); break;
            case '2': stepM = 0.10f; Serial.println("step = 0.10 counted m"); break;
            case '3': stepM = 0.15f; Serial.println("step = 0.15 counted m"); break;
            case 'o':
                zeroSteps = rawSteps();
                Serial.println("position zero set here");
                break;
            default:
                release("x");
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_TMC_EN, OUTPUT);
    pinMode(PIN_TMC_STEP, OUTPUT);
    pinMode(PIN_TMC_DIR, OUTPUT);
    pinMode(PIN_TMC_CS, OUTPUT);
    digitalWrite(PIN_TMC_EN, HIGH);
    digitalWrite(PIN_TMC_STEP, LOW);
    digitalWrite(PIN_TMC_DIR, LOW);
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TMC_CS);
    delay(300);
    detectDriver();
    zeroSteps = rawSteps();
    Serial.println("cart_move_test: cart near the rail centre; a = +step, d = -step, 1/2/3 = 0.05/0.10/0.15 m, o = zero, x = release");
}

void loop() {
    handleSerial();
    const uint32_t now = micros();
    if (moving && (uint32_t)(now - lastTickUs) >= TICK_US) {
        const float dt = (now - lastTickUs) * 1e-6f;
        lastTickUs = now;
        updateMove(dt);
    }
}
