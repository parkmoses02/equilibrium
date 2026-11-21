// Cart-acceleration calibration run: the cart sweeps a gentle sine CHIRP while the pendulum hangs
// (it is NOT balanced). Encoder angle, cart command and camera packets are logged on one clock so
// tools/cart_accel_fit.py can fit  phi'' = K sin(phi) - C phi' + B cos(phi) a  and the cart px/m scale.
//
// Run it only from tools/cart_accel_fit.py capture (it sends 'l', 'z', 'g' and keep-alive 'k' bytes).
// Safety, any one of these stops the cart immediately and disables the driver:
//   - any key other than l/z/g/k,  - no 'k' within 1 s,  - cart more than 9.5 cm from the start,
//   - pendulum more than 120 deg from hanging,  - the run time is over.
// The cart starts wherever it is (put it at the CENTRE of the rail). Position is open-loop STEP counting.
//
// Chirp: 0.5 -> 2.0 Hz over 30 s, acceleration amplitude 1 m/s^2 (position amplitude capped at 8 cm),
//   speed <= 0.5 m/s, acceleration <= 3 m/s^2. Driver setup is copied from pipeline_energy_lqr.cpp.
// Log lines (only while logging; '#' lines are status), USB serial 921600:
//   S,<us>,<encoder count>,<cmd velocity mm/s>,<counted steps>      every 2 ms (2400 counts/rev, hanging = 0)
//   C,<rx_us>,<frame>,<angle i16 0.01deg>,<flags>,<age 0.1ms>,<cart x u16 0.1px>   per camera packet

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>

// --- pins ---
constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr int PIN_UART_RX = 32;  // AI-G TX -> ESP32 GPIO32
constexpr int PIN_UART_TX = 33;  // ESP32 GPIO33 -> AI-G RX (unused)
constexpr uint8_t PIN_TMC_EN   = 13; // active LOW
constexpr uint8_t PIN_TMC_STEP = 14;
constexpr uint8_t PIN_TMC_DIR  = 27;
constexpr uint8_t PIN_TMC_CS   = 5;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_SCK  = 18;

constexpr uint32_t UART_BAUD = 460800;
constexpr size_t PKT_LEN = 14;
constexpr int32_t ENCODER_COUNTS_PER_REV = 2400;
constexpr float STEPS_PER_METRE = (200.0f * 16.0f) / 0.040f; // 80,000 steps/m
constexpr uint32_t TICK_US = 2000; // 500 Hz

// --- chirp and limits ---
constexpr float F0_HZ = 0.5f, F1_HZ = 2.0f, CHIRP_S = 30.0f;
constexpr float ACC_AMP = 1.0f;          // m/s^2, target acceleration amplitude
constexpr float POS_AMP_MAX = 0.08f;     // m
constexpr float RAMP_S = 1.0f;
constexpr float TRIP_POS_M = 0.095f;
constexpr float MAX_SPEED = 0.5f;        // m/s
constexpr float MAX_ACCEL = 3.0f;        // m/s^2
constexpr float POS_KP = 5.0f;           // 1/s
constexpr float PEND_TRIP_DEG = 120.0f;  // from hanging
constexpr uint32_t KEEPALIVE_MS = 1000;
constexpr uint32_t RUN_LIMIT_MS = (uint32_t)((CHIRP_S + 2.0f) * 1000.0f);

// --- TMC5160 ---
constexpr uint8_t REG_GCONF = 0x00;
constexpr uint8_t REG_IOIN  = 0x04;
constexpr uint8_t WRITE_FLAG = 0x80;
SPISettings tmcSpi(1000000, MSBFIRST, SPI_MODE3);
TMCStepDir motor(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_TMC_CS,
                 PIN_TMC_EN, PIN_TMC_STEP, PIN_TMC_DIR);
constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;
uint32_t spsToUnit(float sps) { return (uint32_t)(sps / VEL_UNIT_TO_SPS); }

// --- encoder ---
volatile int32_t encoderCount = 0;
volatile uint8_t previousAB = 0;
constexpr int8_t QUADRATURE_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR updateEncoder() {
    const uint8_t currentAB = (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
                              static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    encoderCount += QUADRATURE_TABLE[(previousAB << 2) | currentAB];
    previousAB = currentAB;
}
int32_t readEncoderCount() {
    noInterrupts();
    const int32_t v = encoderCount;
    interrupts();
    return v;
}
void zeroEncoderDownward() {
    noInterrupts();
    encoderCount = 0;
    previousAB = (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
                 static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    interrupts();
}

// --- driver helpers ---
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
    Serial.printf("# TMC5160 IOIN=0x%08lX (version byte 0x%02X, expected 0x30)\n",
                  (unsigned long)ioin, (unsigned)(ioin >> 24));
    driverOk = (static_cast<uint8_t>(ioin >> 24) == 0x30);
    if (driverOk) {
        motor.init(0.4f, 1.0f, 4, 192); // 1/16 microsteps, spreadCycle
        writeRegister(REG_GCONF, 0x00000000);
        writeRegister(0x13, 0); // TPWMTHRS = 0
        motor.setSpeed(spsToUnit(MAX_SPEED * STEPS_PER_METRE * 1.5f));
        motor.setAcceleration(0xFFFF);
    }
    digitalWrite(PIN_TMC_EN, HIGH);
    return driverOk;
}

// --- motion state ---
static bool running = false;
static bool logging = false;
static float cmdVelocity = 0.0f;
static float cartPosition = 0.0f;
static float prevTarget = 0.0f;
static int32_t stepsNow = 0;
static uint32_t runStartMs = 0, lastKeepaliveMs = 0;
static uint32_t lastTickUs = 0;

static void stopMotion(const char* reason) {
    if (!running) return;
    running = false;
    cmdVelocity = 0.0f;
    motor.setVelocityDirect(0.0f);
    digitalWrite(PIN_TMC_EN, HIGH);
    Serial.printf("# STOPPED: %s\n", reason);
}

static void startMotion() {
    if (running) return;
    if (!driverOk && !detectDriver()) {
        Serial.println("# START REFUSED: TMC5160 not detected (check driver power and SPI wiring).");
        return;
    }
    motor.actualPosition(0);
    cartPosition = 0.0f;
    cmdVelocity = 0.0f;
    prevTarget = 0.0f;
    stepsNow = 0;
    runStartMs = millis();
    lastKeepaliveMs = runStartMs;
    lastTickUs = micros();
    digitalWrite(PIN_TMC_EN, LOW);
    running = true;
    Serial.println("# STARTED: chirp 0.5-2 Hz, 30 s. Any key / lost keep-alive stops.");
}

static float smooth01(float x) {
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    return x * x * (3.0f - 2.0f * x);
}

static void updateMotion(float dt) {
    cartPosition = motor.getSPIPosition() / STEPS_PER_METRE;
    stepsNow = (int32_t)motor.getSPIPosition();
    if (fabsf(cartPosition) > TRIP_POS_M) { stopMotion("position limit"); return; }
    if (millis() - runStartMs > RUN_LIMIT_MS) { stopMotion("run finished"); return; }
    if (millis() - lastKeepaliveMs > KEEPALIVE_MS) { stopMotion("keep-alive lost"); return; }
    float angDown = readEncoderCount() * 2.0f * PI / ENCODER_COUNTS_PER_REV;
    while (angDown > PI) angDown -= 2.0f * PI;
    if (fabsf(angDown) > PEND_TRIP_DEG * PI / 180.0f) { stopMotion("pendulum angle limit"); return; }

    const float t = (millis() - runStartMs) * 1e-3f;
    const float f = F0_HZ + (F1_HZ - F0_HZ) * t / CHIRP_S;
    const float w = 2.0f * PI * f;
    const float phase = 2.0f * PI * (F0_HZ * t + (F1_HZ - F0_HZ) * t * t / (2.0f * CHIRP_S));
    float amp = fminf(ACC_AMP / (w * w), POS_AMP_MAX);
    amp *= smooth01(t / RAMP_S) * smooth01((CHIRP_S - t) / RAMP_S);
    const float target = amp * sinf(phase);
    const float targetVel = (target - prevTarget) / dt;
    prevTarget = target;

    float desired = targetVel + POS_KP * (target - cartPosition);
    desired = constrain(desired, -MAX_SPEED, MAX_SPEED);
    cmdVelocity += constrain(desired - cmdVelocity, -MAX_ACCEL * dt, MAX_ACCEL * dt);
    motor.setVelocityDirect(cmdVelocity * STEPS_PER_METRE);
}

// --- camera packet parser (same as uart_link_test.cpp) ---
static uint8_t crc8(const uint8_t* d, size_t n) {
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}
static uint8_t buf[PKT_LEN];
static size_t idx = 0;
static uint32_t pktOk, pktCrcErr, pktDropped;
static char line[80];

static void writeLine(int n) {
    if (n > 0) Serial.write(reinterpret_cast<const uint8_t*>(line), (size_t)n);
}
static void dropFirstByte() {
    memmove(buf, buf + 1, idx - 1);
    idx--;
    pktDropped++;
}
static void handlePacket(uint32_t rxUs) {
    pktOk++;
    if (!logging) return;
    const uint16_t frame = buf[2] | (buf[3] << 8);
    const int16_t angle = (int16_t)(buf[4] | (buf[5] << 8));
    const uint16_t x = buf[6] | (buf[7] << 8);
    const uint8_t flags = buf[10];
    const uint16_t age = buf[11] | (buf[12] << 8);
    writeLine(snprintf(line, sizeof(line), "C,%lu,%u,%d,%u,%u,%u\n", (unsigned long)rxUs,
                       (unsigned)frame, (int)angle, (unsigned)flags, (unsigned)age, (unsigned)x));
}
static void pollRx() {
    while (Serial2.available()) {
        const uint8_t b = (uint8_t)Serial2.read();
        const uint32_t nowUs = micros();
        buf[idx++] = b;
        while (idx >= 1 && buf[0] != 0xAA) dropFirstByte();
        if (idx >= 2 && buf[1] != 0x55) { dropFirstByte(); continue; }
        if (idx == PKT_LEN) {
            if (crc8(buf + 2, 11) == buf[13]) {
                handlePacket(nowUs);
                idx = 0;
            } else {
                pktCrcErr++;
                dropFirstByte();
                while (idx >= 1 && buf[0] != 0xAA) dropFirstByte();
                if (idx >= 2 && buf[1] != 0x55) dropFirstByte();
            }
        }
    }
}

static void handleSerial() {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') continue;
        if (c == 'k' || c == 'K') { lastKeepaliveMs = millis(); continue; }
        if (c == 'l' || c == 'L') {
            logging = !logging;
            Serial.printf("# logging %s\n", logging ? "ON" : "OFF");
        } else if (c == 'z' || c == 'Z') {
            if (running) stopMotion("zero requested while running");
            zeroEncoderDownward();
            Serial.println("# encoder zeroed (pendulum hanging down)");
        } else if (c == 'g' || c == 'G') {
            startMotion();
        } else {
            stopMotion("key pressed");
        }
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    pinMode(PIN_TMC_EN, OUTPUT);
    pinMode(PIN_TMC_STEP, OUTPUT);
    pinMode(PIN_TMC_DIR, OUTPUT);
    pinMode(PIN_TMC_CS, OUTPUT);
    digitalWrite(PIN_TMC_EN, HIGH);
    digitalWrite(PIN_TMC_STEP, LOW);
    digitalWrite(PIN_TMC_DIR, LOW);
    digitalWrite(PIN_TMC_CS, HIGH);
    previousAB = (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
                 static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TMC_CS);
    delay(300);
    detectDriver();
    Serial2.setRxBufferSize(1024);
    Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("# cart_accel_log: use tools/cart_accel_fit.py capture; hang the pendulum still, cart at the rail centre");
}

void loop() {
    static uint32_t lastStatusUs = micros();
    handleSerial();
    pollRx();
    const uint32_t now = micros();
    if ((uint32_t)(now - lastTickUs) >= TICK_US) {
        const float dt = (now - lastTickUs) * 1e-6f;
        lastTickUs = now;
        if (running) updateMotion(dt);
        if (logging) {
            writeLine(snprintf(line, sizeof(line), "S,%lu,%ld,%ld,%ld\n", (unsigned long)now,
                               (long)readEncoderCount(), (long)lroundf(cmdVelocity * 1000.0f),
                               (long)stepsNow));
        }
    }
    if ((uint32_t)(now - lastStatusUs) >= 1000000) {
        lastStatusUs += 1000000;
        Serial.printf("# %s%s x=%.3f cnt=%ld pkt=%lu crc=%lu drop=%lu\n", running ? "RUN " : "stop",
                      logging ? " LOG" : "", cartPosition, (long)readEncoderCount(),
                      (unsigned long)pktOk, (unsigned long)pktCrcErr, (unsigned long)pktDropped);
        pktOk = pktCrcErr = pktDropped = 0;
    }
}
