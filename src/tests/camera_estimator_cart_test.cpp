// Step 3 of the camera pipeline: the cart MOVES (chirp, pendulum hanging, not balanced) while the camera angle
// estimator runs. Two estimators run on the same camera data and are compared with the encoder:
//   B  : EKF using the cart acceleration input (model term B cos(phi) a)
//   0  : the same EKF with the cart term switched off (a = 0)
// tools/camera_est_cart_check.py then sweeps B on the log to confirm sign and size.
//
// Run it only from tools/cart_accel_fit.py capture (it sends 'z', 'l', 'g' and keep-alive 'k' bytes; it only
// saves the raw output, so it works for this firmware too). Safety, any one of these stops the cart at once
// and disables the driver: any key other than l/z/g/k, no 'k' within 1 s, cart more than 9.5 cm from the start,
// pendulum more than 120 deg from hanging, run over. Put the cart at the CENTRE of the rail.
// Motion and driver setup are copied from cart_accel_log.cpp; estimator from camera_estimator_test.cpp.
//
// Log lines (USB 921600, only while logging; '#' lines are status):
//   X,<us>,<count>,<phi_B 0.01deg>,<w_B mrad/s>,<phi_0 0.01deg>,<w_0 mrad/s>,<cmd vel mm/s>,<cmd acc mm/s^2>,<steps>
//   C,<rx_us>,<frame>,<angle i16 0.01deg>,<flags>,<age 0.1ms>

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>

// --- pins ---
constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr int PIN_UART_RX = 32;
constexpr int PIN_UART_TX = 33;
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
constexpr float RAD_PER_COUNT = 2.0f * PI / ENCODER_COUNTS_PER_REV;
constexpr float STEPS_PER_METRE = (200.0f * 16.0f) / 0.040f; // 80,000 steps/m (counted; real travel is 1.5x)
constexpr uint32_t TICK_US = 2000; // 500 Hz
constexpr float DT = 0.002f;

// --- chirp and limits (same as cart_accel_log.cpp) ---
constexpr float F0_HZ = 0.5f, F1_HZ = 2.0f, CHIRP_S = 30.0f;
constexpr float ACC_AMP = 1.0f;
constexpr float POS_AMP_MAX = 0.08f;
constexpr float RAMP_S = 1.0f;
constexpr float TRIP_POS_M = 0.095f;
constexpr float MAX_SPEED = 0.5f;
constexpr float MAX_ACCEL = 3.0f;
constexpr float POS_KP = 5.0f;
constexpr float PEND_TRIP_DEG = 120.0f;
constexpr uint32_t KEEPALIVE_MS = 1000;
constexpr uint32_t RUN_LIMIT_MS = (uint32_t)((CHIRP_S + 2.0f) * 1000.0f);

// --- estimator parameters (same as camera_estimator_test.cpp) ---
constexpr float MODEL_K = 64.6f;
constexpr float MODEL_C = 0.05f;
constexpr float MODEL_B = -9.4f;            // 1/m, cart acceleration in the firmware's counted units
constexpr float Q_RATE = 200.0f;
constexpr float R_ANGLE = 0.017453f * 0.017453f;
constexpr float CAM_SIGN = -1.0f;
constexpr float CAM_OFFSET_RAD = -0.96f * 0.017453f;
constexpr float PRE_DRIVER_MS = 6.3f;
constexpr int DEFAULT_LAG = 7;
constexpr int MAX_LAG = 14;
constexpr int HIST = 24;

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

static float wrapPi(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}

// --- EKF with fixed-lag retrodiction (bScale = 1: uses the cart acceleration, 0: ignores it) ---
struct Est { float phi, w, p00, p01, p11; };
struct Node { Est e; float a; };

struct Estimator {
    Node hist[HIST];
    int idx = 0;
    bool init = false;
    float bScale = 1.0f;

    Est predict(const Est& e, float a) const {
        const float B = MODEL_B * bScale;
        const float s = sinf(e.phi), c = cosf(e.phi);
        Est n;
        n.phi = e.phi + DT * e.w;
        n.w = e.w + DT * (MODEL_K * s - MODEL_C * e.w + B * c * a);
        const float f10 = DT * (MODEL_K * c - B * s * a);
        const float f11 = 1.0f - MODEL_C * DT;
        const float a00 = e.p00 + DT * e.p01, a01 = e.p01 + DT * e.p11;
        const float a10 = f10 * e.p00 + f11 * e.p01, a11 = f10 * e.p01 + f11 * e.p11;
        n.p00 = a00 + a01 * DT;
        n.p01 = 0.5f * ((a00 * f10 + a01 * f11) + (a10 + a11 * DT));
        n.p11 = a10 * f10 + a11 * f11 + Q_RATE * DT;
        return n;
    }
    static Est update(const Est& e, float z) {
        const float y = wrapPi(z - e.phi);
        const float S = e.p00 + R_ANGLE;
        const float k0 = e.p00 / S, k1 = e.p01 / S;
        Est n;
        n.phi = wrapPi(e.phi + k0 * y);
        n.w = e.w + k1 * y;
        n.p00 = e.p00 - k0 * e.p00;
        n.p01 = e.p01 - k0 * e.p01;
        n.p11 = e.p11 - k1 * e.p01;
        return n;
    }
    void tick(float a, bool hasZ, float z, int lag) {
        if (hasZ && !init) {
            const Est e0 = {z, 0.0f, 1.0f, 0.0f, 100.0f};
            for (int i = 0; i < HIST; i++) { hist[i].e = e0; hist[i].a = 0.0f; }
            init = true;
        }
        if (!init) return;
        const int prev = idx;
        idx = (idx + 1) % HIST;
        hist[idx].e = predict(hist[prev].e, a);
        hist[idx].a = a;
        if (!hasZ) return;
        if (lag < 0) lag = 0;
        if (lag > MAX_LAG) lag = MAX_LAG;
        int k = (idx - lag + HIST) % HIST;
        hist[k].e = update(hist[k].e, z);
        while (k != idx) {
            const int nxt = (k + 1) % HIST;
            hist[nxt].e = predict(hist[k].e, hist[nxt].a);
            k = nxt;
        }
    }
    const Est& cur() const { return hist[idx].e; }
};
static Estimator estB, est0;

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

// --- motion state (from cart_accel_log.cpp) ---
static bool running = false;
static bool logging = false;
static bool zeroed = false;
static float cmdVelocity = 0.0f;
static float prevCmdVelocity = 0.0f;
static float cartPosition = 0.0f;
static float prevTarget = 0.0f;
static int32_t stepsNow = 0;
static uint32_t runStartMs = 0, lastKeepaliveMs = 0;

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
    prevCmdVelocity = 0.0f;
    prevTarget = 0.0f;
    stepsNow = 0;
    runStartMs = millis();
    lastKeepaliveMs = runStartMs;
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

// --- camera packet parser (same as camera_estimator_test.cpp) ---
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
static char line[112];
static uint32_t pktOk, pktValid, pktCrcErr, pktDropped;
static bool pendingValid = false;
static float pendingZ = 0.0f;
static int pendingLag = DEFAULT_LAG;
static float zHold = 0.0f;
static bool zHoldValid = false;

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
    const uint16_t frame = buf[2] | (buf[3] << 8);
    const int16_t angle = (int16_t)(buf[4] | (buf[5] << 8));
    const uint8_t flags = buf[10];
    const uint16_t age = buf[11] | (buf[12] << 8);
    if (logging) {
        writeLine(snprintf(line, sizeof(line), "C,%lu,%u,%d,%u,%u\n", (unsigned long)rxUs,
                           (unsigned)frame, (int)angle, (unsigned)flags, (unsigned)age));
    }
    if (!(flags & 0x01) || angle == (int16_t)0x8000) return;
    pktValid++;
    const float camRad = angle * 0.01f * 0.017453293f;
    pendingZ = wrapPi(CAM_SIGN * camRad + CAM_OFFSET_RAD);
    zHold = pendingZ;
    zHoldValid = true;
    pendingValid = true;
    if (flags & 0x20) {
        pendingLag = (int)lroundf((age * 0.1f + PRE_DRIVER_MS) / (DT * 1000.0f));
    } else {
        pendingLag = DEFAULT_LAG;
    }
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
            zeroed = true;
            Serial.println("# encoder zeroed (pendulum hanging down)");
        } else if (c == 'g' || c == 'G') {
            startMotion();
        } else {
            stopMotion("key pressed");
        }
    }
}

// --- error statistics against the encoder (current time) ---
static double sumB2, sum02, sumRaw2, sumWB2, sumW02;
static uint32_t nErr, nRaw, nErrW;
static uint32_t tickSumUs, tickMaxUs, nTicks;
static int32_t cntHist[11];
static float wBHist[11], w0Hist[11];
static int cntIdx = 0;

static void runTick(float dt) {
    const uint32_t t0 = micros();
    if (running) updateMotion(dt);
    const float accel = (cmdVelocity - prevCmdVelocity) / dt;   // counted m/s^2, firmware sign
    prevCmdVelocity = cmdVelocity;
    const bool hasZ = pendingValid;
    const float z = pendingZ;
    const int lag = pendingLag;
    pendingValid = false;
    estB.tick(accel, hasZ, z, lag);
    est0.tick(0.0f, hasZ, z, lag);
    const uint32_t dtUs = micros() - t0;
    tickSumUs += dtUs;
    if (dtUs > tickMaxUs) tickMaxUs = dtUs;
    nTicks++;

    const int32_t count = readEncoderCount();
    cntIdx = (cntIdx + 1) % 11;
    cntHist[cntIdx] = count;
    wBHist[cntIdx] = estB.init ? estB.cur().w : 0.0f;
    w0Hist[cntIdx] = est0.init ? est0.cur().w : 0.0f;
    if (!estB.init) return;
    if (logging) {
        const uint32_t nowUs = micros();
        writeLine(snprintf(line, sizeof(line), "X,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n", (unsigned long)nowUs,
                           (long)count, (long)lroundf(estB.cur().phi * 5729.578f),
                           (long)lroundf(estB.cur().w * 1000.0f), (long)lroundf(est0.cur().phi * 5729.578f),
                           (long)lroundf(est0.cur().w * 1000.0f), (long)lroundf(cmdVelocity * 1000.0f),
                           (long)lroundf(accel * 1000.0f), (long)stepsNow));
    }
    if (!zeroed) return;
    const float phiEnc = wrapPi(count * RAD_PER_COUNT - PI);
    const float eB = wrapPi(estB.cur().phi - phiEnc);
    const float e0 = wrapPi(est0.cur().phi - phiEnc);
    sumB2 += (double)eB * eB;
    sum02 += (double)e0 * e0;
    nErr++;
    if (zHoldValid) {
        const float er = wrapPi(zHold - phiEnc);
        sumRaw2 += (double)er * er;
        nRaw++;
    }
    const int old = (cntIdx + 1) % 11;
    const float wTrue = (count - cntHist[old]) * RAD_PER_COUNT / (10.0f * DT);
    const int c5 = (cntIdx + 11 - 5) % 11;
    const float ewB = wBHist[c5] - wTrue, ew0 = w0Hist[c5] - wTrue;
    sumWB2 += (double)ewB * ewB;
    sumW02 += (double)ew0 * ew0;
    nErrW++;
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
    estB.bScale = 1.0f;
    est0.bScale = 0.0f;
    Serial2.setRxBufferSize(1024);
    Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("# camera_estimator_cart_test: use tools/cart_accel_fit.py capture; hang the pendulum still, cart at the rail centre");
}

void loop() {
    static uint32_t lastTickUs = micros(), lastStatusUs = micros();
    handleSerial();
    pollRx();
    const uint32_t now = micros();
    if ((uint32_t)(now - lastTickUs) >= TICK_US) {
        const float dt = (now - lastTickUs) * 1e-6f;
        lastTickUs = now;
        runTick(dt);
    }
    if ((uint32_t)(now - lastStatusUs) >= 1000000) {
        lastStatusUs += 1000000;
        const float d = 57.29578f;
        Serial.printf("# %s%s x=%.3f err_B=%.2f err_0=%.2f err_raw=%.2f deg | w_B=%.2f w_0=%.2f rad/s | tick us avg/max=%lu/%lu | pkt=%lu valid=%lu\n",
                      running ? "RUN " : "stop", logging ? " LOG" : "", cartPosition,
                      nErr ? sqrtf((float)(sumB2 / nErr)) * d : 0.0f, nErr ? sqrtf((float)(sum02 / nErr)) * d : 0.0f,
                      nRaw ? sqrtf((float)(sumRaw2 / nRaw)) * d : 0.0f,
                      nErrW ? sqrtf((float)(sumWB2 / nErrW)) : 0.0f, nErrW ? sqrtf((float)(sumW02 / nErrW)) : 0.0f,
                      nTicks ? (unsigned long)(tickSumUs / nTicks) : 0UL, (unsigned long)tickMaxUs,
                      (unsigned long)pktOk, (unsigned long)pktValid);
        sumB2 = sum02 = sumRaw2 = sumWB2 = sumW02 = 0;
        nErr = nRaw = nErrW = 0;
        tickSumUs = tickMaxUs = nTicks = 0;
        pktOk = pktValid = pktCrcErr = pktDropped = 0;
    }
}
