// Step 2 of the camera pipeline: NO MOTOR. Runs the camera angle estimator on the ESP32 at 500 Hz and
// compares it with the encoder in real time. Port of tools/camera_kalman_sim.py.
//
//   - 2-state EKF (angle phi from upright, rate w) with the measured pendulum model
//       phi' = w,  w' = K sin(phi) - C w + B cos(phi) a        (a = cart acceleration, firmware units)
//   - the camera angle arrives ~14 ms late: fixed-lag retrodiction (update the state `lag` ticks ago,
//     then re-propagate to now with the stored inputs)
//   - the camera angle is corrected for sign (opposite to the encoder) and the zero offset
//
// USB serial 921600. Keys: 'z' zero the encoder (pendulum hanging still), 'l' log on/off.
// Once per second (while the encoder is zeroed) a '#' line prints the error of the estimator and of the raw
// camera angle against the encoder at the current time, and the time one estimator tick takes.
// Log lines while logging:   X,<us>,<encoder count>,<phi 0.01 deg>,<w mrad/s>      every tick
//                            C,<rx_us>,<frame>,<angle i16 0.01deg>,<flags>,<age 0.1ms>   per camera packet
// Camera packet (14 bytes): layout in CLAUDE.md section 5. The other camera firmwares copy this parser.

#include <Arduino.h>
#include <math.h>
#include "camera_angle_calib.h"

constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr int PIN_UART_RX = 32;  // AI-G TX -> ESP32 GPIO32
constexpr int PIN_UART_TX = 33;  // ESP32 GPIO33 -> AI-G RX (unused)
constexpr uint32_t UART_BAUD = 460800;
constexpr size_t PKT_LEN = 14;
constexpr int32_t ENCODER_COUNTS_PER_REV = 2400;
constexpr float RAD_PER_COUNT = 2.0f * PI / ENCODER_COUNTS_PER_REV;

// --- estimator parameters (measured, see CLAUDE.md section 5) ---
constexpr uint32_t TICK_US = 2000;          // 500 Hz
constexpr float DT = 0.002f;
constexpr float MODEL_K = 64.6f;            // 1/s^2   (64.2 and 65.0 in two runs)
constexpr float MODEL_C = 0.05f;            // 1/s
constexpr float MODEL_B = -9.4f;            // 1/m, cart acceleration in the firmware's counted units
constexpr float Q_RATE = 200.0f;            // rate process noise density
constexpr float R_ANGLE = 0.017453f * 0.017453f;   // (1 deg)^2 camera angle noise
constexpr float CAM_SIGN = -1.0f;           // camera angle sign relative to the encoder
// (the camera zero offset is part of the calibration table in camera_angle_calib.h)
constexpr float PRE_DRIVER_MS = 6.3f;       // exposure/readout before the driver timestamp + UART
constexpr int DEFAULT_LAG = 7;              // ticks (14 ms)
constexpr int MAX_LAG = 14;
constexpr int HIST = 24;                    // ticks of state history (> MAX_LAG + 1)

// Cart acceleration used by the model (counted m/s^2). 0 here: nothing moves the cart in this test.
float cartAccelCmd = 0.0f;

// --- encoder (same decoding as pipeline_energy_lqr.cpp) ---
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

// --- EKF with fixed-lag retrodiction ---
struct Est { float phi, w, p00, p01, p11; };
struct Node { Est e; float a; };
static Node hist[HIST];
static int histIdx = 0;
static bool estInit = false;

static Est predict(const Est& e, float a) {
    const float s = sinf(e.phi), c = cosf(e.phi);
    Est n;
    n.phi = e.phi + DT * e.w;
    n.w = e.w + DT * (MODEL_K * s - MODEL_C * e.w + MODEL_B * c * a);
    const float f10 = DT * (MODEL_K * c - MODEL_B * s * a);
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

// One 500 Hz tick. hasZ: a camera angle (already sign/offset corrected) arrived, `lag` ticks late.
static void estimatorTick(float a, bool hasZ, float z, int lag) {
    if (hasZ && !estInit) {
        const Est e0 = {z, 0.0f, 1.0f, 0.0f, 100.0f};
        for (int i = 0; i < HIST; i++) { hist[i].e = e0; hist[i].a = 0.0f; }
        estInit = true;
    }
    if (!estInit) return;
    const int prev = histIdx;
    histIdx = (histIdx + 1) % HIST;
    hist[histIdx].e = predict(hist[prev].e, a);
    hist[histIdx].a = a;
    if (!hasZ) return;
    if (lag < 0) lag = 0;
    if (lag > MAX_LAG) lag = MAX_LAG;
    int k = (histIdx - lag + HIST) % HIST;
    hist[k].e = update(hist[k].e, z);
    while (k != histIdx) {
        const int nxt = (k + 1) % HIST;
        hist[nxt].e = predict(hist[k].e, hist[nxt].a);
        k = nxt;
    }
}

// --- camera packet parser (CRC-8 poly 0x07 over bytes 2..12; resyncs on 0xAA 0x55) ---
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
static bool logging = false;
static bool zeroed = false;
static char line[80];
static uint32_t pktOk, pktValid, pktCrcErr, pktDropped;
// a camera angle waiting for the next estimator tick
static bool pendingValid = false;
static float pendingZ = 0.0f;
static int pendingLag = DEFAULT_LAG;
static float zHold = 0.0f;       // last camera angle, held (the "raw" baseline)
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
    pendingZ = camCalibrate(wrapPi(CAM_SIGN * camRad));
    zHold = pendingZ;
    zHoldValid = true;
    pendingValid = true;
    if (flags & 0x20) {
        const float delayMs = age * 0.1f + PRE_DRIVER_MS;
        pendingLag = (int)lroundf(delayMs / (DT * 1000.0f));
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
        if (c == 'l' || c == 'L') {
            logging = !logging;
            Serial.printf("# logging %s\n", logging ? "ON" : "OFF");
        } else if (c == 'z' || c == 'Z') {
            zeroEncoderDownward();
            zeroed = true;
            Serial.println("# encoder zeroed (pendulum hanging down)");
        }
    }
}

// --- error statistics against the encoder (current time) ---
static double sumEkf2, sumRaw2, sumW2;
static uint32_t nErr, nErrW, nRaw;
static uint32_t tickSumUs, tickMaxUs, nTicks;
static int32_t cntHist[11];
static float wEstHist[11];
static int cntIdx = 0;

static void runTick() {
    const uint32_t t0 = micros();
    const bool hasZ = pendingValid;
    const float z = pendingZ;
    const int lag = pendingLag;
    pendingValid = false;
    estimatorTick(cartAccelCmd, hasZ, z, lag);
    const uint32_t dtUs = micros() - t0;
    tickSumUs += dtUs;
    if (dtUs > tickMaxUs) tickMaxUs = dtUs;
    nTicks++;

    const int32_t count = readEncoderCount();
    cntIdx = (cntIdx + 1) % 11;
    cntHist[cntIdx] = count;
    wEstHist[cntIdx] = estInit ? hist[histIdx].e.w : 0.0f;
    if (!estInit) return;
    const float phiEst = hist[histIdx].e.phi;
    if (logging) {
        const uint32_t nowUs = micros();
        writeLine(snprintf(line, sizeof(line), "X,%lu,%ld,%ld,%ld\n", (unsigned long)nowUs, (long)count,
                           (long)lroundf(phiEst * 57.29578f * 100.0f), (long)lroundf(hist[histIdx].e.w * 1000.0f)));
    }
    if (!zeroed) return;
    const float phiEnc = wrapPi(count * RAD_PER_COUNT - PI);
    const float e = wrapPi(phiEst - phiEnc);
    sumEkf2 += (double)e * e;
    nErr++;
    if (zHoldValid) {
        const float er = wrapPi(zHold - phiEnc);
        sumRaw2 += (double)er * er;
        nRaw++;
    }
    // rate truth: encoder counts 10 ticks apart, centred 5 ticks ago; estimator rate from 5 ticks ago
    const int old = (cntIdx + 1) % 11;
    const float wTrue = (count - cntHist[old]) * RAD_PER_COUNT / (10.0f * DT);
    const float ew = wEstHist[(cntIdx + 11 - 5) % 11] - wTrue;
    sumW2 += (double)ew * ew;
    nErrW++;
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    previousAB = (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
                 static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);
    Serial2.setRxBufferSize(1024);
    Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("# camera_estimator_test: hang the pendulum still, press 'z', then swing it by hand");
}

void loop() {
    static uint32_t lastTickUs = micros(), lastStatusUs = micros();
    handleSerial();
    pollRx();
    const uint32_t now = micros();
    if ((uint32_t)(now - lastTickUs) >= TICK_US) {
        lastTickUs += TICK_US;
        runTick();
    }
    if ((uint32_t)(now - lastStatusUs) >= 1000000) {
        lastStatusUs += 1000000;
        const float degPerRad = 57.29578f;
        Serial.printf("# %s err_ekf=%.2f deg err_raw=%.2f deg err_w=%.2f rad/s | tick us avg/max=%lu/%lu | pkt=%lu valid=%lu crc=%lu drop=%lu\n",
                      zeroed ? "ZERO" : "(press z)",
                      nErr ? sqrtf((float)(sumEkf2 / nErr)) * degPerRad : 0.0f,
                      nRaw ? sqrtf((float)(sumRaw2 / nRaw)) * degPerRad : 0.0f,
                      nErrW ? sqrtf((float)(sumW2 / nErrW)) : 0.0f,
                      nTicks ? (unsigned long)(tickSumUs / nTicks) : 0UL, (unsigned long)tickMaxUs,
                      (unsigned long)pktOk, (unsigned long)pktValid, (unsigned long)pktCrcErr,
                      (unsigned long)pktDropped);
        sumEkf2 = sumRaw2 = sumW2 = 0;
        nErr = nErrW = nRaw = 0;
        tickSumUs = tickMaxUs = nTicks = 0;
        pktOk = pktValid = pktCrcErr = pktDropped = 0;
    }
}
