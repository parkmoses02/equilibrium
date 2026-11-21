// Step 1 of the camera pipeline: NO MOTOR. Logs the encoder angle and the AI-G camera
// packets on one clock (micros()) so tools/camera_delay_xcorr.py can measure the total
// camera latency (exposure -> ESP32 reception), the angle sign, and the zero offset by
// cross-correlation. Swing the pendulum by hand while logging.
//
// USB serial 921600. Keys: 'l' start/stop logging, 'z' zero encoder (hang still first).
// Log lines (only while logging; lines starting with '#' are status):
//   E,<us>,<encoder count>                         every 2 ms (2400 counts/rev, hanging = 0)
//   C,<rx_us>,<frame>,<angle i16 0.01deg>,<flags>,<age 0.1ms>   per valid camera packet
// Camera packet (14 bytes) is the same as in uart_link_test.cpp.

#include <Arduino.h>

constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr int PIN_UART_RX = 32;  // AI-G TX -> ESP32 GPIO32
constexpr int PIN_UART_TX = 33;  // ESP32 GPIO33 -> AI-G RX (unused here)
constexpr uint32_t UART_BAUD = 460800;
constexpr size_t PKT_LEN = 14;
constexpr uint32_t ENCODER_PERIOD_US = 2000;
constexpr uint32_t STATUS_US = 1000000;

// --- Encoder (same decoding as pipeline_energy_lqr.cpp) ---
volatile int32_t encoderCount = 0;
volatile uint8_t previousAB = 0;
constexpr int8_t QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

void IRAM_ATTR updateEncoder() {
    const uint8_t currentAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    encoderCount += QUADRATURE_TABLE[(previousAB << 2) | currentAB];
    previousAB = currentAB;
}

int32_t readEncoderCount() {
    noInterrupts();
    const int32_t val = encoderCount;
    interrupts();
    return val;
}

void zeroEncoderDownward() {
    noInterrupts();
    encoderCount = 0;
    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    interrupts();
}

// --- Camera packet parser (same as uart_link_test.cpp) ---
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
static uint32_t pktOk, pktCrcErr, pktDropped, pktAngleValid, pktTotalOk;
static char line[64];

static void writeLine(int n) {
    if (n > 0) Serial.write(reinterpret_cast<const uint8_t*>(line), (size_t)n);
}

static void dropFirstByte() {
    memmove(buf, buf + 1, idx - 1);
    idx--;
    pktDropped++;
}

static void handlePacket(uint32_t rxUs) {
    const uint16_t frame = buf[2] | (buf[3] << 8);
    const int16_t angle = (int16_t)(buf[4] | (buf[5] << 8));
    const uint8_t flags = buf[10];
    const uint16_t age = buf[11] | (buf[12] << 8);
    pktOk++;
    pktTotalOk++;
    const bool valid = (flags & 0x01) && angle != (int16_t)0x8000;
    if (valid) pktAngleValid++;
    if (logging && valid) {
        writeLine(snprintf(line, sizeof(line), "C,%lu,%u,%d,%u,%u\n", (unsigned long)rxUs,
                           (unsigned)frame, (int)angle, (unsigned)flags, (unsigned)age));
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
            Serial.println("# encoder zeroed (pendulum hanging down)");
        }
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);
    Serial2.setRxBufferSize(1024);
    Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("# camera_delay_log: hang the pendulum still, press 'z', then 'l' to log; swing by hand");
}

void loop() {
    static uint32_t lastEncUs = micros(), lastStatusUs = micros();
    handleSerial();
    pollRx();
    const uint32_t now = micros();
    if ((uint32_t)(now - lastEncUs) >= ENCODER_PERIOD_US) {
        lastEncUs += ENCODER_PERIOD_US;
        if (logging) {
            writeLine(snprintf(line, sizeof(line), "E,%lu,%ld\n", (unsigned long)now,
                               (long)readEncoderCount()));
        }
    }
    if ((uint32_t)(now - lastStatusUs) >= STATUS_US) {
        lastStatusUs += STATUS_US;
        Serial.printf("# %s cnt=%ld pkt=%lu angle_valid=%lu crc=%lu drop=%lu total=%lu\n",
                      logging ? "LOG" : "idle", (long)readEncoderCount(), (unsigned long)pktOk,
                      (unsigned long)pktAngleValid, (unsigned long)pktCrcErr,
                      (unsigned long)pktDropped, (unsigned long)pktTotalOk);
        pktOk = pktAngleValid = pktCrcErr = pktDropped = 0;
    }
}
