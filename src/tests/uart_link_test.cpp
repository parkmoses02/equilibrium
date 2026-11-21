// AI-G <-> ESP32 UART link test (no motor, no encoder)
//
// RX (Serial2, GPIO32): AI-G camera packets (14 bytes) are parsed with CRC-8 and
//   counted. Once per second the stats go to USB serial (115200).
// TX (Serial2, GPIO33): every 100 ms a 5-byte packet [0xAA][0x56][seq u16][crc8]
//   is sent back so the AI-G side can check the reverse direction.
//
// Packet (14 bytes, little-endian):
//   0,1 0xAA 0x55 | 2,3 frame u16 | 4,5 angle i16 (0.01 deg, 0x8000 invalid)
//   6,7 cart x u16 (0.1 px, 0xFFFF invalid) | 8,9 cart y u16 | 10 flags
//   11,12 age u16 (0.1 ms) | 13 CRC-8 (poly 0x07, init 0, bytes 2..12)

#include <Arduino.h>

static const uint32_t UART_BAUD = 460800;
static const int PIN_RX = 32;  // AI-G TX -> ESP32 GPIO32
static const int PIN_TX = 33;  // ESP32 GPIO33 -> AI-G RX
static const size_t PKT_LEN = 14;
static const uint32_t REPORT_US = 1000000;
static const uint32_t ECHO_US = 100000;

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

// stats (reset every report)
static uint32_t okCount, crcErr, droppedBytes, frameGaps, lastFrameGapSum;
static uint32_t dtMin, dtMax;
static uint64_t dtSum;
static uint32_t lastPktUs;
static uint16_t lastFrame;
static bool haveLast;
static uint32_t totalOk, totalCrcErr, totalDropped, totalGaps;
// last valid packet
static int16_t lastAngle;
static uint16_t lastX, lastAge;
static uint8_t lastFlags;

static void resetWindow() {
  okCount = crcErr = droppedBytes = frameGaps = lastFrameGapSum = 0;
  dtMin = 0xFFFFFFFF; dtMax = 0; dtSum = 0;
}

static void dropFirstByte() {
  memmove(buf, buf + 1, idx - 1);
  idx--;
  droppedBytes++;
}

static void handlePacket(uint32_t nowUs) {
  uint16_t frame = buf[2] | (buf[3] << 8);
  lastAngle = (int16_t)(buf[4] | (buf[5] << 8));
  lastX = buf[6] | (buf[7] << 8);
  lastFlags = buf[10];
  lastAge = buf[11] | (buf[12] << 8);

  if (haveLast) {
    uint16_t diff = (uint16_t)(frame - lastFrame);
    if (diff == 0) {
      // duplicate frame number: ignore for gap stats
    } else if (diff > 1) {
      frameGaps++;
      lastFrameGapSum += diff - 1;
    }
    uint32_t dt = nowUs - lastPktUs;
    if (dt < dtMin) dtMin = dt;
    if (dt > dtMax) dtMax = dt;
    dtSum += dt;
  }
  haveLast = true;
  lastFrame = frame;
  lastPktUs = nowUs;
  okCount++;
}

static void pollRx() {
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    uint32_t nowUs = micros();
    buf[idx++] = b;

    // header sync: keep only bytes that can be the start of a packet
    while (idx >= 1 && buf[0] != 0xAA) dropFirstByte();
    if (idx >= 2 && buf[1] != 0x55) { dropFirstByte(); continue; }

    if (idx == PKT_LEN) {
      if (crc8(buf + 2, 11) == buf[13]) {
        handlePacket(nowUs);
        idx = 0;
      } else {
        crcErr++;
        dropFirstByte();  // resync: look for next header inside this window
        while (idx >= 1 && buf[0] != 0xAA) dropFirstByte();
        if (idx >= 2 && buf[1] != 0x55) dropFirstByte();
      }
    }
  }
}

static void sendEcho() {
  static uint16_t seq = 0;
  uint8_t p[5] = {0xAA, 0x56, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), 0};
  p[4] = crc8(p + 2, 2);
  Serial2.write(p, sizeof(p));
  seq++;
}

static void report() {
  totalOk += okCount; totalCrcErr += crcErr; totalDropped += droppedBytes; totalGaps += frameGaps;
  Serial.printf("ok=%lu crc_err=%lu drop_bytes=%lu frame_gaps=%lu(lost=%lu)",
                (unsigned long)okCount, (unsigned long)crcErr, (unsigned long)droppedBytes,
                (unsigned long)frameGaps, (unsigned long)lastFrameGapSum);
  if (okCount > 1) {
    Serial.printf(" dt_us min/avg/max=%lu/%lu/%lu", (unsigned long)dtMin,
                  (unsigned long)(dtSum / (okCount - 1)), (unsigned long)dtMax);
  }
  if (haveLast) {
    Serial.printf(" | frame=%u angle=%.2f x=%.1f flags=0x%02X age=%.1fms",
                  lastFrame, lastAngle * 0.01f, lastX * 0.1f, lastFlags, lastAge * 0.1f);
  }
  Serial.printf(" | total ok=%lu crc=%lu drop=%lu gaps=%lu\n", (unsigned long)totalOk,
                (unsigned long)totalCrcErr, (unsigned long)totalDropped, (unsigned long)totalGaps);
  resetWindow();
}

void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  resetWindow();
  Serial.println("uart_link_test: Serial2 460800 8N1 RX=32 TX=33");
}

void loop() {
  static uint32_t lastReport = micros(), lastEcho = micros();
  pollRx();
  uint32_t now = micros();
  if ((uint32_t)(now - lastEcho) >= ECHO_US) { lastEcho += ECHO_US; sendEcho(); }
  if ((uint32_t)(now - lastReport) >= REPORT_US) { lastReport += REPORT_US; report(); }
}
