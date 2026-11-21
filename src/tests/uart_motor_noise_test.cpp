// AI-G <-> ESP32 UART link test WITH the cart moving (motor/driver noise check).
//
// UART part: identical to uart_link_test.cpp (14-byte camera packets in on Serial2,
//   5-byte echo packets out every 100 ms, stats once per second on USB serial 115200).
// Motor part: a slow sine sweep of the cart around the START position (centre of the rail).
//   - Starts only on 'g'. ANY other key (space / x / ...) stops it immediately.
//   - Amplitude 8 cm (hard trip at 9.5 cm from the start), peak speed 0.2 m/s,
//     acceleration limited to 2 m/s^2, auto stop after 20 min.
//   - Put the cart at the CENTRE of the rail before pressing 'g'. Position is open-loop
//     (counted STEP pulses), so a stall makes the real cart drift from the counted one.
// Driver setup / pins / sign convention are copied from pipeline_energy_lqr.cpp.

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>

// --- UART (AI-G) ---
constexpr uint32_t UART_BAUD = 460800;
constexpr int PIN_UART_RX = 32;  // AI-G TX -> ESP32 GPIO32
constexpr int PIN_UART_TX = 33;  // ESP32 GPIO33 -> AI-G RX
constexpr size_t PKT_LEN = 14;
constexpr uint32_t REPORT_US = 1000000;
constexpr uint32_t ECHO_US = 100000;

// --- Motor / driver (same as pipeline_energy_lqr.cpp) ---
constexpr uint8_t PIN_TMC_EN   = 13; // active LOW
constexpr uint8_t PIN_TMC_STEP = 14;
constexpr uint8_t PIN_TMC_DIR  = 27;
constexpr uint8_t PIN_TMC_CS   = 5;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_SCK  = 18;
constexpr float STEPS_PER_METRE = (200.0f * 16.0f) / 0.040f; // 80,000 steps/m
constexpr uint8_t REG_GCONF   = 0x00;
constexpr uint8_t REG_IOIN    = 0x04;
constexpr uint8_t WRITE_FLAG  = 0x80;
SPISettings tmcSpi(1000000, MSBFIRST, SPI_MODE3);
TMCStepDir motor(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_TMC_CS,
                 PIN_TMC_EN, PIN_TMC_STEP, PIN_TMC_DIR);
constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;
uint32_t spsToUnit(float sps) { return (uint32_t)(sps / VEL_UNIT_TO_SPS); }

// --- Sweep limits (small on purpose) ---
constexpr float SWEEP_AMPLITUDE_M = 0.08f;   // +-8 cm around the start position
constexpr float SWEEP_OMEGA = 2.5f;          // rad/s -> peak speed A*w = 0.2 m/s, period 2.5 s
constexpr float RAMP_S = 3.0f;               // amplitude ramp-up time
constexpr float TRIP_POS_M = 0.095f;         // hard stop if the counted position exceeds this
constexpr float MAX_SPEED = 0.25f;           // m/s
constexpr float MAX_ACCEL = 2.0f;            // m/s^2
constexpr float POS_KP = 5.0f;               // 1/s, pulls the counted position onto the target
constexpr uint32_t RUN_LIMIT_MS = 20UL * 60UL * 1000UL;
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz

bool running = false;
float cmdVelocity = 0.0f;
float cartPosition = 0.0f;
float prevTarget = 0.0f;
uint32_t runStartMs = 0;
uint32_t lastControlUs = 0;

// ---------------- UART parser (copied from uart_link_test.cpp) ----------------
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
static uint32_t okCount, crcErr, droppedBytes, frameGaps, lostFrames;
static uint32_t dtMin, dtMax;
static uint64_t dtSum;
static uint32_t lastPktUs;
static uint16_t lastFrame;
static bool haveLast;
static uint32_t totalOk, totalCrcErr, totalDropped, totalGaps;

static void resetWindow() {
  okCount = crcErr = droppedBytes = frameGaps = lostFrames = 0;
  dtMin = 0xFFFFFFFF; dtMax = 0; dtSum = 0;
}

static void dropFirstByte() {
  memmove(buf, buf + 1, idx - 1);
  idx--;
  droppedBytes++;
}

static void handlePacket(uint32_t nowUs) {
  uint16_t frame = buf[2] | (buf[3] << 8);
  if (haveLast) {
    uint16_t diff = (uint16_t)(frame - lastFrame);
    if (diff > 1) { frameGaps++; lostFrames += diff - 1; }
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
    while (idx >= 1 && buf[0] != 0xAA) dropFirstByte();
    if (idx >= 2 && buf[1] != 0x55) { dropFirstByte(); continue; }
    if (idx == PKT_LEN) {
      if (crc8(buf + 2, 11) == buf[13]) {
        handlePacket(nowUs);
        idx = 0;
      } else {
        crcErr++;
        dropFirstByte();
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
  // kept short: Serial.printf blocks while the 115200 TX buffer drains
  Serial.printf("%s x=%.3f ok=%lu crc=%lu drop=%lu gap=%lu/%lu",
                running ? "RUN " : "STOP", cartPosition, (unsigned long)okCount,
                (unsigned long)crcErr, (unsigned long)droppedBytes,
                (unsigned long)frameGaps, (unsigned long)lostFrames);
  if (okCount > 1) {
    Serial.printf(" dt=%lu/%lu/%lu", (unsigned long)dtMin, (unsigned long)(dtSum / (okCount - 1)),
                  (unsigned long)dtMax);
  }
  Serial.printf(" | tot ok=%lu crc=%lu drop=%lu gap=%lu\n", (unsigned long)totalOk,
                (unsigned long)totalCrcErr, (unsigned long)totalDropped, (unsigned long)totalGaps);
  resetWindow();
}

// ---------------- TMC driver ----------------
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

static void configureDriver() {
  motor.init(0.4f, 1.0f, 4, 192); // 1/16 microsteps, spreadCycle
  writeRegister(REG_GCONF, 0x00000000);
  writeRegister(0x13, 0); // TPWMTHRS = 0
  motor.setSpeed(spsToUnit(MAX_SPEED * STEPS_PER_METRE * 1.5f));
  motor.setAcceleration(0xFFFF);
}

static bool driverOk = false;

static void stopMotion(const char* reason) {
  running = false;
  cmdVelocity = 0.0f;
  motor.setVelocityDirect(0.0f);
  digitalWrite(PIN_TMC_EN, HIGH);
  Serial.print("STOPPED: ");
  Serial.println(reason);
}

static void startMotion() {
  if (!driverOk) { Serial.println("START REFUSED: TMC5160 not detected."); return; }
  if (running) return;
  motor.actualPosition(0);
  cartPosition = 0.0f;
  cmdVelocity = 0.0f;
  prevTarget = 0.0f;
  runStartMs = millis();
  lastControlUs = micros();
  digitalWrite(PIN_TMC_EN, LOW);
  running = true;
  Serial.println("STARTED: sine sweep +-8 cm around the current position. Any key stops.");
}

static void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') continue;
    if (c == 'g' || c == 'G') startMotion();
    else if (running) stopMotion("key pressed");
  }
}

static void updateMotion() {
  if (!running) return;
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
  const float dt = (nowUs - lastControlUs) * 1e-6f;
  lastControlUs = nowUs;

  cartPosition = motor.getSPIPosition() / STEPS_PER_METRE;
  if (fabsf(cartPosition) > TRIP_POS_M) { stopMotion("position limit"); return; }
  if (millis() - runStartMs > RUN_LIMIT_MS) { stopMotion("run time limit"); return; }

  const float t = (millis() - runStartMs) * 1e-3f;
  float s = t / RAMP_S;
  s = s >= 1.0f ? 1.0f : s * s * (3.0f - 2.0f * s); // smoothstep amplitude ramp
  const float target = SWEEP_AMPLITUDE_M * s * sinf(SWEEP_OMEGA * t);
  const float targetVel = (target - prevTarget) / dt;
  prevTarget = target;

  float desired = targetVel + POS_KP * (target - cartPosition);
  desired = constrain(desired, -MAX_SPEED, MAX_SPEED);
  const float dv = constrain(desired - cmdVelocity, -MAX_ACCEL * dt, MAX_ACCEL * dt);
  cmdVelocity += dv;
}

static void serviceStepper() {
  if (!running) return;
  motor.setVelocityDirect(cmdVelocity * STEPS_PER_METRE);
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
  const uint32_t ioin = readRegister(REG_IOIN);
  driverOk = (static_cast<uint8_t>(ioin >> 24) == 0x30);
  if (driverOk) configureDriver();
  digitalWrite(PIN_TMC_EN, HIGH);

  Serial2.setRxBufferSize(1024);
  Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  resetWindow();
  Serial.printf("uart_motor_noise_test: driver %s. Centre the cart, then press 'g' to sweep; any other key stops.\n",
                driverOk ? "OK" : "NOT FOUND");
}

void loop() {
  static uint32_t lastReport = micros(), lastEcho = micros();
  handleSerial();
  pollRx();
  updateMotion();
  serviceStepper();
  const uint32_t now = micros();
  if ((uint32_t)(now - lastEcho) >= ECHO_US) { lastEcho += ECHO_US; sendEcho(); }
  if ((uint32_t)(now - lastReport) >= REPORT_US) { lastReport += REPORT_US; report(); }
}
