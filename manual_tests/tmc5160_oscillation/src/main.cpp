#include <Arduino.h>
#include <SPI.h>

// ESP32 WROOM + carrier Rev.J pin map.
constexpr uint8_t PIN_TMC_EN   = 13;  // Active LOW
constexpr uint8_t PIN_TMC_STEP = 14;
constexpr uint8_t PIN_TMC_DIR  = 27;
constexpr uint8_t PIN_TMC_CS   = 5;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_SCK  = 18;

// Conservative first-test motion: 200 microsteps/s for one second.
constexpr uint32_t MOVE_TIME_MS = 1000;
constexpr uint32_t PAUSE_TIME_MS = 1000;
constexpr uint32_t STEP_HALF_PERIOD_US = 2500;

constexpr uint8_t REG_GCONF        = 0x00;
constexpr uint8_t REG_IOIN         = 0x04;
constexpr uint8_t REG_GLOBALSCALER = 0x0B;
constexpr uint8_t REG_IHOLD_IRUN   = 0x10;
constexpr uint8_t REG_TPOWERDOWN   = 0x11;
constexpr uint8_t REG_CHOPCONF     = 0x6C;
constexpr uint8_t WRITE_FLAG       = 0x80;

SPISettings tmcSpi(1000000, MSBFIRST, SPI_MODE3);

void writeRegister(uint8_t address, uint32_t value) {
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

uint32_t readRegister(uint8_t address) {
  // TMC5160 returns the requested register on the following SPI transaction.
  for (uint8_t transaction = 0; transaction < 2; ++transaction) {
    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address & 0x7F);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();
  }

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

[[noreturn]] void stopWithError(const char *message) {
  digitalWrite(PIN_TMC_EN, HIGH);
  Serial.println(message);
  Serial.println("Driver remains DISABLED. Remove VM power before checking wiring.");
  while (true) delay(1000);
}

void configureDriver() {
  // SpreadCycle, normal STEP/DIR operation.
  writeRegister(REG_GCONF, 0x00000000);

  // BTT TMC5160 V1.0 has 0.075 ohm sense resistors.
  // IHOLD=4, IRUN=8, IHOLDDELAY=6 and GLOBALSCALER=128 provide a
  // deliberately conservative first-test current (roughly 0.3-0.4 A RMS).
  writeRegister(REG_GLOBALSCALER, 128);
  writeRegister(REG_IHOLD_IRUN, (6UL << 16) | (8UL << 8) | 4UL);
  writeRegister(REG_TPOWERDOWN, 10);

  // 16 microsteps, TOFF=3, HSTRT=4, HEND=1, TBL=2.
  const uint32_t chopconf = (4UL << 24) | (2UL << 15) |
                            (1UL << 7) | (4UL << 4) | 3UL;
  writeRegister(REG_CHOPCONF, chopconf);
}

void moveForOneSecond(bool direction) {
  digitalWrite(PIN_TMC_DIR, direction ? HIGH : LOW);
  delayMicroseconds(20);

  const uint32_t deadline = millis() + MOVE_TIME_MS;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    digitalWrite(PIN_TMC_STEP, HIGH);
    delayMicroseconds(STEP_HALF_PERIOD_US);
    digitalWrite(PIN_TMC_STEP, LOW);
    delayMicroseconds(STEP_HALF_PERIOD_US);
  }
}

void setup() {
  pinMode(PIN_TMC_EN, OUTPUT);
  pinMode(PIN_TMC_STEP, OUTPUT);
  pinMode(PIN_TMC_DIR, OUTPUT);
  pinMode(PIN_TMC_CS, OUTPUT);

  // Keep the power stage disabled until communication and configuration pass.
  digitalWrite(PIN_TMC_EN, HIGH);
  digitalWrite(PIN_TMC_STEP, LOW);
  digitalWrite(PIN_TMC_DIR, LOW);
  digitalWrite(PIN_TMC_CS, HIGH);

  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TMC_CS);

  Serial.println();
  Serial.println("TMC5160 cautious oscillation test");
  Serial.println("Motor must be unloaded; use 12 V VM and a heatsink.");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  const uint32_t ioin = readRegister(REG_IOIN);
  const uint8_t version = static_cast<uint8_t>(ioin >> 24);
  Serial.printf("IOIN=0x%08lX, VERSION=0x%02X\n", ioin, version);
  if (version != 0x30) {
    stopWithError("ERROR: TMC5160 SPI response is invalid.");
  }

  configureDriver();
  digitalWrite(PIN_TMC_EN, LOW);
  Serial.println("Driver enabled. Beginning 1 s left/right motion.");
}

void loop() {
  Serial.println("Direction A: 1 second");
  moveForOneSecond(false);
  delay(PAUSE_TIME_MS);

  Serial.println("Direction B: 1 second");
  moveForOneSecond(true);
  delay(PAUSE_TIME_MS);
}
