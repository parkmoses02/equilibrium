/*
* TMC.cpp - TMC5160 SPI 공통 베이스 구현. 설계 배경은 TMC.h 상단 주석 참조.
 * Created by Magdi Laoun, July 2025.
 */
#include <TMC.h>
#include <SPI.h>
TMC::TMC(uint8_t sck_, uint8_t  mosi_, uint8_t  miso_, uint8_t cs_, uint8_t en_) {
    // Configuration only - see the note in Encoder.cpp. Global constructors
    // run before the core is up, so pin/SPI setup is deferred to begin().
    cs = cs_;
    en = en_;
    sck = sck_;
    mosi = mosi_;
    miso = miso_;
}
void TMC::begin() {
    pinMode(cs, OUTPUT);
    pinMode(en, OUTPUT);
    digitalWrite(cs, HIGH);
    digitalWrite(en, HIGH); // keep the driver disabled until init() configures it
    SPI.begin(sck, miso, mosi);
    SPI.setDataMode(SPI_MODE0);
}
void TMC::init(float iHold_, float iRun_, float mStep_, uint8_t scaler_) {
  begin();
  setConfiguration(mStep_);
  // GLOBALSCALER resets to 0, which the driver reads as full scale. Set it
  // before setCurrent() so IRUN is never briefly applied at 256/256.
  setGlobalScaler(scaler_);
  setCurrent(iHold_, iRun_);
  // 아래 네 줄은 파생 백엔드의 오버라이드로 들어간다 (가상 호출).
  setRampMode(0);
  actualPosition(0);
  targetPosition(0);
  setAcceleration(2000);
  setSpeed(40000);
  digitalWrite(en, LOW); // driver is configured now, so enable the outputs
}
void TMC::setChopConf(uint32_t mStep_) {
  uint32_t TOFF = 3;
  uint32_t HSTRT = 4;
  uint32_t HEND = 1;
  uint32_t TBL = 2;
  uint32_t TPFD = 0;
  uint32_t value = TOFF<<0
                  | HSTRT<<4
                  | HEND<<7
                  | TBL<<15
                  | TPFD<<20
                  | mStep_<<24;
  uint8_t instruction = CHOPCONF | WRITE;
  transferData(instruction, value);
}
void TMC::setCurrent(float iHold_, float iRun_) {
  // 31/2 was integer division (15, not 15.5), and the Serial.print() dumped
  // ASCII into the binary telemetry stream - both removed.
  uint32_t iH = static_cast<uint32_t>(31.0f / 2.0f * iHold_);
  uint32_t iR = static_cast<uint32_t>(31.0f / 2.0f * iRun_);
  if (iH > 31) iH = 31;
  if (iR > 31) iR = 31;
  uint32_t delay = 6;
  uint32_t value = iH | iR << 8 | delay << 16;
  uint8_t instruction = IHOLD_IRUN | WRITE;
  transferData(instruction, value);
}
void TMC::setTPowerDown(){
  uint32_t value = 10;
  uint8_t instruction = TPOWERDOWN | WRITE;
  transferData(instruction, value);
}
void TMC::setConfiguration(uint32_t mStep_) {
    setChopConf(mStep_); //Set microstepping to 1/16
    setTPowerDown();
    setGlobalConf();
    setTimePwmThrs();
}
void TMC::setGlobalConf() {
  uint32_t en_pwm_mode = 1;
  uint32_t value = en_pwm_mode<<2;
  uint8_t instruction = GCONF | WRITE;
  transferData(instruction, value);
}
void TMC::setGlobalScaler(uint8_t scaler_) {
  uint8_t instruction = GLOBALSCALER | WRITE;
  transferData(instruction, scaler_);
}
void TMC::setTimePwmThrs() {
  uint32_t value = 500;
  uint8_t instruction = TPWMTHRS | WRITE;
  transferData(instruction, value);
}
void TMC::transferData(uint8_t instruction, uint32_t value) {
  uint8_t data[BUFFER_SIZE];
  data[4] = (value >> 0) & 0xFF;
  data[3] = (value >> 8) & 0xFF;
  data[2] = (value >> 16) & 0xFF;
  data[1] = (value >> 24) & 0xFF;
  data[0] = instruction | WRITE;
  digitalWrite(cs, LOW);
  SPI.transfer(data, BUFFER_SIZE);
  digitalWrite(cs, HIGH);
}
