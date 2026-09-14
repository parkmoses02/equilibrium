#pragma once
/*
* TMC.h - Library for controlling TMC5160 stepper motor driver via SPI.
* Created by Magdi Laoun, July 2025.
*
* ===========================================================================
*  이 클래스는 SD_MODE = 0 보드용이다  (예: TMC5160_BOB_V1.0)
* ---------------------------------------------------------------------------
*  TMC5160 의 내장 모션 컨트롤러(RAMPMODE / XTARGET / VMAX / AMAX)로 모터를
*  구동한다. 가감속 적분과 위치 카운트를 전부 칩이 처리하므로 ESP32 는 제어
*  주기마다 레지스터만 갱신하면 되고, 펄스 타이밍 지터가 원천적으로 없다.
*
*  ※ 주의: SD_MODE 는 레지스터가 아니라 하드웨어 입력 핀이며 보드가 결정한다.
*     BIGTREETECH TMC5160T Pro 같은 3D프린터용 스텝스틱은 A4988 드롭인 호환을
*     위해 SD_MODE 를 HIGH 로 묶어 출하한다. 그런 보드에서는 이 클래스로
*     모터가 돌지 않는다 (XACTUAL 은 정확히 세지만 출력단과 끊겨 있다).
*     그 경우 TMCStepDir 을 쓸 것. 전환 스위치는 include/MyData.h 참조.
* ===========================================================================
*/
#include <Arduino.h>
#define FULL_CURRENT 0.4
#define STAND_CURRENT 0.05
class TMC {
    private:
    uint8_t en; //TMC5160 Enable
    uint8_t miso; //SPI MISO -> SDO
    uint8_t mosi; //SPI MOSI -> SDI
    uint8_t sck; //SPI Clock
    uint8_t cs;  //Channel select
    
    static constexpr uint8_t WRITE = 0x80;
    static constexpr uint8_t GCONF = 0x00; //RW 18 , global configuration flags
    static constexpr uint8_t GLOBALSCALER = 0x0B; //Global scaling of Motor current
    static constexpr uint8_t CHOPCONF = 0x6C; //RW 32 Chopper and driver configuration
    static constexpr uint8_t IHOLD_IRUN = 0x10; //W 5+5+4 Setting of current, holding and running
    static constexpr uint8_t TPWMTHRS = 0x13; //W 20 upper velocity of stealthChop voltage PWM mode
    static constexpr uint8_t RAMPMODE = 0x20; //RW 2, 0 = positionning, 1, 2 = velocity, 3 = hold mode
    static constexpr uint8_t XACTUAL = 0x21; //RW 32 actual motor position (signed)
    static constexpr uint8_t VACTUAL = 0x22; //R 24 actual motor velocity from ramp generator
    static constexpr uint8_t VSTART = 0x23; //W 18 motor start velocity (unsigned)
    static constexpr uint8_t A1 = 0x24; //W 16 first acceleration between VStart and V1, unsigned
    static constexpr uint8_t V1 = 0x25; //W 20 first acceleration/deceleratuib phase threshold velocity
    static constexpr uint8_t AMAX = 0x26; //W 16 second acceleration between V1 and Vmax
    static constexpr uint8_t VMAX = 0x27; //W 23 motion ramp target velocity in positionning mode, target velocity in velocity mode.
    static constexpr uint8_t DMAX = 0x28; //W 16 deceleration between vmax and v1
    static constexpr uint8_t D1 = 0x2A; //W 16 deceleration between V1 and VStop
    static constexpr uint8_t VSTOP = 0x2B; //W 18 motor stop velocity (must not be 0)
    static constexpr uint8_t XTARGET = 0x2D; //RW 32 target position
    static constexpr uint8_t TPOWERDOWN = 0x11; //W 8, delay time after stand still of the motor to power down
    static constexpr uint8_t BUFFER_SIZE = 5; //size of buffer for SPI
    void setConfiguration(uint32_t mStep_=0);
    void setChopConf(uint32_t mStep_=5);
    void setTPowerDown();
    void setGlobalConf();
    void setTimePwmThrs();
    void transferData(uint8_t instruction, uint32_t value);
    void setAcceleration1(uint32_t value);
    void setSpeed1(uint32_t value);
    void setDeceleration1(uint32_t value);
    void getSPIValue(uint8_t instruction, uint8_t *data);
    void setSpeedStop();
    
    public:
    static uint8_t statVal;
    TMC(uint8_t sck_, uint8_t  mosi_, uint8_t  miso_, uint8_t cs_, uint8_t en_);
    // 파생 백엔드(TMCStepDir)를 TMC& 로 다루므로 가상 소멸자가 필요하다.
    virtual ~TMC() {}
    void begin(); //Configure pins and SPI; must run from setup(), not a constructor
    void setGlobalScaler(uint8_t scaler_);
    void setCurrent(float iHold_=0.05, float iRun_=0.4);

    // --- 아래 모션 관련 함수들은 백엔드마다 구현이 다르다 -------------------
    // SD_MODE=0 : 여기 구현대로 칩의 램프 제너레이터에 SPI 로 지시
    // SD_MODE=1 : TMCStepDir 이 오버라이드하여 STEP/DIR 펄스로 대체
    // 인자 단위는 두 백엔드가 동일한 "TMC5160 내부 단위"를 쓴다. 덕분에
    // Pendulum.cpp 와 LQR 게인을 백엔드 교체와 무관하게 그대로 쓸 수 있다.
    //
    // scaler_ is applied before the outputs are enabled; leaving it at the
    // register default (0) would mean full scale for the first few hundred
    // microseconds, so it is part of init() rather than a separate call.
    virtual void init(float iHold_=0.05, float iRun_=0.6, float mStep_=0, uint8_t scaler_=192);
    virtual void setAccelerationMax(uint32_t value);
    virtual void setDecelerationMax(uint32_t value);
    virtual void setSpeedMax(uint32_t value);
    virtual void setRampMode(uint32_t mode);
    virtual void targetPosition(int32_t value);
    virtual void actualPosition(int32_t value);
    virtual void setAcceleration(uint32_t value);
    virtual void setSpeed(uint32_t value);
    virtual int16_t getSPISpeed();
    virtual long getSPIPosition();
};

