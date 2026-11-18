#pragma once
/*
* TMC.h - TMC5160 SPI 공통 베이스
* Created by Magdi Laoun, July 2025.
*
* ===========================================================================
*  이 브랜치(stepdir)는 SD_MODE = 1 보드 전용이다.
* ---------------------------------------------------------------------------
*  SD_MODE 는 레지스터가 아니라 하드웨어 입력 핀이고 보드가 결정한다.
*  BIGTREETECH TMC5160T Pro 같은 3D프린터용 스텝스틱은 A4988 드롭인 호환을
*  위해 SD_MODE 를 HIGH 로 묶어 출하하는데, 그 상태에서는 칩 내장 램프
*  제너레이터(RAMPMODE/XTARGET/VMAX)가 출력단과 끊겨 있어 모터가 돌지 않는다.
*  (XACTUAL 은 정확히 세고 position_reached 까지 뜨지만 축은 정지.)
*  실측 확인은 src/tests/tmc5160_diagnostics.cpp 의 IOIN 비트 6.
*
*  그래서 이 클래스는 두 백엔드가 공통으로 쓰는 부분 - SPI 연결, 전류/쵸퍼/
*  마이크로스텝 설정 - 만 담고, 모션 지령은 순수 가상 함수로 비워 두었다.
*  실제 구동은 TMCStepDir(STEP/DIR 펄스)이 구현한다. 이 클래스는 추상 클래스라
*  단독으로 인스턴스화되지 않는다.
*
*  ※ SD_MODE = 0 보드(TMC5160_BOB 등)로 교체할 거라면 칩 내장 램프를 쓰는
*     원본 구현이 magdi 브랜치에 그대로 있다.
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
    static constexpr uint8_t TPOWERDOWN = 0x11; //W 8, delay time after stand still of the motor to power down
    static constexpr uint8_t BUFFER_SIZE = 5; //size of buffer for SPI
    void setConfiguration(uint32_t mStep_=0);
    void setChopConf(uint32_t mStep_=5);
    void setTPowerDown();
    void setGlobalConf();
    void setTimePwmThrs();
    void transferData(uint8_t instruction, uint32_t value);

    public:
    TMC(uint8_t sck_, uint8_t  mosi_, uint8_t  miso_, uint8_t cs_, uint8_t en_);
    // 파생 백엔드(TMCStepDir)를 TMC& 로 다루므로 가상 소멸자가 필요하다.
    virtual ~TMC() {}
    void begin(); //Configure pins and SPI; must run from setup(), not a constructor
    void setGlobalScaler(uint8_t scaler_);
    void setCurrent(float iHold_=0.05, float iRun_=0.4);

    // SPI 로 전류/쵸퍼/마이크로스텝을 세우고 출력을 켠다. 파생 클래스는 자기
    // 하드웨어(LEDC/PCNT)를 먼저 준비한 뒤 이 구현을 호출한다.
    //
    // scaler_ is applied before the outputs are enabled; leaving it at the
    // register default (0) would mean full scale for the first few hundred
    // microseconds, so it is part of init() rather than a separate call.
    virtual void init(float iHold_=0.05, float iRun_=0.6, float mStep_=0, uint8_t scaler_=192);

    // --- 모션 지령: 백엔드가 구현한다 --------------------------------------
    // 인자는 "TMC5160 내부 단위"다. 덕분에 Pendulum.cpp 의 변환 비율과 LQR
    // 게인, 캘리브레이션 값이 구현과 무관하게 그대로 유효하다.
    virtual void setAccelerationMax(uint32_t value) = 0;
    virtual void setDecelerationMax(uint32_t value) = 0;
    virtual void setSpeedMax(uint32_t value) = 0;
    virtual void setRampMode(uint32_t mode) = 0;
    virtual void targetPosition(int32_t value) = 0;
    virtual void actualPosition(int32_t value) = 0;
    virtual void setAcceleration(uint32_t value) = 0;
    virtual void setSpeed(uint32_t value) = 0;
    virtual int16_t getSPISpeed() = 0;
    virtual long getSPIPosition() = 0;
};
