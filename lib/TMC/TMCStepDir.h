#pragma once
/*
* TMCStepDir.h - STEP/DIR 백엔드
*
* ===========================================================================
*  이 클래스는 SD_MODE = 1 보드용이다  (예: BIGTREETECH TMC5160T Pro v1.0)
* ---------------------------------------------------------------------------
*  3D프린터용 스텝스틱은 A4988 드롭인 호환을 위해 SD_MODE 를 기판에서 HIGH 로
*  묶어 출하한다. 이 상태에서는 TMC5160 의 내장 모션 컨트롤러가 출력단과
*  연결되지 않아 RAMPMODE/XTARGET/VMAX 로는 모터가 전혀 돌지 않는다
*  (XACTUAL 은 램프대로 정확히 세고 position_reached 까지 뜨지만 축은 정지).
*  실측 확인: src/tests/tmc5160_diagnostics.cpp 의 IOIN 비트 6.
*
*  그래서 칩이 하던 두 가지 일을 ESP32 가 대신한다:
*    1) 가감속 적분  -> 소프트웨어 램프 (1kHz FreeRTOS 태스크)
*    2) 위치 카운트  -> PCNT 하드웨어 펄스 카운터
*       (SD_MODE=1 에서는 XACTUAL 이 외부 STEP 펄스를 세지 않는다. 실측 확인)
*  펄스 자체는 LEDC 하드웨어 PWM 이 만들므로, 제어 루프의 연산 부하와 무관하게
*  펄스 간격 지터가 없다. ESP32 는 제어 주기마다 "주파수만" 갱신한다 -
*  SD_MODE=0 에서 VMAX 를 SPI 로 덮어쓰는 것과 구조적으로 동일하다.
*
*  ※ SD_MODE = 0 보드(TMC5160_BOB 등)로 교체하면 이 클래스는 필요 없다.
*     칩 내장 램프를 쓰는 원본 구현은 magdi 브랜치에 있다.
*
*  단위: 지령은 "TMC5160 내부 단위"로 받아 내부에서 물리 단위로 환산한다.
*  따라서 Pendulum.cpp 의 speedRatio/accelerationRatio 와 LQR 게인,
*  캘리브레이션 값을 한 줄도 고치지 않아도 된다.
* ===========================================================================
*/
#include <TMC.h>

class TMCStepDir : public TMC {
    public:
    TMCStepDir(uint8_t sck_, uint8_t mosi_, uint8_t miso_, uint8_t cs_,
               uint8_t en_, uint8_t step_, uint8_t dir_);

    void init(float iHold_=0.05, float iRun_=0.6, float mStep_=0, uint8_t scaler_=192) override;
    void setRampMode(uint32_t mode) override;
    void targetPosition(int32_t value) override;
    void actualPosition(int32_t value) override;
    void setSpeed(uint32_t value) override;
    void setSpeedMax(uint32_t value) override;
    void setAcceleration(uint32_t value) override;
    void setAccelerationMax(uint32_t value) override;
    void setDecelerationMax(uint32_t value) override;
    long getSPIPosition() override;
    int16_t getSPISpeed() override;

    // LQR 처럼 컨트롤러가 가속도를 직접 적분해 속도를 만드는 경우를 위한 직결
    // 지령. 내부 램프를 건너뛰고 주어진 속도를 그대로 펄스 주파수로 만든다.
    // (칩 내장 램프를 쓰는 SD_MODE=0 에는 대응물이 없는, 이 백엔드 전용 경로다.)
    // 단위는 마이크로스텝/s, 부호가 방향. setSpeed() 로 준 상한은 계속 적용된다.
    void setVelocityDirect(float stepsPerSec);

    // 램프 1스텝. 태스크가 주기적으로 부르며, 테스트에서 직접 호출할 수도 있다.
    void rampUpdate(float dt);
    // 현재 지령 속도 [마이크로스텝/s]. 진단/로깅용.
    float velocityStepsPerSec() const { return velocity; }

    private:
    // RAMPMODE 값은 TMC5160 과 같은 의미를 유지한다.
    static constexpr uint32_t RAMP_POSITION = 0;
    static constexpr uint32_t RAMP_VELOCITY_POS = 1; // CW
    static constexpr uint32_t RAMP_VELOCITY_NEG = 2; // CCW
    static constexpr uint32_t RAMP_HOLD = 3;
    // TMC5160 에는 없는 이 백엔드 전용 모드. setVelocityDirect() 가 쓴다.
    static constexpr uint32_t RAMP_DIRECT = 100;

    uint8_t stepPin;
    uint8_t dirPin;

    volatile uint32_t mode = RAMP_HOLD;
    volatile int32_t target = 0;      // 마이크로스텝
    volatile float velocity = 0.0f;   // 마이크로스텝/s (부호 있음)
    volatile float accel = 1000.0f;   // 마이크로스텝/s^2
    volatile float vmax = 1000.0f;    // 마이크로스텝/s
    bool hwReady = false;

    uint32_t lastHz = 0;
    uint8_t lastRes = 0;
    bool pulsing = false;

    void setupPulseHardware();
    void applyOutput(float v);
    void setStepRate(uint32_t hz);
    static void rampTask(void *arg);
};
