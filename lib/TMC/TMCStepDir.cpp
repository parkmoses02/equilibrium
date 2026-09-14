/*
* TMCStepDir.cpp - SD_MODE = 1 (STEP/DIR) 백엔드 구현
* 설계 배경과 SD_MODE=0 과의 관계는 TMCStepDir.h 상단 주석 참조.
*/
#include <TMCStepDir.h>
#include <driver/pcnt.h>
#include <driver/gpio.h>
#include <soc/io_mux_reg.h>

namespace {

// ---- TMC5160 내부 단위 <-> 물리 단위 -------------------------------------
// 데이터시트: v[스텝/s]   = VMAX * f_CLK / 2^24
//             a[스텝/s^2] = AMAX * f_CLK^2 / 2^41
// f_CLK = 12MHz (내부 발진기). Pendulum.cpp 의 speedRatio(1.37) /
// accelerationRatio(0.015) 상수가 바로 이 환산의 역수다.
constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;      // 0.715256
constexpr float ACC_UNIT_TO_SPS2 = 12000000.0f * 12000000.0f / 2199023255552.0f; // 65.48

// ---- LEDC ----------------------------------------------------------------
constexpr uint8_t LEDC_CH = 0;
constexpr uint32_t LEDC_SRC_HZ = 80000000UL; // APB
constexpr uint8_t LEDC_MAX_RES = 14;
// 이 아래로는 LEDC 가 안정적으로 못 내려가고, 실용적으로도 정지로 봐도 된다.
constexpr uint32_t MIN_STEP_HZ = 10;

// ---- PCNT ----------------------------------------------------------------
// 카운터가 16bit 라 한계값에서 이벤트를 받아 상위 자릿수를 누적한다.
constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
constexpr int16_t PCNT_LIMIT = 10000;

volatile int32_t g_posAccum = 0; // PCNT 한계 이벤트로 누적되는 상위 위치
volatile int32_t g_posOffset = 0; // actualPosition() 으로 원점을 옮길 때 쓰는 오프셋

void IRAM_ATTR pcntOverflowISR(void *) {
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT, &status);
    if (status & PCNT_EVT_H_LIM) g_posAccum += PCNT_LIMIT;
    if (status & PCNT_EVT_L_LIM) g_posAccum -= PCNT_LIMIT;
}

portMUX_TYPE g_posMux = portMUX_INITIALIZER_UNLOCKED;

} // namespace

TMCStepDir::TMCStepDir(uint8_t sck_, uint8_t mosi_, uint8_t miso_, uint8_t cs_,
                       uint8_t en_, uint8_t step_, uint8_t dir_)
    : TMC(sck_, mosi_, miso_, cs_, en_), stepPin(step_), dirPin(dir_) {}

void TMCStepDir::setupPulseHardware() {
    // STEP/DIR 두 핀은 "출력인 동시에 PCNT 가 되읽는 입력"이어야 한다.
    // 순서가 중요하다. 아래 두 함정을 모두 피해야 펄스와 카운트가 같이 산다:
    //
    //  (1) pcnt_unit_config() 는 지정된 핀을 입력 전용으로 바꾼다. 그래서
    //      PCNT 를 먼저 설정하고, 출력(LEDC / DIR)은 그 뒤에 다시 걸어야 한다.
    //  (2) gpio_set_direction(..., INPUT_OUTPUT) 은 func_out_sel 을 일반 GPIO 로
    //      되돌려 LEDC 라우팅을 지운다. 입력 버퍼는 IO_MUX 의 PIN_INPUT_ENABLE
    //      로만 따로 켠다.
    //
    // 실측: 이 순서를 어기면 ledcReadFreq 는 정상인데 PCNT 가 0 에 머문다.

    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = stepPin;
    cfg.ctrl_gpio_num = dirPin;
    cfg.lctrl_mode = PCNT_MODE_REVERSE; // DIR LOW  -> 감소
    cfg.hctrl_mode = PCNT_MODE_KEEP;    // DIR HIGH -> 증가
    cfg.pos_mode = PCNT_COUNT_INC;      // 상승 엣지에서 카운트
    cfg.neg_mode = PCNT_COUNT_DIS;
    cfg.counter_h_lim = PCNT_LIMIT;
    cfg.counter_l_lim = -PCNT_LIMIT;
    cfg.unit = PCNT_UNIT;
    cfg.channel = PCNT_CHANNEL_0;
    pcnt_unit_config(&cfg);

    // STEP 선의 짧은 글리치만 무시한다. 값은 APB(80MHz) 클럭 수.
    // 너무 키우면 정상 펄스까지 걸러진다: 고속 구간의 펄스 폭은
    // 640kHz / 50% duty 기준 약 0.78us(=62 사이클)뿐이다. 10 사이클=125ns.
    pcnt_set_filter_value(PCNT_UNIT, 10);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT, pcntOverflowISR, nullptr);
    pcnt_counter_resume(PCNT_UNIT);

    // 여기서부터 출력 복구. PCNT 의 gpio_matrix_in 설정은 별도 레지스터라
    // 아래에서 출력을 다시 걸어도 유지된다.

    // STEP: LEDC 가 펄스를 만들고, 같은 핀을 PCNT 가 되읽어 실제 송출된
    // 펄스를 센다. 지령이 아니라 실제 출력을 세므로 위치가 어긋나지 않는다.
    ledcSetup(LEDC_CH, 1000, 10);
    ledcAttachPin(stepPin, LEDC_CH); // 내부 pinMode(OUTPUT) 이 입력을 끈다
    ledcWrite(LEDC_CH, 0);
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[stepPin]); // 입력 버퍼만 되살림

    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, HIGH);
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[dirPin]);

    g_posAccum = 0;
    g_posOffset = 0;
    hwReady = true;
}

void TMCStepDir::init(float iHold_, float iRun_, float mStep_, uint8_t scaler_) {
    // 펄스 하드웨어를 먼저 세운다. TMC::init() 이 setRampMode()/targetPosition()
    // 을 부르는데 그 호출은 이미 이 클래스의 오버라이드로 들어오기 때문이다.
    setupPulseHardware();

    // 전류/쵸퍼/마이크로스텝 설정은 SPI 로 하는 것이 동일하므로 그대로 재사용한다.
    // (램프 관련 레지스터도 함께 쓰이지만 SD_MODE=1 에서는 무시될 뿐 해롭지 않다.)
    TMC::init(iHold_, iRun_, mStep_, scaler_);

    xTaskCreatePinnedToCore(rampTask, "tmc_ramp", 4096, this, 5, nullptr, 1);
}

// ---- 지령 인터페이스 (단위는 부모 TMC 와 동일한 내부 단위) ------------------

void TMCStepDir::setRampMode(uint32_t m) { mode = m; }

void TMCStepDir::targetPosition(int32_t value) {
    target = value;
    mode = RAMP_POSITION;
}

void TMCStepDir::actualPosition(int32_t value) {
    // 칩의 XACTUAL 쓰기에 대응. 현재 카운터를 그대로 두고 오프셋만 옮긴다.
    portENTER_CRITICAL(&g_posMux);
    int16_t raw = 0;
    pcnt_get_counter_value(PCNT_UNIT, &raw);
    g_posOffset = value - (g_posAccum + static_cast<int32_t>(raw));
    portEXIT_CRITICAL(&g_posMux);
}

void TMCStepDir::setSpeed(uint32_t value) { setSpeedMax(value); }

void TMCStepDir::setSpeedMax(uint32_t value) {
    float v = static_cast<float>(value) * VEL_UNIT_TO_SPS;
    vmax = (v < 1.0f) ? 1.0f : v;
}

void TMCStepDir::setAcceleration(uint32_t value) { setAccelerationMax(value); }

void TMCStepDir::setAccelerationMax(uint32_t value) {
    float a = static_cast<float>(value) * ACC_UNIT_TO_SPS2;
    accel = (a < 1.0f) ? 1.0f : a;
}

// 감속도는 소프트 램프에서 가속도와 동일하게 다룬다. 기존 코드가 항상
// setAcceleration() 으로 네 값을 함께 설정하므로 동작상 차이가 없다.
void TMCStepDir::setDecelerationMax(uint32_t value) { setAccelerationMax(value); }

long TMCStepDir::getSPIPosition() {
    if (!hwReady) return 0;
    portENTER_CRITICAL(&g_posMux);
    int16_t raw = 0;
    pcnt_get_counter_value(PCNT_UNIT, &raw);
    long pos = g_posAccum + static_cast<int32_t>(raw) + g_posOffset;
    portEXIT_CRITICAL(&g_posMux);
    return pos;
}

int16_t TMCStepDir::getSPISpeed() {
    return static_cast<int16_t>(velocity / VEL_UNIT_TO_SPS);
}

// ---- 소프트웨어 램프 제너레이터 -------------------------------------------

void TMCStepDir::setVelocityDirect(float stepsPerSec) {
    const float lim = vmax;
    if (stepsPerSec > lim) stepsPerSec = lim;
    if (stepsPerSec < -lim) stepsPerSec = -lim;
    velocity = stepsPerSec;
    mode = RAMP_DIRECT;
    // 램프 태스크를 기다리지 않고 즉시 반영한다. 제어 주기가 램프 주기보다
    // 빠를 수 있고, 그때 한 틱(1ms)이라도 늦으면 위상 지연이 된다.
    applyOutput(velocity);
}

void TMCStepDir::rampUpdate(float dt) {
    const float a = accel;
    const float vLimit = vmax;
    float v = velocity;
    const float dv = a * dt;

    switch (mode) {
    case RAMP_DIRECT:
        // 속도를 컨트롤러가 직접 준다. 램프는 관여하지 않고 상한만 다시 건다.
        break;
    case RAMP_VELOCITY_POS:
        v += dv;
        break;
    case RAMP_VELOCITY_NEG:
        v -= dv;
        break;
    case RAMP_HOLD:
        if (fabsf(v) <= dv) v = 0.0f;
        else v -= (v > 0.0f ? dv : -dv);
        break;
    case RAMP_POSITION:
    default: {
        const float d = static_cast<float>(target - getSPIPosition());
        // 지금 속도로 최대 감속했을 때 더 가는 거리. 이보다 가까우면 감속.
        const float stopDist = (v * v) / (2.0f * a);
        const bool movingAway = (v > 0.0f && d < 0.0f) || (v < 0.0f && d > 0.0f);
        if (movingAway || fabsf(d) <= stopDist) {
            if (fabsf(v) <= dv) v = 0.0f;
            else v -= (v > 0.0f ? dv : -dv);
        } else {
            v += (d > 0.0f ? dv : -dv);
        }
        // 목표 근처에서 1마이크로스텝씩 떠는 것을 막는다.
        if (fabsf(d) < 1.0f && fabsf(v) < dv) v = 0.0f;
        break;
    }
    }

    if (v > vLimit) v = vLimit;
    if (v < -vLimit) v = -vLimit;
    velocity = v;
    applyOutput(v);
}

void TMCStepDir::applyOutput(float v) {
    if (!hwReady) return;
    // DIR HIGH 를 양의 방향으로 둔다. PCNT 의 hctrl_mode 와 짝이 맞으므로
    // 카트 방향이 반대로 나오면 여기 한 줄만 뒤집으면 된다.
    digitalWrite(dirPin, (v >= 0.0f) ? HIGH : LOW);
    setStepRate(static_cast<uint32_t>(fabsf(v) + 0.5f));
}

void TMCStepDir::setStepRate(uint32_t hz) {
    if (hz < MIN_STEP_HZ) {
        if (pulsing) {
            ledcWrite(LEDC_CH, 0);
            pulsing = false;
        }
        return;
    }
    // LEDC 제약: hz * 2^res <= 80MHz. 이를 만족하는 가장 큰 해상도를 고른다.
    // 해상도를 고정하면 저속과 고속을 한 번에 못 담는다 (10bit 면 76Hz~78kHz).
    uint8_t res = 1;
    while (res < LEDC_MAX_RES && hz <= (LEDC_SRC_HZ >> (res + 1))) res++;

    // 주파수 변경은 타이머 재설정을 동반하므로 의미 있는 변화일 때만 한다.
    const uint32_t delta = (hz > lastHz) ? (hz - lastHz) : (lastHz - hz);
    if (!pulsing || res != lastRes || delta * 200 > lastHz) {
        ledcChangeFrequency(LEDC_CH, hz, res);
        ledcWrite(LEDC_CH, 1UL << (res - 1)); // 50% duty
        lastHz = hz;
        lastRes = res;
        pulsing = true;
    }
}

void TMCStepDir::rampTask(void *arg) {
    TMCStepDir *self = static_cast<TMCStepDir *>(arg);
    const TickType_t period = pdMS_TO_TICKS(1); // 1kHz
    const float dt = 0.001f;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        self->rampUpdate(dt);
        vTaskDelayUntil(&last, period);
    }
}
