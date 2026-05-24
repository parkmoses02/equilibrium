/*
 * ESP32-C3 Inverted Pendulum Controller
 * Hardware: NEMA17 Stepper + 600 CPR Encoder + TMC5160 Driver
 *
 * Compile: platformio run -t upload
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

// ==========================================
// 1. 핀 설정 (ESP32-C3)
// ==========================================
#define SPI_SCK      6    // SPI Clock
#define SPI_MOSI     7    // SPI Data Out
#define SPI_MISO     8    // SPI Data In
#define TMC_CS       5    // TMC5160 Chip Select
#define ENCODER_A    0    // 엔코더 채널 A
#define ENCODER_B    1    // 엔코더 채널 B
#define LED_DEBUG    10   // 디버그 LED (선택사항)

// ==========================================
// 2. 상수 정의
// ==========================================
#define CONTROL_DT       0.01f      // 10ms 제어 주기
#define ENCODER_CPR      600        // 600 CPR 엔코더
#define MOTOR_STEPS      200        // 200 steps/rev
#define MICROSTEPPING    16         // TMC5160 1/16 마이크로스텝
#define PULLEY_CIRCUM    0.04f      // 40mm 풀리
#define PENDULUM_LENGTH  0.3f       // 30cm 진자

// 제어 파라미터
#define U_MAX            15.0f      // 최대 제어력 (N)
#define CATCH_ANGLE      0.35f      // LQR 진입 각도 (rad)
#define K_SWING          40.0f      // Swing-up 게인
#define K_CENTER         10.0f      // 중앙 복귀 탄성
#define KD_CENTER        8.0f       // 중앙 복귀 댐핑

// LQR 제어기 게인 (시뮬레이션에서 계산됨)
#define K_X              -10.0f     // x에 대한 게인
#define K_THETA          -318.2f    // theta에 대한 게인
#define K_XDOT           -5.0f      // x_dot에 대한 게인
#define K_THETADOT       -31.6f     // theta_dot에 대한 게인

// ==========================================
// 3. 전역 변수
// ==========================================
volatile int32_t encoder_count = 0;
volatile int32_t encoder_prev = 0;
uint32_t last_control_time = 0;
float pos_resolution = PULLEY_CIRCUM / (MOTOR_STEPS * MICROSTEPPING);
float angle_resolution = (2.0f * M_PI) / ENCODER_CPR;

struct {
    float x, theta, x_dot, theta_dot;
    float x_buffer[5] = {0};
    float theta_buffer[5] = {0};
    uint8_t buffer_idx = 0;
    char mode;  // 'S' = SWING, 'L' = LQR
} state;

// ==========================================
// 4. TMC5160 SPI 통신
// ==========================================
class TMC5160 {
private:
    SPIClass* spi;
    uint8_t cs_pin;

public:
    TMC5160(SPIClass* spi_bus, uint8_t cs) : spi(spi_bus), cs_pin(cs) {}

    void init() {
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);
        spi->begin(SPI_SCK, SPI_MISO, SPI_MOSI, cs_pin);
        spi->setFrequency(1000000);  // 1MHz SPI
        spi->setDataMode(SPI_MODE3);

        // TMC5160 초기화
        write_reg(0x10, 0x00000000);  // GCONF
        write_reg(0x14, 0x00000000);  // IHOLD_IRUN
        delay(10);
    }

    void set_current(uint16_t ihold, uint16_t irun) {
        uint32_t val = ((ihold & 0x1F) << 16) | (irun & 0x1F);
        write_reg(0x14, val);
    }

    void set_target_current(float current_ma) {
        // current_ma를 TMC5160 레지스터값으로 변환 (0-1000 mA)
        uint16_t irun = (uint16_t)((current_ma / 1000.0f) * 31.0f);
        irun = constrain(irun, 0, 31);
        set_current(irun >> 1, irun);
    }

    void write_reg(uint8_t addr, uint32_t data) {
        uint8_t buf[5] = {(uint8_t)(0x80 | addr),
                          (uint8_t)(data >> 24),
                          (uint8_t)(data >> 16),
                          (uint8_t)(data >> 8),
                          (uint8_t)data};
        digitalWrite(cs_pin, LOW);
        spi->writeBytes(buf, 5);
        digitalWrite(cs_pin, HIGH);
    }

    uint32_t read_reg(uint8_t addr) {
        uint8_t buf[5] = {addr, 0, 0, 0, 0};
        digitalWrite(cs_pin, LOW);
        spi->writeBytes(buf, 5);
        digitalWrite(cs_pin, HIGH);
        delay(1);
        buf[0] = addr;
        digitalWrite(cs_pin, LOW);
        spi->writeBytes(buf, 5);
        digitalWrite(cs_pin, HIGH);
        // Read back (간단한 구현)
        return 0;
    }
};

TMC5160 motor(&SPI, TMC_CS);

// ==========================================
// 5. 엔코더 읽기 (ISR)
// ==========================================
void IRAM_ATTR encoder_isr() {
    int a = digitalRead(ENCODER_A);
    int b = digitalRead(ENCODER_B);

    if (a == b) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

// ==========================================
// 6. 센서 데이터 읽기
// ==========================================
void read_sensors() {
    // 엔코더 처리
    float theta_raw = (float)encoder_count * angle_resolution;

    // 포지션 시뮬레이션 (실제로는 리니어 엔코더 또는 적분으로부터)
    float x = state.x;  // 이전값 유지 (또는 리니어 센서에서 읽기)

    // 양자화
    x = round(x / pos_resolution) * pos_resolution;
    theta_raw = round(theta_raw / angle_resolution) * angle_resolution;

    // 버퍼 업데이트
    state.x_buffer[state.buffer_idx] = x;
    state.theta_buffer[state.buffer_idx] = theta_raw;
    state.buffer_idx = (state.buffer_idx + 1) % 5;

    // 미분 계산 (중앙 차분법)
    float dt = 0.04f;  // 5 * 10ms
    state.x_dot = (state.x_buffer[(state.buffer_idx + 4) % 5] - state.x_buffer[state.buffer_idx]) / dt;
    state.theta_dot = (state.theta_buffer[(state.buffer_idx + 4) % 5] - state.theta_buffer[state.buffer_idx]) / dt;

    // 각도 정규화 (-π ~ π)
    state.theta = fmod(theta_raw + M_PI, 2.0f * M_PI) - M_PI;
    state.x = x;
}

// ==========================================
// 7. 제어 알고리즘
// ==========================================
float compute_control() {
    float u_cmd = 0.0f;

    if (fabs(state.theta) < CATCH_ANGLE) {
        // LQR 제어
        state.mode = 'L';
        u_cmd = K_X * state.x + K_THETA * state.theta +
                K_XDOT * state.x_dot + K_THETADOT * state.theta_dot;
    } else {
        // Swing-up 제어
        state.mode = 'S';
        float I_total = 0.006f + 0.2f * (PENDULUM_LENGTH * PENDULUM_LENGTH);
        float energy = 0.5f * I_total * (state.theta_dot * state.theta_dot) +
                      0.2f * 9.81f * PENDULUM_LENGTH * (cos(state.theta) - 1.0f);

        float u_swing = K_SWING * energy * state.theta_dot * cos(state.theta);
        float u_center = -K_CENTER * state.x - KD_CENTER * state.x_dot;
        u_cmd = u_swing + u_center;
    }

    // 포화 처리
    u_cmd = constrain(u_cmd, -U_MAX, U_MAX);
    return u_cmd;
}

// ==========================================
// 8. 모터 제어 (PWM)
// ==========================================
void drive_motor(float force) {
    // force (N) → PWM duty cycle 변환
    // 최대 토크 기반 (0.5 Nm, 풀리 반지름 20mm → 25N)

    // 방향 결정
    if (force > 0.1f) {
        // CW: GPIO에서 모터 드라이버로 PWM 출력
        // ledcWrite(pwm_channel, pwm_value);
    } else if (force < -0.1f) {
        // CCW
        // ledcWrite(pwm_channel, pwm_value);
    } else {
        // Stop
    }

    // TMC5160 전류 설정 (선택사항)
    float current_ma = fabs(force) * 100.0f;  // 임시 변환 (실제 캘리브레이션 필요)
    motor.set_target_current(current_ma);
}

// ==========================================
// 9. Serial 통신 (모니터링)
// ==========================================
void send_telemetry() {
    static uint32_t last_send = 0;
    if (millis() - last_send >= 100) {  // 100ms 주기
        last_send = millis();
        Serial.printf("x=%.3f theta=%.3f xdot=%.3f thetadot=%.3f mode=%c\n",
                     state.x, state.theta, state.x_dot, state.theta_dot, state.mode);
    }
}

// ==========================================
// 10. 메인 루프
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n[ESP32-C3] Inverted Pendulum Controller Starting...");

    // GPIO 설정
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    pinMode(LED_DEBUG, OUTPUT);

    // 엔코더 인터럽트
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);

    // TMC5160 초기화
    motor.init();
    motor.set_current(10, 20);

    // SPI 설정
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TMC_CS);
    SPI.setFrequency(1000000);
    SPI.setDataMode(SPI_MODE3);

    Serial.println("[OK] Hardware initialized");
    last_control_time = millis();
}

void loop() {
    uint32_t now = millis();

    // 10ms 제어 주기
    if (now - last_control_time >= 10) {
        last_control_time = now;
        digitalWrite(LED_DEBUG, !digitalRead(LED_DEBUG));  // 토글 (디버그용)

        // 1. 센서 읽기
        read_sensors();

        // 2. 제어 계산
        float u_cmd = compute_control();

        // 3. 모터 구동
        drive_motor(u_cmd);

        // 4. 텔레메트리 송신
        send_telemetry();
    }

    delay(1);  // CPU 부하 분산
}
