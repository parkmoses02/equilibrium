/*
 * stepdir_backend_test.cpp - TMCStepDir (SD_MODE = 1) 백엔드 검증
 *
 * 본 펌웨어에 넣기 전에 STEP/DIR 백엔드만 따로 확인한다. 확인 항목:
 *   0) 펄스 경로: LEDC 가 발진하고 PCNT 가 그 펄스를 세는가
 *   1) 위치 모드: 지령한 만큼 정확히 이동하는가
 *   2) 속도 모드: 가속도 지령이 의도한 기울기로 속도를 올리는가
 *   3) 왕복 반복 후 원점 복귀 오차가 누적되지 않는가
 *
 * 안전:
 *   - 카트가 조립된 상태를 전제로 소프트 리밋(시작 위치 기준 +-SOFT_LIMIT_M)을
 *     둔다. 위치 피드백이 끊기면 위치 모드는 목표에 영원히 못 닿아 폭주하므로
 *     이 가드가 없으면 카트가 레일 끝을 들이받는다.
 *   - 부팅 시 자동 실행하지 않는다. 카트를 레일 중앙에 두고 키를 눌러 시작할 것.
 *
 * 사용법:
 *   pio run -e stepdir_backend_test -t upload -t monitor
 *   카트를 레일 중앙에 놓고  a (전체)  또는  0/1/2/3 (개별)
 */
#include <Arduino.h>
#include <MyData.h>
#include <driver/pcnt.h>

TMCStepDir motor(SCK, MOSI, MISO, CS, EN, TMC_STEP, TMC_DIR);

// 본 펌웨어와 동일한 기구 파라미터 (main.cpp 의 Pendulum 생성자와 일치)
constexpr uint32_t MOTOR_STEPS = 200;
constexpr uint32_t MICROSTEPS = 128;
constexpr int32_t STEPS_PER_REV = MOTOR_STEPS * MICROSTEPS; // 25600
constexpr float PULLEY_CIRCUMFERENCE_M = 0.040f;
// 모터 1회전 = 풀리 둘레만큼 카트가 이동한다. 자로 재서 이 값과 맞는지 보면
// distanceRatio 가 맞는지 바로 확인된다.
constexpr float METRES_PER_STEP = PULLEY_CIRCUMFERENCE_M / STEPS_PER_REV;

// 시작 위치 기준 허용 이동 범위. 레일이 좁으면 줄일 것.
constexpr float SOFT_LIMIT_M = 0.08f; // +-80mm

// TMC 내부 단위 환산 (TMCStepDir.cpp 와 동일한 식)
constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;
constexpr float ACC_UNIT_TO_SPS2 = 12000000.0f * 12000000.0f / 2199023255552.0f;

uint32_t spsToUnit(float sps) { return static_cast<uint32_t>(sps / VEL_UNIT_TO_SPS); }
uint32_t sps2ToUnit(float sps2) { return static_cast<uint32_t>(sps2 / ACC_UNIT_TO_SPS2); }

long g_origin = 0; // 소프트 리밋 기준점

void hr(const char *t) {
    Serial.println();
    Serial.println(F("------------------------------------------------------------"));
    Serial.print(F("  "));
    Serial.println(t);
    Serial.println(F("------------------------------------------------------------"));
}

void markOrigin() { g_origin = motor.getSPIPosition(); }

// 소프트 리밋을 감시하면서 대기한다. 범위를 벗어나면 즉시 감속 정지하고
// false 를 돌려준다 (호출부는 나머지 테스트를 중단해야 한다).
bool settle(uint32_t ms) {
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        const float d = static_cast<float>(motor.getSPIPosition() - g_origin) * METRES_PER_STEP;
        if (fabsf(d) > SOFT_LIMIT_M) {
            motor.setRampMode(3); // hold - 감속 정지
            Serial.print(F("  !! 소프트 리밋 초과 ("));
            Serial.print(d * 1000.0f, 1);
            Serial.println(F(" mm) - 정지. 테스트 중단."));
            delay(500);
            return false;
        }
        delay(2);
    }
    return true;
}

// 펄스 경로가 어디서 끊기는지 분리해서 본다.
//   ledcReadFreq == 0        -> LEDC 가 발진하지 않음 (설정/라우팅 문제)
//   freq 는 맞는데 raw == 0   -> 펄스는 나가지만 PCNT 입력이 못 받음
bool test0_probe() {
    hr("0. 펄스 경로 프로브");
    markOrigin();
    int16_t raw = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw);
    Serial.print(F("  시작 PCNT raw = "));
    Serial.println(raw);

    const long before = motor.getSPIPosition();
    motor.setSpeed(spsToUnit(1000.0f));
    motor.setAcceleration(sps2ToUnit(1000000.0f));
    motor.setRampMode(CW);
    delay(300);

    Serial.print(F("  구동 중  velocity = "));
    Serial.print(motor.velocityStepsPerSec(), 0);
    Serial.print(F("  ledcReadFreq = "));
    Serial.print(ledcReadFreq(0));
    Serial.print(F("  ledcRead(duty) = "));
    Serial.println(ledcRead(0));

    if (!settle(700)) return false;
    motor.setRampMode(3);
    if (!settle(300)) return false;

    const long moved = motor.getSPIPosition() - before;
    Serial.print(F("  1초 후 이동 = "));
    Serial.print(moved);
    Serial.println(F(" 마이크로스텝  (1000/s 로 1초 -> 약 1000 이 정상)"));
    return true;
}

bool test1_position() {
    hr("1. 위치 모드 정확도 + 거리 스케일");
    markOrigin();
    motor.setRampMode(0);
    motor.setSpeed(spsToUnit(20000.0f));
    motor.setAcceleration(sps2ToUnit(200000.0f));

    Serial.print(F("  모터 1회전 = 카트 "));
    Serial.print(PULLEY_CIRCUMFERENCE_M * 1000.0f, 1);
    Serial.println(F(" mm 이동이어야 한다. 자로 재볼 것."));

    const int32_t moves[] = {STEPS_PER_REV, -STEPS_PER_REV, STEPS_PER_REV / 4, -STEPS_PER_REV / 4};
    for (int32_t d : moves) {
        const long before = motor.getSPIPosition();
        motor.targetPosition(static_cast<int32_t>(before) + d);
        if (!settle(2500)) return false;
        const long moved = motor.getSPIPosition() - before;
        Serial.print(F("  지령 "));
        Serial.print(d);
        Serial.print(F("  실제 "));
        Serial.print(moved);
        Serial.print(F("  오차 "));
        Serial.print(moved - d);
        Serial.print(F("  ( = 카트 "));
        Serial.print(moved * METRES_PER_STEP * 1000.0f, 2);
        Serial.println(F(" mm)"));
    }
    return true;
}

bool test2_velocity() {
    hr("2. 속도 모드 / 가속도 기울기");
    markOrigin();
    const float targetAccel = 100000.0f; // 마이크로스텝/s^2
    motor.setSpeed(spsToUnit(40000.0f));
    motor.setAcceleration(sps2ToUnit(targetAccel));

    // 한 방향으로만 가면 리밋에 닿으므로 정방향 후 역방향으로 되돌아온다.
    for (uint8_t pass = 0; pass < 2; ++pass) {
        motor.setRampMode(pass == 0 ? CW : CCW);
        const uint32_t t0 = millis();
        for (uint8_t i = 0; i < 3; ++i) {
            if (!settle(100)) return false;
            const float v = motor.velocityStepsPerSec();
            const float expected = targetAccel * (millis() - t0) / 1000.0f;
            Serial.print(F("  "));
            Serial.print(pass == 0 ? F("CW ") : F("CCW"));
            Serial.print(F("  t="));
            Serial.print(millis() - t0);
            Serial.print(F("ms  v="));
            Serial.print(v, 0);
            Serial.print(F("  기대 ~"));
            Serial.print(pass == 0 ? expected : -expected, 0);
            Serial.println(F(" 마이크로스텝/s"));
        }
        motor.setRampMode(3); // hold - 감속 정지
        if (!settle(800)) return false;
    }
    Serial.print(F("  정지 후 v = "));
    Serial.println(motor.velocityStepsPerSec(), 1);
    return true;
}

bool test3_roundTrip() {
    hr("3. 왕복 반복 누적 오차");
    markOrigin();
    motor.setRampMode(0);
    motor.setSpeed(spsToUnit(30000.0f));
    motor.setAcceleration(sps2ToUnit(300000.0f));

    const long origin = motor.getSPIPosition();
    for (uint8_t i = 0; i < 5; ++i) {
        motor.targetPosition(static_cast<int32_t>(origin) + STEPS_PER_REV);
        if (!settle(2000)) return false;
        motor.targetPosition(static_cast<int32_t>(origin));
        if (!settle(2000)) return false;
        Serial.print(F("  "));
        Serial.print(i + 1);
        Serial.print(F("회차 복귀 오차 = "));
        Serial.println(motor.getSPIPosition() - origin);
    }
    return true;
}

void runAll() {
    if (!test0_probe()) return;
    if (!test1_position()) return;
    if (!test2_velocity()) return;
    if (!test3_roundTrip()) return;
    hr("완료");
    Serial.println(F("  오차가 수 마이크로스텝 이내이고 누적되지 않으면 백엔드는 정상."));
    Serial.println(F("  카트가 기대와 반대로 움직이면 TMCStepDir::applyOutput() 의"));
    Serial.println(F("  DIR 극성을 뒤집을 것 (PCNT 부호도 함께 따라간다)."));
}

void printMenu() {
    Serial.println();
    Serial.println(F("============================================================"));
    Serial.print(F("  카트를 레일 중앙에 두세요. 이동 허용 범위 +-"));
    Serial.print(SOFT_LIMIT_M * 1000.0f, 0);
    Serial.println(F(" mm"));
    Serial.println(F("  a = 전체 실행"));
    Serial.println(F("  0 = 펄스 경로 프로브   1 = 위치/거리   2 = 속도   3 = 왕복"));
    Serial.println(F("  s = 현재 위치 읽기 (모터 안 움직임)"));
    Serial.println(F("============================================================"));
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println();
    Serial.println(F("############################################################"));
    Serial.println(F("#   TMCStepDir (SD_MODE=1) BACKEND TEST                    #"));
    Serial.println(F("############################################################"));

    motor.init(0.4f, 1.0f, MSTEPS, 192);
    delay(300);
    Serial.print(F("  초기 위치 = "));
    Serial.println(motor.getSPIPosition());
    // 자동 실행하지 않는다. 카트가 조립된 상태에서 업로드 직후 갑자기
    // 움직이면 위험하므로 반드시 사용자가 시작시킨다.
    Serial.println(F("  모터는 홀딩 상태. 명령을 입력해야 움직입니다."));
    printMenu();
}

void loop() {
    if (!Serial.available()) return;
    switch (Serial.read()) {
    case 'a': runAll(); printMenu(); break;
    case '0': test0_probe(); printMenu(); break;
    case '1': test1_position(); printMenu(); break;
    case '2': test2_velocity(); printMenu(); break;
    case '3': test3_roundTrip(); printMenu(); break;
    case 's':
        Serial.print(F("  위치 = "));
        Serial.print(motor.getSPIPosition());
        Serial.print(F(" 마이크로스텝  ("));
        Serial.print(motor.getSPIPosition() * METRES_PER_STEP * 1000.0f, 2);
        Serial.println(F(" mm)"));
        break;
    default: break;
    }
}
