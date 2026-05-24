# 하드웨어 구현 체크리스트

## ✅ 사전 준비

### 물리적 조립
- [ ] NEMA17 모터 마운팅
- [ ] 600 CPR 엔코더 조립 (토스 벨트 연결)
- [ ] 진자 길이 30cm 확인
- [ ] 카트 질량 측정 (목표: M_cart = 0.8kg)
- [ ] 진자 질량 측정 (목표: m_pend = 0.2kg)

### 전자 회로
- [ ] ESP32-C3 보드 준비
- [ ] TMC5160 드라이버 보드
- [ ] 30V 전원 공급 (스위칭 파워서플라이)
- [ ] 엔코더 2채널 케이블
- [ ] SPI 케이블 확인

### 소프트웨어
- [ ] PlatformIO 설치 (`platformio core install`)
- [ ] ESP32 패키지 다운로드
- [ ] C++ 컴파일 도구 확인

---

## 🔌 GPIO 핀 연결 확인

### 엔코더 (2채널 Incremental)

회로도의 **J6** 커넥터:
```
J6 핀배치:
1 ─── Encoder GND
2 ─── Encoder A ──→ GPIO 0
3 ─── Encoder B ──→ GPIO 1  
4 ─── Encoder +5V
```

연결 절차:
```bash
# 연결 후 테스트
platformio device monitor -b 115200
# Serial 출력에서 encoder count 증가/감소 확인
```

### SPI 통신 (TMC5160)

```
ESP32-C3          TMC5160
─────────────────────────
GPIO 5  (CS)  ──→ CS
GPIO 6  (SCK) ──→ CLK
GPIO 7  (MOSI)──→ SDI
GPIO 8  (MISO)──→ SDO
GND     ──→ GND
```

---

## 🔄 제어 루프 흐름

```
[10ms 주기]
   ↓
[1. 엔코더 읽기] → encoder_count (정수값)
   ↓
[2. 양자화] → theta = encoder_count * (2π/600)
   ↓
[3. 미분 필터] → 5개 샘플의 차분으로 각속도 계산
   ↓
[4. LQR/Swing-up 선택]
   ├─ |theta| < 0.35rad → LQR 게인 계산
   └─ |theta| ≥ 0.35rad → 에너지 펌핑
   ↓
[5. 제어력 포화] → u_cmd = clip(-15N ~ 15N)
   ↓
[6. PWM 출력] → 모터 구동
   ↓
[7. Telemetry 송신] → Serial (100ms 주기)
```

---

## 📐 파라미터 검증

### NEMA17 모터 특성

| 파라미터 | 값 | 확인 방법 |
|---------|-----|---------|
| 스텝 각도 | 1.8° | 스펙시트 |
| 200 steps/rev | ✓ | 계산: 360° ÷ 1.8° |
| 최대 토크 | 0.5 Nm | 스펙시트 (no-load) |
| 정격 전류 | 1-2 A | 스펙시트 |
| 마이크로스텝 (TMC5160) | 1/16 | SPI 설정 |
| 해상도 | 200×16 = 3200 step/rev | 계산 |

### 600 CPR 엔코더

| 파라미터 | 값 | 확인 |
|---------|-----|------|
| 펄스/회전 | 600 | 스펙시트 |
| 채널 | A, B (2채널) | 회로도 |
| 각도 해상도 | 2π/600 ≈ 0.01047 rad | 계산 |

### 풀리 및 벨트

| 파라미터 | 값 | 확인 |
|---------|-----|------|
| 풀리 둘레 | 40mm | 측정 |
| 포지션 해상도 | 0.04/(200×16) ≈ 0.125mm | 계산 |

---

## 🧪 단계별 테스트

### Test 1: 보드 통신 확인

```bash
platformio run -t upload
platformio device monitor -b 115200

# 기대 출력:
# [ESP32-C3] Inverted Pendulum Controller Starting...
# [OK] Hardware initialized
```

**패스 조건**: Serial 메시지 수신

---

### Test 2: 엔코더 읽기

```cpp
// main.cpp 수정 (임시 테스트 코드)
void loop() {
    Serial.printf("Encoder count: %d\n", encoder_count);
    delay(100);
}
```

**테스트 절차**:
1. 풀리를 손으로 회전
2. encoder_count가 증가/감소하는지 확인

**패스 조건**: ±600 범위 내에서 선형 증가/감소

---

### Test 3: SPI 통신 (TMC5160)

```cpp
// main.cpp 수정
void setup() {
    // ...
    motor.init();
    uint32_t reg_val = motor.read_reg(0x21);  // READ_IOIN
    Serial.printf("TMC5160 IOIN register: 0x%08X\n", reg_val);
}
```

**패스 조건**: 0x00~0xFF 범위의 값 수신

---

### Test 4: 제어 루프 안정성

```bash
# 시리얼 모니터에서 데이터 수집
# 약 100프레임 (1초 × 100 Hz)
# CSV로 저장 및 python으로 분석

import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
with open('telemetry.csv', 'w') as f:
    f.write("time,x,theta,xdot,thetadot,mode\n")
    start = time.time()
    for i in range(1000):
        line = ser.readline().decode()
        t = time.time() - start
        f.write(f"{t},{line}")
```

**분석 체크리스트**:
- [ ] 진자 주기 약 1-2초
- [ ] 제어 주기 10ms ±2% (9.8-10.2ms)
- [ ] 노이즈 수준 정상 범위

---

## 🎛️ 실제 운영 시작

### 초기 전원 온

1. **안전 조치**:
   - 진자를 수평으로 고정
   - 카트를 중앙에 위치
   - 주변 물체 제거 (안전거리 1m)

2. **전원 인가**:
   - ESP32-C3 USB 연결
   - 30V 전원 활성화 (모터)
   - Serial 모니터 확인

3. **모드 확인**:
   - 처음 상태: mode=S (Swing-up)
   - 진자 각도 작아짐: mode=L (LQR)

### 게인 튜닝 절차

1. **기본 게인으로 시작** (BASIC)
2. **현상 관찰**:
   - Swing-up 성공? → 다음 단계
   - Swing-up 실패? → K_swing ↑, 또는 모터 전류 ↑
3. **LQR 안정성 확인**:
   - 부드러운 수렴? → 완료
   - 진동? → Q_theta ↑
   - 과도한 카트 움직임? → Q_x ↑

---

## 🚨 비상 상황 대응

### 진자가 제어 불가능하게 움직임

**즉시 조치**:
1. 모터 전원 차단
2. 진자를 손으로 안정화
3. Serial 로그 확인: `mode=?`

**원인 분석**:
- mode=S (Swing-up)인데 회전 안 함 → 모터 토크 부족
- mode=L (LQR)인데 발산 → 게인 과도

### USB 통신 끊김

```bash
# 재연결
platformio device reset
platformio device monitor -b 115200
```

### 보드 먹통

```bash
# 브릭 상태 복구
esptool.py --chip esp32c3 erase_flash
platformio run -t upload
```

---

## 📊 성능 지표

| 항목 | 목표값 | 측정값 |
|------|--------|--------|
| 제어 주기 | 10ms | ? ms |
| Swing-up 시간 | < 3초 | ? 초 |
| LQR 수렴 시간 | < 2초 | ? 초 |
| 정상 상태 오버슈트 | < 5° | ? |
| 포지션 안정도 | ±50mm | ? mm |

---

## 📞 Support

각 테스트 단계별로 결과를 기록하고,
문제 발생 시 `docs/TROUBLESHOOTING.md` 참고.
