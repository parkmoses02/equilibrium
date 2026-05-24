# ESP32-C3 역진자 제어 펌웨어 통합 가이드

## 🎯 프로젝트 구성

### 하드웨어
```
[MacOS/PC]
    ↓ USB Serial
[ESP32-C3]
    ↓ SPI
[TMC5160 Motor Driver] ← 30V 전원
    ↓
[NEMA17 Stepper Motor] (2상)
    ↓ Toothed Belt
[600 CPR Incremental Encoder]
    ↓ 2 channels (A, B)
[ESP32-C3] (complete loop)
```

### 파일 구조
```
esp32_firmware/
├── main.cpp                  # 메인 제어 루프
├── platformio.ini           # PlatformIO 빌드 설정
├── controller_gains.h       # LQR 게인 정의
├── README.md               # 이 파일
└── docs/
    ├── PINOUT.md           # GPIO 핀 배치
    ├── CALIBRATION.md      # 캘리브레이션 절차
    └── TROUBLESHOOTING.md  # 문제 해결
```

---

## 📋 하드웨어 연결 (회로도 기반)

### GPIO 할당

| 기능 | GPIO | 설명 |
|------|------|------|
| Encoder A | 0 | 인크리멘탈 엔코더 채널 A |
| Encoder B | 1 | 인크리멘탈 엔코더 채널 B |
| SPI CS | 5 | TMC5160 칩 선택 |
| SPI SCK | 6 | SPI 클록 |
| SPI MOSI | 7 | SPI 데이터 출력 (Master Out) |
| SPI MISO | 8 | SPI 데이터 입력 (Master In) |
| LED Debug | 10 | 디버그용 LED (선택사항) |

### 전원 연결

| 핀 | 전압 | 설명 |
|----|------|------|
| J1-1 | +5V | ESP32-C3 전원 |
| J1-2 | +3.3V | GPIO 로직 전원 |
| J1-3 | GND | 그라운드 |
| TMC5160 Supply | +30V | 모터 드라이버 전원 |

---

## 🚀 세팅 및 업로드

### 1단계: 환경 설정

```bash
# PlatformIO CLI 설치 (VS Code + PlatformIO extension)
# 또는 명령줄: pip install platformio

# ESP32-C3 보드 패키지 자동 설치됨
```

### 2단계: 하드웨어 연결

1. ESP32-C3를 MacOS/PC에 USB로 연결
2. 엔코더 2채널을 GPIO 0, 1에 연결
3. SPI 핀 연결 확인:
   - SCK (GPIO 6) → TMC5160 CLK
   - MOSI (GPIO 7) → TMC5160 SDI
   - MISO (GPIO 8) → TMC5160 SDO
   - CS (GPIO 5) → TMC5160 CS
4. TMC5160 전원 (30V) 연결

### 3단계: 코드 컴파일 및 업로드

```bash
# 디렉토리 이동
cd esp32_firmware

# 빌드 및 업로드
platformio run -t upload -e esp32-c3-devkitm-1

# 또는 VS Code에서
# [PlatformIO: Upload] 클릭
```

### 4단계: 시리얼 모니터 확인

```bash
# 모니터 열기
platformio device monitor -b 115200

# 예상 출력:
# [ESP32-C3] Inverted Pendulum Controller Starting...
# [OK] Hardware initialized
# x=0.000 theta=0.000 xdot=0.000 thetadot=0.000 mode=S
# x=0.001 theta=0.005 xdot=0.002 thetadot=0.015 mode=S
```

---

## 🔧 제어 파라미터 튜닝

### 시뮬레이션 → 펌웨어 게인 이전

1. **Python 시뮬레이션 실행**
   ```bash
   python singlePendulum_improved.py
   ```

2. **슬라이더로 LQR 가중치 조정**
   - Q_x, Q_theta, Q_xdot, Q_thetadot 변경
   - 화면에 표시되는 K 게인 확인

3. **최적 게인을 `controller_gains.h`에 입력**
   ```cpp
   static const float LQR_GAINS_CUSTOM[] = {
       K_x_value,       // 시뮬레이션에서 읽은 값
       K_theta_value,
       K_xdot_value,
       K_thetadot_value
   };
   #define CURRENT_GAINS LQR_GAINS_CUSTOM
   ```

4. **펌웨어 재컴파일 및 업로드**
   ```bash
   platformio run -t upload
   ```

### 온라인 튜닝 (Swift 앱 필요)

MacOS의 Swift 앱 (`SwiftCode`)에서:
1. "Reset K gains" 버튼으로 초기화
2. "Adjust Kx, Ktheta, ..." 슬라이더로 실시간 조정
3. "Save to EEPROM" (구현 예정)

---

## 📊 모니터링 및 디버깅

### Serial 데이터 형식

```
x=0.123 theta=-0.045 xdot=-0.012 thetadot=0.005 mode=L
```

- **x**: 카트 위치 (m)
- **theta**: 진자 각도 (rad, -π ~ π)
- **xdot**: 카트 속도 (m/s)
- **thetadot**: 진자 각속도 (rad/s)
- **mode**: 'L' (LQR) 또는 'S' (Swing-up)

### 상태 진단

| 현상 | 원인 | 해결책 |
|------|------|--------|
| 진자가 하강하지 않음 | 모터 부족 | 모터 전류 ↑ 또는 K_swing ↑ |
| Swing-up 후 LQR에서 불안정 | 게인 부족 | Q_theta 또는 Q_thetadot ↑ |
| 카트가 과도하게 흔들림 | Q_x 부족 | Q_x ↑ |
| USB 통신 끊김 | 보드 리셋 | `platformio device reset` |

---

## ⚙️ 향후 개선사항

### Phase 1 (현재)
- [x] 기본 LQR + Swing-up 제어
- [x] 엔코더 읽기
- [x] Serial 모니터링

### Phase 2 (예정)
- [ ] EEPROM에 게인 저장
- [ ] USB 통신으로 실시간 K값 조정
- [ ] 포지션 센서 (리니어 엔코더) 통합
- [ ] 안전 종료 (watchdog)

### Phase 3 (장기)
- [ ] 더블 역진자 확장
- [ ] Kalman 필터 추정기
- [ ] 모터 포화 모델 개선

---

## 📞 기술 지원

문제 발생 시:
1. `Serial Monitor` 출력 확인
2. GPIO 연결 재확인
3. `platformio device monitor -b 115200` 으로 보드 로그 확인
4. `docs/TROUBLESHOOTING.md` 참고

---

## 📝 라이선스

이 코드는 교육 목적으로 자유롭게 수정 및 배포 가능합니다.
