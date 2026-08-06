# ESP32 WROOM 업로드 및 하드웨어 체크리스트

대상 하드웨어는 `Freenove ESP32 WROOM + TMC5160 carrier Rev.C`이다. 이전 ESP32-C3 및 WROVER-CAM 핀맵은 사용하지 않는다.

## 1. 확정 핀맵

| 기능 | ESP32 GPIO | TMC5160/커넥터 | 비고 |
|---|---:|---|---|
| 엔코더 A | GPIO34 | ENC_A | 입력 전용 GPIO |
| 엔코더 B | GPIO35 | ENC_B | 입력 전용 GPIO |
| TMC Enable | GPIO13 | EN | Active Low |
| VSPI CS | GPIO5 | CSN/CFG3 | 부팅 중 High 유지 |
| VSPI MISO | GPIO19 | SDO/CFG0 | 드라이버 → ESP32 |
| VSPI MOSI | GPIO23 | SDI/CFG1 | ESP32 → 드라이버 |
| VSPI SCK | GPIO18 | SCK/CFG2 | SPI clock |
| TMC STEP | GPIO14 | STEP | PCB에 배선됨; 현재 코드는 직접 사용하지 않음 |
| TMC DIR | GPIO27 | DIR | PCB에 배선됨; 현재 코드는 직접 사용하지 않음 |

GPIO16과 GPIO17은 Rev.C PCB에서 NC다. GPIO6~GPIO11은 ESP32 플래시 연결용이므로 외부 회로에 사용하지 않는다.

코드의 기준 정의는 `include/MyData.h`, PCB 기준표는 `../PCB/pinmap.csv`다. 두 파일의 GPIO 번호가 항상 일치해야 한다.

## 2. 전원 및 배선 확인

- ESP32 로직 전원과 TMC5160 모터 전원 36 V를 혼동하지 않는다.
- ESP32 GND, TMC5160 GND, 엔코더 GND는 공통 접지한다.
- 모터 전원은 전원을 끈 상태에서 연결하거나 분리한다.
- TMC5160 방열판을 장착하고, 초기 시험에서는 모터 전류와 동작 시간을 낮게 시작한다.
- 엔코더 커넥터 순서가 `A, B, GND, +5V`인지 실물 라벨과 다시 대조한다.

## 3. 빌드 및 업로드

PlatformIO 기본 환경은 `esp32dev`로 지정되어 있다.

1. VS Code에서 이 프로젝트 폴더를 연다.
2. PlatformIO → Project Tasks → `esp32dev` → Build를 실행한다.
3. USB 포트를 확인하고 Upload를 실행한다.
4. 업로드 후 115200 baud로 Serial Monitor를 연다.

명령줄을 사용할 경우:

```powershell
platformio run --environment esp32dev
platformio run --environment esp32dev --target upload
platformio device monitor --environment esp32dev
```

## 4. 최초 통전 시험

1. 모터 36 V 전원을 끈 채 ESP32만 USB로 켜고 시리얼 출력과 엔코더 A/B 값을 확인한다.
2. TMC5160의 CS가 부팅 중 High이고 EN이 의도대로 제어되는지 확인한다.
3. 모터를 공중에 띄우거나 기구를 안전하게 고정한 뒤 36 V 전원을 넣는다.
4. 낮은 전류 및 저속 명령으로 회전 방향과 엔코더 방향을 확인한다.
5. 과열, 진동, 비정상 소음이 있으면 즉시 모터 전원을 차단한다.

현재 펌웨어는 TMC5160 내부 램프 제어를 SPI로 명령하므로 STEP/DIR 선은 PCB에 존재하지만 직접 펄스를 출력하지 않는다.
