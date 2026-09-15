# stepdir 브랜치 — STEP/DIR (SD_MODE = 1) 전용

BIGTREETECH TMC5160T Pro 처럼 **SD_MODE 가 기판에서 HIGH 로 묶여 나오는**
드라이버 보드용 코드만 모아둔 브랜치다. 이 보드들은 칩 내장 램프 제너레이터로는
모터가 돌지 않고, XACTUAL 도 세지 않는다. 그래서 펄스는 ESP32 의 LEDC 로 만들고
위치는 PCNT 로 센다.

- 구동 백엔드: `lib/TMC/TMCStepDir.{h,cpp}`. 펄스는 LEDC, 위치는 PCNT.
- `lib/TMC/TMC.{h,cpp}` 는 SPI 연결과 전류/쵸퍼/마이크로스텝 설정만 담은 추상
  베이스다. 모션 지령은 전부 순수 가상이라 이 브랜치에 칩 내장 램프 코드는
  한 줄도 남아 있지 않다.
- `src/main.cpp` 는 `TMCStepDir` 로 고정되어 있다. 백엔드 전환 매크로
  (`USE_STEPDIR_BACKEND`)는 없앴다.
- PCB 원본, 파이썬 캐시 등 펌웨어 빌드에 필요 없는 파일은 이 브랜치에 없다
  (magdi / main 브랜치에 그대로 남아 있다).

## 빌드/업로드

PlatformIO 환경 목록 (`platformio.ini`):

| 환경 | 용도 |
|---|---|
| `esp32dev` (기본) | 본 펌웨어. LQR 제어 + 시리얼 패널 프로토콜 |
| `stepdir_backend_test` | `TMCStepDir` 백엔드 검증. 제일 먼저 돌릴 것 |
| `upright_balance_test` | 손으로 세워보는 단계별 업라이트 밸런스 테스트 |
| `encoder_calibration` | 엔코더 카운트/극성 확인 |
| `tmc5160_oscillation` | 저전류 STEP/DIR 왕복 동작 확인 |
| `tmc5160_diagnostics` | SPI 레지스터 진단. IOIN 으로 SD_MODE 판정 |

```
pio run -e stepdir_backend_test -t upload
pio device monitor -b 115200
```

핀맵과 하드웨어 점검 순서는 `UPLOAD_CHECKLIST.md`, 핀 정의 기준은
`include/MyData.h` 다.

호스트 쪽 GUI/로깅은 `panel/` 과 `src/tests/monitor_and_log.py` 에 있다.
