# CLAUDE.md — equilibrium (도립진자)

카트 위 도립진자. ESP32 펌웨어(PlatformIO) + PC 패널(PyQt5). **지금 진행 중인 작업: 엔코더 대신 TOPST AI-G + IMX219 카메라로 각도(와 카트 위치)를 받는다.**
응답은 한국어로. 코드의 식별자/주석은 주변 파일의 스타일을 따른다.

## 0. 가장 중요한 규칙

1. **`seminar` 브랜치에는 수정/커밋/푸시를 하지 않는다.** `seminar`는 엔코더 기반의 검증된 기준이다. 카메라 작업은 전부 **`cam` 브랜치에서** 한다.
   ```
   git status                      # 작업 트리가 깨끗한지 확인 (깨끗하지 않으면 사용자에게 먼저 물어볼 것)
   git branch --show-current       # cam이 아니면: git switch cam
   ```
   `cam`에서는 바로 수정하고 커밋해도 된다. 새 브랜치를 만들지 않는다 (`cam`이 있어서 `cam/...` 이름은 git이 만들 수 없기도 하다). 사용자가 따로 요청할 때만 다른 브랜치를 쓴다. 병합(merge)과 PR은 사용자가 한다.
2. **푸시는 사용자가 요청할 때만.** `git push -u origin <브랜치>`. force push 금지.
3. **`git add .` / `git add -A` 금지.** 바꾼 파일만 이름으로 add 한다. 아래는 절대 stage하지 않는다:
   `panel/angle_logs/`, `__pycache__/`, `.pio/`, `src/tests/logs/`, 사용자 로컬 전용 스크립트 `panel/inverse_lqr.py`, `panel/inverse_lqr_sweep.py`, `panel/balance_sim.py`.
4. **하드웨어를 움직이는 일을 자동으로 하지 않는다.** `pio ... -t upload`, 시리얼 포트 열기, 모터가 움직일 수 있는 명령은 사용자가 요청했을 때만. 컴파일 확인은 `pio run -e <env>`(업로드 없이)까지만.
5. **하드웨어 검증이 끝난 파일은 고치지 않는다.** 새 기능은 새 파일 + 새 env로 만든다 (아래 3번).

## 1. 토큰 절약 규칙

- `tree`, `ls -R`, 넓은 glob 금지. 아래 "파일 지도"를 먼저 보고 필요한 파일만 연다.
- 큰 파일은 통째로 읽지 말고 `Grep`으로 위치를 찾은 뒤 `offset/limit`으로 필요한 줄만 읽는다.
  `panel/pipeline_controller.py`(약 1.5k줄), `src/tests/pipeline_energy_lqr.cpp`(약 650줄), `src/tests/upright_balance_test.cpp`(약 940줄), `docs/` 의 노트.
- 읽지 말 것: `.git/`, `.pio/`, `.venv/`, `__pycache__/`, `*.png`, `*.csv`, `*.raw`, `*.bin`, `*.zip`, `panel/angle_logs/`, `src/tests/logs/`, `lib/` 전체(벤더 라이브러리).
- 같은 세션에서 이미 읽은 파일을 다시 읽지 않는다. `Edit` 후 검증용으로 다시 `Read` 하지 않는다 (실패하면 Edit가 에러를 낸다).
- 파일 전체를 다시 쓰지 말고 `Edit`로 바뀐 부분만 고친다.
- 명령 출력은 `head`/`tail`/`grep`으로 줄인다. 긴 diff나 파일 내용을 답변에 붙여넣지 않는다. 결과는 5줄 이내로 요약한다.
- 300줄 넘는 문서를 읽거나, 빌드/패키지 설치를 하기 전에는 사용자에게 먼저 묻는다.
- 카메라 노트 전체는 `docs/AI-G_camera_pendulum_notes.md`. 이 CLAUDE.md의 "카메라" 절로 부족할 때만, 해당 소제목만 찾아서 읽는다.

## 2. 파일 지도

| 경로 | 설명 | 수정 |
|---|---|---|
| `src/tests/pipeline_energy_lqr.cpp` | 현재 PC 패널과 연동되는 펌웨어 (스윙업 상태머신 + 리카티 LQR, 500Hz) | 요청 시 |
| `src/tests/upright_balance_test.cpp` | **하드웨어에서 검증된 기준 코드** (글자 명령 전용) | **금지** (읽기만) |
| `src/main.cpp`, `src/Pendulum.cpp`, `src/Speed.cpp`, `include/*` | 원래 펌웨어 | 요청 시에만 |
| `lib/TMC/*`, `lib/Encoder/*` | TMC5160 STEP/DIR 드라이버, 엔코더 라이브러리 | **금지** |
| `src/tests/encoder_calibration.cpp`, `tmc5160_diagnostics.cpp`, `tmc5160_oscillation.cpp`, `stepdir_backend_test.cpp` | 개별 하드웨어 시험 | 금지 (읽지 않음) |
| `src/tests/monitor_and_log.py` | 시리얼 로그 도구 | 무시 |
| `panel/test_swingup_statemachine.py` | 스윙업 상태머신의 Python 포팅 (시뮬 검증용) | 요청 시 |
| `panel/__init__.py`, `panel/README.md`, `panel/requirements.txt` | 패키지 표식, 설명, 의존성 | 무시 (의존성 추가는 요청 시) |
| `include/MyData.h` | 공용 헤더 | 요청 시 |
| `include/README`, `lib/README`, `test/README` | PlatformIO 기본 문서 | 무시 |
| `platformio.ini` | env마다 `build_src_filter`로 파일 하나만 빌드 | env 추가만 |
| `panel/pipeline_controller.py` | pipeline용 PC GUI (시뮬, 텔레메트리, 게인 전송, 기록) | 요청 시 |
| `panel/simulator.py`, `panel/serial_protocol.py` | `pipeline_controller.py`와 `panel.py`가 공용으로 씀 (A, B 행렬, DARE, 시리얼) | 요청 시에만 |
| `panel/panel.py` | 원래 패널 (`src/main.cpp`용) | 금지 |
| `README_STEPDIR.md`, `UPLOAD_CHECKLIST.md`, `.vscode/` | 문서/설정 | 무시 |

AI-G 쪽 프로그램(`pend_cam.c`)과 보드 설정은 **이 repo 밖**에서 관리한다고 가정한다. repo 안에서 찾지 말고, 필요하면 사용자에게 경로를 물어본다.

## 3. 새 펌웨어 코드를 만드는 방법

- 예: 카메라 입력용이면 `src/tests/camera_pipeline.cpp` 를 새로 만들고, `pipeline_energy_lqr.cpp`에서 필요한 부분만 복사해서 시작한다.
- `platformio.ini`에는 기존 패턴 그대로 env를 **추가**한다 (기본 `esp32dev`의 `build_src_filter`는 건드리지 않는다).
  ```
  [env:camera_pipeline]
  platform = espressif32
  board = esp32dev
  framework = arduino
  monitor_speed = 115200
  build_src_filter =
      -<*>
      +<tests/camera_pipeline.cpp>
  ```
- 확인: `pio run -e camera_pipeline` (업로드 금지). Python은 `python -m py_compile <파일>`.

## 4. 현재 시스템 요약

- 제어 500Hz (`CONTROL_PERIOD_US = 2000`), 텔레메트리 50Hz, GUI 갱신 약 30Hz. PC↔ESP32 시리얼 115200.
- 카트 위치는 **엔코더가 아니라 모터에 보낸 STEP 수를 센 값**(open-loop, `STEPS_PER_METRE = 80000`). 탈조가 나면 실제와 어긋난다.
- 진자 각도는 엔코더(2400 count/rev). 수직 위가 0, 아래가 ±180. 각속도는 카운트 차분 + 저역통과(0.25).
- 한계(기본): 가속도 12 m/s², 속도 0.8 m/s, 레일 ±0.35 m(arm한 자리가 0), 직접 balance arm window ±45°(트립 = window + 10°, 최소 25°).
- **부호 규약:** 펌웨어 극성 `controlPolarity = -1`. GUI의 시뮬레이션 좌표계와는 카트 방향이 반대다. 리카티 `K`(u = -Kx)는 **`-K`(양수 크기)** 로 펌웨어에 보낸다. 이 규칙을 바꾸지 않는다.
- PC→ESP32 명령 프레임: `[0xAA][주소][4바이트]`(6바이트). 모드 명령(0x50)은 첫 바이트가 모드(0 정지, 1 스윙업, 2 밸런스). ESP32→PC 텔레메트리도 같은 헤더, 주소 0x00~0x06(각도 rad, 각속도, 위치 m, 속도, 에너지, 가속도, 모드).
- 주소: 0x01 속도 한계, 0x02 가속도 한계, 0x07 레일 한계, 0x0B 진자 길이, 0x0C 에너지 마진, 0x20~0x23 게인(kAngle, kRate, kCartPos, kCartVel), 0x24~0x26 소프트 밸런스, 0x27 arm window(deg), 0x50 모드, 0x52 엔코더 영점, 0x53 카트 위치 영점.
- 글자 명령(시리얼 모니터): Z 영점, A 밸런스, S 스윙업, X/스페이스 정지, P 극성, D 소프트 밸런스, O 카트 위치 영점.

## 5. 카메라 (AI-G + IMX219) 작업

**구조:** AI-G가 영상에서 마커를 찾아 각도와 카트 위치(px)를 UART로 ESP32에 보낸다. **칼만 필터, 속도 추정, 모터 제어는 ESP32**가 한다 (모터 명령을 필터 입력으로 쓰고, 늦게 도착한 카메라 값에 맞춰 상태를 앞으로 굴리는 지연 보정이 필요하다). 엔코더 경로는 **지우지 말고** 컴파일 옵션 등으로 남겨 둔다 (검증된 동작 보존).

**카메라:** 640x480 모드(출력 632x464 UYVY), 90fps(프레임 주기 11.1ms). 빨간 마커 = 회전 중심(카트), 파란 마커 2개 = 막대 가운데와 끝. 보드 안 처리 시간 평균 7.56ms. **카메라 전체 지연(실제 움직임 → ESP32 수신)은 엔코더와 상호상관으로 측정함: 약 14ms** (로그 전/후반 13.7/14.0ms, 패킷 age 7.6ms + 나머지 약 6.3ms). 제어 500Hz 기준 약 7틱이다. 제어는 500Hz이므로 프레임 사이(약 5~6 틱)는 예측으로 메운다. 측정 도구: `src/tests/camera_delay_log.cpp`(env `camera_delay_log`) + `tools/camera_delay_xcorr.py`.

**UART:** AI-G 헤더 8번(TX, `/dev/ttyAMA1`) → ESP32 GPIO32(**RX**), 10번(RX) → ESP32 GPIO33(**TX**), GND 공통. 460800bps, 8N1, 흐름 제어 없음. ESP32: `Serial2.begin(460800, SERIAL_8N1, 32, 33)`. AI-G GPIO 전압(3.3V 여부)은 **미확인**. 헤더 2, 4번은 5V이니 ESP32 입력에 닿게 하지 않는다.

**패킷 (14바이트, little-endian):**

| 바이트 | 내용 |
|---|---|
| 0, 1 | `0xAA`, `0x55` |
| 2~3 | 프레임 번호 u16 (V4L2 시퀀스, 넘치면 처음부터) |
| 4~5 | 각도 i16, 0.01° 단위, `0x8000` = 무효 |
| 6~7 | 카트 x u16, 0.1px 단위, `0xFFFF` = 무효 |
| 8~9 | 카트 y u16, 0.1px 단위 |
| 10 | 플래그: b0 각도 유효, b1 끝 마커 사용, b2 가운데 마커 사용, b3 카트 위치 유효, b4 막대 길이 보정 중, b5 age 유효 |
| 11~12 | age u16, 0.1ms 단위 (드라이버 프레임 시각부터 전송 직전까지) |
| 13 | CRC-8 (다항식 0x07, 초기값 0, 바이트 2~12) |

수신 순서: `0xAA 0x55` 동기 → 14바이트 → CRC 확인. 수신 시각을 `micros()`로 같이 기록한다. **실제 UART와 ESP32 수신은 아직 시험하지 않았다.**

**주의할 점**
- 카메라 각도: 0° = 막대가 위로 섬, **오른쪽으로 기울면 +**, 범위 ±180(경계에서 뛰므로 ESP32에서 이어 붙인다). 펌웨어(엔코더)의 각도 부호와 같다고 **가정하지 않는다**. 측정 결과(진자를 크게 흔든 20초 로그): 카메라 부호는 엔코더(수직 위 0)와 **반대**이고 영점 차이 약 +0.96° (enc ≈ -cam - 0.96°), 배율은 카메라가 약 0.9% 크게 나온다(원인 미확인: 엔코더 카운트 수, 카메라 비스듬함, 렌즈 왜곡 중 무엇인지 모름). 0점은 손으로 늘어뜨려 잡은 것이라 ±1° 정도 불확실하다.
- 카트 위치는 px 단위다. mm로 바꾸려면 막대/레일 길이로 보정이 필요하다 (둘 다 아직 모른다). 화면 폭은 632px.
- 빠른 움직임(스윙업)에서는 마커가 번진다. 노출 조정은 AI-G 쪽 일이다.
- 조명이 바뀌면 AI-G 쪽 색 임계값을 다시 맞춰야 한다 (이 repo의 일이 아니다).

**확인됨:** AI-G GPIO 전압 3.3V(8번 핀 측정), UART 양방향 460800 / 2m 점퍼선 / 카트 구동 중에도 오류 없음(더미 송신기 기준, 느린 왕복만 시험).
**미해결 (임의로 정하지 말고 사용자에게 확인):** 막대 길이와 레일 길이, 빠른 동작(0.8 m/s, 12 m/s²)에서의 UART 노이즈, 카메라 시야에서 막대 원이 일부 잘리는 구간(어느 쪽인지 미확인).

## 6. AI-G 보드에 SSH로 들어갈 때만

- 보드 `192.168.0.100`, 계정 `topst` (`sudo` 없음, root는 `su -`). PC 쪽 `enp1s0`은 `192.168.0.1/24`.
- `/dev/ttyAMA2`는 커널 콘솔이다. **`stty`/`echo` 등으로 절대 건드리지 않는다.**
- 종료는 `su -` 후 `shutdown -h now`. 사용자가 시킬 때만 한다.

## 7. 작업을 끝낼 때

- 바뀐 파일, 확인한 것(컴파일/문법/모의 실행), **확인 못 한 것(하드웨어 검증 여부)** 을 짧게 알린다.
- 커밋 목록을 알리고, 푸시/병합 여부는 사용자에게 묻는다. 커밋은 기능 단위로 작게 나눈다 (되돌리기 쉽게).
- 커밋 메시지는 영어 한 줄 요약으로 쓴다 (예: `Add camera UART packet parser`).
