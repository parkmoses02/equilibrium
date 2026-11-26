# 카메라 인식 뷰어 실행 방법 (처음부터 끝까지)

VS Code를 방금 처음 켰다고 생각하고 순서대로 따라 하면 된다. 괄호 안의 `PS>` 는 입력하지 않는다.

## 이 뷰어가 하는 일

- ESP32 USB(COM3) 하나로 **카메라 인식값을 받아 화면에 표시**하고, 터미널 2(시리얼 도구)가 하던 **키 입력(O, Z, E, C, S, A, X)을 같은 창에서** 한다. `panel/double_console.py` 와 같은 방식이다.
- 그래서 **뷰어가 켜져 있는 동안은 기존 시리얼 도구를 켜면 안 된다.** COM3 은 한 프로그램만 열 수 있다.
- 추가된 파일은 전부 새 파일이다. 기존 파일은 고치지 않았다. 언제든 기존 방식으로 돌아갈 수 있다(맨 아래).

| 파일 | 역할 |
|---|---|
| `src/tests/camera_view_fw.cpp` | `camera_control.cpp` 복사본 + 카메라 데이터를 USB로 보내는 코드 |
| `platformio_camview.ini` | 위 펌웨어용 env `camera_view_windows` |
| `panel/camera_view.py` | 뷰어 창 |
| `panel/camera_link.py` | ESP32 시리얼 읽기/쓰기 |
| `panel/camera_detect.py` | (선택) AI-G 영상 스트림용. ESP32 모드에서는 안 쓴다 |

---

## 0. 시작하기 전에 (장비 확인)

전원을 켜기 전에 눈으로 확인한다.

1. **ESP32** 가 USB 케이블로 PC에 연결되어 있다 (ESP32 쪽 CH340 USB).
2. **AI-G 보드** 가 랜선으로 PC에 직접 연결되어 있고 전원(USB-C)이 켜져 있다. 부팅에 1~2분 걸린다.
3. AI-G 콘솔 UART ↔ ESP32 배선이 연결되어 있다 (AI-G 콘솔 TX → ESP32 GPIO32, RX → GPIO33, GND 공통). 콘솔용 USB-TTL(CP2102)은 **뺀다**.
4. **모터 전원(24~36 V)** 이 켜져 있다. 12 V 는 스윙업에서 탈조가 난다. 드라이버 보드에 전원이 들어오는지(LED) 본다.
5. **카트를 레일 가운데**에 두고, 진자는 아래로 **늘어뜨려 가만히** 둔다.
6. **드라이버 전원 스위치에 바로 손이 닿게** 한다. USB가 끊기면 소프트웨어 정지 키가 전달되지 않는다.
7. 카메라에 **빨간 마커(카트)와 파란 마커 2개(막대 가운데, 끝)** 가 모두 보이고, 막대를 움직여도 화면 밖으로 안 나가게 고정되어 있다.

## 1. VS Code 열기

1. VS Code 를 실행한다.
2. 메뉴 **파일 → 폴더 열기** 를 누르고 아래 폴더를 선택한다.
   `C:\Users\tjdgu\OneDrive\문서\GitHub\equilibrium`
3. "작성자를 신뢰합니까?" 창이 뜨면 **예, 작성자를 신뢰합니다** 를 누른다.
4. 화면 **왼쪽 아래** 의 브랜치 이름이 **`cam`** 인지 확인한다. `cam` 이 아니면 터미널에서 `git switch cam` 을 입력한다 (`seminar` 브랜치에서는 작업하지 않는다).

## 2. 터미널 두 개 열기

1. 메뉴 **터미널 → 새 터미널** (또는 키보드 `Ctrl` + `` ` ``). 화면 아래에 PowerShell 터미널이 열린다. 이것이 **터미널 1** (AI-G 카메라용).
2. 터미널 패널 오른쪽 위의 **`+`** 버튼을 눌러 한 개 더 연다. 이것이 **터미널 2** (뷰어용).
3. 두 터미널 모두 현재 위치가 `...\GitHub\equilibrium>` 로 끝나는지 본다. 아니면 다음을 입력한다.
   ```powershell
   cd "C:\Users\tjdgu\OneDrive\문서\GitHub\equilibrium"
   ```
4. 오른쪽 터미널 목록에서 이름을 우클릭 → **이름 바꾸기** 로 `AI-G`, `뷰어` 라고 붙여 두면 헷갈리지 않는다.

> **명령은 전체를 한 줄로 입력한다.** `Run` 만 따로 치면 "인식되지 않습니다" 오류가 난다. `Run` 은 명령이 아니라 `.\tools\AI-G_run_windows.cmd` 에 주는 옵션이다.

## 3. (처음 한 번) 새 펌웨어 올리기

이미 올렸다면 건너뛴다. 아래처럼 마지막에 `[SUCCESS]` 와 `camera_view_windows  SUCCESS` 가 나왔으면 끝난 것이다.

- 올리기 전에 COM3 을 쓰는 프로그램(PlatformIO 모니터, 예전 시리얼 도구, 이 뷰어)을 **모두 닫는다**.
- **터미널 2** 에 입력한다.
  ```powershell
  & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -c platformio_camview.ini -e camera_view_windows -t upload --upload-port COM3
  ```
- 약 30초~몇 분 걸린다. 올라가는 동안 ESP32 는 멈추므로 모터가 움직이지 않게 전원/카트를 확인한다.
- COM 번호가 COM3 이 아니면 장치 관리자(윈도우 키 → "장치 관리자" → 포트(COM & LPT)) 에서 **USB-SERIAL CH340** 옆의 번호를 확인하고 `COM3` 부분을 바꾼다.

## 4. 터미널 1: AI-G 카메라 켜기

### 4-1. Setup (보드를 켤 때마다 한 번)

센서를 90 fps, 노출 300 으로 설정하고 보드의 콘솔 UART 를 연다.

```powershell
.\tools\AI-G_run_windows.cmd Setup
```

- `topst@192.168.0.100's password:` 가 뜨면 AI-G 의 **topst 계정 비밀번호**를 입력하고 `Enter` 를 누른다. **입력해도 화면에는 아무것도 안 보이는 게 정상**이다. (비밀번호 : topst \ 비밀번호 두번 입력하면 아래 새로운 비번 요청함)
- 두 번째 접속에서 `Password:` 가 또 나오면 **root 비밀번호**를 입력한다 (같은 방식으로 안 보인다). (비밀번호 : root)
- 정상이면 `exposure: 300`, `vertical_blanking: 108`, `analogue_gain: 232`, `digital_gain: 512` 가 나오고 마지막에 `Connection to 192.168.0.100 closed.` 로 끝난다.
- `ping`/접속이 안 되면: AI-G 부팅이 끝났는지(1~2분), 랜선, PC 이더넷 어댑터 IP 가 `192.168.0.1` / `255.255.255.0` 인지 확인한다.

### 4-2. Run (카메라 추적 시작)

```powershell
.\tools\AI-G_run_windows.cmd Run
```

- 비밀번호(topst)를 묻는다. 입력하고 `Enter`. (비밀번호 : topst)
- 시작 줄에 `calibrated rod radius: tip ...px, middle ...px` 가 나와야 한다. **tip 약 139~152, middle 약 53~58** 이면 정상이다.
  - 값이 이상하면(예: 383/354) 마커를 잘못 잡은 것이다. 조명, 카메라 시야, 배경 물체를 확인하고 `Ctrl+C` 로 끄고 다시 `Run` 한다.
  - 이 값은 처음 30프레임 동안 마커 2개가 다 보이는 상태에서 잰다. 막대를 세워 마커가 모두 보이게 둔 채로 시작하는 것이 좋다.
- **이 터미널은 닫지 말고 그대로 둔다.** 실험이 끝날 때까지 계속 돌아간다.
- 기본 54,000 프레임은 90 fps 에서 약 10분이다. 더 오래 쓰려면 프레임 수를 늘린다 (30분 = 162000).
  ```powershell
  .\tools\AI-G_run_windows.cmd Run -Frames 162000
  ```
  프레임이 끝나면 추적이 저절로 멈춘다. 실험 중에는 그 전에 카트를 정지한다.

## 5. 터미널 2: 뷰어 켜기

> **먼저 확인:** 예전 시리얼 도구(`pipeline_serial_windows.py`)가 켜져 있으면 그 터미널에서 `q` 를 눌러 끈다. PlatformIO 시리얼 모니터도 닫는다.

터미널 2 에 입력한다.

```powershell
python panel\camera_view.py
```

- 어두운 색의 큰 창이 뜬다. 처음에는 **시뮬레이션 영상(진자가 혼자 움직이는 화면)** 이 재생된다. 아직 ESP32 에 연결 전이라는 뜻이고 정상이다.
- `ModuleNotFoundError: No module named ...` 가 나오면 필요한 패키지를 설치한다.
  ```powershell
  python -m pip install PyQt5 pyqtgraph numpy pyserial
  ```
- 창이 뜨지 않고 바로 끝나면 터미널에 나온 오류 메시지를 그대로 기록해 둔다.

## 6. 뷰어에서 ESP32 연결하기

1. 창 **왼쪽 위 "1. Camera Link / ESP32 Serial"** 의 `Port:` 칸에 `COM3` 이 적혀 있는지 본다 (다르면 고친다).
2. **`Connect ESP32`** 를 누른다.
3. 성공하면 버튼이 빨간 **`Disconnect`** 로 바뀌고, 왼쪽 아래 로그 상자에 `connected COM3 ...` 가 뜬다.
   - 연결하는 순간 ESP32 가 재시작할 수 있다. 로그에 `=== CAMERA CONTROL + VIEW TELEMETRY ...` 가 뜨면 재시작된 것이고, 영점 정보가 사라졌으니 **`Z` 를 다시 눌러야 한다.**
4. 연결이 안 되고 오류 창이 뜨면 "Access denied / 포트를 열 수 없음" 이다 → 다른 프로그램이 COM3 을 쓰고 있다. 시리얼 도구(`q`), PlatformIO 모니터, 다른 뷰어를 닫고 다시 누른다.

### 연결이 제대로 되었는지 보는 법

- 카메라 화면 맨 위 왼쪽 글자가 `ESP32 USB camera data relayed (25 Hz)` 로 바뀐다. (`AI-G STREAM` 이나 시뮬레이션 글자가 보이면 아직 아니다.)
- 화면 아래 오른쪽 작은 칸 `ANG TIP MID CART` 가 **초록색**이면 AI-G 가 각도와 마커를 잡고 있다는 뜻이다.
- 맨 위 줄의 `last ... ms` 가 몇 ms(수 ~ 20 이하)로 계속 바뀐다.
- 화면에 카메라 값이 안 오면(시뮬레이션 그대로, 또는 `last` 가 계속 커짐): 터미널 1 의 `Run` 이 돌고 있는지, AI-G ↔ ESP32 배선(TX↔RX, GND), `Setup` 을 했는지 확인한다.

## 7. 실험 순서 (뷰어 왼쪽 아래 "5. ESP32 Operator" 그룹)

왼쪽 패널은 스크롤된다. 마우스 휠로 맨 아래까지 내리면 **O / Z / E / C 버튼, `Allow motion buttons`, S / A 버튼, 큰 빨간 STOP 버튼, 로그 상자** 가 있다. 모두 터미널 2 에서 치던 키와 같다.

| 순서 | 버튼 | 하는 일 / 확인할 것 |
|---|---|---|
| 1 | **O  Zero cart** | **카트를 레일 가운데에 둔 상태**에서 누른다. 로그에 `OK: Cart position zeroed` |
| 2 | **Z  Zero encoder** | **진자를 아래로 늘어뜨려 완전히 멈춘 상태**에서 누른다. 로그에 `OK: Downward Zero Set`. 진자가 흔들리면 가만히 멈출 때까지 기다린 뒤 누른다 |
| 3 | **E  Stats on/off** | 상태 줄을 켠다. 카메라 패킷이 잘 들어오는지 보는 용도 |
| 4 | **C  Source cam/enc** | **한 번만** 누른다. 로그에 `OK: balance source = CAMERA` 가 떠야 한다. 두 번 누르면 `ENCODER` 로 돌아간다 |
| 5 | `Allow motion buttons` 체크 | 모터를 움직일 준비가 되었을 때만 체크한다 (잠금 해제) |
| 6 | **S  Swing-up** 또는 **A  Balance** | 아래 설명 |

- **S (스윙업)**: 진자가 아래로 가만히 늘어져 있어야 한다(5° 이내). 처음 약 2초는 카트가 천천히 한쪽으로 10 cm 정도 간 뒤 킥으로 진자를 올리고, 수직 ±25° 안에 들어오면 자동으로 밸런스로 넘어간다. 로그에 `ARMED` → `CATCH` 가 차례로 뜬다.
- **A (밸런스)**: **손으로 진자를 수직 ±45° 안에 잡고** 누른 뒤 손을 뗀다. 처음 시험할 때는 이 방법이 더 안전하다.
- S/A 가 거부될 때 로그에 `REFUSED` 가 뜬다.
  - `zero downward with Z first` → Z 를 먼저 누른다.
  - `let it hang still` → 진자가 아직 흔들린다. 멈출 때까지 기다린다.
  - `no fresh camera data` → 카메라 값이 안 들어온다 (6절 참고).
  - `camera ... and encoder ... disagree` → 카메라와 엔코더 각도가 3° 이상 다르다. 카메라 보정이나 마운트를 확인한다.
- 스윙업이 끝나고 `REC dump received` 가 나온 뒤에 다음 `S` 를 누른다.
- 카트가 가운데에서 벗어났으면 멈춘 뒤 손으로 가운데로 옮기고 `O` 를 다시 누른다.

## 8. 멈추기

- **스페이스바, `Esc`, 또는 큰 빨간 `X  STOP`** 버튼. (창을 클릭하지 않아도 앱이 켜져 있으면 동작한다.)
- 창을 닫거나 `Disconnect` 를 누르면 `X` 를 두 번 보내고 포트를 닫는다.
- **USB 가 빠졌거나 PC 가 멈추면 정지 키가 전달되지 않는다. 드라이버 전원 스위치로 멈춘다.**
- 자동 해제: 진자가 트립 각도를 넘음, 카트가 레일 한계(센 값 ±0.20 m), 카메라 데이터가 60 ms 넘게 끊김, 카메라와 엔코더 각도가 8° 이상 어긋남(스윙 중 12°), 스윙업 10초 초과. 이유는 로그에 `DISARMED: ...` 로 나온다.

## 9. 끝내기

1. 카트가 멈춘 것을 확인한다.
2. 뷰어 창을 닫는다 (`X` 가 전송된다).
3. 터미널 1 에서 `Ctrl+C` 로 AI-G 추적을 끝낸다.
4. 모터 전원을 끈다.

로그: 뷰어가 받은 모든 데이터는 `C:\Users\tjdgu\equilibrium_logs\camview_날짜_시간.log` 에 원본 그대로 저장된다 (연결할 때마다 새 파일).

## 화면에서 알아둘 것

- ESP32 가 받는 것은 **피벗(빨간 마커)의 픽셀 위치와 막대 각도**뿐이다. 끝/가운데 마커의 픽셀 위치는 오지 않아서 **박스 위치는 각도와 막대 반지름(148/60 px)으로 계산한 값**이다. 어느 마커를 썼는지는 `TIP`, `MID` 표시(플래그)로 안다.
- 데이터가 AI-G → UART → ESP32 → USB 를 거치므로 AI-G 경로의 지연이 그대로 남는다. 갱신은 25 Hz 이다.
- 빨간 `CAMERA DATA LOST` 배너는 ESP32 가 마지막 카메라 패킷을 받은 지 60 ms 가 넘었다는 뜻이고, 이때 펌웨어도 정지한다.
- 오른쪽 아래 그래프: 카메라 각도(엔코더와 비교), 마커별 검출, 카트 x, 패킷 지연.

## 문제가 생겼을 때

| 증상 | 원인과 해결 |
|---|---|
| `Run : 'Run' 용어가 ... 인식되지 않습니다` | `Run` 만 입력했다. 전체 명령을 입력한다: `.\tools\AI-G_run_windows.cmd Run` |
| 비밀번호를 입력해도 아무것도 안 보임 | 정상이다. 입력하고 `Enter` |
| `Connect ESP32` 에서 오류 창 | COM3 을 다른 프로그램이 쓰는 중. 시리얼 도구(`q`)/PlatformIO 모니터/다른 뷰어를 닫는다 |
| 연결은 됐는데 화면이 시뮬레이션 그대로 | 카메라 패킷이 안 온다. 터미널 1 의 `Run` 이 돌고 있는지, AI-G↔ESP32 배선, `Setup` 을 했는지 확인. 업로드한 펌웨어가 `camera_view_windows` 인지 확인(로그의 `VIEW TELEMETRY`) |
| 엔코더 값만 보이고 카메라 박스가 없다 | 예전 펌웨어(`camera_control`)가 올라가 있다. 3절로 새 펌웨어를 올린다. 이 경우에도 키와 로그는 정상 동작한다 |
| 연결하니 ESP32 가 재시작되었다 | `Z` 와 `O` 를 다시 누른다 |
| `S`/`A` 를 눌러도 로그에 `blocked` | `Allow motion buttons` 를 체크하지 않았다 |
| `ARMED` 후 카트가 안 움직인다 | 모터 전원(24~36 V)이 꺼졌거나, 부팅 때 드라이버 설정이 건너뛰어졌다. 전원 연결 상태에서 ESP32 리셋 후 `O`, `Z`, `C` 다시. 이전 시험에서 이 경우 STEP 만 세고 카트는 안 움직였다 |
| `calibrated rod radius` 값이 이상하다 | 마커를 잘못 잡음. 조명/시야/배경 확인 후 `Run` 다시 |
| 카트가 한쪽 끝(±0.200 m)으로 밀려가서 `Cart exceeded soft limit` | 각도 오프셋 때문일 수 있다 (`Z` 영점 정확도, 카메라 보정). 이 뷰어의 문제가 아니다 |

## 언제든 기존 방식으로 돌아가기

**방법 1. 뷰어만 끄고 기존 도구 사용 (펌웨어는 그대로 둬도 된다)**

1. 뷰어 창을 닫는다 (`X` 전송).
2. 터미널 2 에서 기존 시리얼 도구를 연다. 새 펌웨어도 키와 텍스트 출력이 기존과 같아서 그대로 동작한다. 추가 텔레메트리는 도구가 걸러낸다.
   ```powershell
   $port = 'COM3'
   $logs = Join-Path $env:USERPROFILE 'equilibrium_logs'
   New-Item -ItemType Directory -Force $logs | Out-Null
   $log = Join-Path $logs ("cam_{0:yyyyMMdd_HHmmss_fff}.log" -f (Get-Date))
   .\.venv_windows\Scripts\python.exe .\tools\pipeline_serial_windows.py $port $log
   ```

**방법 2. 펌웨어까지 원래대로** (뷰어와 시리얼 도구를 모두 닫고)

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e camera_control_windows -t upload --upload-port COM3
```

원래 환경 `camera_control_windows` 는 건드리지 않았다.

**방법 3. 추가한 파일 전부 삭제** (추적 안 된 새 파일뿐이라 `git checkout` 이 필요 없다)

```powershell
Remove-Item panel\camera_view.py, panel\camera_link.py, panel\camera_detect.py, platformio_camview.ini, src\tests\camera_view_fw.cpp, docs\CAMERA_VIEW.md
```

삭제 후 `git status` 에는 `logs/` 만 남아야 한다.

## 확인된 것과 아닌 것

- 확인: 새 펌웨어 업로드 성공(`camera_view_windows SUCCESS`), 파서와 화면(가짜 ESP32 를 루프백 시리얼로 연결해 시험), 키 잠금, 정지 키 전송, repo 위치에서 창 실행.
- 확인 못 함: **새 펌웨어가 실제 AI-G 와 연결된 상태에서 카메라 값을 제대로 보내는지**, 실제 하드웨어에서의 뷰어 동작. 처음에는 모터 전원을 끄거나 카트를 고정한 상태에서 6절의 `ANG TIP MID CART` 가 초록으로 들어오는지만 먼저 확인한다.
