# AI-G 카메라 실행 · Windows

VS Code에서 이 저장소를 열고 PowerShell 터미널 두 개를 사용한다. 터미널 1은 SSH로 AI-G의 IMX219 카메라를 실행하고, 터미널 2는 USB로 연결한 ESP32를 조작한다. PC에서는 WSL 없이 실행할 수 있다. Linux PC에서 작업할 때는 [기존 실행 문서](AI-G_run.md)를 따른다.

## 처음 한 번

1. Windows에서 AI-G에 직결한 이더넷 어댑터의 IPv4 주소를 `192.168.0.1`, 서브넷 마스크를 `255.255.255.0`으로 설정한다. 이 직결 어댑터의 게이트웨이는 비워 둔다. 보드 IP는 `192.168.0.100`이다. 같은 IP를 쓰는 보드 중 IMX219 보드만 켠다.
2. 콘솔 USB-TTL(CP2102)을 분리하고 AI-G 콘솔 TX의 대기 전압이 약 3.3V인지 확인한다. AI-G 콘솔 TX → ESP32 GPIO32(RX), RX → GPIO33(TX), GND → GND로 연결한다. 현재 장치에서 40핀 헤더 8·10번은 쇼트 상태다. ESP32의 CH340 USB는 PC에 연결한다.
3. Windows OpenSSH Client(`ssh`, `scp`), Python 3.10 이상, VS Code의 PlatformIO IDE가 필요하다. 저장소 루트에서 아래 명령을 실행한다. 이미 준비된 가상환경은 바로 사용할 수 있다.

```powershell
python -m venv .venv_windows
.\.venv_windows\Scripts\python.exe -m pip install -r .\tools\requirements_windows.txt
.\.venv_windows\Scripts\python.exe -m serial.tools.list_ports -v
Test-NetConnection 192.168.0.100 -Port 22
```

4. 목록에서 **USB-SERIAL CH340**의 COM 번호를 찾는다. 이 PC에서 확인한 값은 **COM3**이다. 번호가 달라지면 [platformio_windows.ini](../platformio_windows.ini)의 `[windows_serial]` 아래 `port` 한 곳과 아래 명령의 포트 번호를 바꾼다. 보드 암호는 SSH나 `su`가 요청할 때 터미널에 입력한다.

## 평소 실행

**터미널 1 · AI-G 카메라.** 보드 부팅 뒤 `Setup`을 한 번 실행한다. 센서를 90 fps, 노출 300으로 설정하고 보드의 디버그 콘솔 UART를 여는 단계다. 두 번째 SSH 접속에서 보드의 root 암호를 요청할 수 있다. `Setup`이 성공하면 `Run`으로 추적을 시작한다.

```powershell
.\tools\AI-G_run_windows.cmd Setup
.\tools\AI-G_run_windows.cmd Run
```

`calibrated rod radius`에서 tip 약 139~152 px, middle 약 53~58 px인지 확인하고 이 터미널을 열어 둔다. 기본 54,000프레임은 90 fps에서 약 10분이다. 예를 들어 30분 실험은 `Run -Frames 162000`으로 시작한다. 추적기는 프레임 수에 비례해 보드 메모리를 할당하고 종료할 때 CSV를 쓰므로 실험 길이에 맞춘다. 실험 중에는 카메라 프레임 한도에 도달하기 전에 카트를 정지한다.

**터미널 2 · ESP32.** PlatformIO 시리얼 모니터를 닫는다. VS Code의 **PlatformIO → Project Tasks → camera_control_windows → Upload**를 실행한다. 펌웨어가 이미 올라가 있으면 바로 아래 명령으로 연결한다. 매 실험의 로그는 Windows 사용자 폴더 아래 `equilibrium_logs`에 저장한다.

**Project Tasks에 환경이 보이지 않을 때**는 저장소 루트의 PowerShell에서 아래 명령으로 직접 업로드한다. 시리얼 도구가 실행 중이면 `q`로 종료해 포트를 비운다. CH340의 포트가 바뀌었으면 `COM3`를 현재 번호로 바꾼다.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e camera_control_windows -t upload --upload-port COM3
```

마지막에 `SUCCESS`가 나오면 업로드가 끝난 것이다. 다음 명령으로 시리얼 도구를 연다.

```powershell
$port = 'COM3'
$logs = Join-Path $env:USERPROFILE 'equilibrium_logs'
New-Item -ItemType Directory -Force $logs | Out-Null
$log = Join-Path $logs ("cam_{0:yyyyMMdd_HHmmss_fff}.log" -f (Get-Date))
.\.venv_windows\Scripts\python.exe .\tools\pipeline_serial_windows.py $port $log
```

키는 Enter 없이 입력한다.

| 순서 | 키 | 확인 |
|---|---|---|
| 1 | `O` | 카트를 레일 가운데에 두고 카트 위치 영점 설정 |
| 2 | `Z` | 진자를 아래로 늘어뜨려 정지시킨 뒤 엔코더 영점 설정 |
| 3 | `E` | `# SH` 상태줄의 `v`가 약 90인지 확인 |
| 4 | `C` | 정지 상태에서 소스 전환. 출력에 `CAMERA`가 선택됐는지 확인 |
| 5 | `S` / `A` | 아래에서 스윙업 / 진자를 세운 뒤 밸런스 |
| 6 | `X` 또는 스페이스 | 카트 정지. 다음 스윙업은 `# REC end` 출력 뒤에 시작 |

카트를 정지한 뒤 `q`로 시리얼 도구를 종료하고, 터미널 1에서 Ctrl+C로 추적을 끝낸다. USB가 끊기면 정지 키가 전달되지 않으므로 드라이버 전원 스위치로 멈춘다. 보드 콘솔 로그인을 다시 쓰려면 ESP32 UART를 분리하고 `.\tools\AI-G_run_windows.cmd Restore`를 실행한다.

## 위치를 바꾸거나 보정할 때

1. 카트를 정지하고 `pend_cam`을 종료한 뒤 `.\tools\AI-G_run_windows.cmd Frame`을 실행한다. AI-G에서 밝은 보기 모드로 한 프레임을 찍고 Windows의 `equilibrium_logs`에 원본 RAW와 상하 반전한 PNG를 저장한다. 빨간 피벗과 파란 마커 2개가 카트 양 끝에서도 보이고, 회전 중심 주변에 약 150 px 여유가 있는지 확인한다. 카메라를 고정한 뒤 **Setup, Run을 다시 실행**해 추적용 노출을 복원한다.
2. 시리얼 도구를 닫고 PlatformIO에서 **camera_estimator_test_windows → Upload**를 실행한다. AI-G → ESP32 UART는 115200이고, 보정용 ESP32 → PC USB는 921600이다. Windows 환경이 두 속도를 각각 설정한다.
3. 아래 명령을 터미널 2에서 실행한다. 각 캡처를 시작할 때 진자를 아래로 늘어뜨린 채 Enter를 누른다. 녹화 시작 문구가 뜨면 손으로 움직인다. 첫 번째 로그는 수직 ±45°를 천천히 여러 번 지나고, 두 번째는 큰 각도까지 움직인다. 포트와 로그 폴더 변수는 위의 터미널 2 예시와 같다.

```powershell
$python = '.\.venv_windows\Scripts\python.exe'
$stamp = Get-Date -Format yyyyMMdd_HHmmss
$near = Join-Path $logs "near_$stamp.log"
$wide = Join-Path $logs "wide_$stamp.log"
& $python .\tools\camera_angle_calib_windows.py capture $port 60 $near
& $python .\tools\camera_angle_calib_windows.py capture $port 60 $wide
& $python .\tools\camera_angle_calib_windows.py fit $near $wide
```

4. 홀드아웃 결과에서 수직 ±5° RMS가 약 0.2° 이내인지, 수직 근처 표본 수가 충분한지 확인한다. 보정값을 적용하려면 같은 `fit` 명령 끝에 `--write-header src/tests/camera_angle_calib.h`를 붙여 실행한 뒤 **camera_control_windows → Upload**로 돌아온다. 이 헤더는 두 운영체제의 펌웨어가 공유한다. 카트 위치 기울기까지 바뀌었으면 [원본 문서 5절](AI-G_run.md)의 절차대로 새 로그의 `b`와 `x`로 기울기를 확인한다.

## 다른 Windows 도구

| 작업 | PlatformIO 환경 | PowerShell에서 실행할 Python 스크립트와 인자 예시 |
|---|---|---|
| 카메라 지연 캡처 / 분석 | `camera_delay_log_windows` | `camera_delay_xcorr_windows.py capture COM3 60 delay.log`, `analyze delay.log` |
| 카트 가속도 실험 / 분석 | `cart_accel_log_windows` | `cart_accel_fit_windows.py capture COM3 chirp.log`, `analyze chirp.log`. capture는 Enter 확인 후 카트를 구동한다. |
| 카트를 손으로 옮겨 위치 측정 | `cart_accel_log_windows` | `cart_accel_fit_windows.py static COM3 30 static.log`, `pxsegments static.log` |
| 엔코더 제어와 카메라 추정 비교 | `camera_pipeline_windows` | `pipeline_serial_windows.py COM3 shadow.log` |

스크립트는 모두 `tools`에 있다. 사용 예는 `& $python .\tools\camera_delay_xcorr_windows.py --help`이다. 오프라인 분석용 `camera_ekf_check.py`와 `camera_kalman_sim.py`는 같은 Windows Python으로 실행한다.

## 보드 파일 복구와 연결 문제

- 보드에 `~/cam.sh`가 없으면 Windows 터미널에서 `scp .\aig\cam_windows.sh topst@192.168.0.100:cam.sh`로 복구한 뒤 `Setup`을 다시 실행한다. 이 파일은 [카메라 노트 4절](AI-G_camera_pendulum_notes.md)의 설정을 담고 있으며 AI-G에서 실행된다.
- 보드에 `~/pend_cam`이 없으면 Windows에서 보드용 실행 파일을 전송한다. 예시는 `.\tools\AI-G_run_windows.cmd Deploy -Binary C:\build\pend_cam`이다. Windows에 AArch64 **Linux** 대상 GCC와 sysroot가 있으면 `Deploy -Compiler C:\toolchain\bin\aarch64-none-linux-gnu-gcc.exe`로 저장소의 `aig/pend_cam.c`를 빌드해 배포할 수 있다.
- `Access denied`로 COM 포트가 열리지 않으면 PlatformIO 모니터, GUI, 이전 시리얼 터미널을 닫는다. 로그 경로가 이미 존재하면 새 이름을 사용한다.
- `v0 ... B0`이면 터미널 1의 추적 프로세스, UART 권한과 배선을 확인한다. `v0 p30 c0 B420`이면 `Setup`으로 90 fps와 노출을 다시 설정한다.
