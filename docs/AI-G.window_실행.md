# AI-G.window_실행

VS Code 아래에 **PowerShell 터미널 두 개**를 엽니다. **터미널 1**은 AI-G 카메라를 켜 두는 창이고, **터미널 2**는 ESP32의 상태를 보고 카트를 조작하는 창입니다. 처음에는 카트를 움직이지 않고 **5단계에서 `v90` 근처의 숫자**가 나오는 것까지 확인하세요.

## 1. 장비를 연결하고 터미널 열기

1. PC와 AI-G를 **랜선**으로 연결합니다. 같은 IP를 쓰는 카메라 보드가 두 개라면 **IMX219가 달린 보드만** 켭니다.
2. PC와 ESP32를 **USB 케이블**로 연결합니다. Windows에서 이 장치는 `USB-SERIAL CH340`으로 표시됩니다.
3. AI-G와 ESP32 사이에 이미 연결해 둔 선을 확인합니다. 새로 배선할 때는 콘솔 USB-TTL 어댑터(CP2102)를 분리하고, AI-G 콘솔 TX의 대기 전압이 약 3.3V인지 확인한 뒤 전원을 끄고 연결합니다.

   | AI-G 디버그 콘솔 | ESP32 연결 위치 |
   |---|---|
   | TX | GPIO32 (RX) |
   | RX | GPIO33 (TX) |
   | GND | GND |

   현재 장비의 **40핀 헤더 8·10번은 쇼트 상태**이므로 사용하지 마세요. 위 표의 TX와 RX는 디버그 콘솔 단자입니다.

4. VS Code에서 **터미널 → 새 터미널**을 누릅니다. **터미널 1**에 아래 명령을 한 줄씩 붙여넣고 매번 **Enter**를 누릅니다.

   ```powershell
   cd "C:\Users\tjdgu\OneDrive\문서\GitHub\equilibrium"
   Test-NetConnection 192.168.0.100 -Port 22
   ```

   **`TcpTestSucceeded : True`**가 나오면 PC에서 AI-G에 접속할 수 있습니다. `False`이면 전원과 랜선을 확인하세요. 처음 연결하는 PC에서는 **Windows 키 + R → `ncpa.cpl` → AI-G에 연결한 이더넷 우클릭 → 속성 → 인터넷 프로토콜 버전 4(TCP/IPv4)**를 엽니다. IP 주소를 `192.168.0.1`, 서브넷 마스크를 `255.255.255.0`으로 입력하고 게이트웨이는 비워 둡니다. 확인을 누르고 위 명령을 다시 실행하세요.

## 2. 터미널 2에서 ESP32 포트 확인하고 업로드하기

터미널 오른쪽 위 **`+`**를 눌러 **터미널 2**를 엽니다. 아래 두 줄을 차례로 실행합니다.

```powershell
cd "C:\Users\tjdgu\OneDrive\문서\GitHub\equilibrium"
.\.venv_windows\Scripts\python.exe -m serial.tools.list_ports -v
```

`USB-SERIAL CH340(COM3)`처럼 표시된 번호를 확인합니다. 이 PC에서 확인한 번호는 **COM3**입니다. 다른 번호가 표시되면 아래 명령의 `COM3`를 그 번호로 바꾸세요.

**이미 `camera_control_windows`를 업로드했다면 3단계로 가세요.** 처음이거나 다른 펌웨어가 들어 있다면 카트를 정지하고 PlatformIO의 시리얼 모니터를 닫습니다. 키보드 조작 프로그램이 열려 있으면 그 창에서 `q`를 누릅니다. 그런 다음 터미널 2에 아래 한 줄을 **맨 앞의 `&`까지** 복사해 실행하세요. PlatformIO 목록에 환경 이름이 보이지 않아도 쓸 수 있습니다.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e camera_control_windows -t upload --upload-port COM3
```

마지막에 **`camera_control_windows  SUCCESS`**가 나오면 업로드가 끝난 것입니다. `Access is denied`가 나오면 COM 포트를 사용 중인 창을 닫고 다시 시도하세요.

## 3. 터미널 1에서 카메라 켜기

**터미널 1**을 클릭합니다. AI-G를 부팅할 때마다 먼저 아래 명령을 실행합니다.

```powershell
.\tools\AI-G_run_windows.cmd Setup
```

`topst@192.168.0.100's password:`에는 보드의 **topst 암호**를, 이후 `su`가 요구하는 암호에는 **root 암호**를 입력합니다. 입력하는 동안 화면에 글자가 표시되지 않아도 Enter를 누르면 됩니다. 오류 없이 `PS ...equilibrium>`이 돌아오면 다음 명령을 실행합니다.
(보통 topst 두번, 그 이후 root 한번 하면 통과 됨)


```powershell
.\tools\AI-G_run_windows.cmd Run
```

카메라에 빨간 마커 1개와 파란 마커 2개가 보이게 합니다. **`calibrated rod radius`** 줄에서 `tip`이 약 **139~152 px**, `middle`이 약 **53~58 px**인지 확인합니다. **이 창은 켜 둔 채로** 터미널 2로 이동합니다. 기본 실행 시간은 90 fps에서 약 10분이므로 이 시간이 끝나기 전에 카트를 정지하세요.

## 4. 터미널 2에서 ESP32 조작 창과 로그 열기

**아래 네 줄은 한 번에 복사해서 붙여넣으세요.** PowerShell이 위에서 아래로 실행합니다.

1. 첫 줄 `$logs =`부터 마지막 줄 끝의 `COM3 $log`까지 **네 줄 전체**를 복사합니다. 문서 편집 화면에 보이는 백틱 세 개와 `powershell` 표시는 복사 범위에서 뺍니다.
2. **터미널 2**에서 `PS ...\equilibrium>`이 보이는 입력 줄을 클릭하고 **Ctrl+V**를 누릅니다. 여러 줄 붙여넣기 확인 창이 뜨면 **붙여넣기(Paste)**를 선택합니다.
3. 마지막 명령이 입력 줄에 남아 있으면 **Enter를 한 번** 누릅니다. 이미 `O cart zero | Z encoder zero`가 보이면 실행된 상태이므로 바로 5단계로 갑니다.

```powershell
$logs = Join-Path $env:USERPROFILE 'equilibrium_logs'
New-Item -ItemType Directory -Force $logs | Out-Null
$log = Join-Path $logs ("cam_{0:yyyyMMdd_HHmmss_fff}.log" -f (Get-Date))
.\.venv_windows\Scripts\python.exe .\tools\pipeline_serial_windows.py COM3 $log
```

**`O cart zero | Z encoder zero`**로 시작하는 키 설명이 보이면 연결됐습니다. 지금부터는 터미널 2 안을 클릭하고 **영문 키를 한 번씩** 누르세요. **Enter는 필요 없습니다.**

## 5. 카트를 움직이기 전에 수신 확인하기

| 순서 | 할 일 | 터미널 2에서 확인할 내용 |
|---|---|---|
| ① | 카트를 레일 가운데에 두고 영문자 **O**를 누릅니다. | `OK: Cart position zeroed` |
| ② | 진자를 아래로 늘어뜨립니다. 완전히 멈추면 **Z**를 누릅니다. | `OK: Downward Zero Set` |
| ③ | **E**를 눌러 상태 표시를 켭니다. | `OK: shadow stats ON` 다음에 `# SH` 줄이 반복됩니다. |
| ④ | `# SH` 줄에서 **소문자 v 뒤의 숫자**를 봅니다. | `v89`, `v90`처럼 약 90이면 유효한 카메라 각도가 초당 약 90번 들어옵니다. |
| ⑤ | 카트를 정지한 상태에서 **C**를 누릅니다. | `OK: balance source = CAMERA`가 나오면 준비 완료입니다. `ENCODER`가 나오면 한 번 더 누릅니다. |

실험 중 `CAMERA CONTROL ... READY`가 다시 뜨면 ESP32가 재시작했을 수 있습니다. 다시 구동하기 전에 이 단계부터 반복하세요. 연결 확인만 할 때는 여기서 7단계로 갑니다.

### `v0 r0 p0 c0 B0`가 계속 나올 때

**`B0`는 그 1초 동안 ESP32가 AI-G에서 받은 바이트가 0이라는 뜻입니다.** 조작 창에 `# SH`가 출력되는 것으로 PC와 ESP32의 USB 연결은 확인할 수 있습니다. 아래 점검은 카트를 정지한 상태에서 진행하세요.

1. **터미널 1**을 봅니다. 이미 `PS ...equilibrium>`으로 돌아왔다면 마지막 오류나 종료 메시지를 확인하고 3단계를 다시 실행합니다.
2. 카메라가 실행 중인데도 `B0`이면 터미널 1에서 **Ctrl+C**를 한 번 누릅니다. 종료 요약의 **`frames:`**, **`UART packets sent:`**, **`write errors:`** 숫자를 확인합니다.
3. 패킷을 여러 개 보냈고 `write errors: 0`인데 같은 시간 터미널 2가 계속 `B0`였다면, 전원을 끄고 **AI-G 콘솔 TX → ESP32 GPIO32**와 **공통 GND**부터 확인합니다. 다시 켜면 3단계부터 진행합니다.
4. `B`와 `p` 뒤 숫자가 양수인데 `v0`이면 ESP32까지 패킷은 도착한 상태입니다. 터미널 1의 마커 반지름, 마커 가림, 조명과 노출을 확인합니다. 특히 `p30 c0 B420`처럼 보이면 추적을 종료한 뒤 3단계의 **Setup → Run**으로 90 fps 설정을 복원합니다.

## 6. 실제로 진자를 흔들어 올리기

카트와 막대가 움직일 공간에서 손과 케이블을 치우고 드라이버 전원 스위치 위치를 확인합니다. **터미널 2에서 `v`가 약 90이고 `CAMERA`가 선택됐을 때** 아래 키를 사용합니다.

- **진자가 아래에 있을 때**는 **S**를 한 번 누릅니다. `ARMED: Energy Swing-up started`가 나오면 스윙업이 시작됩니다. 성공하면 자동으로 밸런스 제어로 넘어갑니다.
- **이미 진자를 세운 상태에서 밸런스만 시작할 때**는 **A**를 사용합니다.
- **정지할 때**는 **X 또는 스페이스바**를 누릅니다. USB가 끊겨 정지 키가 전달되지 않으면 **드라이버 전원 스위치**로 멈춥니다. 다음 스윙업은 `# REC end`가 출력된 뒤 시작합니다.

## 7. 종료하고 로그 열기

1. **터미널 2**에서 **X**로 카트를 멈춥니다. 스윙업을 했다면 `# REC end`를 확인한 뒤 **q**를 눌러 조작 프로그램을 닫습니다.
2. **터미널 1**에서 **Ctrl+C**로 카메라 추적을 끝냅니다.
3. **터미널 2**에 아래 명령을 입력합니다. 열린 폴더에서 방금 실행한 시각의 `cam_날짜_시간_밀리초.log` 파일을 찾습니다.

```powershell
explorer.exe $logs
```

다음 실험에서 배선과 COM 번호가 같고 같은 펌웨어가 올라가 있다면 두 터미널을 프로젝트 폴더에서 열고 **3단계부터** 시작하세요. 새 PC에 Python 환경을 설치하거나 카메라 위치를 바꿔 다시 보정할 때는 [Windows 설정과 보정 안내](AI-G_run_windows.md)를 참고하세요.
