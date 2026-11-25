# AI-G 카메라 실행

Windows PC에서는 [Windows용 실행 방법](AI-G_run_windows.md)을 따른다. 아래 PC 명령은 Linux용이다.

## 1. 연결

**PC ↔ AI-G (이더넷)**
- PC `enp1s0` = `192.168.0.1/24`, 보드 = `192.168.0.100`. IMX219 보드만 켠다(OV5647 보드와 IP가 같다).
- 접속: `ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null topst@192.168.0.100`
- root는 `su -`.

**AI-G ↔ ESP32 (UART, 3.3V)**

| AI-G | ESP32 |
|---|---|
| TX | GPIO32 (RX) |
| RX | GPIO33 (TX) |
| GND | GND |

- 2026-11-25부터 **디버그 콘솔 UART(`/dev/ttyAMA2`, 115200)** 를 쓴다. 40핀 헤더 8, 10번(`ttyAMA1`)이 GND로 쇼트돼서 못 쓴다.
- 연결 전에 콘솔 TX 핀 idle 전압이 약 3.3V인지 잰다. 콘솔 USB-TTL(CP2102)은 뺀다.
- 40핀(8번 TX, 10번 RX)을 쓸 수 있게 되면 `--uart /dev/ttyAMA1 --baud 460800`, 펌웨어 `SH_UART_BAUD = 460800`.

## 2. 부팅 후 설정 (재부팅할 때마다)

AI-G에서:

```sh
sh ~/cam.sh 90
v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl exposure=300
v4l2-ctl -d /dev/v4l-subdev2 --get-ctrl vertical_blanking,exposure,analogue_gain,digital_gain
```

root에서 (`su -`, 끝나면 `exit`):

```sh
systemctl stop serial-getty@ttyAMA2.service
dmesg -n 1
chmod 666 /dev/ttyAMA2
```

- `cam.sh`가 없으면(재플래시) 새로 만들어야 한다.
- 되돌리기: `systemctl start serial-getty@ttyAMA2.service`.
- `pend_cam`이 보드에 없으면: `aarch64-linux-gnu-gcc -O2 -static -o pend_cam pend_cam.c -lm` 후 `scp pend_cam topst@192.168.0.100:~/`.

## 3. 실행

**AI-G** (`topst` 셸, 종료 Ctrl+C):

```sh
./pend_cam --vflip --uart /dev/ttyAMA2 --baud 115200 --frames 10000000
```

- 시작 줄 `calibrated rod radius`가 tip 약 139~152, middle 약 53~58이어야 정상.
- `--frames`는 90 fps에서 100만이 약 3시간. 넘으면 프로세스가 끝난다.
- 마커 색 조정: `--red V최소,U최대,차이`, `--blue U최소,V최대,차이`, 배경 파란 점 제외 `--ignore X,Y,R`.

**ESP32** (PC): VS Code에서 env `camera_control` 빌드/업로드(시리얼 모니터는 닫는다), 그다음

```bash
python3 tools/pipeline_serial.py /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 ~/equilibrium_logs/<이름>.log
```

- 키: `O` 카트 위치 영점(레일 가운데), `Z` 엔코더 영점(진자 늘어뜨림), `E` 상태줄, `C` 소스 전환(CAMERA), `S` 스윙업, `A` 밸런스, `X`/스페이스 정지, `q` 종료.
- 스윙업 뒤 `# REC end`가 나올 때까지 기다린 다음 다음 `S`.
- 로그는 시도마다 새 이름으로 저장한다.

## 4. 확인과 문제

`E`를 누른 뒤 `# SH` 줄의 `v`가 약 90이면 정상.

| 증상 | 원인 |
|---|---|
| `v0 ... B0` | 바이트가 안 온다: 배선, `pend_cam` 미실행, 권한 |
| `v0 p30 c0 B420` | 패킷은 오는데 30 fps에 각도 무효: `cam.sh 90`/노출 설정이 풀림 |
| `v0`, `c`가 큼 | TX/RX 바뀜, GND 불량 |
| `rod radius`가 이상함(예: 383/354) | 마커를 잘못 잡음: 조명, 시야, 배경 물체 |
| `Camera data lost` | 카메라가 끊김: `pend_cam` 종료, 마커 가림 |

## 5. 카메라 위치를 옮긴 뒤 보정

카메라나 진자 위치가 바뀌면 아래 값이 모두 틀어진다: 각도 보정 표 `src/tests/camera_angle_calib.h`, 카트 위치 보정 기울기 `camXSlopeDegPerM`(기본 -6.0°/m). 이전 로그와 섞지 않는다.

**1) 카메라 화면을 PC로 보내서 대략 위치 맞추기**

AI-G에서 밝은 확인용 설정으로 바꾸고 1프레임을 저장한다(`pend_cam`은 끈다):

```sh
sh ~/cam.sh view
rm -f /dev/shm/frame.raw
v4l2-ctl -d /dev/video2 --stream-mmap --stream-count=1 --stream-to=/dev/shm/frame.raw
```

PC에서 가져와 그림으로 바꾼다(`vflip`은 `pend_cam --vflip`과 같은 방향으로 보려는 것):

```bash
scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null topst@192.168.0.100:/dev/shm/frame.raw ~/frame.raw
ffmpeg -y -f rawvideo -pix_fmt uyvy422 -video_size 632x464 -i ~/frame.raw -vf vflip -frames:v 1 ~/frame.png
```

카메라를 조금 움직이고 위 세 단계를 반복하며 맞춘다. 1프레임 캡처는 `docs/AI-G_camera_pendulum_notes.md`의 녹화 명령에서 `--stream-count`만 1로 줄인 것이고 아직 시험하지 않았다.

볼 것:
- 빨간 마커(카트)와 파란 마커 2개가 모두 보이는가.
- 막대 끝 반지름이 약 150 px이므로 회전 중심에서 화면 가장자리까지 150 px 이상 남는가(화면 632x464).
- 카트를 레일 양 끝으로 손으로 밀어도 마커가 화면 안에 있는가.

위치가 정해지면 카메라를 단단히 고정한다. 보정 중에 움직이면 모두 다시 해야 한다.

**2) 설정과 `pend_cam` 확인**

`sh ~/cam.sh 90`, 노출 설정 후 `pend_cam`을 켠다(3절). `calibrated rod radius`가 tip 약 139~152, middle 약 53~58이면 마커를 제대로 잡은 것이다. 색이 안 잡히면 `--red`, `--blue`, `--ignore`로 조정한다.

**3) 각도 보정 표 다시 만들기**

- `src/tests/camera_estimator_test.cpp`의 `UART_BAUD`(기본 460800)를 `pend_cam --baud`와 같게 맞춘다(지금은 115200). env `camera_estimator_test`를 올린다.
- `pend_cam`을 켠 채로, 진자를 늘어뜨려 가만히 둔 상태에서 캡처한다:

```bash
python3 tools/camera_angle_calib.py capture /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 <초> ~/equilibrium_logs/<이름>.log
```

- Enter 뒤 손으로 진자를 모든 각도로 움직인다. 수직을 천천히 자주 지나가게 하고 마커를 가리지 않는다. 로그를 2개 만든다(수직 ±45° 왕복, 큰 스윙).
- 표를 만든다:

```bash
python3 tools/camera_angle_calib.py fit ~/equilibrium_logs/<로그1>.log ~/equilibrium_logs/<로그2>.log --write-header src/tests/camera_angle_calib.h
```

- 출력의 홀드아웃 결과에서 수직 ±5° rms가 약 0.2° 안쪽이면 쓸 만하다. 표본이 없는 칸은 이웃 값으로 채워지므로 수직 근처 칸의 표본 수를 본다.
- `camera_control`을 다시 빌드해 올린다.

**4) 카트 위치 보정 기울기 확인**

- `camera_control`에서 카트를 레일 가운데에 두고 `O`, `Z`, `E`, `C` 후 `A`(또는 `S`)로 밸런스를 1~3분 유지하는 로그를 저장한다.
- `# SH` 줄의 `b`(EKF - 엔코더 편향, 보정 전 값)와 `x`(카트 위치)를 회귀해 기울기(°/m)를 구한다. 이전 값은 약 +6.0°/m였고, 펌웨어 보정은 그 반대 부호(-6.0)다.
- 값이 다르면 `camXSlopeDegPerM` 기본값을 바꾸거나 정지 중에 키 `.` / `,`(±0.5)로 맞춘다.
