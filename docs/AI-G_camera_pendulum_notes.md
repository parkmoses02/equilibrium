# TOPST AI-G + IMX219 카메라 + 도립진자 마커 추적: 정리 노트

다른 채팅으로 이어가기 위한 요약입니다. 직접 확인한 것과 추정, 아직 안 해 본 것을 구분해서 적었습니다.

## 1. 목표와 현재 상태

- 목표: AI-G 보드에서 IMX219 카메라로 도립진자(Inverted pendulum)의 마커 위치와 막대 각도를 추출하고, UART로 ESP32에 보낸다. 모터 제어, 칼만 필터는 ESP32가 한다. 기존에는 엔코더로 각도를 받았고, 이번에는 일부러 엔코더 대신 카메라를 쓴다. 카트 위치도 카메라(빨간 마커)에서 구한다.
- 이 노트 시점까지 **확인된 것**: 카메라 인식, 640x480 센서 모드(출력 632x464 UYVY) 90fps 안정 캡처, 보드 안에서 마커 검출과 각도 계산(프레임당 평균 7.56ms), 8, 10번 핀 UART 루프백.
- **아직 안 한 것**: ESP32와 실제 UART 통신, 엔코더와의 지연 비교, ESP32 쪽 코드, AI-G GPIO 전압 확인.

## 2. 하드웨어와 소프트웨어 환경

- 보드: TOPST AI-G (Telechips TCC750x), Yocto kirkstone 4.0.17, 커널 5.10.223-tcc
- 펌웨어는 직접 다시 빌드해서 플래싱했다: meta-topst release/1.3.1 + meta-topst-bsp 1.3.0-r01 + 커널 release/ai/1.1.0, `TOPST_CAM_MODULE="imx219"`, 공식 tccvin2 sizeimage 버그 패치 적용 (PC의 `~/topst/`에서 빌드, 결과물 백업 `~/backup/output_aig-imx219-20261120.fwdn.zip`)
- 카메라: IMX219, CSI 커넥터 1개. 장치: `/dev/video2`(tccvin), `/dev/media0`, `/dev/v4l-subdev0`(tcc-isp-0), `/dev/v4l-subdev1`(mipi_csi2_0), `/dev/v4l-subdev2`(imx219 3-0010)
- 계정: `topst`. `sudo`는 설정되어 있지 않고 `su -`로 root가 된다.
- dmesg에 `tcc-isp-fw` request_firmware error(-2)가 매번 나오지만 지금까지 영상에는 문제가 없었다(영향은 확인하지 못함).

## 3. 보드 연결 방법

### 3.1 물리 연결
1. PC와 보드를 랜선으로 직결 (PC의 랜 포트가 `enp1s0`)
2. USB-C 전원 연결
3. 시리얼 콘솔용 USB-TTL을 PC에 연결 (선택, 부팅 로그를 볼 때)

### 3.2 PC 네트워크 설정 (처음 한 번만 만들고 이후에는 `up`만)
```sh
nmcli con show | grep topst || sudo nmcli con add type ethernet ifname enp1s0 con-name topst \
  ipv4.method manual ipv4.addresses 192.168.0.1/24 ipv4.never-default yes ipv6.method ignore
nmcli con up topst
ip -br a show enp1s0        # UP, 192.168.0.1/24 가 나와야 함
```
`enp1s0`이 DOWN이면 랜선이 헐거운 경우가 많았다.

### 3.3 SSH 접속
```sh
ping -c 2 192.168.0.100
ssh topst@192.168.0.100
```
- 보드 IP는 `192.168.0.100`
- 나갈 때는 `exit`(root 상태면 두 번), 또는 Ctrl+D

### 3.4 시리얼 콘솔 (필요할 때만)
```sh
sudo usermod -aG dialout $USER     # 처음 한 번, 재로그인 필요 (Permission denied가 나올 때)
picocom -b 115200 /dev/ttyUSB0     # 종료: Ctrl+A, Ctrl+X
```
- 이 콘솔은 보드 쪽 `/dev/ttyAMA2`에 해당한다. **`ttyAMA2`에는 절대 `stty`나 `echo`를 하지 말 것.** 터미널이 깨진다 (복구: `stty sane`).

### 3.5 종료
`su -` 후 `/sbin/shutdown` (예: `shutdown -h now`)을 실행한 뒤 전원을 뽑는다.

## 4. 카메라 초기 설정 (보드에서)

`~/cam.sh`는 보드에 이미 있고, 재플래싱하지 않으면 재부팅해도 남아 있다.

```sh
#!/bin/sh
media-ctl -d /dev/media0 -V '"imx219 3-0010":0 [fmt:SBGGR10_1X10/640x480]'
media-ctl -d /dev/media0 -V '"tcc-isp-0":0 [fmt:SBGGR10_1X10/640x480]'
case "$1" in
  view) v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl vertical_blanking=10100,exposure=8000,analogue_gain=100,digital_gain=256 ;;
  60)   v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl vertical_blanking=402,exposure=860,analogue_gain=200,digital_gain=512 ;;
  90)   v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl vertical_blanking=108,exposure=580,analogue_gain=232,digital_gain=512 ;;
  *)    echo "usage: $0 view|60|90"; exit 1 ;;
esac
v4l2-ctl -d /dev/video2 --set-fmt-video width=632,height=464,pixelformat=UYVY
```

현재 쓰는 설정:
```sh
sh ~/cam.sh 90
v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl exposure=300
v4l2-ctl -d /dev/v4l-subdev2 --get-ctrl vertical_blanking,exposure,analogue_gain,digital_gain   # 확인
```
(`cam.sh` 실행 시 출력 끝이 `gain,digital_gain`처럼 잘려 보이는 건 터미널 표시 문제이고, `--get-ctrl`로 값이 맞게 들어간 걸 확인했다.)

### 알아 둘 사실
- 출력 포맷은 **UYVY**만 쓴다. RGB3로 캡처하면 R=G=0인 쓰레기 이미지가 나왔다.
- ISP가 출력을 사방 (4, 8)px씩 잘라서 640x480 → 632x464가 된다.
- fps = 182.4M ÷ (3448 × (높이 + vertical_blanking)). 640x480에서 측정: vb 1283/402/108/32 → 30.01/59.98/89.97/103.32 fps. 90fps는 900프레임에서 프레임 드롭 0이었다.
- 다른 모드: 1640x1232 vb=32 → 41.85fps, 1920x1080 vb=32 → 47.57fps.
- 120fps는 센서 클럭(pixel_rate) 변경이 필요해서 커널 재빌드와 재플래싱이 필요하다. 아직 하지 않았다(미루기로 함).
- 자동 노출이 없다(`isp-server`가 돌고 있지 않다). 기본값은 어두워서 gain과 노출을 직접 정한다. analogue_gain 0~232, digital_gain 256~4095, exposure 최대는 대략 프레임 길이(줄 수).
- **노출 시간**: 한 줄 18.9µs이므로 exposure=300줄 ≈ 5.7ms, 580줄 ≈ 11ms. (센서 읽기는 480줄 ≈ 9ms로 추정, rolling shutter)
- 영상이 **상하로 뒤집혀** 있다(좌우는 그대로인 거울상). 추적 코드에서 `--vflip`으로 보정한다.

### 녹화해서 PC로 가져오기 (분석용)
```sh
# 보드: /dev/shm이 100MB라 632x464 UYVY는 120프레임이 한계
rm -f /dev/shm/e300.raw
v4l2-ctl -d /dev/video2 --stream-mmap --stream-count=120 --stream-to=/dev/shm/e300.raw
ls -l /dev/shm/e300.raw      # 70,379,520 바이트면 정상
```
```sh
# PC: 90fps를 30fps로 재생하는 3배 슬로모션 mp4
scp topst@192.168.0.100:/dev/shm/e300.raw ~/
ffmpeg -f rawvideo -pix_fmt uyvy422 -video_size 632x464 -framerate 30 -i ~/e300.raw \
  -c:v libx264 -crf 18 -pix_fmt yuv420p ~/e300_slow3x.mp4
```

## 5. 마커 추적 프로그램 (`pend_cam.c`)

- 마커: **빨간 마커 = 회전 중심(카트)**, **파란 마커 2개 = 막대 가운데와 끝**
- 빌드와 배포 (보드에 gcc가 없어서 PC에서 크로스 컴파일):
```sh
sudo apt install gcc-aarch64-linux-gnu
aarch64-linux-gnu-gcc -O2 -static -o pend_cam pend_cam.c -lm
scp pend_cam topst@192.168.0.100:~/
```
- 실행 (카메라 설정 후):
```sh
./pend_cam --vflip --frames 900 --out /dev/shm/track.csv                      # CSV만 기록
./pend_cam --vflip --uart /dev/ttyAMA1 --baud 460800 --frames 100000          # ESP32로 전송
```
- 방식: UYVY의 U, V 값을 바로 임계값에 적용(RGB 변환 없음) → 연결 성분으로 마커 덩어리 검출 → 막대 반지름(끝 약 148px, 가운데 약 60px)을 처음 30개 정상 프레임에서 스스로 잰 뒤 그 값으로 끝/가운데 구분 → 각도 계산, 이상값 제거(앞 프레임과 45° 초과 변화는 버림)
- 각도 기준: 0° = 막대가 위로 섬(`--vflip` 적용 후), 오른쪽으로 기울면 +, 범위 -180~180(CSV에는 이어 붙인 값 `angle_unwrapped_deg`도 있음)
- 색 임계값(YUV)은 한 번의 조명에서 정한 값이다. 조명이 달라지면 `--red V최소,U최대,차이`, `--blue U최소,V최대,차이`로 조정한다. 배경의 파란 점은 `--ignore X,Y,R`로 제외한다.
- CSV 열: frame, seq, t_s, pivot_x/y, tip_x/y/n, mid_x/y/n, angle_tip/mid/angle/unwrapped(deg), proc_ms, lat_ms, flags
- PC에서 그래프: `python3 plot_track.py track.csv` → `track.png` (numpy, matplotlib 필요). 보드에서 `scp topst@192.168.0.100:/dev/shm/track.csv .`로 가져온다.

### 보드에서 측정한 성능 (막대를 움직이며 1회 실행, 120프레임)
- 캡처 89.8fps, 놓친 프레임 0, 끝/가운데 마커 모두 120/120 검출
- 끝과 가운데로 각각 잰 각도 차이: 중앙값 0.59°, 상위 5% 1.78° (빠르게 지날 때 최대 2.4° 안팎)
- 카트 중심 좌표 흔들림: 정지 시 x, y 모두 표준편차 약 0.5~0.7px
- **처리 시간: 프레임당 평균 7.56ms (첫 프레임 제외 7.51~7.65ms), 최대 9.0ms** / 프레임 주기 11.1ms의 약 68%
- 이 처리 시간에는 노출, 센서 읽기, ISP 시간이 들어 있지 않다. 카메라 전체 지연은 아직 모른다 (ESP32 엔코더와 비교해서 구할 계획).
- 이 `/dev/video2`는 multi-planar로 인식됐고 직접 캡처 코드가 그대로 동작했다.

### 처리 시간을 줄이는 방법 (아직 안 함, 아이디어만)
마커 주변 영역만 검색(ROI), NEON 벡터화와 컴파일 옵션, 프레임 읽는 메모리(비캐시 영역일 가능성)가 느린지 `--dump`로 저장한 프레임을 `--file`로 돌려 비교.

## 6. ESP32와 UART 연결

### 6.1 핀 (AI-G 40핀 헤더 JH1)
| AI-G 헤더 핀 | 신호 | ESP32 연결 |
|---|---|---|
| 8번 | GPIO 88, UART TX | ESP32 GPIO32 (ESP32에서 **RX**로 설정) |
| 10번 | GPIO 89, UART RX | ESP32 GPIO33 (ESP32에서 **TX**로 설정) |
| 6, 9, 14, 20, 25, 30, 34, 39번 | GND | ESP32 GND |

- 사용자가 "esp 32번/33번"이라고 적은 것을 ESP32의 **GPIO 번호**로 해석했다. 보드 핀 위치 번호라면 확인이 필요하다.
- 헤더의 2, 4번은 **5V**이다. ESP32의 GPIO나 3.3V 입력에 닿지 않게 한다.
- **AI-G GPIO의 I/O 전압은 아직 확인하지 못했다.** 라즈베리파이 HAT 호환 헤더라 3.3V일 가능성이 높지만, AI-G Hardware-User Guide에서 확인할 것. ESP32는 3.3V이다.

### 6.2 리눅스 장치와 시험 결과
- 8, 10번 핀 UART = **`/dev/ttyAMA1`** (root:dialout). 8번과 10번을 점퍼선으로 이어서 루프백 시험을 했고, 115200bps에서 `hello`가 되돌아왔다 (보내고 받는 것 모두 동작).
- `/dev/ttyAMA2`는 커널 콘솔이니 건드리지 않는다.
- `topst` 계정으로 쓰려면 root로 `usermod -aG dialout topst`(또는 `addgroup topst dialout`) 후 재로그인. 급하면 `chmod 666 /dev/ttyAMA1` (재부팅하면 풀림). 이 명령들이 이 이미지에서 되는지는 확인하지 못했다.

### 6.3 패킷 형식 (14바이트, little-endian, 8N1, 흐름 제어 없음)
| 위치 | 내용 |
|---|---|
| 0, 1 | 0xAA, 0x55 |
| 2~3 | 프레임 번호 u16 (V4L2 시퀀스, 넘치면 처음부터) |
| 4~5 | 각도 i16, 0.01° 단위, 0x8000이면 무효 |
| 6~7 | 카트 위치 x, u16, 0.1px 단위, 0xFFFF이면 무효 |
| 8~9 | 카트 위치 y, u16, 0.1px 단위 |
| 10 | 플래그: b0 각도 유효, b1 끝 마커 사용, b2 가운데 마커 사용, b3 카트 위치 유효, b4 막대 길이 보정 중, b5 age 유효 |
| 11~12 | age u16, 0.1ms 단위: 드라이버 프레임 시각부터 전송 직전까지의 시간 |
| 13 | CRC-8 (다항식 0x07, 초기값 0, 2~12번 바이트) |

- 기본 460800bps에서 패킷 하나가 약 0.3ms. 이 서버에서 가상 시리얼로 120패킷 검증했고 CRC 오류는 없었다. **실제 UART와 ESP32 수신은 아직 시험하지 않았다.**
- ESP32 수신 예시 코드는 `Serial2.begin(460800, SERIAL_8N1, 32 /*RX*/, 33 /*TX*/)` 형태로 만들 수 있고, 헤더 0xAA 0x55 동기 → 14바이트 수신 → CRC 확인 순서로 파싱한다. 수신 시각을 `micros()`로 같이 기록하면 지연 비교에 쓸 수 있다.
- ±180° 경계에서 각도가 뛰니 ESP32에서 이어 붙여야 한다.

## 7. 설계 결정과 앞으로 할 일

- 칼만 필터와 속도 추정은 **ESP32에서** 한다. 모터 명령을 입력으로 쓸 수 있고, 늦게 도착한 카메라 값에 맞춰 상태를 앞으로 굴리는 지연 보정이 필요하기 때문이다. 카트 위치, 속도, 각도, 각속도를 함께 추정하는 4상태 필터를 가정하고 있다.
- 카메라 지연은 ESP32의 엔코더 값과 카메라 값을 같은 시계로 기록해서 시간축으로 밀며 상호상관으로 구할 계획이다. 먼저 부호, 0점, 배율을 맞추고, 막대를 빠르게 움직인 구간을 쓰고, 실제 쓸 노출과 설정에서 잰다.
- 카메라 각도는 영상의 수직선 기준이라, 막대를 수직으로 세운 상태를 0°로 보정하는 단계가 필요하다.
- 카트 위치를 길이로 바꾸려면 막대 길이나 레일 길이 같은 실제 길이로 px/mm 보정이 필요하다. 이동 범위가 화면에 다 들어오는지도 확인이 필요하다(화면 폭 632px).
- 스윙업처럼 빠른 움직임에서는 마커 번짐이 커진다. 노출을 150~200줄 정도로 더 낮춰 비교해 보기(값은 추정, 낮추면 어두워지니 조명이나 digital_gain 필요).
- 촬영 구도: 막대가 위로 올라올 때 프레임 위쪽에서 잘리지 않아야 하고, 막대가 오른쪽으로 넘어갈 때 끝 마커가 화면 오른쪽 가장자리에 아주 가깝다(x≈603/632).

### 미해결, 확인 필요
1. AI-G GPIO 전압 (3.3V 여부)
2. ESP32와 실제 UART 통신 시험
3. 카메라 전체 지연 (엔코더 비교)
4. 막대 길이, 레일 길이 (카트 위치 보정과 지연 허용 판단에 필요)
5. 120fps 재빌드 (센서 클럭 변경), 필요할 때만
6. 이미지 단위 잔여 항목: PC의 `tar` 패키지가 `hold` 상태이니 재빌드가 끝나면 `sudo apt-mark unhold tar && sudo apt install tar`. 새 이미지에는 플래싱 가이드 6단계(NPU, sudo)를 다시 적용하지 않았다.
7. ISP 튜닝이 안 된 것으로 보이는 윤곽선 흰 테두리(헤일로)가 영상에 있다. 설정으로 해결되는지는 확인하지 못했다.
