# AI-G 영상 -> 노트북 인식 보기 (임시 작업물, 커밋 안 함)

제어 경로(AI-G -> UART -> ESP32)는 그대로 두고, 같은 영상을 노트북으로도 보내 마커 검출과 추적을 노트북에서 계산해 보여주는 보기 전용 도구.

## 파일
| 파일 | 역할 |
|---|---|
| `aig/pend_cam.c` | cam 브랜치 `aig/pend_cam.c` + `--stream`, `--stream-div` 옵션 (변경분은 `aig/pend_cam_stream.patch`) |
| `camera_detect.py` | `pend_cam.c` 검출/추적의 Python 포팅 + TCP 수신 스레드 |
| `camera_view.py` | 시각화 창. 오프라인 시뮬 / ESP32 텔레메트리 / **AI-G 영상(노트북 인식)** 중 영상이 오면 자동으로 그쪽을 씀 |
| `tools/fake_aig_stream.py` | 보드 없이 시험하는 가짜 스트림 서버 |
| `panel/` | cam 브랜치 panel 복사본 (시뮬레이션 재사용) |

## 보드 (AI-G)
1. 빌드와 배포 (PC에서 크로스 컴파일, `-pthread` 필요):
   ```
   aarch64-linux-gnu-gcc -O2 -static -pthread -o pend_cam pend_cam.c -lm
   scp pend_cam topst@192.168.0.100:~/pend_cam_stream
   ```
   기존 `~/pend_cam` 은 덮어쓰지 말고 다른 이름으로 올려 둔다.
2. 실행 (기존 인자 + 스트림):
   ```
   ./pend_cam_stream --vflip --uart /dev/ttyAMA2 --baud 115200 --frames 162000 --stream 5000 --stream-div 6
   ```
   - `--stream-div N`: N번째 프레임마다 전송. 6 = 15 fps(약 70 Mbps), 3 = 30 fps(약 140 Mbps), 1 = 90 fps(약 420 Mbps).
   - 노트북이 연결돼 있을 때만 프레임을 복사한다. 연결이 없으면 기존과 동일하게 동작한다.
3. 종료할 때 나오는 요약의 `stream: ... frame copy mean X ms` 를 본다. 프레임 복사가 느리면(처리 7.5 ms에 더해 11.1 ms 예산을 넘으면) `--stream-div` 를 키운다.

## 노트북
```powershell
python camera_view.py
```
왼쪽 1번 그룹의 `AI-G` 칸에 `192.168.0.100`, 포트 `5000` 을 넣고 `Connect AI-G video`.
처음 30개의 깨끗한 프레임으로 막대 반지름을 스스로 잰다(`rod R 148/60px` 이면 정상). 임계값은 1b 그룹에서 바꾼다 (`pend_cam --red/--blue` 와 같은 값으로 맞출 것).

## 보드 없이 시험
```powershell
python tools/fake_aig_stream.py 5055
python camera_view.py     # AI-G 칸을 127.0.0.1 / 5055 로
```

## 확인된 것과 아닌 것
- 확인: 가짜 서버 -> 노트북 수신 -> 검출 -> 추적 -> 화면 전체 경로 (반지름 147.9/59.9 px, 각도 오차 0.1° 이내).
- 확인 못 함: `pend_cam.c` 의 실제 AI-G 빌드와 실행 (Linux 헤더 스텁으로 문법만 검사). 실제 이더넷 전송량, 프레임 복사 시간, 제어 경로에 미치는 영향.
- 처음 시험은 모터 없이(밸런스를 걸지 않고) 스트림만 켜서 `pend_cam` 요약의 dropped frames 와 처리 시간이 기존과 같은지 비교할 것.
