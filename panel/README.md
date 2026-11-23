# Pendulum Panel (PC)

Lightweight Python panel to interact with the ESP pendulum firmware.

Requirements
- Python 3.8+
- Install packages from `requirements.txt`:

```bash
python -m pip install -r panel/requirements.txt
```

Run (recommended)

```bash
python -m panel.panel
```

Or run directly (may require Python path adjustments):

```bash
python panel/panel.py
```

Usage notes
- Set the correct serial port in the GUI (e.g. COM3) and click Connect.
- The panel listens for packets in the form `0xAA addr float32` (device->PC).
- To send commands the panel writes 5-byte packets `addr + float32` (PC->device).

This is a minimal starter panel. Extend controls to send specific parameters (addresses are in the source comments).

## Rev.L 이중진자 실시간 표시 (읽기 전용)

1. PlatformIO `double_upright`에서 Upload and Monitor를 실행한다. Monitor가 출력하는 로그 경로를 확인한다.
2. 저장소 루트의 별도 터미널에서 `python panel/live_pose.py`를 실행한다 (Tkinter 필요). 자동 선택이 안 되면 로그 경로를 인자로 지정한다. Monitor를 다시 시작해 로그가 바뀌면 뷰어도 다시 실행한다.
3. 기존 제어 명령은 **PlatformIO Monitor에서만** 입력한다. 뷰어는 COM 포트를 열지 않는다. 동작 중 약 20 Hz로 실제 두 절대각과 램프 기반 카트 **추정** 위치를 그린다. 정지/통신 중단 시 마지막 자세를 고정한다.

## DD에서 UU로 스윙업

일반 `double_upright`의 모니터에서 카트를 중앙에 놓고 두 링크를 아래로 늘어뜨린 후 `z`를 입력한다. 두 링크를 정지시킨 상태에서 `rq`를 한 줄로 보내면 DD→UU 궤적을 한 번 실행한다. `x`로 정지할 수 있다. `re`는 DD→DU 명령이다.

744개 명령 프레임(각 5 ms) 직후 한 번 캐치를 검사한다. θ는 두 링크의 수직 위쪽 기준 각도 오차 절댓값 중 큰 값, ω는 두 필터링된 각속도 절댓값 중 큰 값, v는 카트 램프 속도의 절댓값이다. 다음 두 경우 중 하나이면 캐치한다.

- 기본 범위는 θ ≤ 0.05 rad, ω < 0.50 rad/s, v ≤ 0.20 m/s이다.
- 확장 범위는 두 링크의 각도 차이 ≤ 0.01 rad, ω ≤ 0.05 rad/s이고 `θ / 0.10 + v / 0.30 ≤ 1`인 경우다.

카트 위치 판정은 `re`, `rq`, `p` 모두 마지막 `z`를 기준으로 한다. 좌우 어느 쪽이든 300 mm를 초과하면 시작을 차단하거나 동작 중 정지한다. 스윙과 유지 제어는 진자의 각도·각속도와 카트 속도를 사용한다. 위치는 TMC5160의 램프 카운터로 추정하므로 손으로 카트를 옮기거나 탈조가 생기면 카트를 중앙에 놓고 두 링크를 늘어뜨린 뒤 `z`를 다시 입력한다.

UU 캐치에 성공하면 `CATCH UU -> q`와 함께 UU 유지 제어로 전환한다. 캐치 실패 시 측정값과 `OFF: missed UU; no retry`를 출력한다. 유지 중 어느 한 링크의 오차가 0.17 rad를 넘으면 정지한다.

오프라인 검증은 저장소 루트에서 `node simulation/dd-uu/verify.js`로 실행한다. UU 궤적을 다시 설계하는 스크립트는 `simulation/dd-uu/design.py`에 있다. 일반 보드 빌드에는 최적화 도구가 필요하지 않다.

### 엔코더 오류가 한 번 발생한 경우

스윙과 유지 중 HEDS 한 채널에서 A/B 두 비트가 동시에 바뀐 것으로 관측되면 해당 관절의 직전 각도와 각속도로 한 제어 주기(통상 5 ms)의 각도를 추정한다. 다른 채널은 현재 측정값을 사용한다. 정지해 카운트가 같은 프레임은 정상이다. CSV의 `estimated_heds`는 0이면 두 채널 모두 측정값, 1 또는 2이면 해당 HEDS 채널의 추정값을 사용했다는 뜻이다.

한 프레임에 여러 오류가 있거나 직전 오류 후 50 ms 이내에 다시 발생하면 정지한다. 캐치 프레임에 단발 오류가 있으면 다음 정상 프레임에서 캐치를 판정한다. 원시 카운트에는 누락이 남을 수 있어, 오류가 있었던 회차가 끝나면 카트를 중앙에 두고 두 링크를 늘어뜨린 뒤 `z`로 다시 영점을 잡는다.

## 이중진자 실험 재생

저장소 루트에서 `python -m panel.replay`를 실행하거나 대시보드의 **실험 재생** 탭을 연다. 최근 기록이 열리며 **로그 열기…**로 다른 파일을 고를 수 있다. 회차를 선택하고 **재생**을 누른다. 시간 막대와 배속으로 원하는 구간을 살펴볼 수 있다.

두 링크는 위쪽을 0°로 한 절대각으로 그린다. 카트 위치는 드라이버 램프 추정값이다. 저장된 스윙 CSV와 시리얼 자세 기록을 사용하며, 표본 사이에는 직전 자세를 표시한다. 특정 파일을 열려면 `python -m panel.replay "logs/파일명.log"`를 사용한다.

## DU 이중진자 LQR

`python -m panel.panel`에서 **DU 이중진자 LQR** 탭을 연다. 실시간 자세 뷰어에서도 같은 이름의 버튼으로 열 수 있다. 계산기만 실행하려면 저장소 루트에서 `python -m panel.du_lqr`를 실행한다.

Q의 여섯 대각 성분과 가속도 입력 가중치 R을 편집하면 각 상태에 대응하는 이득 K가 즉시 갱신된다. 기본값은 펌웨어의 DU 이득 `K_e`를 재현한다. 계산은 PC에서 수행하며 보드 연결 없이 사용할 수 있다.

## 실험 후 외란 분석

성공 회차의 `RE_CSV_BEGIN`부터 `RE_CSV_END`까지 한 블록을 텍스트로 저장한 뒤
`python simulation/dd-du/inverse_residual.py run.txt` (NumPy, SciPy)를 실행한다.
같은 기준 궤적 헤더를 사용해야 한다. 결과는 램프 가속도를 실측 대신 사용한 **등가 토크 잔차**이며,
관절 마찰의 독립 측정값이나 포착 이후 유지 구간 분석이 아니다.
