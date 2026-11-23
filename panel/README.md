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

## 실험 후 외란 분석

성공 회차의 `RE_CSV_BEGIN`부터 `RE_CSV_END`까지 한 블록을 텍스트로 저장한 뒤
`python simulation/dd-du/inverse_residual.py run.txt` (NumPy, SciPy)를 실행한다.
같은 기준 궤적 헤더를 사용해야 한다. 결과는 램프 가속도를 실측 대신 사용한 **등가 토크 잔차**이며,
관절 마찰의 독립 측정값이나 포착 이후 유지 구간 분석이 아니다.
