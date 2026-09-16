"""upright_balance_test.cpp용 시리얼 모니터 + 세션 로그 저장 스크립트.

기존 PlatformIO 시리얼 모니터를 대신해서 이 스크립트를 실행하면:
  - 명령 키(z/a/s/m/t/x/p/h)는 Windows 에선 누르는 즉시(Enter 없이) 보드로 전달되고,
  - 보드가 출력하는 모든 줄이 콘솔에 그대로 표시되며,
  - "ARMED: ..." 부터 "DISARMED: ..." 까지 한 번의 밸런싱 시도(세션) 로그가
    src/tests/logs/ 에 자동으로 저장된다.

로그는 누적되지 않고 최신 3개 세션만 남도록 오래된 파일부터 자동 삭제된다.

사용법:
    python src/tests/monitor_and_log.py --port COM4
    (포트를 안 주면 COM4가 기본값)
"""
import argparse
import threading
import time
from pathlib import Path

import serial

LOG_DIR = Path(__file__).parent / "logs"
MAX_LOGS = 3


def prune_old_logs():
    logs = sorted(LOG_DIR.glob("run_*.log"), key=lambda p: p.stat().st_mtime)
    while len(logs) > MAX_LOGS:
        logs.pop(0).unlink(missing_ok=True)


def next_log_path():
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return LOG_DIR / f"run_{stamp}.log"


def reader_thread(ser: serial.Serial):
    session_lines = None
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").rstrip()
        print(line)

        if line.startswith("ARMED:"):
            session_lines = [line]
        elif session_lines is not None:
            session_lines.append(line)
            if line.startswith("DISARMED:"):
                path = next_log_path()
                path.write_text("\n".join(session_lines) + "\n", encoding="utf-8")
                prune_old_logs()
                print(f"--- session log saved: {path.name} ---")
                session_lines = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    threading.Thread(target=reader_thread, args=(ser,), daemon=True).start()

    try:
        import msvcrt  # Windows: 키를 누르는 즉시 보낸다 (X 비상정지에 Enter 불필요)
    except ImportError:
        msvcrt = None

    try:
        if msvcrt:
            print(f"Connected to {args.port} @ {args.baud}. Keys are sent immediately "
                  "(x/space = stop). Ctrl+C to quit.")
            while True:
                key = msvcrt.getwch()
                if key == "\x03":
                    break
                if key.isascii() and key.isprintable():
                    ser.write(key.encode())
        else:
            print(f"Connected to {args.port} @ {args.baud}. command + Enter to send. Ctrl+C to quit.")
            while True:
                command = input()
                if command:
                    ser.write(command.encode())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
