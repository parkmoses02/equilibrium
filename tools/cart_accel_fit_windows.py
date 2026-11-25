"""Cart chirp, static camera capture, and analysis for cart_accel_log on Windows."""
import argparse

from cart_accel_fit import analyze, px_segments
from capture_windows import capture_timed, duration, recording


def chirp(port, path):
    print("Centre the cart with at least 8 cm free each way. Hang the pendulum still.")
    print("Clear the rail and swing area. Driver power ON. Ctrl+C stops; driver power is the hard stop.")
    input("Press Enter to start the 30 s cart sweep. ")
    with recording(port, path, zero=True) as cap:
        cap.pump(1.5)
        try:
            cap.device.write(b"g")
            if not cap.pump(2, keepalive=True, until=b"# STARTED"):
                print("No STARTED acknowledgement. See the recording for a refusal.")
                return 1
            if not cap.pump(40, keepalive=True, until=b"# STOPPED"):
                print("Timed out waiting for STOPPED.")
                return 1
        finally:
            cap.device.write(b"x")
        cap.pump(3)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    capture = modes.add_parser("capture", help="motor-driven chirp")
    capture.add_argument("port")
    capture.add_argument("log")
    static = modes.add_parser("static", help="manual movement, driver disabled")
    static.add_argument("port")
    static.add_argument("seconds", type=duration)
    static.add_argument("log")
    modes.add_parser("analyze").add_argument("log")
    modes.add_parser("pxsegments").add_argument("log")
    args = parser.parse_args()
    if args.mode == "capture":
        return chirp(args.port, args.log)
    if args.mode == "static":
        return capture_timed(args.port, args.seconds, args.log)
    if args.mode == "analyze":
        return analyze(args.log)
    return px_segments(args.log)


if __name__ == "__main__":
    raise SystemExit(main())
