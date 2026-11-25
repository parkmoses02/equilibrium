"""Measure camera delay with camera_delay_log over Windows COM."""
import argparse

from camera_delay_xcorr import analyze
from capture_windows import capture_timed, duration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    capture = modes.add_parser("capture")
    capture.add_argument("port")
    capture.add_argument("seconds", type=duration)
    capture.add_argument("log")
    modes.add_parser("analyze").add_argument("log")
    args = parser.parse_args()
    if args.mode == "analyze":
        return analyze(args.log)
    capture_timed(args.port, args.seconds, args.log, zero=True)


if __name__ == "__main__":
    raise SystemExit(main())
