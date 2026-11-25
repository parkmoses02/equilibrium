"""Capture camera_estimator_test over Windows COM, or fit a calibration table."""
import argparse

from camera_angle_calib import fit
from capture_windows import capture_timed, duration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    capture = modes.add_parser("capture")
    capture.add_argument("port")
    capture.add_argument("seconds", type=duration)
    capture.add_argument("log")
    analyze = modes.add_parser("fit")
    analyze.add_argument("logs", nargs="+")
    analyze.add_argument("--write-header")
    args = parser.parse_args()
    if args.mode == "fit":
        return fit(args.logs, args.write_header)
    capture_timed(args.port, args.seconds, args.log, zero=True)


if __name__ == "__main__":
    raise SystemExit(main())
