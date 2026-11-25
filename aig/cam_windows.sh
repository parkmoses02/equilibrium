#!/bin/sh
# Installed as ~/cam.sh on the AI-G from the Windows checkout.
set -e
media-ctl -d /dev/media0 -V '"imx219 3-0010":0 [fmt:SBGGR10_1X10/640x480]'
media-ctl -d /dev/media0 -V '"tcc-isp-0":0 [fmt:SBGGR10_1X10/640x480]'
case "$1" in
    view) controls='vertical_blanking=10100,exposure=8000,analogue_gain=100,digital_gain=256' ;;
    60)   controls='vertical_blanking=402,exposure=860,analogue_gain=200,digital_gain=512' ;;
    90)   controls='vertical_blanking=108,exposure=580,analogue_gain=232,digital_gain=512' ;;
    *)    echo "usage: $0 view|60|90" >&2; exit 1 ;;
esac
v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl "$controls"
v4l2-ctl -d /dev/video2 --set-fmt-video width=632,height=464,pixelformat=UYVY
