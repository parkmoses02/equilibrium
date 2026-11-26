"""
Stand-in for `pend_cam --stream PORT` so the laptop viewer can be tested without the AI-G board.

Serves the same wire format (StreamHdr + raw UYVY). It renders the simulated swing-up/balance as a camera
image (red pivot marker, two blue rod markers, grey background), flips it vertically (the real camera image
is upside down; pend_cam runs with --vflip) and sends it at ~30 fps.

    python tools/fake_aig_stream.py [port]        (default 5000, listens on 127.0.0.1)
"""
import math
import os
import socket
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from panel.pipeline_controller import simulate_full_pipeline, solve_riccati_lqr   # noqa: E402
from camera_detect import HDR_FMT, STREAM_MAGIC                                   # noqa: E402

W, H = 632, 464
PIVOT_Y, R_TIP, R_MID = 300.0, 148.0, 60.0
PX_PER_M = 1000.0


def rgb_to_uyvy(rgb):
    f = rgb.astype(np.float32)
    y = 0.299 * f[..., 0] + 0.587 * f[..., 1] + 0.114 * f[..., 2]
    u = 128 - 0.168736 * f[..., 0] - 0.331264 * f[..., 1] + 0.5 * f[..., 2]
    v = 128 + 0.5 * f[..., 0] - 0.418688 * f[..., 1] - 0.081312 * f[..., 2]
    ua = 0.5 * (u[:, 0::2] + u[:, 1::2])
    va = 0.5 * (v[:, 0::2] + v[:, 1::2])
    out = np.empty((rgb.shape[0], rgb.shape[1] // 2, 4), np.uint8)
    out[..., 0] = np.clip(ua, 0, 255)
    out[..., 1] = np.clip(y[:, 0::2], 0, 255)
    out[..., 2] = np.clip(va, 0, 255)
    out[..., 3] = np.clip(y[:, 1::2], 0, 255)
    return out.reshape(rgb.shape[0], -1)


def render(angle_deg, pos_m, rng, hide_tip=False):
    img = np.zeros((H, W, 3), np.float32)
    gy = np.linspace(70, 28, H)[:, None]
    img[:] = gy[..., None]
    yy, xx = np.mgrid[0:H, 0:W]
    px = W / 2 + pos_m * PX_PER_M
    a = math.radians(angle_deg)
    pv = (px, PIVOT_Y)
    mid = (px + R_MID * math.sin(a), PIVOT_Y - R_MID * math.cos(a))
    tip = (px + R_TIP * math.sin(a), PIVOT_Y - R_TIP * math.cos(a))
    img[int(PIVOT_Y) + 12:int(PIVOT_Y) + 20, 18:W - 18] = (110, 118, 132)           # rail
    img[int(PIVOT_Y) + 2:int(PIVOT_Y) + 26, int(px) - 42:int(px) + 42] = (110, 115, 125)   # cart (neutral: must not look blue to the detector)
    for t in np.linspace(0, 1, 120):                                                  # rod
        x, y = int(pv[0] + (tip[0] - pv[0]) * t), int(pv[1] + (tip[1] - pv[1]) * t)
        if 0 <= x < W - 3 and 0 <= y < H - 3:
            img[y:y + 3, x:x + 3] = (176, 182, 192)

    def disc(c, r, col):
        m = (xx - c[0]) ** 2 + (yy - c[1]) ** 2 <= r * r
        img[m] = col
    disc(pv, 11, (255, 40, 40))
    disc(mid, 9, (40, 80, 255))
    if not hide_tip:
        disc(tip, 11, (40, 80, 255))
    img += rng.normal(0, 2.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8), pv, mid, tip


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
    K, _, _, _ = solve_riccati_lqr(0.20, 0.04, 9.81)
    res = simulate_full_pipeline(length_m=0.20, friction=0.04, g=9.81, K_lqr=K, swing_speed=1.2, swing_accel=8.0,
                                 cart_limit=0.20, max_speed=0.8, max_accel=12.0, dt=0.002, total_time_s=10.0,
                                 initial_angle_deg=175.0)
    rng = np.random.default_rng(3)
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', port))
    srv.listen(1)
    print(f'fake AI-G stream on 127.0.0.1:{port}', flush=True)
    while True:
        conn, _ = srv.accept()
        print('viewer connected', flush=True)
        seq, idx, t0 = 0, 0, time.monotonic()
        try:
            while True:
                t = time.monotonic() - t0
                idx = int((t % 10.0) / 0.002)
                ang = float(res['theta_deg'][idx])
                pos = float(res['pos_mm'][idx]) / 1000.0
                hide = 3.4 < (t % 4.0) < 3.75 and t > 1.0              # cover the tip now and then
                rgb, pv, mid, tip = render(ang, pos, rng, hide)
                upside_down = np.ascontiguousarray(rgb[::-1])           # real camera image is upside down
                payload = rgb_to_uyvy(upside_down).tobytes()
                nan = float('nan')
                hdr = struct.pack(HDR_FMT, STREAM_MAGIC, seq & 0xFFFFFFFF, int(t * 1e6), W, H, W * 2, len(payload), 1,
                                  8 if not hide else 0, pv[0], pv[1], nan if hide else tip[0], nan if hide else tip[1],
                                  mid[0], mid[1], ang, 7.5, 7.6)
                conn.sendall(hdr + payload)
                seq += 3
                time.sleep(max(0.0, (seq / 3) / 30.0 - (time.monotonic() - t0)))
        except (BrokenPipeError, ConnectionResetError, OSError):
            print('viewer gone', flush=True)
        finally:
            conn.close()


if __name__ == '__main__':
    main()
