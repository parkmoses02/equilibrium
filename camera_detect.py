"""
Laptop-side marker detection for the AI-G camera stream (display only).

Python port of the detection + tracking in aig/pend_cam.c (same thresholds, same logic) so the viewer shows
what the camera recognises without depending on the AI-G -> ESP32 path:

    Detector  : UYVY frame -> red pivot blob + blue blobs  (classify, 3x3 dilate, 4-connected labelling)
    RodTracker: tip / mid selection with the self-calibrated rod radii, causal outlier rejection, angle
    StreamReceiver (QThread): TCP client for the stream that `pend_cam --stream PORT` serves

Wire format (little-endian): StreamHdr (68 bytes) + raw UYVY payload (stride * h bytes), see aig/pend_cam.c.
"""
import math
import socket
import struct
import time

import numpy as np
from scipy import ndimage

HDR_FMT = '<IIQHHIIHH9f'
HDR_SIZE = struct.calcsize(HDR_FMT)          # 68
STREAM_MAGIC = 0x31464350                    # 'P','C','F','1'


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------------
class Thresholds:
    """YUV thresholds, defaults identical to pend_cam.c (override on the board with --red / --blue)."""
    def __init__(self):
        self.r_vmin, self.r_umax, self.r_diff = 148, 130, 30
        self.b_umin, self.b_vmax, self.b_diff = 150, 120, 50
        self.min_blue, self.min_red = 30, 10          # in macropixels (2 px wide)
        self.tol, self.jump, self.cal_n = 0.30, 45.0, 30
        self.ignore = []                              # [(x, y, r)] blue blobs to skip


_DIL = np.ones((3, 3), bool)


class Detector:
    def __init__(self, th=None):
        self.th = th or Thresholds()

    def classify(self, img, w, h, stride):
        """img: bytes/ndarray of stride*h. Returns (red mask, blue mask), each (h, w/2)."""
        a = np.frombuffer(img, np.uint8, count=stride * h).reshape(h, stride)[:, :w * 2]
        p = a.reshape(h, w // 2, 4).astype(np.int16)
        u, v = p[..., 0], p[..., 2]
        y = (p[..., 1] + p[..., 3]) >> 1
        t = self.th
        red = (v >= t.r_vmin) & (u <= t.r_umax) & (v - u >= t.r_diff)
        blue = (u >= t.b_umin) & (v <= t.b_vmax) & (u - v >= t.b_diff) & (y >= 15)
        return red, blue

    @staticmethod
    def blobs(mask, min_area, maxb=64):
        """Port of label_blobs(): dilate 3x3, label (4-connectivity), centroid/area from the undilated mask."""
        if not mask.any():
            return []
        lab, n = ndimage.label(ndimage.binary_dilation(mask, structure=_DIL))
        ys, xs = np.nonzero(mask)
        ls = lab[ys, xs]
        keep = ls > 0
        ys, xs, ls = ys[keep], xs[keep], ls[keep]
        cnt = np.bincount(ls, minlength=n + 1)
        sx = np.bincount(ls, weights=xs, minlength=n + 1)
        sy = np.bincount(ls, weights=ys, minlength=n + 1)
        out = []
        for k in range(1, n + 1):
            if cnt[k] >= min_area:
                out.append({'x': 2.0 * sx[k] / cnt[k] + 0.5, 'y': sy[k] / cnt[k], 'n': int(cnt[k]) * 2})
                if len(out) >= maxb:
                    break
        return out

    def detect(self, img, w, h, stride, vflip):
        """Returns (pivot or None, [blue blobs]); y already flipped when vflip (like pend_cam)."""
        red, blue = self.classify(img, w, h, stride)
        t = self.th
        rb = self.blobs(red, t.min_red)
        piv = None
        if rb:
            piv = dict(max(rb, key=lambda b: b['n']))
            if vflip:
                piv['y'] = (h - 1) - piv['y']
        bl = []
        for b in self.blobs(blue, t.min_blue):
            if vflip:
                b['y'] = (h - 1) - b['y']
            if any(math.hypot(b['x'] - gx, b['y'] - gy) < gr for gx, gy, gr in t.ignore):
                continue
            bl.append(b)
        return piv, bl


# ------------------------------------------------------------------------------------------------
class RodTracker:
    """Port of track(): rod radii self-calibration, tip/mid selection, causal outlier rejection."""
    def __init__(self, th):
        self.th = th
        self.reset()

    def reset(self):
        self.calibrated = False
        self.dt, self.dm = [], []
        self.Lt = self.Lm = 0.0
        self.last_tip = self.last_mid = self.last_best = None
        self.since_tip = self.since_mid = 0
        self.unw = 0.0

    @staticmethod
    def _ang(p, b):
        return math.degrees(math.atan2(b['x'] - p['x'], p['y'] - b['y']))

    def update(self, piv, bl):
        """Returns dict(tip, mid (blobs or None), at, am, ang (deg or nan), calibrating)."""
        t = self.th
        res = {'tip': None, 'mid': None, 'at': math.nan, 'am': math.nan, 'ang': math.nan,
               'calibrating': False, 'no_pivot': piv is None}
        if piv is None:
            return res
        d = [math.hypot(b['x'] - piv['x'], b['y'] - piv['y']) for b in bl]
        tip = mid = None
        if not self.calibrated:
            if len(bl) == 2:
                ti = 0 if d[0] > d[1] else 1
                mi = 1 - ti
                if d[mi] > 0.3 * d[ti]:
                    self.dt.append(d[ti])
                    self.dm.append(d[mi])
                    tip, mid = bl[ti], bl[mi]
                    res['calibrating'] = True
                    if len(self.dt) >= t.cal_n:
                        self.Lt = float(np.median(self.dt))
                        self.Lm = float(np.median(self.dm))
                        self.calibrated = True
        if self.calibrated and not res['calibrating']:
            bt = bm = 1e9
            for i, b in enumerate(bl):
                et = abs(d[i] - self.Lt) / self.Lt
                em = abs(d[i] - self.Lm) / self.Lm
                if et < t.tol and et < bt and et <= em:
                    tip, bt = b, et
                if em < t.tol and em < bm and em < et:
                    mid, bm = b, em
            if tip and mid:
                if abs(wrap180(self._ang(piv, tip) - self._ang(piv, mid))) > 25:
                    if bt <= bm:
                        mid = None
                    else:
                        tip = None
        at = self._ang(piv, tip) if tip else math.nan
        am = self._ang(piv, mid) if mid else math.nan
        if not math.isnan(at) and self.last_tip is not None and self.since_tip < 10:
            lim = min(180.0, t.jump * (1 + self.since_tip))
            if abs(wrap180(at - self.last_tip)) > lim:
                at, tip = math.nan, None
        if not math.isnan(am) and self.last_mid is not None and self.since_mid < 10:
            lim = min(180.0, t.jump * (1 + self.since_mid))
            if abs(wrap180(am - self.last_mid)) > lim:
                am, mid = math.nan, None
        if not math.isnan(at):
            self.last_tip, self.since_tip = at, 0
        else:
            self.since_tip += 1
        if not math.isnan(am):
            self.last_mid, self.since_mid = am, 0
        else:
            self.since_mid += 1
        best = at if not math.isnan(at) else am
        res.update(tip=tip, mid=mid, at=at, am=am, ang=best)
        return res


# ------------------------------------------------------------------------------------------------
def uyvy_to_rgb(img, w, h, stride, vflip):
    """UYVY -> RGB888 ndarray (h, w, 3) for display (BT.601 full range), flipped like the detector."""
    a = np.frombuffer(img, np.uint8, count=stride * h).reshape(h, stride)[:, :w * 2]
    p = a.reshape(h, w // 2, 4).astype(np.float32)
    u, y0, v, y1 = p[..., 0] - 128.0, p[..., 1], p[..., 2] - 128.0, p[..., 3]
    rv, gu, gv, bu = 1.402 * v, -0.344 * u, -0.714 * v, 1.772 * u
    out = np.empty((h, w // 2, 2, 3), np.float32)
    for k, yy in enumerate((y0, y1)):
        out[:, :, k, 0] = yy + rv
        out[:, :, k, 1] = yy + gu + gv
        out[:, :, k, 2] = yy + bu
    rgb = np.clip(out.reshape(h, w, 3), 0, 255).astype(np.uint8)
    return np.ascontiguousarray(rgb[::-1]) if vflip else rgb


def recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError('stream closed')
        got += k
    return buf


def parse_header(raw):
    f = struct.unpack(HDR_FMT, bytes(raw))
    if f[0] != STREAM_MAGIC:
        raise ValueError('bad stream magic %08x' % f[0])
    keys = ('magic', 'seq', 'ts_us', 'w', 'h', 'stride', 'payload', 'flags', 'rflags',
            'px', 'py', 'tx', 'ty', 'mx', 'my', 'ang', 'proc_ms', 'lat_ms')
    return dict(zip(keys, f))


# ------------------------------------------------------------------------------------------------
try:
    from PyQt5 import QtCore
except Exception:           # the detector can be used without Qt
    QtCore = None

if QtCore is not None:
    class StreamReceiver(QtCore.QThread):
        """Connects to pend_cam --stream, runs detection + tracking on every received frame and keeps
        the newest result in `self.latest` (the GUI polls it). Nothing here touches the control path."""
        status = QtCore.pyqtSignal(str)

        def __init__(self, host, port, thresholds, parent=None):
            super().__init__(parent)
            self.host, self.port = host, port
            self.th = thresholds
            self.det = Detector(thresholds)
            self.trk = RodTracker(thresholds)
            self._stop = False
            self.latest = None
            self.version = 0
            self.rx_frames = 0
            self.gaps = 0
            self.connected = False

        def stop(self):
            self._stop = True
            s = getattr(self, '_sock', None)
            if s is not None:                       # unblock a recv() that is waiting for the next frame
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

        def reset_tracker(self):
            self.trk.reset()

        def run(self):
            sock = None
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                self._sock = sock
                sock.settimeout(5.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
                self.connected = True
                self.status.emit(f'stream connected to {self.host}:{self.port}')
                prev_seq, prev_t = None, None
                while not self._stop:
                    hdr = parse_header(recv_exact(sock, HDR_SIZE))
                    img = recv_exact(sock, hdr['payload'])
                    t_rx = time.monotonic()
                    t0 = time.perf_counter()
                    w, h, stride, vflip = hdr['w'], hdr['h'], hdr['stride'], bool(hdr['flags'] & 1)
                    piv, bl = self.det.detect(img, w, h, stride, vflip)
                    tr = self.trk.update(piv, bl)
                    rgb = uyvy_to_rgb(img, w, h, stride, vflip)
                    proc_ms = (time.perf_counter() - t0) * 1e3
                    gap = 0
                    if prev_seq is not None:
                        gap = (hdr['seq'] - prev_seq) & 0xFFFF
                    interval = (t_rx - prev_t) * 1e3 if prev_t is not None else math.nan
                    prev_seq, prev_t = hdr['seq'], t_rx
                    self.rx_frames += 1
                    self.latest = {'hdr': hdr, 'piv': piv, 'blobs': bl, 'track': tr, 'rgb': rgb,
                                   'proc_ms': proc_ms, 'interval_ms': interval, 'seq_step': gap,
                                   't_rx': t_rx, 'calibrated': self.trk.calibrated,
                                   'Lt': self.trk.Lt, 'Lm': self.trk.Lm}
                    self.version += 1
            except Exception as exc:                       # connection lost / refused / bad data
                if not self._stop:
                    self.status.emit(f'stream stopped: {exc}')
            finally:
                self.connected = False
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
