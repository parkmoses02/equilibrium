"""
Camera Recognition View: same window structure as panel/pipeline_controller.py
(left control column + right canvas + 2x2 telemetry graphs), but the canvas is the
AI-G / IMX219 camera frame (632x464) with detection boxes and tracking overlays,
in the style of an object-detection demo:

    red box    = CART  (red marker, rotation centre)         -> like "CAR"
    yellow box = MID / TIP (blue markers on the rod)          -> like "Human"
    green fan  = LQR catch window (+-25 deg) around upright
    blue arcs  = motion cue at the tip
    wifi icon  = AI-G -> ESP32 UART link (460800 bps, 14-byte packets)

Sources
  * Offline (default): the nonlinear swing-up + LQR simulation from pipeline_controller.py is
    rendered as a synthetic camera frame (pixel noise, camera delay, marker dropouts).
  * Live: connect to the ESP32; the angle / cart position telemetry (0xAA frames) drive the markers.
    No camera pixels are available over that link, so boxes are drawn from the telemetry geometry.

Run:  python camera_view.py
"""

import os
import sys
import math
import time
import datetime
from collections import deque

import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:   # repo layout: this file sits in panel/ next to pipeline_controller.py
    from pipeline_controller import simulate_full_pipeline, solve_riccati_lqr  # noqa: E402
except ImportError:   # workspace layout: panel/ is a sub folder
    from panel.pipeline_controller import simulate_full_pipeline, solve_riccati_lqr  # noqa: E402
from camera_detect import StreamReceiver, Thresholds  # noqa: E402
from camera_link import SerialLink  # noqa: E402

# ---- camera constants (from the AI-G notes: 632x464 UYVY, 90 fps) -------------------------
FRAME_W, FRAME_H = 632, 464
CAM_FPS = 90.0
PIVOT_Y = 300.0            # rail height in the frame: the tip stays inside the frame for most angles
R_TIP_PX = 148.0           # measured on the board (pend_cam.c self-calibration)
R_MID_PX = 60.0
CATCH_WINDOW_DEG = 25.0
CAM_TIMEOUT_MS = 60.0      # firmware disarms when no accepted packet for this long

RED = QtGui.QColor(235, 40, 40)
YELLOW = QtGui.QColor(255, 220, 20)
ORANGE = QtGui.QColor(255, 140, 0)
GREEN = QtGui.QColor(46, 204, 113)
BLUE = QtGui.QColor(88, 166, 255)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


# ==============================================================================
# 1. Tracker (alpha-beta per marker; stands in for the ESP32 fixed-lag EKF in this view)
# ==============================================================================
class AlphaBeta:
    def __init__(self):
        self.reset()

    def reset(self):
        self.init = False
        self.rejects = 0
        self.x = self.y = self.vx = self.vy = 0.0
        self.age_s = 0.0          # time since last accepted measurement

    def step(self, dt, meas, alpha, beta, gate_px):
        """meas = (x, y) or None. Returns True when the measurement was accepted."""
        if not self.init:
            if meas is None:
                return False
            self.x, self.y = meas
            self.vx = self.vy = 0.0
            self.init = True
            self.age_s = 0.0
            return True
        px, py = self.x + self.vx * dt, self.y + self.vy * dt
        if meas is None:
            self.x, self.y = px, py
            self.age_s += dt
            return False
        rx, ry = meas[0] - px, meas[1] - py
        # outlier gate; it widens while coasting (the marker keeps moving while it is hidden)
        if math.hypot(rx, ry) > gate_px * (1.0 + 10.0 * self.age_s):
            self.rejects += 1
            if self.rejects < 4:                    # ~0.1 s of frames, like the firmware's resync
                self.x, self.y = px, py
                self.age_s += dt
                return False
            self.x, self.y = meas                   # estimate is lost: restart from the camera
            self.vx = self.vy = 0.0
            self.rejects = 0
            self.age_s = 0.0
            return True
        self.rejects = 0
        self.x = px + alpha * rx
        self.y = py + alpha * ry
        self.vx += beta * rx / dt
        self.vy += beta * ry / dt
        self.age_s = 0.0
        return True

    def ahead(self, lag_s):
        return self.x + self.vx * lag_s, self.y + self.vy * lag_s


# ==============================================================================
# 2. Camera frame canvas
# ==============================================================================
class CameraViewWidget(QtWidgets.QWidget):
    """Draws a synthetic 632x464 camera frame + detection overlay. Data comes from set_frame()."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(340)
        self.f = None
        self.rail_px = 200.0
        self.trail = deque(maxlen=45)
        self.ghost = deque(maxlen=4)
        self.opt = dict(boxes=True, labels=True, trail=True, predict=True, fan=True,
                        motion=True, roi=True, rodbox=True, hud=True)
        self._pulse = 0.0

    def set_option(self, key, val):
        self.opt[key] = bool(val)
        self.update()

    def clear_trail(self):
        self.trail.clear()
        self.ghost.clear()

    def set_frame(self, f):
        self.f = f
        if f is not None and f['est']['tip'] is not None:
            self.trail.append(f['est']['tip'])
            if f['true']['mid'] is not None and f['true']['tip'] is not None:
                self.ghost.append((f['true']['mid'], f['true']['tip']))
        self._pulse = (self._pulse + 0.25) % 1.0
        self.update()

    # ---- drawing helpers (all in camera pixel coordinates) ----
    @staticmethod
    def _font(px, bold=False):
        fnt = QtGui.QFont('Segoe UI')
        fnt.setPixelSize(int(px))
        fnt.setBold(bold)
        return fnt

    def _draw_tag(self, p, x, y, text, color, big=False):
        """Filled label tab sitting on the top-left corner of a box (above it)."""
        fnt = self._font(15 if big else 11, True)
        p.setFont(fnt)
        fm = QtGui.QFontMetrics(fnt)
        tw = fm.horizontalAdvance(text) + 8
        th = fm.height() + 2
        rect = QtCore.QRectF(x - 1.5, y - th, tw, th)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(color)
        p.drawRect(rect)
        p.setPen(QtGui.QColor(255, 255, 255) if color == RED else QtGui.QColor(20, 20, 20))
        p.drawText(rect, QtCore.Qt.AlignCenter, text)

    def _draw_box(self, p, det):
        x, y, w, h = det['box']
        color = det['color']
        if det['lost']:
            pen = QtGui.QPen(ORANGE, 2.5, QtCore.Qt.DashLine)
        else:
            pen = QtGui.QPen(color, 3.0)
        p.setBrush(QtCore.Qt.NoBrush)
        p.setPen(pen)
        p.drawRect(QtCore.QRectF(x, y, w, h))
        if self.opt['labels']:
            if det['lost']:
                self._draw_tag(p, x, y, f"{det['name']} LOST (pred)", ORANGE)
            else:
                txt = f"{det['name']} #{det['id']}  {det['conf']:.2f}"
                self._draw_tag(p, x, y, txt, color, big=(color == RED))

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setRenderHint(QtGui.QPainter.TextAntialiasing)
        try:
            p.fillRect(self.rect(), QtGui.QColor('#0b0f14'))
            w, h = self.width(), self.height()
            s = min((w - 16) / FRAME_W, (h - 16) / FRAME_H)
            if s <= 0:
                return
            ox = (w - FRAME_W * s) / 2.0
            oy = (h - FRAME_H * s) / 2.0
            p.save()
            p.translate(ox, oy)
            p.scale(s, s)
            frame_rect = QtCore.QRectF(0, 0, FRAME_W, FRAME_H)
            p.setClipRect(frame_rect)
            self._draw_scene(p)
            if self.f is not None:
                self._draw_overlay(p)
            p.restore()
            # frame border
            p.setPen(QtGui.QPen(QtGui.QColor('#30363d'), 2))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawRect(QtCore.QRectF(ox, oy, FRAME_W * s, FRAME_H * s))
        finally:
            p.end()

    # ---- the synthetic camera image ----
    def _draw_scene(self, p):
        if self.f is not None and self.f.get('image') is not None:
            p.drawImage(QtCore.QRectF(0, 0, FRAME_W, FRAME_H), self.f['image'])   # real camera frame (AI-G stream)
            return
        grad = QtGui.QLinearGradient(0, 0, 0, FRAME_H)
        grad.setColorAt(0.0, QtGui.QColor(58, 64, 76))
        grad.setColorAt(0.65, QtGui.QColor(38, 42, 52))
        grad.setColorAt(1.0, QtGui.QColor(24, 26, 32))
        p.fillRect(QtCore.QRectF(0, 0, FRAME_W, FRAME_H), grad)
        # faint perspective grid so the "road" reads as a surface
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 14), 1))
        for gx in range(0, FRAME_W + 1, 79):
            p.drawLine(gx, 0, gx, FRAME_H)
        for gy in range(0, FRAME_H + 1, 58):
            p.drawLine(0, gy, FRAME_W, gy)
        # rail
        rail_y = PIVOT_Y + 16
        p.setPen(QtGui.QPen(QtGui.QColor(110, 118, 132), 6))
        p.drawLine(18, int(rail_y), FRAME_W - 18, int(rail_y))
        p.setPen(QtGui.QPen(QtGui.QColor(20, 22, 28, 160), 10))
        p.drawLine(18, int(rail_y + 9), FRAME_W - 18, int(rail_y + 9))
        if self.f is None:
            return
        t = self.f['true']
        # rod (grey, slightly thicker than the markers' stem)
        p.setPen(QtGui.QPen(QtGui.QColor(176, 182, 192), 7, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(QtCore.QPointF(*t['pivot']), QtCore.QPointF(*t['tip']))
        # cart body
        cx = t['pivot'][0]
        p.setPen(QtGui.QPen(QtGui.QColor(225, 230, 238), 2))
        p.setBrush(QtGui.QColor(52, 120, 180))
        p.drawRoundedRect(QtCore.QRectF(cx - 42, PIVOT_Y + 2, 84, 24), 5, 5)
        p.setBrush(QtGui.QColor(30, 40, 52))
        p.drawEllipse(QtCore.QPointF(cx - 24, rail_y + 2), 7, 7)
        p.drawEllipse(QtCore.QPointF(cx + 24, rail_y + 2), 7, 7)
        # motion blur ghosts (markers smear at speed)
        if self.f['blur'] and self.opt['motion']:
            n = len(self.ghost)
            for i, (gm, gt) in enumerate(self.ghost):
                a = int(20 + 30 * i / max(1, n))
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(60, 120, 255, a))
                p.drawEllipse(QtCore.QPointF(*gm), 8, 8)
                p.drawEllipse(QtCore.QPointF(*gt), 10, 10)
        # markers: red = pivot, blue = mid and tip (the board detects these by U/V thresholds)
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 200), 1.5))
        p.setBrush(QtGui.QColor(225, 40, 40))
        p.drawEllipse(QtCore.QPointF(*t['pivot']), 11, 11)
        p.setBrush(QtGui.QColor(50, 110, 240))
        p.drawEllipse(QtCore.QPointF(*t['mid']), 9, 9)
        p.drawEllipse(QtCore.QPointF(*t['tip']), 11, 11)

    # ---- detection overlay ----
    def _draw_overlay(self, p):
        f = self.f
        est = f['est']
        pv = est['pivot'] or f['true']['pivot']

        # green fan = catch window around upright (like the radar fan in the demo picture)
        if self.opt['fan']:
            r = R_TIP_PX * 1.18
            rect = QtCore.QRectF(pv[0] - r, pv[1] - r, 2 * r, 2 * r)
            path = QtGui.QPainterPath()
            path.moveTo(pv[0], pv[1])
            path.arcTo(rect, 90 - CATCH_WINDOW_DEG, 2 * CATCH_WINDOW_DEG)
            path.closeSubpath()
            rg = QtGui.QRadialGradient(QtCore.QPointF(*pv), r)
            inside = f['in_window']
            c0 = QtGui.QColor(46, 204, 113, 150 if inside else 95)
            c1 = QtGui.QColor(46, 204, 113, 15)
            rg.setColorAt(0.0, c0)
            rg.setColorAt(1.0, c1)
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(rg)
            p.drawPath(path)
            p.setPen(QtGui.QPen(QtGui.QColor(46, 204, 113, 170), 1.5, QtCore.Qt.DashLine))
            for sgn in (-1, 1):
                a = math.radians(sgn * CATCH_WINDOW_DEG)
                p.drawLine(QtCore.QPointF(*pv),
                           QtCore.QPointF(pv[0] + r * math.sin(a), pv[1] - r * math.cos(a)))

        # tip trail (fades with age)
        if self.opt['trail'] and len(self.trail) > 2:
            pts = list(self.trail)
            n = len(pts)
            for i in range(1, n):
                if math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) > 70:
                    continue            # tracker restart / resync jump: do not draw a streak across it
                a = int(30 + 200 * i / n)
                p.setPen(QtGui.QPen(QtGui.QColor(255, 220, 20, a), 1 + 3.0 * i / n,
                                    QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawLine(QtCore.QPointF(*pts[i - 1]), QtCore.QPointF(*pts[i]))

        # blue motion arcs at the tip, opening toward the direction of travel
        if self.opt['motion'] and est['tip'] is not None:
            vx, vy = f['vel']['tip']
            sp = math.hypot(vx, vy)
            if sp > 40:
                ang = math.degrees(math.atan2(-vy, vx))
                tx, ty = est['tip']
                for k, rr in enumerate((30, 44, 58)):
                    a = max(20, min(150, int(sp / 6)) - 35 * k)
                    p.setPen(QtGui.QPen(QtGui.QColor(88, 166, 255, a), 4 - k,
                                        QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                    p.setBrush(QtCore.Qt.NoBrush)
                    p.drawArc(QtCore.QRectF(tx - rr, ty - rr, 2 * rr, 2 * rr),
                              int((ang - 55) * 16), int(110 * 16))

        # search ROI around each target
        if self.opt['roi']:
            half = f['roi']
            p.setBrush(QtCore.Qt.NoBrush)
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 70), 1, QtCore.Qt.DotLine))
            for key in ('pivot', 'mid', 'tip'):
                e = est[key]
                if e is not None:
                    p.drawRect(QtCore.QRectF(e[0] - half, e[1] - half, 2 * half, 2 * half))

        # whole-rod box
        if self.opt['rodbox'] and est['pivot'] and est['tip']:
            xs = [est['pivot'][0], est['tip'][0]]
            ys = [est['pivot'][1], est['tip'][1]]
            pad = 16
            p.setBrush(QtCore.Qt.NoBrush)
            p.setPen(QtGui.QPen(QtGui.QColor(120, 220, 255, 190), 1.8, QtCore.Qt.DashLine))
            p.drawRect(QtCore.QRectF(min(xs) - pad, min(ys) - pad,
                                     max(xs) - min(xs) + 2 * pad, max(ys) - min(ys) + 2 * pad))

        # detection boxes
        if self.opt['boxes']:
            for det in f['dets']:
                self._draw_box(p, det)

        # predicted (lag-compensated) position: dashed ghost box + arrow
        if self.opt['predict']:
            for det in f['dets']:
                if det['pred'] is None:
                    continue
                (ex, ey), (qx, qy) = det['center'], det['pred']
                if math.hypot(qx - ex, qy - ey) < 3:
                    continue
                p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 200), 1.6, QtCore.Qt.DashLine))
                p.drawLine(QtCore.QPointF(ex, ey), QtCore.QPointF(qx, qy))
                p.setBrush(QtCore.Qt.NoBrush)
                bw, bh = det['box'][2], det['box'][3]
                p.drawRect(QtCore.QRectF(qx - bw / 2, qy - bh / 2, bw, bh))

        # raw measurement (the detector's output before tracking)
        if self.opt['boxes']:
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 230), 1.4))
            for key in ('pivot', 'mid', 'tip'):
                m = f['meas'][key]
                if m is not None:
                    p.drawLine(QtCore.QPointF(m[0] - 4, m[1]), QtCore.QPointF(m[0] + 4, m[1]))
                    p.drawLine(QtCore.QPointF(m[0], m[1] - 4), QtCore.QPointF(m[0], m[1] + 4))

        if self.opt['hud']:
            self._draw_hud(p)

    def _draw_hud(self, p):
        f = self.f
        # top-left stream info
        p.setFont(self._font(11, True))
        lines = [f.get('hud1') or f"AI-G  IMX219  {FRAME_W}x{FRAME_H} UYVY  {CAM_FPS:.0f} fps",
                 f.get('hud2') or f"frame #{f['frame']:05d}   proc {f['proc_ms']:.1f} ms   age {f['age_ms']:.1f} ms"]
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(0, 0, 0, 150))
        p.drawRect(QtCore.QRectF(6, 6, 292, 38))
        p.setPen(QtGui.QColor(230, 237, 243))
        for i, ln in enumerate(lines):
            p.drawText(QtCore.QPointF(12, 22 + 15 * i), ln)

        # bottom readout
        ang = f['angle_est']
        txt = (f"angle {ang:+6.1f} deg   cart x {f['cart_x_px']:6.1f} px   "
               f"{f['mode_text']}")
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(0, 0, 0, 160))
        p.drawRect(QtCore.QRectF(6, FRAME_H - 30, 420, 24))
        p.setPen(GREEN if f['in_window'] else QtGui.QColor(230, 237, 243))
        p.setFont(self._font(12, True))
        p.drawText(QtCore.QPointF(12, FRAME_H - 13), txt)

        # flags b0..b5 of the 14-byte packet
        names = ['ANG', 'TIP', 'MID', 'CART', 'CAL', 'AGE']
        x0 = FRAME_W - 6 - 6 * 38
        p.setFont(self._font(10, True))
        for i, nm in enumerate(names):
            on = f['flags'][i]
            r = QtCore.QRectF(x0 + i * 38, FRAME_H - 30, 34, 20)
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor(46, 160, 67, 210) if on else QtGui.QColor(80, 86, 96, 190))
            p.drawRoundedRect(r, 3, 3)
            p.setPen(QtGui.QColor(255, 255, 255) if on else QtGui.QColor(170, 176, 186))
            p.drawText(r, QtCore.Qt.AlignCenter, nm)

        # UART link icon (wifi-style arcs) top-right, pulses with each packet
        cx, cy = FRAME_W - 52, 50
        ok = f['link_ok']
        col = BLUE if ok else QtGui.QColor(248, 81, 73)
        for k, rr in enumerate((10, 20, 30)):
            a = 255 if ok and (k / 3.0 <= self._pulse + 0.34) else 90
            p.setPen(QtGui.QPen(QtGui.QColor(col.red(), col.green(), col.blue(), a), 4,
                                QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawArc(QtCore.QRectF(cx - rr, cy - rr, 2 * rr, 2 * rr), 45 * 16, 90 * 16)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(col)
        p.drawEllipse(QtCore.QPointF(cx, cy), 4, 4)
        p.setFont(self._font(10, True))
        p.setPen(col)
        p.drawText(QtCore.QRectF(cx - 70, cy + 12, 140, 14), QtCore.Qt.AlignCenter,
                   f"UART 460800  {'OK' if ok else 'NO DATA'}")

        # firmware guard banner
        if f['banner']:
            p.setFont(self._font(20, True))
            fm = QtGui.QFontMetrics(p.font())
            tw = fm.horizontalAdvance(f['banner']) + 30
            r = QtCore.QRectF((FRAME_W - tw) / 2, 66, tw, 34)
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255), 2))
            p.setBrush(QtGui.QColor(218, 54, 51, 225))
            p.drawRoundedRect(r, 6, 6)
            p.drawText(r, QtCore.Qt.AlignCenter, f['banner'])


# ==============================================================================
# 3. Main window (same layout as PipelineMainWindow)
# ==============================================================================
STYLE = """
    QMainWindow { background-color: #0f141c; color: #e6edf3; }
    QGroupBox { font-weight: bold; border: 1px solid #30363d; border-radius: 6px; margin-top: 10px; background-color: #161b22; color: #f0f6fc; }
    QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 6px; color: #58a6ff; }
    QLabel { color: #c9d1d9; font-size: 12px; }
    QDoubleSpinBox, QSpinBox, QLineEdit { background-color: #0d1117; border: 1px solid #30363d; border-radius: 4px; padding: 4px; color: #58a6ff; font-weight: bold; }
    QPushButton { background-color: #21262d; border: 2px solid #8b949e; border-radius: 5px; padding: 6px; color: #e6edf3; font-weight: bold; }
    QPushButton:hover { background-color: #30363d; border-color: #c9d1d9; }
    QPushButton:pressed { background-color: #0d1117; border-color: #58a6ff; }
    QPushButton#primaryBtn { background-color: #238636; color: white; border: 2px solid #56d364; font-size: 13px; }
    QPushButton#primaryBtn:hover { background-color: #2ea043; border-color: #9be9a8; }
    QPushButton#dangerBtn { background-color: #da3633; color: white; border: 2px solid #ff7b72; font-size: 13px; }
    QPushButton#dangerBtn:hover { background-color: #f85149; border-color: #ffa198; }
    QPushButton#actionBtn { background-color: #1f6feb; color: white; border: 2px solid #79c0ff; font-size: 13px; }
    QPushButton#actionBtn:hover { background-color: #388bfd; border-color: #a5d6ff; }
    QCheckBox { color: #ffffff; font-size: 12px; font-weight: bold; spacing: 8px; }
    QCheckBox::indicator { width: 16px; height: 16px; border: 2px solid #8b949e; border-radius: 3px; background-color: #0d1117; }
    QCheckBox::indicator:hover { border-color: #c9d1d9; }
    QCheckBox::indicator:checked { background-color: #1f6feb; border-color: #79c0ff; }
    QScrollArea { border: none; background-color: #0f141c; }
    QWidget#leftPane { background-color: #0f141c; }
    QScrollBar:vertical { background: #0d1117; width: 12px; margin: 0; }
    QScrollBar::handle:vertical { background: #30363d; border-radius: 5px; min-height: 24px; }
    QScrollBar::handle:vertical:hover { background: #484f58; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""

MODE_NAMES = {0: 'SAFE', 1: 'ENERGY SWING-UP', 2: 'LQR BALANCE'}
HIST = 300   # graph samples (about 10 s at 30 fps)


class CameraMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Inverted Pendulum - Camera Recognition View (AI-G + IMX219)')
        self.resize(1750, 960)

        self.serial = None
        self._rx_count = 0
        self._live = {'angle': 0.0, 'pos': 0.0, 'mode': 0}
        self._sim = None
        self._sim_idx = 0
        self._frame_no = 0
        self._rng = np.random.default_rng(7)
        self._trk = {k: AlphaBeta() for k in ('pivot', 'mid', 'tip')}
        self._conf_ema = {'pivot': 0.97, 'mid': 0.97, 'tip': 0.97}
        self._last_pos_px = None
        self._buf = {k: deque(maxlen=HIST) for k in
                     ('t', 'true', 'meas', 'est', 'cx_m', 'cx_e', 'c_cart', 'c_mid', 'c_tip', 'age', 'proc')}
        self._t0 = time.monotonic()
        self._lost_since = None
        self._sim_wall_t = 0.0
        self.th = Thresholds()           # YUV thresholds shared with the stream receiver (same defaults as pend_cam)
        self.receiver = None
        self._rx_ver = 0
        self._last_stream_t = None
        self._n_ref = {'pivot': None, 'mid': None, 'tip': None}
        self._lost_frames = 0
        self._names_mode = 'sim'
        # camera data relayed by the ESP32 (camera_view_fw.cpp, telemetry 0x10..0x19) and operator state
        self._cam = {}
        self._cam_t = 0.0
        self._cam_ver = 0
        self._cam_seen = 0
        self._last_cam_t = None
        self._last_pivot = (FRAME_W / 2.0, PIVOT_Y)
        self._last_ang = 0.0
        self._lines = deque()            # text lines from the serial thread
        self._link_err = None
        self._source_text = '?'
        self._rec_n = 0
        self._sh_line = ''

        self._init_ui()
        self.run_simulation(initial_angle_deg=175.0)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(33)

    # ------------------------------------------------------------------ UI
    def _init_ui(self):
        self.setStyleSheet(STYLE)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        left_pane = QtWidgets.QWidget()
        left_pane.setFixedWidth(480)
        left_pane.setObjectName('leftPane')
        left = QtWidgets.QVBoxLayout(left_pane)
        left.setContentsMargins(0, 12, 0, 0)
        left.setSpacing(8)

        def dspin(lo, hi, val, dec=1, step=None, suffix=''):
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(dec)
            sp.setValue(val)
            if step:
                sp.setSingleStep(step)
            if suffix:
                sp.setSuffix(suffix)
            return sp

        # 1. connection
        grp = QtWidgets.QGroupBox('1. Camera Link & ESP32 Serial')
        lay = QtWidgets.QVBoxLayout(grp)
        h = QtWidgets.QHBoxLayout()
        self.port_edit = QtWidgets.QLineEdit('COM3')
        self.connect_btn = QtWidgets.QPushButton('Connect ESP32')
        self.connect_btn.setObjectName('actionBtn')
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.connect_btn.setToolTip('COM 포트를 115200bps로 열고 ESP32 텔레메트리(각도, 카트 위치)로 마커를 그립니다. 모터는 움직이지 않습니다.')
        h.addWidget(QtWidgets.QLabel('Port:'))
        h.addWidget(self.port_edit)
        h.addWidget(self.connect_btn)
        lay.addLayout(h)
        self.status_label = QtWidgets.QLabel('Status: Offline (Simulated Camera Active)')
        self.status_label.setStyleSheet('color: #8b949e; font-family: monospace;')
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(34)
        lay.addWidget(self.status_label)
        cam = QtWidgets.QLabel('Camera: IMX219 632x464 UYVY, 90 fps, exposure 300 lines\n'
                               'AI-G /dev/ttyAMA1 -> ESP32 GPIO32, 460800 8N1, 14-byte packet')
        cam.setStyleSheet('color: #7ee787; font-family: monospace; font-size: 11px;'
                          'background-color: #0d1117; border: 1px solid #30363d; border-radius: 4px; padding: 6px;')
        lay.addWidget(cam)
        hs = QtWidgets.QHBoxLayout()
        self.host_edit = QtWidgets.QLineEdit('192.168.0.100')
        self.sport_edit = QtWidgets.QLineEdit('5000')
        self.sport_edit.setMaximumWidth(70)
        self.stream_btn = QtWidgets.QPushButton('Connect AI-G video')
        self.stream_btn.setObjectName('actionBtn')
        self.stream_btn.clicked.connect(self._toggle_stream)
        self.stream_btn.setToolTip('AI-G 의 pend_cam --stream <포트> 에 TCP로 접속해 영상 원본을 받고, 마커 검출과 추적은 이 노트북에서 계산합니다.\n'
                                   '제어 경로(AI-G -> ESP32 UART)와는 무관한 보기 전용입니다.')
        hs.addWidget(QtWidgets.QLabel('AI-G:'))
        hs.addWidget(self.host_edit)
        hs.addWidget(self.sport_edit)
        hs.addWidget(self.stream_btn)
        lay.addLayout(hs)
        left.addWidget(grp)

        # 1b. laptop detector thresholds (same meaning as pend_cam --red / --blue)
        grp = QtWidgets.QGroupBox('1b. Laptop Detector (YUV, same as pend_cam)')
        form = QtWidgets.QFormLayout(grp)

        def ispin(attr, lo, hi, tip=''):
            sp = QtWidgets.QSpinBox()
            sp.setRange(lo, hi)
            sp.setValue(int(getattr(self.th, attr)))
            sp.valueChanged.connect(lambda v, a=attr: setattr(self.th, a, int(v)))
            if tip:
                sp.setToolTip(tip)
            return sp
        form.addRow('Red  V min:', ispin('r_vmin', 0, 255, 'pend_cam --red V최소,U최대,차이 의 첫 번째 값'))
        form.addRow('Red  U max:', ispin('r_umax', 0, 255))
        form.addRow('Red  V-U min:', ispin('r_diff', 0, 255))
        form.addRow('Blue U min:', ispin('b_umin', 0, 255, 'pend_cam --blue U최소,V최대,차이 의 첫 번째 값'))
        form.addRow('Blue V max:', ispin('b_vmax', 0, 255))
        form.addRow('Blue U-V min:', ispin('b_diff', 0, 255))
        form.addRow('Min blue area (macropx):', ispin('min_blue', 1, 500))
        form.addRow('Min red area (macropx):', ispin('min_red', 1, 500))
        self.recal_btn = QtWidgets.QPushButton('Re-calibrate rod radius')
        self.recal_btn.clicked.connect(lambda: self.receiver and self.receiver.reset_tracker())
        self.recal_btn.setToolTip('막대를 세워 마커 2개가 모두 보이는 상태에서 누르면 처음 30개의 깨끗한 프레임으로 끝/가운데 반지름을 다시 잽니다.')
        form.addRow(self.recal_btn)
        left.addWidget(grp)

        # 2. detection
        grp = QtWidgets.QGroupBox('2. Marker Detection')
        form = QtWidgets.QFormLayout(grp)
        self.spin_conf = dspin(0.0, 1.0, 0.50, 2, 0.05)
        self.spin_pad = dspin(0, 40, 14, 0, 1, ' px')
        self.spin_roi = dspin(20, 160, 48, 0, 4, ' px')
        self.spin_rtip = dspin(60, 200, R_TIP_PX, 0, 2, ' px')
        self.spin_rmid = dspin(20, 120, R_MID_PX, 0, 2, ' px')
        self.spin_noise = dspin(0.0, 6.0, 0.6, 2, 0.1, ' px')
        self.spin_pxm = dspin(200, 1500, 1000, 0, 25, ' px/m')
        self.spin_pxm.setToolTip('카트 위치 [센 m] -> 화면 x [px] 배율. 실제 AI-G 보정값은 약 1157 px/센 m 이지만, '
                                 '이 화면은 레일 한계(±0.20 m)가 프레임 안에 들어오게 1000 으로 둡니다.')
        form.addRow('Confidence threshold:', self.spin_conf)
        form.addRow('Box padding:', self.spin_pad)
        form.addRow('Search ROI half-size:', self.spin_roi)
        form.addRow('Rod radius (tip):', self.spin_rtip)
        form.addRow('Rod radius (mid):', self.spin_rmid)
        form.addRow('Detector noise (sigma):', self.spin_noise)
        form.addRow('Cart scale:', self.spin_pxm)
        left.addWidget(grp)

        # 3. tracker
        grp = QtWidgets.QGroupBox('3. Tracker, Latency & Fault Injection')
        form = QtWidgets.QFormLayout(grp)
        self.spin_delay = dspin(0, 60, 14, 1, 1, ' ms')
        self.spin_alpha = dspin(0.05, 1.0, 0.60, 2, 0.05)
        self.spin_beta = dspin(0.0, 1.0, 0.25, 2, 0.05)
        self.spin_gate = dspin(1, 45, 6, 1, 1, ' deg')
        self.spin_slope = dspin(-20, 20, -6.0, 1, 0.5, ' deg/m')
        self.spin_slope.setToolTip('camera_control 의 camXSlopeDegPerM. 카메라 각도는 카트 위치에 따라 약 +6 deg/m 편향되고 '
                                   '이 값(-6)을 위치에 곱해 더해 보정합니다.')
        self.chk_dropout = QtWidgets.QCheckBox('Inject marker dropout (cover tip)')
        self.chk_dropout.setChecked(True)
        self.chk_dropout.setToolTip('4초마다 0.3초 동안 끝 마커를 가립니다. 60 ms 넘게 데이터가 끊기면 펌웨어는 disarm 합니다.')
        self.chk_bias = QtWidgets.QCheckBox('Camera angle bias vs cart x (+6 deg/m)')
        self.chk_bias.setChecked(True)
        form.addRow('Camera delay:', self.spin_delay)
        form.addRow('Tracker alpha:', self.spin_alpha)
        form.addRow('Tracker beta:', self.spin_beta)
        form.addRow('Outlier gate:', self.spin_gate)
        form.addRow('Camera x correction:', self.spin_slope)
        form.addRow(self.chk_dropout)
        form.addRow(self.chk_bias)
        left.addWidget(grp)

        # 4. execution
        grp = QtWidgets.QGroupBox('4. Replay & Safety')
        lay = QtWidgets.QVBoxLayout(grp)
        h = QtWidgets.QHBoxLayout()
        self.swing_btn = QtWidgets.QPushButton('🚀 Replay Swing-up (camera)')
        self.swing_btn.setObjectName('primaryBtn')
        self.swing_btn.clicked.connect(self._cmd_swingup)
        self.swing_btn.setToolTip('오프라인: 스윙업 -> 캐치 -> LQR 시뮬레이션을 카메라 화면으로 다시 재생합니다.\n'
                                  '연결 중: 하드웨어에는 아무것도 보내지 않습니다(제어는 ESP32 패널에서).')
        self.balance_btn = QtWidgets.QPushButton('⚖ Replay Balance (camera)')
        self.balance_btn.clicked.connect(self._cmd_balance)
        self.balance_btn.setToolTip('오프라인: 수직 근처(10도)에서 시작하는 밸런스 재생. 연결 중에는 아무 일도 하지 않습니다.')
        h.addWidget(self.swing_btn)
        h.addWidget(self.balance_btn)
        lay.addLayout(h)
        self.stop_btn = QtWidgets.QPushButton('⛔ Emergency Disarm / Stop')
        self.stop_btn.setObjectName('dangerBtn')
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.clicked.connect(self._cmd_stop)
        self.stop_btn.setToolTip('연결 중이면 ESP32 를 SAFE 모드(0x50, 0)로 보냅니다. 오프라인이면 재생을 일시정지/재개합니다.')
        lay.addWidget(self.stop_btn)
        self.shot_btn = QtWidgets.QPushButton('Save Frame -> PNG')
        self.shot_btn.clicked.connect(self._save_frame)
        self.shot_btn.setToolTip('지금 보이는 카메라 화면(박스 포함)을 camera_logs/ 에 PNG 로 저장합니다.')
        lay.addWidget(self.shot_btn)
        self.shot_label = QtWidgets.QLabel('')
        self.shot_label.setWordWrap(True)
        self.shot_label.setStyleSheet('color: #8b949e; font-family: monospace; font-size: 10px;')
        lay.addWidget(self.shot_label)
        left.addWidget(grp)

        # 5. ESP32 operator keys: the same keys as tools/pipeline_serial_windows.py (terminal 2)
        grp = QtWidgets.QGroupBox('5. ESP32 Operator  (replaces terminal 2 while connected)')
        lay = QtWidgets.QVBoxLayout(grp)
        grid = QtWidgets.QGridLayout()
        for i, (txt, key, tip) in enumerate((
                ('O  Zero cart', 'O', '카트를 레일 가운데에 둔 뒤 카트 위치 영점'),
                ('Z  Zero encoder', 'Z', '진자를 아래로 늘어뜨려 완전히 멈춘 뒤 엔코더 영점'),
                ('E  Stats on/off', 'E', '# SH 상태 줄 켜기/끄기 (v 가 약 90 이면 정상)'),
                ('C  Source cam/enc', 'C', '밸런스 소스 전환. 정지 상태에서만 가능'))):
            b = QtWidgets.QPushButton(txt)
            b.setToolTip(tip)
            b.clicked.connect(lambda _=False, k=key: self._send_key(k))
            grid.addWidget(b, i // 2, i % 2)
        lay.addLayout(grid)
        self.chk_motion = QtWidgets.QCheckBox('Allow motion buttons (S / A)')
        self.chk_motion.setToolTip('체크해야 S / A 버튼이 동작합니다. 실수로 모터가 움직이는 것을 막는 잠금입니다.')
        lay.addWidget(self.chk_motion)
        h = QtWidgets.QHBoxLayout()
        sb = QtWidgets.QPushButton('S  Swing-up')
        sb.setObjectName('primaryBtn')
        sb.clicked.connect(lambda: self._send_key('S', motion=True))
        ab = QtWidgets.QPushButton('A  Balance')
        ab.clicked.connect(lambda: self._send_key('A', motion=True))
        h.addWidget(sb)
        h.addWidget(ab)
        lay.addLayout(h)
        xb = QtWidgets.QPushButton('⛔  X  STOP   (Space / Esc)')
        xb.setObjectName('dangerBtn')
        xb.setMinimumHeight(46)
        xb.clicked.connect(lambda: self._send_key('X'))
        lay.addWidget(xb)
        for seq in ('Space', 'Esc'):
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(seq), self)
            sc.setContext(QtCore.Qt.ApplicationShortcut)
            sc.activated.connect(lambda: self._send_key('X'))
        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(400)
        self.log_box.setMinimumHeight(170)
        self.log_box.setStyleSheet('background-color: #0d1117; color: #7ee787; font-family: Consolas, monospace;'
                                   'font-size: 11px; border: 1px solid #30363d;')
        lay.addWidget(self.log_box)
        left.addWidget(grp)
        left.addStretch()

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(left_pane)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(500)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        main_layout.addWidget(scroll)

        # ---------------- right column ----------------
        right_pane = QtWidgets.QWidget()
        right = QtWidgets.QVBoxLayout(right_pane)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(8)

        self.view = CameraViewWidget()
        right.addWidget(self.view, stretch=3)

        opts = QtWidgets.QHBoxLayout()
        self._opt_checks = {}
        for key, text, on in (('boxes', 'Boxes', True), ('labels', 'Labels', True), ('trail', 'Trail', True),
                              ('predict', 'Prediction', True), ('fan', 'Catch fan', True),
                              ('motion', 'Motion cues', True), ('roi', 'Search ROI', True),
                              ('rodbox', 'Rod box', True), ('hud', 'HUD', True)):
            cb = QtWidgets.QCheckBox(text)
            cb.setChecked(on)
            cb.toggled.connect(lambda v, k=key: self.view.set_option(k, v))
            opts.addWidget(cb)
        opts.addStretch()
        right.addLayout(opts)

        flips = QtWidgets.QHBoxLayout()
        self.chk_flip_cart = QtWidgets.QCheckBox('Flip cart left/right (live)')
        self.chk_flip_cart.setChecked(True)
        self.chk_flip_cart.setToolTip('하드웨어 + 방향과 화면 + 방향(오른쪽)이 반대일 때 카트 위치 부호를 뒤집어 표시합니다.')
        self.chk_flip_tilt = QtWidgets.QCheckBox('Flip pendulum tilt (live)')
        self.chk_flip_tilt.setToolTip('진자가 기우는 방향이 실제와 반대로 보이면 체크하세요.')
        flips.addWidget(self.chk_flip_cart)
        flips.addWidget(self.chk_flip_tilt)
        flips.addStretch()
        right.addLayout(flips)

        grid_widget = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(grid_widget)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(6)
        pg.setConfigOption('background', '#161b22')
        pg.setConfigOption('foreground', '#8b949e')
        self.plot_angle = pg.PlotWidget(title='Camera Angle (deg) [Upright = 0]')
        self.plot_conf = pg.PlotWidget(title='Detection Confidence [per marker]')
        self.plot_x = pg.PlotWidget(title='Cart x in Frame (px)')
        self.plot_lat = pg.PlotWidget(title='Packet Age & Processing Time (ms)')
        for pl in (self.plot_angle, self.plot_conf, self.plot_x, self.plot_lat):
            pl.showGrid(x=True, y=True, alpha=0.3)
            pl.setLabel('bottom', 'Time (s)')
            pl.addLegend(offset=(5, 5))
        self.plot_angle.addItem(pg.InfiniteLine(pos=CATCH_WINDOW_DEG, angle=0, pen=pg.mkPen('#2ea043', width=1, style=QtCore.Qt.DashLine)))
        self.plot_angle.addItem(pg.InfiniteLine(pos=-CATCH_WINDOW_DEG, angle=0, pen=pg.mkPen('#2ea043', width=1, style=QtCore.Qt.DashLine)))
        self.plot_conf.setYRange(0, 1.05)
        self.thr_line = pg.InfiniteLine(pos=0.5, angle=0, pen=pg.mkPen('#f85149', width=1, style=QtCore.Qt.DashLine))
        self.plot_conf.addItem(self.thr_line)
        self.cv_true = self.plot_angle.plot([], [], pen=pg.mkPen('#8b949e', width=1.5, style=QtCore.Qt.DashLine), name='encoder (truth)')
        self.cv_meas = self.plot_angle.plot([], [], pen=pg.mkPen('#58a6ff', width=1), name='camera (delayed)')
        self.cv_est = self.plot_angle.plot([], [], pen=pg.mkPen('#f1e05a', width=2), name='tracked estimate')
        self.cv_cart = self.plot_conf.plot([], [], pen=pg.mkPen('#ff7b72', width=2), name='CART')
        self.cv_mid = self.plot_conf.plot([], [], pen=pg.mkPen('#d2a8ff', width=1.5), name='MID')
        self.cv_tip = self.plot_conf.plot([], [], pen=pg.mkPen('#f1e05a', width=2), name='TIP')
        self.cv_cxm = self.plot_x.plot([], [], pen=pg.mkPen('#58a6ff', width=1), name='measured')
        self.cv_cxe = self.plot_x.plot([], [], pen=pg.mkPen('#79c0ff', width=2), name='tracked')
        self.cv_age = self.plot_lat.plot([], [], pen=pg.mkPen('#ff7b72', width=2), name='packet age')
        self.cv_proc = self.plot_lat.plot([], [], pen=pg.mkPen('#7ee787', width=2), name='board processing')
        grid.addWidget(self.plot_angle, 0, 0)
        grid.addWidget(self.plot_conf, 0, 1)
        grid.addWidget(self.plot_x, 1, 0)
        grid.addWidget(self.plot_lat, 1, 1)
        right.addWidget(grid_widget, stretch=2)
        main_layout.addWidget(right_pane, stretch=1)

    # ------------------------------------------------------------------ simulation source
    def run_simulation(self, initial_angle_deg=175.0):
        K, _, _, _ = solve_riccati_lqr(0.20, 0.04, 9.81)
        self._sim = simulate_full_pipeline(
            length_m=0.20, friction=0.04, g=9.81, K_lqr=K,
            swing_speed=1.20, swing_accel=8.0, cart_limit=0.20,
            max_speed=0.80, max_accel=12.0, dt=0.002, total_time_s=10.0,
            initial_angle_deg=float(initial_angle_deg))
        self._sim_idx = 0
        self._sim_wall_t = 0.0
        self._reset_tracking()

    def _reset_tracking(self):
        for t in self._trk.values():
            t.reset()
        self.view.clear_trail()
        for b in self._buf.values():
            b.clear()
        self._lost_since = None
        self._t0 = time.monotonic()

    def _cmd_swingup(self):
        if self.serial is None:
            self.run_simulation(175.0)
        else:
            self.status_label.setText('Live: no command sent. Start swing-up from the ESP32 control panel.')

    def _cmd_balance(self):
        if self.serial is None:
            self.run_simulation(10.0)
        else:
            self.status_label.setText('Live: no command sent. Start balance from the ESP32 control panel.')

    def _cmd_stop(self):
        if self.serial is None:
            if self.timer.isActive():
                self.timer.stop()
                self.stop_btn.setText('▶ Resume Replay')
            else:
                self.timer.start(33)
                self.stop_btn.setText('⛔ Emergency Disarm / Stop')
            return
        self._send_key('X')
        self.status_label.setText('TX: X (stop) sent.')

    def _save_frame(self):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_logs')
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, datetime.datetime.now().strftime('frame_%Y%m%d_%H%M%S.png'))
        self.view.grab().save(path)
        self.shot_label.setText(f'saved: {path}')

    # ------------------------------------------------------------------ serial (live)
    def _toggle_connection(self):
        if self.serial is None:
            port = self.port_edit.text().strip()
            log_path = Path.home() / 'equilibrium_logs' / datetime.datetime.now().strftime('camview_%Y%m%d_%H%M%S_%f.log')
            try:
                link = SerialLink(port, self._on_packet_received, self._on_line, self._on_link_error, log_path=log_path)
                link.open()
                self.serial = link
                self.connect_btn.setText('Disconnect')
                self.connect_btn.setObjectName('dangerBtn')
                self._restyle(self.connect_btn)
                self.status_label.setText(f'Connected to {port}. Waiting for telemetry...')
                self._rx_count = 0
                self._cam_t = 0.0
                self._link_err = None
                self._reset_tracking()
                self._log_line(f'connected {port}  (raw log: {log_path.name})')
                self._log_line('The ESP32 may restart on connect: press Z again if it says "zero downward".')
            except Exception as e:
                self.serial = None
                hint = ''
                if 'denied' in str(e).lower() or 'permission' in str(e).lower() or 'could not open' in str(e).lower():
                    hint = ('\n\nThe port is probably used by another program. Close the terminal-2 serial tool '
                            '(press q there), PlatformIO monitor, or any other viewer, then try again.')
                QtWidgets.QMessageBox.critical(self, 'Connection Error', str(e) + hint)
        else:
            self._disconnect_serial('disconnected (X stop sent first)')

    def _disconnect_serial(self, note):
        link, self.serial = self.serial, None
        try:
            if link is not None:
                link.close()                 # sends X twice before closing, like the terminal tool
        finally:
            self.connect_btn.setText('Connect ESP32')
            self.connect_btn.setObjectName('actionBtn')
            self._restyle(self.connect_btn)
            self.status_label.setText('Status: Offline (Simulated Camera Active)')
            self._log_line(note)
            self._reset_tracking()

    def _on_link_error(self, msg):           # serial thread
        self._link_err = msg

    def _on_line(self, text):                # serial thread
        self._lines.append(text)

    def _on_packet_received(self, addr, value):   # serial thread
        self._rx_count += 1
        if addr == 0x00:
            self._live['angle'] = value
        elif addr == 0x02:
            self._live['pos'] = value
        elif addr == 0x06:
            self._live['mode'] = int(value)
        elif 0x10 <= addr <= 0x19:
            self._cam[addr] = value
            if addr == 0x19:                 # last frame of the camera group
                self._cam_t = time.monotonic()
                self._cam_ver += 1

    # ------------------------------------------------------------------ operator keys + log
    def _log_line(self, text):
        self.log_box.appendPlainText(datetime.datetime.now().strftime('%H:%M:%S ') + text)

    def _send_key(self, key, motion=False):
        if self.serial is None:
            self._log_line(f'{key}: not connected (press Connect ESP32 first)')
            return
        if motion and not self.chk_motion.isChecked():
            self._log_line(f'{key}: blocked - tick "Allow motion buttons" first')
            return
        try:
            for _ in range(2 if key == 'X' else 1):
                self.serial.send_key(key)
            self._log_line(f'TX {key}')
        except Exception as exc:
            self._log_line(f'TX {key} FAILED: {exc}  (if the cart moves, use the driver power switch)')

    def _drain_lines(self):
        if self._link_err:
            err, self._link_err = self._link_err, None
            if self.serial is not None:
                self._disconnect_serial(f'LINK LOST: {err}  (motor stop NOT confirmed - use the driver power switch if needed)')
            return
        for _ in range(200):
            try:
                ln = self._lines.popleft()
            except IndexError:
                break
            if ln.startswith('# REC'):
                if ln.startswith('# REC end'):
                    self._log_line(f'REC dump received ({self._rec_n} samples, saved in the raw log)')
                    self._rec_n = 0
                elif ln.startswith('# REC none'):
                    self._log_line('REC: none recorded')
                elif ln.startswith('# REC n'):          # header line of a dump
                    self._rec_n = 0
                else:
                    self._rec_n += 1
                continue
            if ln.startswith('# SH'):
                self._sh_line = ln
                continue
            self._log_line(ln)
            low = ln.lower()
            if low.startswith('ok: balance source'):
                self._source_text = ln.split('=')[-1].strip()
            elif low.startswith('=== camera control') or low.startswith('==='):
                self._log_line('(ESP32 restarted - press Z, O again)')

    # ------------------------------------------------------------------ camera data relayed over USB
    def _cam_active(self):
        return self.serial is not None and (time.monotonic() - self._cam_t) < 1.5

    def _geometry_px(self, pivot, ang_deg):
        """Marker pixels from pivot + camera angle (image convention: 0 = up, + = right) and rod radii."""
        a = math.radians(ang_deg)
        rt, rm = self.spin_rtip.value(), self.spin_rmid.value()
        return {'pivot': pivot,
                'mid': (pivot[0] + rm * math.sin(a), pivot[1] - rm * math.cos(a)),
                'tip': (pivot[0] + rt * math.sin(a), pivot[1] - rt * math.cos(a))}

    def _tick_camera(self):
        if self._cam_ver == self._cam_seen:
            return
        self._cam_seen = self._cam_ver
        c = dict(self._cam)
        if self._names_mode != 'cam':
            self._set_names('cam')
            self._reset_tracking()
            self._last_cam_t = None
        t_now = self._cam_t
        dt = 0.04 if self._last_cam_t is None else max(0.005, min(0.25, t_now - self._last_cam_t))
        self._last_cam_t = t_now

        def val(addr, default=math.nan):
            v = c.get(addr, default)
            return v
        flags_f = val(0x13, 0.0)
        flags = int(flags_f) if flags_f == flags_f else 0
        raw, px, py = val(0x10), val(0x11), val(0x12)
        age, since = val(0x14, 0.0), val(0x19, 0.0)
        pkt_ok, crc = val(0x17, 0.0), val(0x18, 0.0)
        est_phi = val(0x15)
        lost_link = since > CAM_TIMEOUT_MS
        have_p = bool(flags & 8) and px == px and py == py and not lost_link
        a_ok = raw == raw and not lost_link
        meas = {'pivot': (px, py) if have_p else None, 'mid': None, 'tip': None}
        if have_p and a_ok:
            g = self._geometry_px((px, py), raw)
            if flags & 2:
                meas['tip'] = g['tip']
            if flags & 4:
                meas['mid'] = g['mid']
        if have_p:
            self._last_pivot = (px, py)
        if a_ok:
            self._last_ang = raw
        true = self._geometry_px(self._last_pivot, self._last_ang)
        conf = {k: (0.99 if meas[k] is not None else 0.0) for k in ('pivot', 'mid', 'tip')}

        gate_px = max(10.0, math.radians(self.spin_gate.value()) * self.spin_rtip.value() * 1.5)
        alpha, beta = self.spin_alpha.value(), self.spin_beta.value()
        lag_s = self.spin_delay.value() / 1000.0
        est, vel, accepted = {}, {}, {}
        for key in ('pivot', 'mid', 'tip'):
            trk = self._trk[key]
            accepted[key] = trk.step(dt, meas[key], alpha, beta, gate_px)
            if trk.init:
                est[key] = trk.ahead(lag_s)
                vel[key] = (trk.vx, trk.vy)
            else:
                est[key] = None
                vel[key] = (0.0, 0.0)
            self._conf_ema[key] += 0.5 * (conf[key] - self._conf_ema[key])

        pad = self.spin_pad.value()
        dets = []
        for key, name, color, did, rr in (('pivot', 'CART', RED, 1, 11), ('mid', 'MID', YELLOW, 2, 9),
                                          ('tip', 'TIP', YELLOW, 3, 11)):
            if est[key] is None:
                continue
            cxp, cyp = self._trk[key].x, self._trk[key].y
            e = est[key]
            half = rr + pad
            lost = (not accepted[key]) and self._trk[key].age_s > 0.02
            dets.append({'name': name, 'id': did, 'color': color, 'conf': self._conf_ema[key],
                         'center': (cxp, cyp), 'pred': e if math.hypot(e[0] - cxp, e[1] - cyp) > 3 else None,
                         'box': (cxp - half, cyp - half, 2 * half, 2 * half), 'lost': lost})

        if lost_link:
            banner = f'CAMERA DATA LOST {since:.0f} ms > {CAM_TIMEOUT_MS:.0f} ms : DISARM'
        elif flags & 0x10:
            banner = 'CALIBRATING ROD RADIUS (pend_cam)'
        elif have_p and not (flags & 2):
            banner = 'TIP MARKER NOT USED'
        else:
            banner = ''
        angle_est = math.nan
        if est['pivot'] and est['tip']:
            angle_est = self._angle_of(est['pivot'], est['tip'])
        a_show = angle_est if angle_est == angle_est else 0.0
        enc_img = -math.degrees(self._live['angle'])         # encoder angle in the camera's image convention
        mode = self._live['mode']
        frame = {
            'true': true, 'meas': meas, 'est': est, 'vel': vel, 'dets': dets,
            'frame': int(pkt_ok) & 0xFFFF, 'proc_ms': 0.0, 'age_ms': float(age) if age == age else 0.0,
            'angle_est': a_show, 'cart_x_px': est['pivot'][0] if est['pivot'] else 0.0,
            'flags': [bool(flags & (1 << i)) for i in range(6)],
            'link_ok': not lost_link, 'banner': banner,
            'mode_text': f"{MODE_NAMES.get(mode, '?')}  src {self._source_text}",
            'blur': False, 'roi': self.spin_roi.value(), 'in_window': abs(a_show) < CATCH_WINDOW_DEG,
            'hud1': 'ESP32 USB  camera data relayed (25 Hz)',
            'hud2': f"pkts {int(pkt_ok)}  crc-fail {int(crc)}  age {age:.1f} ms  last {since:.0f} ms",
        }
        self.view.set_frame(frame)
        phi_txt = f"  fw-est {-math.degrees(est_phi):+.1f}°" if est_phi == est_phi else ''
        self.status_label.setText(
            f"ESP32 | {MODE_NAMES.get(mode, '?')} | src {self._source_text} | cam {a_show:+.1f}° enc {enc_img:+.1f}°"
            f"{phi_txt} | age {age:.1f} ms")
        self._push_graphs(False, enc_img, raw, angle_est, meas, est, conf, age if age == age else 0.0,
                          since if since == since else 0.0)

    # ------------------------------------------------------------------ AI-G video stream (laptop vision)
    @staticmethod
    def _restyle(btn):
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    def _toggle_stream(self):
        if self.receiver is None:
            try:
                port = int(self.sport_edit.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(self, 'Port', 'Port must be a number.')
                return
            self.receiver = StreamReceiver(self.host_edit.text().strip(), port, self.th, self)
            self.receiver.status.connect(self._on_stream_status)
            self.receiver.finished.connect(self._on_stream_finished)
            self._rx_ver = 0
            self._last_stream_t = None
            self.receiver.start()
            self.stream_btn.setText('Disconnect AI-G video')
            self.stream_btn.setObjectName('dangerBtn')
            self._restyle(self.stream_btn)
            self.status_label.setText(f'Connecting to AI-G {self.host_edit.text().strip()}:{port} ...')
        else:
            self.stream_btn.setText('Disconnecting...')
            self.receiver.stop()

    def _on_stream_status(self, text):
        self.status_label.setText(text)

    def _on_stream_finished(self):
        if self.sender() is self.receiver:
            self.receiver = None
        self.stream_btn.setText('Connect AI-G video')
        self.stream_btn.setObjectName('actionBtn')
        self._restyle(self.stream_btn)

    def closeEvent(self, ev):
        if self.serial is not None:
            try:
                self.serial.close()          # X (stop) is sent before the port closes
            except Exception:
                pass
            self.serial = None
        if self.receiver is not None:
            self.receiver.stop()
            self.receiver.wait(1500)
        super().closeEvent(ev)

    def _stream_active(self):
        r = self.receiver
        return r is not None and r.latest is not None and (time.monotonic() - r.latest['t_rx']) < 1.5

    def _set_names(self, mode):
        if mode == self._names_mode:
            return
        self._names_mode = mode
        names = {'sim': ('encoder (truth)', 'camera (delayed)', 'tracked estimate', 'packet age', 'board processing'),
                 'stream': ('AI-G on-board angle', 'laptop raw angle', 'laptop tracked', 'frame interval',
                            'laptop processing'),
                 'cam': ('encoder (image convention)', 'camera raw angle', 'tracked estimate', 'packet age',
                         'ms since last packet')}[mode]
        for pl, cv, nm in ((self.plot_angle, self.cv_true, names[0]), (self.plot_angle, self.cv_meas, names[1]),
                           (self.plot_angle, self.cv_est, names[2]), (self.plot_lat, self.cv_age, names[3]),
                           (self.plot_lat, self.cv_proc, names[4])):
            try:
                pl.plotItem.legend.getLabel(cv).setText(nm)
            except Exception:
                pass
        self.plot_lat.setTitle('Frame Interval & Laptop Processing (ms)' if mode == 'stream'
                               else 'Packet Age & Processing Time (ms)')
        self.plot_angle.setTitle('Camera Angle (deg) [Upright = 0]  -  laptop vision' if mode == 'stream'
                                 else 'Camera Angle (deg) [Upright = 0]')

    def _tick_stream(self):
        r = self.receiver
        if r.version == self._rx_ver:
            return                                   # no new frame since the last tick
        self._rx_ver = r.version
        d = r.latest
        hdr, tr = d['hdr'], d['track']
        if self._names_mode != 'stream':
            self._set_names('stream')
            self._reset_tracking()
            self._last_stream_t = None

        t_rx = d['t_rx']
        dt = 0.066 if self._last_stream_t is None else max(0.004, min(0.25, t_rx - self._last_stream_t))
        self._last_stream_t = t_rx

        def xy(b):
            return (b['x'], b['y']) if b else None
        meas = {'pivot': xy(d['piv']), 'mid': xy(tr['mid']), 'tip': xy(tr['tip'])}
        blob = {'pivot': d['piv'], 'mid': tr['mid'], 'tip': tr['tip']}
        conf = {}
        for key in ('pivot', 'mid', 'tip'):
            b = blob[key]
            if b is None:
                conf[key] = 0.0
                continue
            ref = self._n_ref[key]
            if ref is None:
                ref = float(b['n'])
            elif b['n'] > 0.5 * ref:
                ref += 0.05 * (b['n'] - ref)         # slow reference for the blob area
            self._n_ref[key] = ref
            conf[key] = float(min(0.99, b['n'] / max(ref, 1.0)))

        lt = d['Lt'] if d['calibrated'] else 150.0
        gate_px = max(10.0, math.radians(self.spin_gate.value()) * lt * 1.5)
        alpha, beta = self.spin_alpha.value(), self.spin_beta.value()
        lag_s = self.spin_delay.value() / 1000.0
        est, vel, accepted = {}, {}, {}
        for key in ('pivot', 'mid', 'tip'):
            trk = self._trk[key]
            accepted[key] = trk.step(dt, meas[key], alpha, beta, gate_px)
            if trk.init:
                est[key] = trk.ahead(lag_s)
                vel[key] = (trk.vx, trk.vy)
            else:
                est[key] = None
                vel[key] = (0.0, 0.0)
            self._conf_ema[key] += 0.5 * (conf[key] - self._conf_ema[key])

        pad = self.spin_pad.value()
        dets = []
        for key, name, color, did, rr in (('pivot', 'CART', RED, 1, 11), ('mid', 'MID', YELLOW, 2, 9),
                                          ('tip', 'TIP', YELLOW, 3, 11)):
            e = est[key]
            if e is None:
                continue
            cxp, cyp = self._trk[key].x, self._trk[key].y
            half = rr + pad
            lost = (not accepted[key]) and self._trk[key].age_s > 0.02
            dets.append({'name': name, 'id': did, 'color': color, 'conf': self._conf_ema[key],
                         'center': (cxp, cyp), 'pred': e if math.hypot(e[0] - cxp, e[1] - cyp) > 3 else None,
                         'box': (cxp - half, cyp - half, 2 * half, 2 * half), 'lost': lost})

        self._lost_frames = self._lost_frames + 1 if meas['tip'] is None else 0
        if tr['calibrating']:
            banner = f"CALIBRATING ROD RADIUS {len(r.trk.dt)}/{self.th.cal_n}"
        elif tr['no_pivot']:
            banner = 'NO RED MARKER (pivot) FOUND'
        elif self._lost_frames >= 2:
            banner = f'TIP MARKER LOST  {self._lost_frames} frames'
        else:
            banner = ''

        angle_est = math.nan
        if est['pivot'] and est['tip']:
            angle_est = self._angle_of(est['pivot'], est['tip'])
        a_show = angle_est if not math.isnan(angle_est) else 0.0
        board_ang = hdr['ang']
        rgb = d['rgb']
        hh, ww = rgb.shape[0], rgb.shape[1]
        image = QtGui.QImage(rgb.data, ww, hh, 3 * ww, QtGui.QImage.Format_RGB888).copy()
        fps = 1000.0 / d['interval_ms'] if d['interval_ms'] and not math.isnan(d['interval_ms']) else 0.0
        true = {k: est[k] or meas[k] for k in ('pivot', 'mid', 'tip')}
        frame = {
            'image': image, 'true': true, 'meas': meas, 'est': est, 'vel': vel, 'dets': dets,
            'frame': hdr['seq'], 'proc_ms': d['proc_ms'], 'age_ms': d['interval_ms'] if not math.isnan(d['interval_ms']) else 0.0,
            'angle_est': a_show, 'cart_x_px': est['pivot'][0] if est['pivot'] else 0.0,
            'flags': [meas['tip'] is not None, meas['tip'] is not None, meas['mid'] is not None,
                      meas['pivot'] is not None, tr['calibrating'], not math.isnan(hdr['proc_ms'])],
            'link_ok': r.connected, 'banner': banner,
            'mode_text': 'AI-G video -> laptop vision', 'blur': False, 'roi': self.spin_roi.value(),
            'in_window': abs(a_show) < CATCH_WINDOW_DEG,
            'hud1': f'AI-G STREAM  {ww}x{hh}  rx {fps:4.1f} fps  laptop vision',
            'hud2': f"frame #{hdr['seq'] & 0xFFFF:05d}  laptop {d['proc_ms']:.1f} ms  board {hdr['proc_ms']:.1f} ms",
        }
        self.view.set_frame(frame)

        diff = ''
        if not math.isnan(board_ang) and not math.isnan(angle_est):
            diff = f" | board {board_ang:+.1f}° Δ{abs(((angle_est - board_ang + 180) % 360) - 180):.1f}°"
        self.status_label.setText(
            f"AI-G STREAM | {fps:.1f} fps | est {a_show:+.1f}°{diff} | rod R {d['Lt']:.0f}/{d['Lm']:.0f}px"
            f"{'' if d['calibrated'] else ' (calibrating)'} | seq step {d['seq_step']}")
        self._push_graphs(False, board_ang, tr['ang'], angle_est, meas, est, conf, d['proc_ms'],
                          d['interval_ms'] if not math.isnan(d['interval_ms']) else 0.0)

    # ------------------------------------------------------------------ geometry
    def _geometry(self, angle_deg, pos_m):
        """True marker pixels from pendulum state. Upright = 0, + = tilting right on screen."""
        rt, rm = self.spin_rtip.value(), self.spin_rmid.value()
        px_per_m = self.spin_pxm.value()
        rail = 0.20
        px = FRAME_W / 2.0 + max(-rail * 1.3, min(rail * 1.3, pos_m)) * px_per_m
        a = math.radians(angle_deg)
        pv = (px, PIVOT_Y)
        return {'pivot': pv,
                'mid': (px + rm * math.sin(a), PIVOT_Y - rm * math.cos(a)),
                'tip': (px + rt * math.sin(a), PIVOT_Y - rt * math.cos(a))}

    @staticmethod
    def _angle_of(pv, tp):
        return math.degrees(math.atan2(tp[0] - pv[0], -(tp[1] - pv[1])))

    # ------------------------------------------------------------------ per-tick update
    def _on_tick(self):
        self._drain_lines()
        if self._cam_active():
            self._tick_camera()
            return
        if self._stream_active():
            self._tick_stream()
            return
        if self._names_mode != 'sim':
            self._set_names('sim')
            self._reset_tracking()
        dt = 0.033
        live = self.serial is not None and self._rx_count > 0
        sim_t = None
        if live:
            sc = -1.0 if self.chk_flip_cart.isChecked() else 1.0
            st = -1.0 if self.chk_flip_tilt.isChecked() else 1.0
            true_ang = st * math.degrees(self._live['angle'])
            pos_m = sc * self._live['pos']
            true_ang = wrap_deg(true_ang)
            mode = self._live['mode']
            lag_ang, lag_pos = true_ang, pos_m
        else:
            res = self._sim
            n = len(res['time'])
            step = max(1, int(dt / 0.002))
            self._sim_idx += step
            if self._sim_idx >= n:
                self._sim_idx = 0
                self._reset_tracking()
            i = self._sim_idx
            sim_t = float(res['time'][i])
            true_ang = float(res['theta_deg'][i])
            pos_m = float(res['pos_mm'][i]) / 1000.0
            mode = 1 + int(res['mode'][i])
            li = max(0, i - int(self.spin_delay.value() / 1000.0 / 0.002))
            lag_ang = float(res['theta_deg'][li])
            lag_pos = float(res['pos_mm'][li]) / 1000.0
            rate = float(res['rate'][i]) if 'rate' in res else 0.0

        true_g = self._geometry(true_ang, pos_m)
        lag_g = self._geometry(lag_ang, lag_pos)
        rate_deg = 0.0
        if not live:
            rate_deg = math.degrees(float(self._sim['rate'][self._sim_idx]))

        # ---- detector: delayed, noisy pixel measurements (+ injected dropout of the tip marker)
        sigma = self.spin_noise.value()
        conf_thr = self.spin_conf.value()
        speed_blur = min(0.35, abs(rate_deg) / 900.0) if not live else 0.0
        covered = False
        if sim_t is not None and self.chk_dropout.isChecked():
            covered = (sim_t % 4.0) > 3.4 and sim_t > 1.0
        meas, conf = {}, {}
        for key in ('pivot', 'mid', 'tip'):
            c = 0.985 - speed_blur * (1.0 if key != 'pivot' else 0.3) + self._rng.normal(0, 0.012)
            if key == 'tip' and covered:
                c = 0.0
            if sim_t is not None and key == 'mid' and sim_t > 1.0 and (sim_t % 6.7) > 6.55:
                c = 0.0
            c = float(max(0.0, min(0.999, c)))
            conf[key] = c
            if c >= conf_thr:
                gx, gy = lag_g[key]
                meas[key] = (gx + self._rng.normal(0, sigma), gy + self._rng.normal(0, sigma))
            else:
                meas[key] = None

        # camera angle bias vs cart position (what camXSlopeDegPerM corrects)
        bias_deg = 6.0 * lag_pos if self.chk_bias.isChecked() else 0.0
        if bias_deg and meas['tip'] is not None and meas['pivot'] is not None:
            a = math.radians(bias_deg)
            dx, dy = meas['tip'][0] - meas['pivot'][0], meas['tip'][1] - meas['pivot'][1]
            meas['tip'] = (meas['pivot'][0] + dx * math.cos(a) - dy * math.sin(a),
                           meas['pivot'][1] + dx * math.sin(a) + dy * math.cos(a))

        # ---- tracker
        alpha, beta = self.spin_alpha.value(), self.spin_beta.value()
        gate_px = max(10.0, math.radians(self.spin_gate.value()) * self.spin_rtip.value() * 1.5)
        lag_s = self.spin_delay.value() / 1000.0
        est, vel, accepted = {}, {}, {}
        for key in ('pivot', 'mid', 'tip'):
            trk = self._trk[key]
            accepted[key] = trk.step(dt, meas[key], alpha, beta, gate_px)
            if trk.init:
                est[key] = trk.ahead(lag_s)          # lag-compensated: where the marker is NOW
                vel[key] = (trk.vx, trk.vy)
            else:
                est[key] = None
                vel[key] = (0.0, 0.0)
        for key in ('pivot', 'mid', 'tip'):
            self._conf_ema[key] += 0.5 * (conf[key] - self._conf_ema[key])

        # ---- guard: no accepted tip packet for CAM_TIMEOUT_MS
        age_tip_ms = self._trk['tip'].age_s * 1000.0
        banner = ''
        if self._trk['tip'].init and age_tip_ms > CAM_TIMEOUT_MS:
            banner = f'CAMERA DATA LOST {age_tip_ms:.0f} ms > {CAM_TIMEOUT_MS:.0f} ms : DISARM'
        elif self._trk['tip'].init and age_tip_ms > 0.5 * CAM_TIMEOUT_MS:
            banner = f'tip marker lost {age_tip_ms:.0f} ms'

        # ---- build detections
        pad = self.spin_pad.value()
        defs = (('pivot', 'CART', RED, 1, 11), ('mid', 'MID', YELLOW, 2, 9), ('tip', 'TIP', YELLOW, 3, 11))
        dets = []
        for key, name, color, did, r in defs:
            e = est[key]
            if e is None:
                continue
            # the box is centred on the tracked position of the marker as it is now
            cxp, cyp = self._trk[key].x, self._trk[key].y
            half = r + pad
            lost = (not accepted[key]) and self._trk[key].age_s > 0.02
            dets.append({'name': name, 'id': did, 'color': color, 'conf': self._conf_ema[key],
                         'center': (cxp, cyp), 'pred': e if (math.hypot(e[0] - cxp, e[1] - cyp) > 3) else None,
                         'box': (cxp - half, cyp - half, 2 * half, 2 * half), 'lost': lost})

        # ---- numbers for HUD / graphs
        angle_est = true_ang
        angle_meas = float('nan')
        if est['pivot'] and est['tip']:
            angle_est = self._angle_of(est['pivot'], est['tip']) + self.spin_slope.value() * lag_pos \
                if self.chk_bias.isChecked() else self._angle_of(est['pivot'], est['tip'])
            angle_est = wrap_deg(angle_est)
        if meas['pivot'] and meas['tip']:
            angle_meas = wrap_deg(self._angle_of(meas['pivot'], meas['tip']))
        cart_x_px = est['pivot'][0] if est['pivot'] else true_g['pivot'][0]
        proc_ms = 7.56 + float(self._rng.normal(0, 0.08))
        age_ms = 7.6 + float(self._rng.normal(0, 0.4))
        self._frame_no = (self._frame_no + int(round(CAM_FPS * dt))) % 65536
        link_ok = not covered or True
        no_tip = meas['tip'] is None
        flags = [not no_tip, not no_tip, meas['mid'] is not None, meas['pivot'] is not None, False, True]
        if live:
            mode_text = f"{MODE_NAMES.get(mode, 'UNKNOWN')} (live telemetry)"
        else:
            mode_text = f"{MODE_NAMES.get(mode, 'UNKNOWN')} (simulated frame)"

        frame = {
            'true': true_g, 'meas': meas, 'est': est, 'vel': vel, 'dets': dets,
            'frame': self._frame_no, 'proc_ms': proc_ms, 'age_ms': age_ms,
            'angle_est': angle_est, 'cart_x_px': cart_x_px, 'flags': flags,
            'link_ok': link_ok, 'banner': banner, 'mode_text': mode_text,
            'blur': speed_blur > 0.04, 'roi': self.spin_roi.value(),
            'in_window': abs(angle_est) < CATCH_WINDOW_DEG,
        }
        self.view.set_frame(frame)

        # ---- status line + graphs
        if live:
            self.status_label.setText(
                f"ONLINE | Mode: {MODE_NAMES.get(mode, 'UNKNOWN')} | Ang: {true_ang:+.1f}° | Pos: {pos_m * 1000:+.1f}mm")
        else:
            self.status_label.setText(
                f"SIM CAMERA | {MODE_NAMES.get(mode, '?')} | est {angle_est:+.1f}° | tip conf {self._conf_ema['tip']:.2f} | "
                f"{'TIP COVERED' if covered else 'tracking'}")
        self._push_graphs(live, true_ang, angle_meas, angle_est, meas, est, conf, proc_ms, age_ms)

    @staticmethod
    def _break_wraps(t, y):
        t = np.asarray(t, float)
        y = np.asarray(y, float)
        if len(y) < 2:
            return t, y
        idx = np.where(np.abs(np.diff(y)) > 180.0)[0] + 1
        if len(idx) == 0:
            return t, y
        return np.insert(t, idx, np.nan), np.insert(y, idx, np.nan)

    def _push_graphs(self, live, true_ang, angle_meas, angle_est, meas, est, conf, proc_ms, age_ms):
        b = self._buf
        b['t'].append(time.monotonic() - self._t0)
        b['true'].append(true_ang)
        b['meas'].append(angle_meas)
        b['est'].append(angle_est)
        b['cx_m'].append(meas['pivot'][0] if meas['pivot'] else float('nan'))
        b['cx_e'].append(est['pivot'][0] if est['pivot'] else float('nan'))
        b['c_cart'].append(conf['pivot'])
        b['c_mid'].append(conf['mid'])
        b['c_tip'].append(conf['tip'])
        b['age'].append(age_ms)
        b['proc'].append(proc_ms)
        t = np.array(b['t'])
        self.thr_line.setValue(self.spin_conf.value())
        for cv, key in ((self.cv_true, 'true'), (self.cv_meas, 'meas'), (self.cv_est, 'est')):
            tt, yy = self._break_wraps(t, b[key])
            cv.setData(tt, yy, connect='finite')
        self.cv_cart.setData(t, np.array(b['c_cart']))
        self.cv_mid.setData(t, np.array(b['c_mid']))
        self.cv_tip.setData(t, np.array(b['c_tip']))
        self.cv_cxm.setData(t, np.array(b['cx_m']), connect='finite')
        self.cv_cxe.setData(t, np.array(b['cx_e']), connect='finite')
        self.cv_age.setData(t, np.array(b['age']))
        self.cv_proc.setData(t, np.array(b['proc']))


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = CameraMainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
