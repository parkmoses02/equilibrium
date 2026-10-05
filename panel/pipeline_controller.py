"""
Pipeline Controller: Hardware Serial + Real Riccati LQR + Acceleration Control + Energy Swing-up
Reuses modules from panel/simulator.py and panel/serial_protocol.py.
"""

import sys
import math
import time
import struct
from collections import deque
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

try:
    from .serial_protocol import SerialProtocol
    from . import simulator
except ImportError:
    from serial_protocol import SerialProtocol
    import simulator


class PipelineSerial(SerialProtocol):
    """SerialProtocol with the pipeline firmware's PC -> ESP32 framing.

    pipeline_energy_lqr.cpp expects every command as a 6-byte frame
        [0xAA][addr][4 payload bytes]
    (same 0xAA header as its telemetry). Without the header, addresses 0x20 (' ') and 0x50
    ('P') collide with the firmware's ASCII commands (disarm / flip polarity) and corrupt
    the stream. The base class is left untouched because panel.py / main.cpp still use the
    5-byte header-less format.
    """
    HEADER = 0xAA

    def send_float(self, addr, value):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('Serial port not open')
        self.ser.write(bytes([self.HEADER, addr & 0xFF]) + struct.pack('<f', float(value)))

    def set_move_mode(self, mode):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('Serial port not open')
        # 0x50 = mode command, payload[0] = mode byte (0 safe, 1 swing, 2 balance)
        self.ser.write(bytes([self.HEADER, 0x50, int(mode) & 0xFF, 0, 0, 0]))


# ==============================================================================
# 1. Riccati LQR Solver & Full Nonlinear Simulation Engine
# ==============================================================================

def solve_riccati_lqr(length_m=0.20, friction=0.04, g=9.81,
                      q_diag=(100.0, 10.0, 5.0, 2.0), r_val=1.0, dt=0.002):
    """
    Solves Discrete Algebraic Riccati Equation (DARE) for the acceleration-controlled
    inverted pendulum system.
    State x = [theta (rad from upright), theta_dot (rad/s), pos (m), vel (m/s)]
    Control input u = cart acceleration (ddot{pos}) [m/s^2]
    """
    length_mm = length_m * 1000.0
    Ac, Bc = simulator.model_matrices(length_mm=length_mm, friction=friction, g=g)
    Ad, Bd = simulator.discretize(Ac, Bc, dt)

    Q = np.diag(q_diag)
    R = np.array([[r_val]], dtype=float)

    P, K = simulator.dare_lqr(Ad, Bd, Q, R)
    # K is shape (1, 4): [K_angle, K_rate, K_cartPos, K_cartVel]
    return K.flatten(), P, Ad, Bd


# --- Swing-up state machine constants (ported from upright_balance_test.cpp /
#     pipeline_energy_lqr.cpp, which are hardware-verified) ---
SWING_START_OFFSET_M = 0.10                      # slow pre-position distance before the kick
SWING_DIRECTION = -1.0                           # first kick direction (same as firmware)
SWING_ENERGY_MARGIN = 0.0                        # target energy margin (bottom = -2, top = 0)
SWING_RETRY_GAIN = 0.3                           # raise target by this fraction of missed energy
CATCH_WINDOW_RAD = math.radians(25.0)            # hand over to LQR inside +-25 deg
SWING_START_ANGLE_RAD = math.radians(5.0)        # "hanging still" thresholds for SETTLE
SWING_START_RATE = 0.5                           # rad/s
SWING_SETTLE_TIMEOUT_S = 3.0

PHASE_PREPOSITION, PHASE_SETTLE, PHASE_PUMP, PHASE_COAST, PHASE_RISE = range(5)


def simulate_full_pipeline(length_m=0.20, friction=0.04, g=9.81,
                           K_lqr=None,
                           swing_speed=1.20, swing_accel=16.0,
                           cart_limit=0.35, max_speed=0.80, max_accel=12.0,
                           dt=0.002, total_time_s=10.0,
                           initial_angle_deg=175.0): # 180 = hanging downward
    """
    Full nonlinear simulation:
    - Starts hanging downward (or near bottom)
    - Swing-up with the firmware's 5-phase state machine
        PREPOSITION -> SETTLE -> PUMP -> COAST -> RISE (+ apex retry, rail-limit guard)
      ported from upright_balance_test.cpp / pipeline_energy_lqr.cpp
    - Handover to Riccati / manual-gain acceleration balancing once |theta| < 25 deg

    Conventions: u = cart acceleration, plant
        theta_ddot = (g*sin(theta) - u*cos(theta) - friction*theta_dot) / L
    Because this plant uses -u*cos(theta), the firmware's controlPolarity (-1, hardware)
    corresponds to +1 here; the kick direction is computed with that polarity.
    K_lqr uses the u = -K @ x convention.
    """
    if K_lqr is None:
        K_lqr, _, _, _ = solve_riccati_lqr(length_m, friction, g, dt=dt)
    K_lqr = np.asarray(K_lqr, dtype=float).flatten()

    polarity = 1.0  # polarity of THIS plant (hardware firmware uses -1)

    total_steps = int(total_time_s / dt)
    t_arr = np.linspace(0, total_time_s, total_steps)

    # State: [theta (rad from upright), theta_dot, pos (m), vel (m/s)]
    # Hanging straight down is theta = pi or -pi. Upright is 0.
    theta = (math.radians(initial_angle_deg) + np.pi) % (2 * np.pi) - np.pi
    theta_dot = 0.0
    pos = 0.0
    vel = 0.0

    mode = "SWING"  # SWING or LQR_BALANCE
    w2 = g / length_m

    swing_phase = PHASE_PREPOSITION
    swing_target_vel = 0.0
    swing_energy_bias = 0.0
    previous_rate = 0.0
    kick_pending = True
    pushed_this_pass = False
    apex_handled = False
    t_phase_start = 0.0
    catch_time_s = None

    traj_theta = np.zeros(total_steps)
    traj_rate = np.zeros(total_steps)
    traj_pos = np.zeros(total_steps)
    traj_vel = np.zeros(total_steps)
    traj_u = np.zeros(total_steps)
    traj_energy = np.zeros(total_steps)
    traj_mode = np.zeros(total_steps)   # 0: swing, 1: balance
    traj_phase = np.full(total_steps, -1, dtype=int)

    def wrap_pi(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    # Cart starts at the centre; the pre-position target is the far end opposite the kick.
    start_x = -polarity * SWING_DIRECTION * SWING_START_OFFSET_M

    for i in range(total_steps):
        t_now = t_arr[i]
        # Normalized mechanical energy: E = 0.5 * theta_dot^2 * (L/g) + cos(theta) - 1
        energy = 0.5 * (theta_dot ** 2) * length_m / g + math.cos(theta) - 1.0

        traj_theta[i] = theta
        traj_rate[i] = theta_dot
        traj_pos[i] = pos
        traj_vel[i] = vel
        traj_energy[i] = energy
        traj_mode[i] = 0 if mode == "SWING" else 1
        traj_phase[i] = swing_phase if mode == "SWING" else -1

        u = 0.0

        if mode == "SWING":
            psi = wrap_pi(theta - np.pi)                 # angle from the bottom
            toward_bottom = (psi * theta_dot < 0.0)
            upper_half = (abs(theta) < 0.5 * np.pi)

            if swing_phase == PHASE_PREPOSITION:
                # +a for T0, -a for T0 (T0 = pendulum period): nearly no residual swing.
                period = 2.0 * np.pi * math.sqrt(length_m / g)
                t = t_now - t_phase_start
                a = start_x / (period * period)
                if t < period:
                    u = a
                elif t < 2.0 * period:
                    u = -a
                else:
                    vel = 0.0
                    swing_phase = PHASE_SETTLE
                    t_phase_start = t_now
                    u = 0.0
            else:
                if swing_phase == PHASE_SETTLE:
                    still = (abs(psi) < SWING_START_ANGLE_RAD and abs(theta_dot) < SWING_START_RATE)
                    if not still and (t_now - t_phase_start < SWING_SETTLE_TIMEOUT_S):
                        u = 0.0
                    else:
                        swing_phase = PHASE_PUMP
                        previous_rate = theta_dot

                if swing_phase in (PHASE_PUMP, PHASE_COAST, PHASE_RISE):
                    # Apex of the upper half-circle outside the catch window: energy was short.
                    if not upper_half:
                        apex_handled = False
                    if (upper_half and not apex_handled and previous_rate * theta_dot < 0.0
                            and abs(theta) >= CATCH_WINDOW_RAD):
                        apex_handled = True
                        swing_energy_bias += SWING_RETRY_GAIN * (1.0 - math.cos(theta))
                        swing_phase = PHASE_PUMP
                        pushed_this_pass = False
                    # COAST turned back in the lower half: not enough energy went in.
                    if swing_phase == PHASE_COAST and not upper_half and previous_rate * theta_dot < 0.0:
                        swing_phase = PHASE_PUMP
                        pushed_this_pass = False
                    previous_rate = theta_dot

                    if abs(theta) < CATCH_WINDOW_RAD:
                        # Caught: hand over to the balance controller (velocity carries over).
                        mode = "LQR_BALANCE"
                        catch_time_s = t_now
                    else:
                        if swing_phase == PHASE_RISE and not upper_half and toward_bottom:
                            swing_phase = PHASE_PUMP   # failed to rise, falling back
                            pushed_this_pass = False

                        if swing_phase != PHASE_RISE:
                            if not toward_bottom:
                                pushed_this_pass = False

                            if swing_phase == PHASE_PUMP and (kick_pending or not pushed_this_pass):
                                # Cart velocity change dv at the bottom changes the pendulum rate
                                # by polarity*dv/L; size it from the missing energy.
                                bottom_rate = math.sqrt(max(0.0, 2.0 * w2 * (energy + 2.0)))
                                target_rate = math.sqrt(2.0 * w2 * (SWING_ENERGY_MARGIN + swing_energy_bias + 2.0))
                                dv_mag = max(0.0, target_rate - bottom_rate) * length_m
                                direction = SWING_DIRECTION if kick_pending else (1.0 if theta_dot >= 0.0 else -1.0)
                                desired = vel + polarity * direction * dv_mag
                                nxt = float(np.clip(desired, -swing_speed, swing_speed))
                                # Start early so the middle of the acceleration is at the bottom.
                                time_to_bottom = abs(psi) / max(abs(theta_dot), 1e-3)
                                lead_time = abs(nxt - vel) / (2.0 * swing_accel) + dt
                                if kick_pending or (toward_bottom and time_to_bottom < lead_time):
                                    swing_target_vel = nxt
                                    kick_pending = False
                                    pushed_this_pass = True
                                    if abs(desired - nxt) < 1e-4:
                                        swing_phase = PHASE_COAST  # this step reaches the target energy

                            # Horizontal pass while rising: cos ~ 0, stopping here barely changes energy.
                            brake_lead = abs(theta_dot) * (abs(vel) / swing_accel) * 0.5
                            if (not toward_bottom) and abs(theta) < 0.5 * np.pi + brake_lead:
                                swing_target_vel = 0.0
                                if swing_phase == PHASE_COAST:
                                    swing_phase = PHASE_RISE

                        # Rail-limit guard: block only targets heading INTO the limit.
                        stop_x = pos + vel * abs(vel) / (2.0 * swing_accel)
                        if abs(stop_x) > cart_limit - 0.03 and swing_target_vel * stop_x > 0.0:
                            swing_target_vel = 0.0

                        u = float((swing_target_vel - vel) / dt)

            if mode == "SWING":
                u = float(np.clip(u, -swing_accel, swing_accel))

        if mode == "LQR_BALANCE":
            # Riccati law: u = -K @ x (standard LQR — all four states share the same
            # sign convention that dare_lqr()/simulator.simulate() were solved for).
            # NOTE: do NOT flip the sign of the pos/vel terms here. That convention
            # only applies to the hand-tuned firmware gains in upright_balance_test.cpp,
            # where it corrects a historical bug in *manually chosen* constants. A
            # Riccati-solved K already has the correct coupling baked in by the DARE
            # solve, so splitting the sign reintroduces that historical bug and
            # destabilizes the cart-return loop (verified: diverges within ~1s).
            u_lqr = -(K_lqr[0] * theta + K_lqr[1] * theta_dot + K_lqr[2] * pos + K_lqr[3] * vel)
            u = float(np.clip(u_lqr, -max_accel, max_accel))

        # Kinematic acceleration integration
        vel_cap = max(max_speed, swing_speed) if mode == "SWING" else max_speed
        vel_next = float(np.clip(vel + u * dt, -vel_cap, vel_cap))
        u_actual = (vel_next - vel) / dt
        pos_next = pos + vel_next * dt

        # Nonlinear pendulum acceleration:
        # theta_ddot = (g * sin(theta) - u_actual * cos(theta) - friction * theta_dot) / length_m
        theta_ddot = (g * math.sin(theta) - u_actual * math.cos(theta) - friction * theta_dot) / length_m

        theta_dot_next = theta_dot + theta_ddot * dt
        theta_next = wrap_pi(theta + theta_dot_next * dt)

        theta, theta_dot, pos, vel = theta_next, theta_dot_next, pos_next, vel_next
        traj_u[i] = u_actual

    return {
        'time': t_arr,
        'theta_deg': np.degrees(traj_theta),
        'rate': traj_rate,
        'pos_mm': traj_pos * 1000.0,
        'vel_m_s': traj_vel,
        'accel': traj_u,
        'energy': traj_energy,
        'mode': traj_mode,
        'phase': traj_phase,
        'catch_time_s': catch_time_s,
    }


# ==============================================================================
# 2. Pendulum Visualization Canvas Widget (Reused & Enhanced)
# ==============================================================================

class PendulumWidget(QtWidgets.QWidget):
    """Horizontal rail with cart and pendulum rod."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle_deg = 180.0 # upright = 180 deg on screen (or 0 rad)
        self.position_mm = 0.0
        self.range_mm = 350.0
        self.setMinimumHeight(300)

    def set_state(self, angle_deg, position_mm):
        self.angle_deg = angle_deg
        self.position_mm = position_mm
        self.update()

    def set_range(self, range_mm):
        self.range_mm = max(10.0, float(range_mm))
        self.update()

    def _mm_to_px(self, mm, w):
        return (mm + self.range_mm) / (2.0 * self.range_mm) * (w - 80) + 40

    def _draw_ruler(self, p, w, rail_y, cart_px):
        """mm scale under the rail: 0 at the centre, + to the right, soft limits in red."""
        y0 = rail_y + 16
        small = QtGui.QFont(); small.setPointSize(8)
        p.setFont(small)
        fm = QtGui.QFontMetrics(small)

        # baseline
        p.setPen(QtGui.QPen(QtGui.QColor(139, 148, 158), 1.5))
        p.drawLine(QtCore.QPointF(self._mm_to_px(-self.range_mm, w), y0),
                   QtCore.QPointF(self._mm_to_px(self.range_mm, w), y0))

        # tick steps: 10 mm minor, 50 mm medium, 100 mm major (labelled)
        n = int(self.range_mm // 10)
        for i in range(-n, n + 1):
            mm = i * 10
            x = self._mm_to_px(mm, w)
            if mm % 100 == 0:
                length, width, col = 12, 1.6, QtGui.QColor(201, 209, 217)
            elif mm % 50 == 0:
                length, width, col = 8, 1.2, QtGui.QColor(160, 168, 178)
            else:
                length, width, col = 4, 1.0, QtGui.QColor(110, 118, 129)
            if mm == 0:
                length, width, col = 16, 2.2, QtGui.QColor(88, 166, 255)
            p.setPen(QtGui.QPen(col, width))
            p.drawLine(QtCore.QPointF(x, y0), QtCore.QPointF(x, y0 + length))
            if mm % 100 == 0:
                label = '0' if mm == 0 else f"{mm:+d}"
                p.setPen(QtGui.QPen(QtGui.QColor(201, 209, 217)))
                p.drawText(QtCore.QPointF(x - fm.horizontalAdvance(label) / 2.0, y0 + 28), label)

        # soft limit markers
        p.setPen(QtGui.QPen(QtGui.QColor(248, 81, 73), 2))
        for sgn in (-1, 1):
            x = self._mm_to_px(sgn * self.range_mm, w)
            p.drawLine(QtCore.QPointF(x, y0 - 6), QtCore.QPointF(x, y0 + 16))
        lim = f"limit {self.range_mm:.0f} mm"
        p.setPen(QtGui.QPen(QtGui.QColor(248, 81, 73)))
        p.drawText(QtCore.QPointF(self._mm_to_px(self.range_mm, w) - fm.horizontalAdvance(lim), y0 + 42), lim)
        p.drawText(QtCore.QPointF(self._mm_to_px(-self.range_mm, w), y0 + 42), lim)

        # cart position marker + numeric readout
        p.setPen(QtGui.QPen(QtGui.QColor(46, 204, 113), 1.5, QtCore.Qt.DashLine))
        p.drawLine(QtCore.QPointF(cart_px, rail_y + 8), QtCore.QPointF(cart_px, y0 + 16))
        txt = f"{self.position_mm:+.1f} mm"
        bold = QtGui.QFont(); bold.setPointSize(10); bold.setBold(True)
        p.setFont(bold)
        p.setPen(QtGui.QPen(QtGui.QColor(46, 204, 113)))
        tw = QtGui.QFontMetrics(bold).horizontalAdvance(txt)
        p.drawText(QtCore.QPointF(cart_px - tw / 2.0, y0 + 56), txt)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        try:
            w, h = self.width(), self.height()
            rail_y = h * 0.58

            # Background clear
            p.fillRect(self.rect(), QtGui.QColor("#1e222d"))

            # Rail line & limit markers
            p.setPen(QtGui.QPen(QtGui.QColor(80, 90, 110), 3))
            p.drawLine(20, int(rail_y), w - 20, int(rail_y))

            # Rail soft limit boundaries
            pos_clamped = max(-self.range_mm, min(self.range_mm, self.position_mm))
            px = self._mm_to_px(pos_clamped, w)

            # Ruler under the rail (same mm -> pixel mapping as the cart)
            self._draw_ruler(p, w, rail_y, px)

            # Cart
            cart_w, cart_h = 50, 22
            cart_rect = QtCore.QRectF(px - cart_w / 2, rail_y - cart_h, cart_w, cart_h)
            p.setBrush(QtGui.QBrush(QtGui.QColor(41, 128, 185)))
            p.setPen(QtGui.QPen(QtGui.QColor(236, 240, 241), 1.5))
            p.drawRoundedRect(cart_rect, 4, 4)

            # Wheels
            p.setBrush(QtGui.QBrush(QtGui.QColor(52, 73, 94)))
            p.drawEllipse(QtCore.QPointF(px - 14, rail_y + 2), 5, 5)
            p.drawEllipse(QtCore.QPointF(px + 14, rail_y + 2), 5, 5)

            # Pendulum pivot
            pivot_x = px
            pivot_y = rail_y - cart_h / 2

            # Length in px
            rod_len = 0.45 * h
            # angle_deg: 0 = hanging down, 180 = upright
            angle_rad = -self.angle_deg * math.pi / 180.0
            bob_x = pivot_x + rod_len * math.sin(angle_rad)
            bob_y = pivot_y + rod_len * math.cos(angle_rad)

            # Catch window guide arcs (when near top 180 deg)
            p.setPen(QtGui.QPen(QtGui.QColor(46, 204, 113, 70), 2, QtCore.Qt.DashLine))
            cw_rad = math.radians(25.0)
            p.drawLine(QtCore.QPointF(pivot_x, pivot_y),
                       QtCore.QPointF(pivot_x - rod_len * math.sin(cw_rad), pivot_y - rod_len * math.cos(cw_rad)))
            p.drawLine(QtCore.QPointF(pivot_x, pivot_y),
                       QtCore.QPointF(pivot_x + rod_len * math.sin(cw_rad), pivot_y - rod_len * math.cos(cw_rad)))

            # Rod
            p.setPen(QtGui.QPen(QtGui.QColor(241, 196, 15), 4))
            p.drawLine(QtCore.QPointF(pivot_x, pivot_y), QtCore.QPointF(bob_x, bob_y))

            # Pivot circle
            p.setBrush(QtGui.QBrush(QtGui.QColor(231, 76, 60)))
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255), 1))
            p.drawEllipse(QtCore.QPointF(pivot_x, pivot_y), 4, 4)

            # Pendulum Bob
            p.setBrush(QtGui.QBrush(QtGui.QColor(231, 76, 60)))
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255), 2))
            p.drawEllipse(QtCore.QPointF(bob_x, bob_y), 10, 10)

        finally:
            p.end()


# ==============================================================================
# 3. Main GUI Window
# ==============================================================================

class PipelineMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Inverted Pendulum: Real Riccati LQR + Accel Control + Energy Swing-up')
        self.resize(1750, 960)

        self.serial = None
        self._rx_count = 0
        self._live_telemetry = {
            'angle': 0.0,
            'rate': 0.0,
            'pos': 0.0,
            'vel': 0.0,
            'energy': -2.0,
            'accel': 0.0,
            'mode': 0
        }

        # Live plot buffers (about 10 s of 30 fps samples)
        self._live_plots_active = False
        self._live_t0 = None
        self._live_buf = {k: deque(maxlen=400) for k in ('t', 'angle', 'energy', 'pos', 'accel', 'vel')}

        # Simulation playback state
        self._sim_result = None
        self._sim_idx = 0
        self._use_simulation_playback = True

        self._init_ui()
        self._calculate_riccati()
        self.run_simulation()

        # 30 fps visual update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(33)

    def _init_ui(self):
        self.setStyleSheet("""
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
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        # -------------------------------------------------------------
        # LEFT COLUMN: Parameters, Riccati Solver, and Device Control
        # -------------------------------------------------------------
        left_pane = QtWidgets.QWidget()
        left_pane.setFixedWidth(480)
        left_pane.setObjectName('leftPane')
        left_layout = QtWidgets.QVBoxLayout(left_pane)
        left_layout.setContentsMargins(0, 12, 0, 0)  # room so the first group title is not clipped
        left_layout.setSpacing(8)

        # 1. Hardware Serial Connection
        grp_port = QtWidgets.QGroupBox('1. Hardware Serial Communication')
        l_port = QtWidgets.QVBoxLayout(grp_port)
        h_conn = QtWidgets.QHBoxLayout()
        self.port_edit = QtWidgets.QLineEdit('COM3')
        self.connect_btn = QtWidgets.QPushButton('Connect ESP32')
        self.connect_btn.setObjectName("actionBtn")
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.connect_btn.setToolTip("입력한 COM 포트를 115200bps로 열어 ESP32와 통신을 시작합니다. 모터는 움직이지 않습니다.")
        h_conn.addWidget(QtWidgets.QLabel('Port:'))
        h_conn.addWidget(self.port_edit)
        h_conn.addWidget(self.connect_btn)
        l_port.addLayout(h_conn)

        self.status_label = QtWidgets.QLabel('Status: Offline (Local Simulation Active)')
        self.status_label.setStyleSheet("color: #8b949e; font-family: monospace;")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(34)
        l_port.addWidget(self.status_label)
        left_layout.addWidget(grp_port)

        # 2. Physical & Riccati LQR Parameters
        grp_lqr = QtWidgets.QGroupBox('2. Physical Model & Riccati DARE Solver')
        l_lqr = QtWidgets.QFormLayout(grp_lqr)

        self.spin_L = QtWidgets.QDoubleSpinBox(); self.spin_L.setRange(0.05, 1.0); self.spin_L.setDecimals(3); self.spin_L.setValue(0.200)
        self.spin_fric = QtWidgets.QDoubleSpinBox(); self.spin_fric.setRange(0.0, 1.0); self.spin_fric.setDecimals(4); self.spin_fric.setValue(0.040)
        self.spin_g = QtWidgets.QDoubleSpinBox(); self.spin_g.setRange(1.0, 20.0); self.spin_g.setValue(9.81)

        l_lqr.addRow('Pendulum Length L [m]:', self.spin_L)
        l_lqr.addRow('Pivot Friction [N*m*s]:', self.spin_fric)
        l_lqr.addRow('Gravity g [m/s^2]:', self.spin_g)

        # Weights Q and R
        self.spin_q0 = QtWidgets.QDoubleSpinBox(); self.spin_q0.setRange(0.1, 10000); self.spin_q0.setValue(100.0) # Angle
        self.spin_q1 = QtWidgets.QDoubleSpinBox(); self.spin_q1.setRange(0.1, 10000); self.spin_q1.setValue(10.0)  # Rate
        self.spin_q2 = QtWidgets.QDoubleSpinBox(); self.spin_q2.setRange(0.1, 10000); self.spin_q2.setValue(5.0)   # Cart Pos
        self.spin_q3 = QtWidgets.QDoubleSpinBox(); self.spin_q3.setRange(0.1, 10000); self.spin_q3.setValue(2.0)   # Cart Vel
        self.spin_r = QtWidgets.QDoubleSpinBox(); self.spin_r.setRange(0.001, 1000); self.spin_r.setDecimals(3); self.spin_r.setValue(1.0) # Accel effort

        l_lqr.addRow('Q_theta (Angle penalty):', self.spin_q0)
        l_lqr.addRow('Q_rate (Rate damping):', self.spin_q1)
        l_lqr.addRow('Q_pos (Cart centering):', self.spin_q2)
        l_lqr.addRow('Q_vel (Cart speed damp):', self.spin_q3)
        l_lqr.addRow('R (Accel effort penalty):', self.spin_r)

        self.calc_lqr_btn = QtWidgets.QPushButton('⚡ Solve Riccati Equation (DARE)')
        self.calc_lqr_btn.clicked.connect(self._calculate_riccati)
        self.calc_lqr_btn.setToolTip("위의 L, 마찰, g, Q, R로 리카티 방정식을 풀어 K를 다시 계산합니다. (하드웨어로 전송은 안 함)")
        l_lqr.addRow(self.calc_lqr_btn)

        self.gain_result_label = QtWidgets.QLabel('K: -')
        self.gain_result_label.setStyleSheet(
            "color: #7ee787; font-weight: bold; font-family: monospace; font-size: 12px;"
            "background-color: #0d1117; border: 1px solid #30363d; border-radius: 4px; padding: 6px;")
        self.gain_result_label.setWordWrap(True)
        self.gain_result_label.setMinimumHeight(120)
        self.gain_result_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        l_lqr.addRow(self.gain_result_label)

        self.send_k_btn = QtWidgets.QPushButton('Transmit Gains to ESP32')
        self.send_k_btn.clicked.connect(self._send_k_to_firmware)
        self.send_k_btn.setToolTip("현재 선택된 게인(리카티 또는 수동)을 ESP32로 전송합니다. 즉시 덮어씌워집니다.")
        l_lqr.addRow(self.send_k_btn)

        # Manual (hand-tuned) gains, firmware convention: positive magnitudes applied as
        #   a = -1*(kAngle*theta + kRate*rate) + 1*(kCartPos*x + kCartVel*v)
        # Defaults are the hardware-verified values from upright_balance_test.cpp.
        self.chk_manual_gains = QtWidgets.QCheckBox('Use manual gains (hardware-verified)')
        l_lqr.addRow(self.chk_manual_gains)

        def _make_gain_spin(value):
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(0.0, 500.0); sp.setDecimals(2); sp.setValue(value)
            sp.valueChanged.connect(self._refresh_gain_label)
            return sp
        self.spin_kA = _make_gain_spin(50.0)  # gKangle   (0x20)
        self.spin_kR = _make_gain_spin(8.0)   # gKrate    (0x21)
        self.spin_kP = _make_gain_spin(3.0)   # gKcartPos (0x22)
        self.spin_kV = _make_gain_spin(5.0)   # gKcartVel (0x23)
        l_lqr.addRow('Manual kAngle:', self.spin_kA)
        l_lqr.addRow('Manual kRate:', self.spin_kR)
        l_lqr.addRow('Manual kCartPos:', self.spin_kP)
        l_lqr.addRow('Manual kCartVel:', self.spin_kV)
        self.chk_manual_gains.toggled.connect(self._refresh_gain_label)

        # Soft (demo) balance = firmware 'D' toggle: lower pendulum-loop gains so the tilt is
        # visible before the cart catches it. Applies on top of whichever gains were sent.
        self.chk_soft_balance = QtWidgets.QCheckBox('Soft balance (demo gains, same as D)')
        self.chk_soft_balance.setToolTip(
            "ESP32 의 D 토글과 같은 기능. 밸런싱 중에도 즉시 적용되며(카트는 튀지 않음) 체크 시 연결돼 있으면 바로 전송합니다.\n"
            "캐치 직후에는 원래 게인을 쓰다가 서서히 아래 soft 게인으로 넘어갑니다.\n"
            "너무 낮추면(약 28/3 미만) 진자가 1~1.5Hz로 흔들리다 넘어질 수 있습니다. 흔들리면 kRate 부터 올리세요.")
        l_lqr.addRow(self.chk_soft_balance)
        self.spin_kAS = QtWidgets.QDoubleSpinBox()   # gKangleSoft (0x24)
        self.spin_kRS = QtWidgets.QDoubleSpinBox()   # gKrateSoft  (0x25)
        for sp, v in ((self.spin_kAS, 15.0), (self.spin_kRS, 2.0)):
            sp.setRange(0.0, 500.0); sp.setDecimals(2); sp.setValue(v)
        l_lqr.addRow('Soft kAngle:', self.spin_kAS)
        l_lqr.addRow('Soft kRate:', self.spin_kRS)
        self.chk_soft_balance.toggled.connect(self._send_soft_state)
        left_layout.addWidget(grp_lqr)

        # 3. Kinematic Limits & Energy Parameters
        grp_limits = QtWidgets.QGroupBox('3. Acceleration & Energy Parameters')
        l_limits = QtWidgets.QFormLayout(grp_limits)

        self.spin_max_accel = QtWidgets.QDoubleSpinBox(); self.spin_max_accel.setRange(1.0, 50.0); self.spin_max_accel.setValue(12.0)
        self.spin_max_speed = QtWidgets.QDoubleSpinBox(); self.spin_max_speed.setRange(0.1, 5.0); self.spin_max_speed.setValue(0.80)
        self.spin_cart_limit = QtWidgets.QDoubleSpinBox(); self.spin_cart_limit.setRange(0.05, 1.0); self.spin_cart_limit.setValue(0.35)
        self.spin_swing_accel = QtWidgets.QDoubleSpinBox(); self.spin_swing_accel.setRange(1.0, 50.0); self.spin_swing_accel.setValue(16.0)
        self.spin_swing_speed = QtWidgets.QDoubleSpinBox(); self.spin_swing_speed.setRange(0.1, 5.0); self.spin_swing_speed.setValue(1.20)

        l_limits.addRow('Max Cart Accel [m/s^2]:', self.spin_max_accel)
        l_limits.addRow('Max Cart Speed [m/s]:', self.spin_max_speed)
        l_limits.addRow('Rail Soft Limit [m]:', self.spin_cart_limit)
        l_limits.addRow('Swing Accel [m/s^2]:', self.spin_swing_accel)
        l_limits.addRow('Swing Speed [m/s]:', self.spin_swing_speed)

        self.send_params_btn = QtWidgets.QPushButton('Transmit Limits to ESP32')
        self.send_params_btn.clicked.connect(self._send_limits_to_firmware)
        self.send_params_btn.setToolTip("최대 속도/가속도, 레일 한계, 진자 길이를 ESP32로 전송합니다.\n"
                                        "(Swing Accel/Speed는 시뮬레이션 전용이라 전송되지 않습니다.)")
        l_limits.addRow(self.send_params_btn)
        left_layout.addWidget(grp_limits)

        # 4. Pipeline Execution Controls
        grp_actions = QtWidgets.QGroupBox('4. Pipeline Execution & Safety')
        l_actions = QtWidgets.QVBoxLayout(grp_actions)

        h_cmds = QtWidgets.QHBoxLayout()
        self.swing_btn = QtWidgets.QPushButton('🚀 Start Energy Swing-up')
        self.swing_btn.setObjectName("primaryBtn")
        self.swing_btn.clicked.connect(self._cmd_swingup)
        self.swing_btn.setToolTip("연결됨: 한계값+게인 전송 후 스윙업 시작(펌웨어가 스윙업→캐치→밸런싱 수행).\n"
                                  "연결 안 됨: 화면 시뮬레이션을 실행합니다.")

        self.balance_btn = QtWidgets.QPushButton('⚖ Direct LQR Balance')
        self.balance_btn.clicked.connect(self._cmd_balance)
        self.balance_btn.setToolTip("스윙업 없이 곧바로 밸런싱. 진자를 손으로 세워 잡은 상태(±몇 도 이내)에서 누르세요.\n"
                                    "게인을 먼저 전송합니다. 연결이 없으면 아무 일도 안 합니다.")

        h_cmds.addWidget(self.swing_btn)
        h_cmds.addWidget(self.balance_btn)
        l_actions.addLayout(h_cmds)

        h_cmds2 = QtWidgets.QHBoxLayout()
        self.zero_btn = QtWidgets.QPushButton('Zero Encoder')
        self.zero_btn.clicked.connect(self._cmd_zero)
        self.zero_btn.setToolTip("진자를 아래로 늘어뜨려 완전히 멈춘 상태에서 누르세요.\n"
                                 "현재 엔코더 위치를 '아래(hanging down)' 기준으로 설정합니다.\n"
                                 "펌웨어는 이걸 한 번 하기 전엔 Swing-up/Balance를 거부합니다.")

        self.zero_pos_btn = QtWidgets.QPushButton('Zero Position')
        self.zero_pos_btn.clicked.connect(self._cmd_zero_position)
        self.zero_pos_btn.setToolTip(
            "카트의 현재 위치를 0으로 다시 정의합니다(진자 엔코더 영점과는 무관, disarm 하지 않음).\n"
            "위치는 모터에 보낸 스텝 수를 센 값이라 탈조가 나면 실제와 어긋납니다.\n"
            "스윙업 후 카트를 손으로 천천히 레일 중앙에 놓은 뒤 누르세요.")

        self.stop_btn = QtWidgets.QPushButton('⛔ Emergency Disarm / Stop')
        self.stop_btn.setObjectName("dangerBtn")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.clicked.connect(self._cmd_stop)
        self.stop_btn.setToolTip("즉시 SAFE 모드(모터 정지)로 전환합니다. 이상하면 언제든 누르세요.")

        h_cmds2.addWidget(self.zero_btn)
        h_cmds2.addWidget(self.zero_pos_btn)
        l_actions.addLayout(h_cmds2)
        l_actions.addWidget(self.stop_btn)   # full width, easy to hit in an emergency

        self.sim_btn = QtWidgets.QPushButton('🔄 Run & Replay Nonlinear Simulation')
        self.sim_btn.clicked.connect(self.run_simulation)
        self.sim_btn.setToolTip("PC 안에서만 돌리는 비선형 시뮬레이션을 다시 계산해 재생합니다. 하드웨어와 무관.")
        l_actions.addWidget(self.sim_btn)

        left_layout.addWidget(grp_actions)
        left_layout.addStretch()
        left_scroll = QtWidgets.QScrollArea()
        left_scroll.setWidget(left_pane)
        left_scroll.setWidgetResizable(True)
        left_scroll.setFixedWidth(left_pane.width() + 20 if left_pane.width() > 480 else 500)
        left_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        left_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        main_layout.addWidget(left_scroll)

        # -------------------------------------------------------------
        # RIGHT COLUMN: Visualization & Telemetry Graphs
        # -------------------------------------------------------------
        right_pane = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_pane)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        # Pendulum Canvas
        self.pendulum_widget = PendulumWidget()
        right_layout.addWidget(self.pendulum_widget, stretch=2)

        # View options for LIVE hardware data only (the simulation already uses the screen
        # convention: + = right for both cart position and pendulum tilt).
        h_view = QtWidgets.QHBoxLayout()
        self.chk_flip_cart = QtWidgets.QCheckBox('Flip cart left/right (live)')
        self.chk_flip_cart.setChecked(True)
        self.chk_flip_cart.setToolTip(
            "하드웨어의 +방향과 화면의 +방향(오른쪽)이 반대일 때 카트 위치/속도/가속도의 부호를 뒤집어 표시합니다.\n"
            "화면, 그래프, 상태 문구에 모두 적용되고 펌웨어 값 자체는 바뀌지 않습니다.\n"
            "(펌웨어 극성 -1 은 시뮬레이션과 카트 방향이 반대인 좌표계입니다.)")
        self.chk_flip_tilt = QtWidgets.QCheckBox('Flip pendulum tilt (live)')
        self.chk_flip_tilt.setChecked(False)
        self.chk_flip_tilt.setToolTip(
            "진자가 기우는 방향이 실제와 반대로 보이면 체크하세요. 각도/각속도의 부호를 뒤집어 표시합니다.")
        h_view.addWidget(self.chk_flip_cart)
        h_view.addWidget(self.chk_flip_tilt)
        h_view.addStretch()
        right_layout.addLayout(h_view)

        # 2x2 Telemetry Graphs
        grid_widget = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(grid_widget)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setSpacing(6)

        pg.setConfigOption('background', '#161b22')
        pg.setConfigOption('foreground', '#8b949e')

        self.plot_angle = pg.PlotWidget(title='Angle (deg)  [180 = hanging down, 0/360 = upright]')
        self.plot_energy = pg.PlotWidget(title='Normalized Energy E [Target = 0]')
        self.plot_pos = pg.PlotWidget(title='Cart Position (mm)')
        self.plot_accel = pg.PlotWidget(title='Cart Acceleration (m/s^2) & Speed (m/s)')

        for p in (self.plot_angle, self.plot_energy, self.plot_pos, self.plot_accel):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel('bottom', 'Time (s)')

        # Angle plot: the angle is shown modulo 360 so that the hanging position (180) is the
        # centre and the upright position sits at the edges (0 and 360). The trace is unwrapped
        # and drawn as 5 copies shifted by 360 deg, so a swing through the top continues across
        # the edge instead of jumping; the view only shows -60..420.
        self.angle_view = (-60.0, 420.0)
        self.plot_angle.getAxis('left').setTicks([[
            (0, '0 top'), (90, '90'), (180, '180 down'), (270, '270'), (360, '360 top')]])
        # catch window (+-25 deg around upright) at both edges, hanging reference at 180
        self.cw_lines = []
        for v in (-25.0, 25.0, 335.0, 385.0):
            ln = pg.InfiniteLine(pos=v, angle=0, pen=pg.mkPen('#2ea043', width=1, style=QtCore.Qt.DashLine))
            self.plot_angle.addItem(ln)
            self.cw_lines.append(ln)
        self.plot_angle.addItem(pg.InfiniteLine(pos=180.0, angle=0,
                                                pen=pg.mkPen('#6e7681', width=1, style=QtCore.Qt.DotLine)))

        # Energy target line
        self.energy_target_line = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#2ea043', width=1.5))
        self.plot_energy.addItem(self.energy_target_line)

        # Rail limit lines
        self.pos_limit_hi = pg.InfiniteLine(pos=350.0, angle=0, pen=pg.mkPen('#f85149', width=1))
        self.pos_limit_lo = pg.InfiniteLine(pos=-350.0, angle=0, pen=pg.mkPen('#f85149', width=1))
        self.plot_pos.addItem(self.pos_limit_hi)
        self.plot_pos.addItem(self.pos_limit_lo)

        self.curves_angle = [self.plot_angle.plot([], [], pen=pg.mkPen('#58a6ff', width=2))
                             for _ in range(5)]  # copies shifted by -720..+720 deg
        self.curve_angle = self.curves_angle[2]   # the unshifted one
        self.curve_energy = self.plot_energy.plot([], [], pen=pg.mkPen('#f1e05a', width=2))
        self.curve_pos = self.plot_pos.plot([], [], pen=pg.mkPen('#79c0ff', width=2))
        self.curve_accel = self.plot_accel.plot([], [], pen=pg.mkPen('#ff7b72', width=2), name='Accel')
        self.curve_vel = self.plot_accel.plot([], [], pen=pg.mkPen('#d2a8ff', width=1.5), name='Vel')

        grid_layout.addWidget(self.plot_angle, 0, 0)
        grid_layout.addWidget(self.plot_energy, 0, 1)
        grid_layout.addWidget(self.plot_pos, 1, 0)
        grid_layout.addWidget(self.plot_accel, 1, 1)
        right_layout.addWidget(grid_widget, stretch=3)

        main_layout.addWidget(right_pane, stretch=1)

    # -------------------------------------------------------------
    # Riccati Solver & Calculation
    # -------------------------------------------------------------
    def _calculate_riccati(self):
        L = float(self.spin_L.value())
        fric = float(self.spin_fric.value())
        g = float(self.spin_g.value())
        Q = (float(self.spin_q0.value()), float(self.spin_q1.value()),
             float(self.spin_q2.value()), float(self.spin_q3.value()))
        R = float(self.spin_r.value())

        self.K_opt, self.P_opt, _, _ = solve_riccati_lqr(
            length_m=L, friction=fric, g=g, q_diag=Q, r_val=R, dt=0.002
        )

        self._refresh_gain_label()

    def _active_gains(self):
        """Return (K_sim, fw_gains, source).

        K_sim   : gains for simulate_full_pipeline, convention u = -K @ x
        fw_gains: gains to transmit to firmware (positive magnitudes), applied there as
                  a = -1*(kAngle*theta + kRate*rate) + 1*(kCartPos*x + kCartVel*v)
        The firmware plant is the simulation plant with the cart direction flipped
        (controlPolarity = -1), which turns u = -K@x with K<0 into the firmware form with
        fw = -K > 0. Hence fw_gains = -K_sim in both the Riccati and manual cases.
        """
        if self.chk_manual_gains.isChecked():
            fw = np.array([self.spin_kA.value(), self.spin_kR.value(),
                           self.spin_kP.value(), self.spin_kV.value()], dtype=float)
            return -fw, fw, 'MANUAL'
        K = np.asarray(self.K_opt, dtype=float).flatten()
        return K, -K, 'RICCATI'

    def _refresh_gain_label(self, *_):
        if getattr(self, 'K_opt', None) is None:
            return
        _, fw, source = self._active_gains()
        K = np.asarray(self.K_opt, dtype=float).flatten()
        self.gain_result_label.setText(
            f"Riccati K (-K@x):\n"
            f"  [{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}, {K[3]:.2f}]\n"
            f"Active ({source}) -> ESP32:\n"
            f"  kAngle={fw[0]:.2f}  kRate={fw[1]:.2f}\n"
            f"  kCartPos={fw[2]:.2f}  kCartVel={fw[3]:.2f}")

    # -------------------------------------------------------------
    # Simulation Execution
    # -------------------------------------------------------------
    def run_simulation(self):
        self._calculate_riccati()
        res = simulate_full_pipeline(
            length_m=float(self.spin_L.value()),
            friction=float(self.spin_fric.value()),
            g=float(self.spin_g.value()),
            K_lqr=self._active_gains()[0],
            swing_speed=float(self.spin_swing_speed.value()),
            swing_accel=float(self.spin_swing_accel.value()),
            cart_limit=float(self.spin_cart_limit.value()),
            max_speed=float(self.spin_max_speed.value()),
            max_accel=float(self.spin_max_accel.value()),
            dt=0.002, total_time_s=10.0,
            initial_angle_deg=175.0
        )
        self._sim_result = res
        self._sim_idx = 0
        self._show_sim_plots()

    def _apply_rail_range(self):
        limit_mm = float(self.spin_cart_limit.value()) * 1000.0
        self.pos_limit_hi.setValue(limit_mm)
        self.pos_limit_lo.setValue(-limit_mm)
        self.pendulum_widget.set_range(limit_mm)

    def _show_sim_plots(self):
        """Plot the full simulated trajectories (also used to restore them after a disconnect)."""
        res = self._sim_result
        if res is None:
            return
        t = res['time']
        for plot in (self.plot_energy, self.plot_pos, self.plot_accel):
            plot.enableAutoRange()
        self.plot_angle.enableAutoRange(axis='x')
        self._set_angle_curves(t, res['theta_deg'])
        self.curve_energy.setData(t, res['energy'])
        self.curve_pos.setData(t, res['pos_mm'])
        self.curve_accel.setData(t, res['accel'])
        self.curve_vel.setData(t, res['vel_m_s'])
        self._apply_rail_range()

    # -------------------------------------------------------------
    # Live telemetry view (graphs + display sign options)
    # -------------------------------------------------------------
    def _live_view(self):
        """Telemetry converted to display units/signs (flip options applied)."""
        tel = self._live_telemetry
        sc = -1.0 if self.chk_flip_cart.isChecked() else 1.0   # cart: pos / vel / accel
        st = -1.0 if self.chk_flip_tilt.isChecked() else 1.0   # pendulum: angle / rate
        return {
            'angle_deg': st * math.degrees(tel['angle']),
            'pos_mm': sc * tel['pos'] * 1000.0,
            'vel': sc * tel['vel'],
            'accel': sc * tel['accel'],
            'energy': tel['energy'],
            'mode': int(tel['mode']),
        }

    def _set_angle_curves(self, t, deg):
        """Draw the angle (any wrap, upright = 0) centred on 180 deg = hanging down.

        The angle is taken modulo 360, unwrapped into a continuous trace and drawn as copies
        shifted by multiples of 360 so that a swing through the upright position (the plot
        edges) continues across the edge.
        """
        t = np.asarray(t, dtype=float)
        deg = np.asarray(deg, dtype=float)
        if len(deg) == 0:
            return
        base = np.mod(deg, 360.0)
        u = np.degrees(np.unwrap(np.radians(base)))
        u = u - 360.0 * np.floor(u[0] / 360.0)      # start inside [0, 360)
        for k, curve in zip((-2, -1, 0, 1, 2), self.curves_angle):
            curve.setData(t, u + 360.0 * k)
        self.plot_angle.setYRange(self.angle_view[0], self.angle_view[1], padding=0)

    def _enter_live_plots(self):
        for k in self._live_buf:
            self._live_buf[k].clear()
        self._live_t0 = time.monotonic()
        self._live_plots_active = True
        self._apply_rail_range()

    def _exit_live_plots(self):
        """Back to the simulation plots (called on disconnect)."""
        self._live_plots_active = False
        self._show_sim_plots()

    def _update_live_plots(self, v):
        b = self._live_buf
        t = time.monotonic() - self._live_t0
        b['t'].append(t)
        b['angle'].append(v['angle_deg'])
        b['energy'].append(v['energy'])
        b['pos'].append(v['pos_mm'])
        b['accel'].append(v['accel'])
        b['vel'].append(v['vel'])

        ts = np.fromiter(b['t'], dtype=float)
        self._set_angle_curves(ts, np.fromiter(b['angle'], dtype=float))
        self.curve_energy.setData(ts, np.fromiter(b['energy'], dtype=float))
        self.curve_pos.setData(ts, np.fromiter(b['pos'], dtype=float))
        self.curve_accel.setData(ts, np.fromiter(b['accel'], dtype=float))
        self.curve_vel.setData(ts, np.fromiter(b['vel'], dtype=float))
        # scrolling ~10 s window, right edge = now
        lo, hi = max(0.0, t - 10.0), max(10.0, t)
        for plot in (self.plot_angle, self.plot_energy, self.plot_pos, self.plot_accel):
            plot.setXRange(lo, hi, padding=0)

    # -------------------------------------------------------------
    # Animation and Telemetry Tick
    # -------------------------------------------------------------
    def _on_tick(self):
        if self.serial is not None and self._rx_count > 0:
            # Live hardware display (flip options applied to screen, graphs and text)
            if not self._live_plots_active:
                self._enter_live_plots()
            v = self._live_view()
            self.pendulum_widget.set_state(180.0 + v['angle_deg'], v['pos_mm'])
            mode_names = {0: "SAFE", 1: "ENERGY SWING-UP", 2: "RICCATI LQR BALANCE"}
            mode_str = mode_names.get(v['mode'], "UNKNOWN")
            self.status_label.setText(
                f"ONLINE | Mode: {mode_str} | Ang: {v['angle_deg']:+.1f}° "
                f"E: {v['energy']:+.2f} Pos: {v['pos_mm']:+.1f}mm"
            )
            self._update_live_plots(v)
        elif self._sim_result is not None:
            # Simulation playback
            n = len(self._sim_result['time'])
            step = max(1, int(0.033 / 0.002))
            self._sim_idx = (self._sim_idx + step) % n
            # Upright is 0 in simulation; canvas expects 180 as upright
            angle_screen_deg = 180.0 + self._sim_result['theta_deg'][self._sim_idx]
            pos_mm = self._sim_result['pos_mm'][self._sim_idx]
            self.pendulum_widget.set_state(angle_screen_deg, pos_mm)

    # -------------------------------------------------------------
    # Serial Communication Handlers
    # -------------------------------------------------------------
    def _toggle_connection(self):
        if self.serial is None:
            port = self.port_edit.text().strip()
            try:
                self.serial = PipelineSerial(port, 115200, callback=self._on_packet_received)
                self.serial.open()
                self.connect_btn.setText('Disconnect')
                self.connect_btn.setObjectName("dangerBtn")
                self.status_label.setText(f"Connected to {port}. Waiting for telemetry...")
                self._rx_count = 0
                self._live_plots_active = False
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Connection Error", str(e))
        else:
            try:
                self.serial.close()
            finally:
                self.serial = None
                self.connect_btn.setText('Connect ESP32')
                self.connect_btn.setObjectName("actionBtn")
                self.status_label.setText("Status: Offline (Local Simulation Active)")
                self._exit_live_plots()

    def _on_packet_received(self, addr, value):
        self._rx_count += 1
        if addr == 0x00: self._live_telemetry['angle'] = value
        elif addr == 0x01: self._live_telemetry['rate'] = value
        elif addr == 0x02: self._live_telemetry['pos'] = value
        elif addr == 0x03: self._live_telemetry['vel'] = value
        elif addr == 0x04: self._live_telemetry['energy'] = value
        elif addr == 0x05: self._live_telemetry['accel'] = value
        elif addr == 0x06: self._live_telemetry['mode'] = value

    def _send_k_to_firmware(self):
        if not self.serial:
            QtWidgets.QMessageBox.warning(self, "Not Connected", "Connect to ESP32 first.")
            return
        self._calculate_riccati()
        _, fw_gains, source = self._active_gains()
        # 0x20: kAngle, 0x21: kRate, 0x22: kCartPos, 0x23: kCartVel
        # The firmware expects POSITIVE gains (its own controlPolarity/cartSign supply the
        # signs). A raw Riccati K is negative (u = -K@x), so it must be sent as -K.
        addrs = [0x20, 0x21, 0x22, 0x23]
        for a, v in zip(addrs, fw_gains):
            self.serial.send_float(a, float(v))
        self._send_soft_state()
        self.status_label.setText(
            f"TX: {source} gains -> ESP32: "
            f"[{fw_gains[0]:.2f}, {fw_gains[1]:.2f}, {fw_gains[2]:.2f}, {fw_gains[3]:.2f}]"
            f"  soft={'ON' if self.chk_soft_balance.isChecked() else 'off'}")

    def _send_soft_state(self, *_):
        """Send soft-balance gains (0x24/0x25) and on/off (0x26). No-op when offline."""
        if not self.serial:
            return
        self.serial.send_float(0x24, float(self.spin_kAS.value()))
        self.serial.send_float(0x25, float(self.spin_kRS.value()))
        self.serial.send_float(0x26, 1.0 if self.chk_soft_balance.isChecked() else 0.0)

    def _send_limits_to_firmware(self):
        if not self.serial:
            QtWidgets.QMessageBox.warning(self, "Not Connected", "Connect to ESP32 first.")
            return
        params = [
            (0x01, float(self.spin_max_speed.value())),
            (0x02, float(self.spin_max_accel.value())),
            (0x07, float(self.spin_cart_limit.value())),
            (0x0B, float(self.spin_L.value())),
        ]
        for a, v in params:
            self.serial.send_float(a, v)
        self.status_label.setText("Transmitted kinematics & limits to ESP32.")

    def _cmd_swingup(self):
        if not self.serial:
            self.run_simulation()
            return
        self._send_limits_to_firmware()
        self._send_k_to_firmware()
        self.serial.set_move_mode(1) # PIPELINE_SWING
        self.status_label.setText("TX: Energy Swing-up commanded.")

    def _cmd_balance(self):
        if not self.serial:
            return
        self._send_k_to_firmware()
        self.serial.set_move_mode(2) # PIPELINE_BALANCE
        self.status_label.setText("TX: Direct LQR Balance commanded.")

    def _cmd_stop(self):
        if not self.serial:
            return
        self.serial.set_move_mode(0) # PIPELINE_SAFE
        self.status_label.setText("TX: Emergency Stop / Safe mode commanded.")

    def _cmd_zero_position(self):
        if not self.serial:
            self.status_label.setText("Zero Position: not connected.")
            return
        self.serial.send_float(0x53, 0.0)  # re-define current cart position as 0
        self.status_label.setText("TX: Cart position zeroed (encoder zero unchanged).")

    def _cmd_zero(self):
        if not self.serial:
            return
        self.serial.send_float(0x52, 0.0) # Reset encoder downward
        self.status_label.setText("TX: Encoder zeroed downward.")


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = PipelineMainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
