
import sys
import struct
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import math
import numpy as np

try:
    from .serial_protocol import SerialProtocol
    from . import simulator
except ImportError:
    from serial_protocol import SerialProtocol
    import simulator


class PendulumWidget(QtWidgets.QWidget):
    """Simple visualization: horizontal rail with a cart and a pendulum rod.
    set_state(angle_deg, position_mm) updates the display.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle_deg = 0.0
        self.position_mm = 0.0
        self.setMinimumHeight(200)

    def set_state(self, angle_deg, position_mm):
        self.angle_deg = angle_deg
        self.position_mm = position_mm
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        try:
            rect = self.rect()
            w, h = rect.width(), rect.height()
            # draw baseline (rail)
            rail_y = h * 0.65
            rail_y_i = int(round(rail_y))
            p.setPen(QtGui.QPen(QtGui.QColor(50, 50, 50), 3))
            p.drawLine(10, rail_y_i, w - 10, rail_y_i)
            # map position_mm to x coordinate
            # assume ±150 mm range maps to widget width
            px = (self.position_mm + 150.0) / 300.0 * (w - 40) + 20
            # draw cart as rectangle
            cart_w, cart_h = 40, 18
            cart_rect = QtCore.QRectF(px - cart_w / 2, rail_y - cart_h, cart_w, cart_h)
            p.setBrush(QtGui.QBrush(QtGui.QColor(30, 144, 255)))
            p.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20)))
            p.drawRect(cart_rect)
            # pendulum pivot on cart top center
            pivot_x = px
            pivot_y = rail_y - cart_h
            # pendulum length in pixels
            length_px = 0.35 * h
            angle_rad = -self.angle_deg * math.pi / 180.0
            bob_x = pivot_x + length_px * math.sin(angle_rad)
            bob_y = pivot_y + length_px * math.cos(angle_rad)
            p.setPen(QtGui.QPen(QtGui.QColor(10, 10, 10), 3))
            p.drawLine(QtCore.QPointF(pivot_x, pivot_y), QtCore.QPointF(bob_x, bob_y))
            # draw bob
            p.setBrush(QtGui.QBrush(QtGui.QColor(220, 20, 60)))
            p.drawEllipse(QtCore.QPointF(bob_x, bob_y), 8, 8)
        finally:
            p.end()



class OptimizeWorker(QtCore.QThread):
    """K 4개를 Nelder-Mead로 직접 탐색하는 백그라운드 스레드."""
    finished = QtCore.pyqtSignal(dict)   # 최종 best 결과 emit
 
    def __init__(self, Ad, Bd, x0, steps, dt,
                 lim_acc, lim_pos, lim_vel,
                 Q_ui, r_ui, init_k, parent=None):
        super().__init__(parent)
        self.Ad, self.Bd         = Ad, Bd
        self.x0                  = x0
        self.steps, self.dt      = steps, dt
        self.lim_acc             = lim_acc
        self.lim_pos             = lim_pos
        self.lim_vel             = lim_vel
        self.Q_ui, self.r_ui     = Q_ui, r_ui
        self.init_k              = init_k.copy()
        self._stop               = False
        self._best               = {'cost': np.inf, 'k': init_k.copy(),
                                    'ev': None, 'nit': 0}
 
    def request_stop(self):
        self._stop = True
 
    # ── 1회 평가: K → 시뮬 → (ev, cost) ──────────────────────────────
    def _eval(self, k_vals):
        try:
            k_use = np.array([k_vals])
            traj  = simulator.simulate(self.Ad, self.Bd, -k_use,
                                       self.x0, self.steps, u_limit=4000.0)
            if np.any(np.isnan(traj)) or np.max(np.abs(traj)) > 1e5:
                return None, np.inf
 
            u_arr    = traj @ k_use.flatten()
            peak_acc = float(np.max(np.abs(u_arr)))
            peak_pos = float(np.max(np.abs(traj[:, 2])))
            peak_vel = float(np.max(np.abs(traj[:, 3])))
 
            # 리카티 비용 적분 (벡터화)
            xQx  = np.einsum('ij,jk,ik->i', traj, self.Q_ui, traj)
            cost = float(np.sum(xQx + self.r_ui * u_arr ** 2) * self.dt)
 
            # 한계 초과 페널티
            PENALTY = 1e8
            def pen(val, lim):
                return PENALTY * ((val / lim) - 1.0) ** 2 if val > lim else 0.0
            cost += pen(peak_acc, self.lim_acc)
            cost += pen(peak_pos, self.lim_pos)
            cost += pen(peak_vel, self.lim_vel)
 
            ev = dict(u=u_arr, traj=traj,
                      peak_acc=peak_acc, peak_pos=peak_pos, peak_vel=peak_vel)
            return ev, cost
        except Exception:
            return None, np.inf
 
    def run(self):
        from scipy.optimize import minimize
        nit = [0]
 
        def objective(k_vals):
            ev, cost = self._eval(k_vals)
            if ev is not None and cost < self._best['cost']:
                self._best.update(cost=cost, k=k_vals.copy(),
                                  ev=ev, nit=nit[0])
            return cost
 
        def callback(xk):
            nit[0] += 1
            if self._stop:
                raise StopIteration
 
        try:
            minimize(objective, self.init_k, method='Nelder-Mead',
                     callback=callback,
                     options={'maxiter': 2000, 'xatol': 1e-3,
                              'fatol': 1e-3, 'adaptive': True})
        except (StopIteration, Exception):
            pass
 
        self.finished.emit(dict(self._best))
 
 
ADDR = {
    'angle': 0x00,
    'angularVelocity': 0x01,
    'position': 0x02,
    'velocity': 0x03,
    'magnitude': 0x00,
    'speed': 0x01,
    'acceleration': 0x02,
    'threshold': 0x03,
    'setMoveMode': 0x50,
    'kpa': 0x20,
    'kda': 0x21,
    'kpm': 0x22,
    'kdm': 0x23,
}


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.is_initialized = False
        self.setWindowTitle('Pendulum Tuning Dashboard')
        # 전체 창 크기를 1700 -> 1800으로 늘려 가로 공간을 더 확보했습니다.
        self.resize(1800, 950)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #f0f2f5; }
            QGroupBox { font-weight: bold; border: 1px solid #b0b5bd; border-radius: 5px; margin-top: 10px; background-color: #ffffff; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; color: #333333; }
            QPushButton { background-color: #e0e4e8; border: 1px solid #a0a5ad; border-radius: 4px; padding: 5px; font-weight: bold; }
            QPushButton:hover { background-color: #d0d4d8; }
            QPushButton#startBtn { background-color: #1f66ff; color: white; border: none; font-size: 14px; padding: 10px;}
            QPushButton#stopBtn { background-color: #ff3b30; color: white; border: none; font-size: 14px; padding: 10px;}
            QLabel#riccatiLabel { background-color: #4a90e2; color: white; font-size: 16px; font-weight: bold; padding: 8px; border-radius: 5px; }
            QLabel#sectionHeader { font-size: 16px; font-weight: bold; color: #2c3e50; }
        """)

        widget = QtWidgets.QWidget()
        self.setCentralWidget(widget)
        main_layout = QtWidgets.QHBoxLayout(widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # ==========================================
        # LEFT PANE: Device Control
        # ==========================================
        left_widget = QtWidgets.QWidget()
        # 글씨가 잘리지 않도록 460 -> 500으로 대폭 확대
        left_widget.setFixedWidth(500)
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        header_dev = QtWidgets.QLabel("Device control")
        header_dev.setObjectName("sectionHeader")
        header_dev.setAlignment(QtCore.Qt.AlignCenter)
        left_layout.addWidget(header_dev)

        grp_reg = QtWidgets.QGroupBox('Regulation coefficients')
        grp_reg.setStyleSheet("QGroupBox { background-color: #eef4fc; border: 1px solid #aec2e8; }")
        lreg = QtWidgets.QVBoxLayout()
        self.kpa_container, self.kpa_spin, self.kpa_slider = self.create_gain_slider('kpa [m/rad/s^2] =', -500.0, 500.0, 144.6164)
        self.kda_container, self.kda_spin, self.kda_slider = self.create_gain_slider('kda [m/rad/s] =', -100.0, 100.0, 22.0)
        self.kpm_container, self.kpm_spin, self.kpm_slider = self.create_gain_slider('kpm [1/s^2] =', -500.0, 500.0, 224.0)
        self.kdm_container, self.kdm_spin, self.kdm_slider = self.create_gain_slider('kdm [1/s] =', -100.0, 100.0, 62.0)
        lreg.addWidget(self.kpa_container); lreg.addWidget(self.kda_container)
        lreg.addWidget(self.kpm_container); lreg.addWidget(self.kdm_container)
        self.send_k_btn = QtWidgets.QPushButton('Send K to Firmware')
        lreg.addWidget(self.send_k_btn)
        grp_reg.setLayout(lreg)
        left_layout.addWidget(grp_reg)

        grp_limits = QtWidgets.QGroupBox('Limits parameters')
        grp_limits.setStyleSheet("QGroupBox { background-color: #fceeee; border: 1px solid #e8aec0; }")
        llim = QtWidgets.QFormLayout()
        self.max_magnitude = QtWidgets.QDoubleSpinBox(); self.max_magnitude.setRange(0,1000); self.max_magnitude.setValue(120)
        self.max_speed = QtWidgets.QDoubleSpinBox(); self.max_speed.setRange(0,10000); self.max_speed.setValue(600)
        self.max_acc = QtWidgets.QDoubleSpinBox(); self.max_acc.setRange(0,100); self.max_acc.setValue(3.3)
        llim.addRow('Max magnitude (rail) [mm] =', self.max_magnitude)
        llim.addRow('Max speed [mm/s] =', self.max_speed)
        llim.addRow('Max acceleration [m/s^2] =', self.max_acc)
        grp_limits.setLayout(llim)
        left_layout.addWidget(grp_limits)

        grp_loop = QtWidgets.QGroupBox('Start parameters')
        grp_loop.setStyleSheet("QGroupBox { background-color: #fcf8ee; border: 1px solid #e8dea8; }")
        lloop = QtWidgets.QFormLayout()
        self.mag1 = QtWidgets.QDoubleSpinBox(); self.mag1.setRange(0,1000); self.mag1.setValue(56.21)
        self.mag2 = QtWidgets.QDoubleSpinBox(); self.mag2.setRange(0,1000); self.mag2.setValue(100)
        self.progression = QtWidgets.QDoubleSpinBox(); self.progression.setRange(0,10); self.progression.setValue(1)
        self.threshold = QtWidgets.QDoubleSpinBox(); self.threshold.setRange(0,1000); self.threshold.setValue(160)
        self.balance_pos = QtWidgets.QDoubleSpinBox(); self.balance_pos.setRange(-360, 360); self.balance_pos.setValue(180)
        lloop.addRow('Magnitude 1 [mm] =', self.mag1); lloop.addRow('Magnitude 2 [mm] =', self.mag2)
        lloop.addRow('Progression =', self.progression); lloop.addRow('Threshold [degree] =', self.threshold)
        lloop.addRow('Balance Pos. [degree] =', self.balance_pos)
        grp_loop.setLayout(lloop)
        left_layout.addWidget(grp_loop)

        grp_looping = QtWidgets.QGroupBox('Looping parameters')
        grp_looping.setStyleSheet("QGroupBox { background-color: #f4eefc; border: 1px solid #c8a8e8; }")
        llooping = QtWidgets.QFormLayout()
        self.first_impulse = QtWidgets.QDoubleSpinBox(); self.first_impulse.setRange(-1000, 1000); self.first_impulse.setValue(10)
        self.second_impulse = QtWidgets.QDoubleSpinBox(); self.second_impulse.setRange(-1000, 1000); self.second_impulse.setValue(13)
        self.angle_2nd_impulse = QtWidgets.QDoubleSpinBox(); self.angle_2nd_impulse.setRange(-360, 360); self.angle_2nd_impulse.setValue(140)
        llooping.addRow('First impulse [mm] =', self.first_impulse); llooping.addRow('Second impulse [mm] =', self.second_impulse)
        llooping.addRow('Angle of 2nd impulse [deg] =', self.angle_2nd_impulse)
        grp_looping.setLayout(llooping)
        left_layout.addWidget(grp_looping)

        grp_port = QtWidgets.QGroupBox('Port / Control')
        grp_port.setStyleSheet("QGroupBox { background-color: #eefce8; border: 1px solid #a4d19b; }")
        lport = QtWidgets.QVBoxLayout()
        hport = QtWidgets.QHBoxLayout()
        self.port_edit = QtWidgets.QLineEdit('COM3')
        self.connect_btn = QtWidgets.QPushButton('Connect')
        hport.addWidget(self.port_edit); hport.addWidget(self.connect_btn)
        lport.addLayout(hport)
        btns = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton('Start')
        self.start_btn.setObjectName("startBtn")
        self.balance_btn = QtWidgets.QPushButton('Balance')
        self.stop_btn = QtWidgets.QPushButton('Stop')
        self.stop_btn.setObjectName("stopBtn")
        btns.addWidget(self.start_btn); btns.addWidget(self.balance_btn); btns.addWidget(self.stop_btn)
        lport.addLayout(btns)
        grp_port.setLayout(lport)
        left_layout.addWidget(grp_port)

        main_layout.addWidget(left_widget)

        # ==========================================
        # MIDDLE PANE: Simulation Parameters
        # ==========================================
        mid_widget = QtWidgets.QWidget()
        # 글씨가 잘리지 않도록 380 -> 420으로 확대
        mid_widget.setFixedWidth(420)
        mid_layout = QtWidgets.QVBoxLayout(mid_widget)
        header_sim = QtWidgets.QLabel("Simulation")
        header_sim.setObjectName("sectionHeader")
        header_sim.setAlignment(QtCore.Qt.AlignCenter)
        mid_layout.addWidget(header_sim)

        grp_sim = QtWidgets.QGroupBox('Simulation parameters')
        grp_sim.setStyleSheet("QGroupBox { background-color: #fcf3ee; border: 1px solid #e8ba9b; }")
        lsim = QtWidgets.QFormLayout()
        self.sim_g = QtWidgets.QDoubleSpinBox(); self.sim_g.setRange(0,20); self.sim_g.setValue(9.81)
        self.sim_friction = QtWidgets.QDoubleSpinBox(); self.sim_friction.setRange(0.0,10.0); self.sim_friction.setDecimals(4); self.sim_friction.setValue(0.04)
        self.sim_l = QtWidgets.QDoubleSpinBox(); self.sim_l.setRange(1.0,1000.0); self.sim_l.setDecimals(1); self.sim_l.setValue(305.0)
        self.sim_dt = QtWidgets.QDoubleSpinBox(); self.sim_dt.setRange(0.0001,1.0); self.sim_dt.setDecimals(4); self.sim_dt.setValue(0.001)
        self.sim_steps = QtWidgets.QSpinBox(); self.sim_steps.setRange(1,50000); self.sim_steps.setValue(6000)
        self.initial_angle = QtWidgets.QDoubleSpinBox(); self.initial_angle.setRange(-360,360); self.initial_angle.setDecimals(3); self.initial_angle.setValue(12.0)
        lsim.addRow('Gravity [m/s^2]', self.sim_g); lsim.addRow('Friction coefficient []', self.sim_friction)
        lsim.addRow('Length of pendulum [mm]', self.sim_l); lsim.addRow('Time step [s]', self.sim_dt)
        lsim.addRow('Step Counts', self.sim_steps); lsim.addRow('Initial angle [degree]', self.initial_angle)
        grp_sim.setLayout(lsim)
        mid_layout.addWidget(grp_sim)

        grp_qr = QtWidgets.QGroupBox('Ponderation coefficients')
        grp_qr.setStyleSheet("QGroupBox { background-color: #eef4fc; border: 1px solid #9bb5e8; }")
        lqr = QtWidgets.QFormLayout()
        self.q0 = QtWidgets.QDoubleSpinBox(); self.q0.setRange(0,1e6); self.q0.setValue(1.0)
        self.q1 = QtWidgets.QDoubleSpinBox(); self.q1.setRange(0,1e6); self.q1.setValue(1.0)
        self.q2 = QtWidgets.QDoubleSpinBox(); self.q2.setRange(0,1e6); self.q2.setValue(1.0)
        self.q3 = QtWidgets.QDoubleSpinBox(); self.q3.setRange(0,1e6); self.q3.setValue(1.0)
        self.r1 = QtWidgets.QDoubleSpinBox(); self.r1.setRange(1e-9,1e6); self.r1.setValue(1.0)
        lqr.addRow('q0', self.q0); lqr.addRow('q1', self.q1); lqr.addRow('q2', self.q2); lqr.addRow('q3', self.q3); lqr.addRow('r', self.r1)
        self.calc_lqr_btn = QtWidgets.QPushButton('Auto Calculate optimal K')
        self.calc_lqr_btn.setStyleSheet("background-color: #4a90e2; color: white; padding: 6px;")
        self.stop_opt_btn = QtWidgets.QPushButton('■ Stop')
        self.stop_opt_btn.setStyleSheet("background-color: #ff3b30; color: white; padding: 6px; font-weight: bold;")
        self.stop_opt_btn.setVisible(False)
        btn_row = QtWidgets.QWidget()
        btn_hl  = QtWidgets.QHBoxLayout(btn_row)
        btn_hl.setContentsMargins(0, 0, 0, 0)
        btn_hl.addWidget(self.calc_lqr_btn, stretch=3)
        btn_hl.addWidget(self.stop_opt_btn, stretch=1)
        lqr.addRow(btn_row)
        grp_qr.setLayout(lqr)
        mid_layout.addWidget(grp_qr)

        self.riccati_label = QtWidgets.QLabel('Ricatti Integration: 0.000000')
        self.riccati_label.setObjectName("riccatiLabel")
        self.riccati_label.setAlignment(QtCore.Qt.AlignCenter)
        mid_layout.addWidget(self.riccati_label)
        mid_layout.addStretch()
        main_layout.addWidget(mid_widget)

        # ==========================================
        # RIGHT PANE: 2x2 Grid Graphs
        # ==========================================
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QGridLayout(right_widget)
        right_layout.setContentsMargins(0, 30, 0, 0)

        pg.setConfigOption('background', '#e8f0eb')
        pg.setConfigOption('foreground', 'k')

        self.plot_angle = pg.PlotWidget(title='Angle (degree)')
        self.plot_pos = pg.PlotWidget(title='Position (mm)')
        self.plot_acc = pg.PlotWidget(title='Control / Acceleration (m/s^2)')
        self.plot_vel = pg.PlotWidget(title='Cart Speed (mm/s)')
        
        for p in [self.plot_angle, self.plot_pos, self.plot_acc, self.plot_vel]:
            p.showGrid(x=True, y=True, alpha=0.5)
            p.setLabel('bottom', 'Time (ms)')

        right_layout.addWidget(self.plot_angle, 0, 0)
        right_layout.addWidget(self.plot_pos, 1, 0)
        right_layout.addWidget(self.plot_acc, 0, 1)
        right_layout.addWidget(self.plot_vel, 1, 1)
        main_layout.addWidget(right_widget, stretch=1)

        self.sim_angle_curve = self.plot_angle.plot([], [], pen=pg.mkPen('r', width=2))
        self.sim_pos_curve = self.plot_pos.plot([], [], pen=pg.mkPen('r', width=2))
        self.sim_acc_curve = self.plot_acc.plot([], [], pen=pg.mkPen('r', width=2))
        self.sim_vel_curve = self.plot_vel.plot([], [], pen=pg.mkPen('r', width=2))

        self.pos_limit_high = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#1fc4d4', width=3))
        self.pos_limit_low = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#1fc4d4', width=3))
        self.plot_pos.addItem(self.pos_limit_high); self.plot_pos.addItem(self.pos_limit_low)

        self.acc_limit_high = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#1fc4d4', width=3))
        self.acc_limit_low = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#1fc4d4', width=3))
        self.plot_acc.addItem(self.acc_limit_high); self.plot_acc.addItem(self.acc_limit_low)

        self.vel_limit_high = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#1fc4d4', width=3))
        self.vel_limit_low = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#1fc4d4', width=3))
        self.plot_vel.addItem(self.vel_limit_high); self.plot_vel.addItem(self.vel_limit_low)

        self.serial = None

        self.connect_btn.clicked.connect(self.on_connect)
        self.start_btn.clicked.connect(self.on_start)
        self.balance_btn.clicked.connect(self.on_balance)
        self.stop_btn.clicked.connect(self.on_stop)
        
        self.calc_lqr_btn.clicked.connect(self.calculate_lqr)
        self.stop_opt_btn.clicked.connect(self._stop_optimization)
        self.send_k_btn.clicked.connect(self.send_manual_gains)
        self.max_magnitude.valueChanged.connect(self.update_limit_lines)
        self.max_acc.valueChanged.connect(self.update_limit_lines)
        self.max_speed.valueChanged.connect(self.update_limit_lines)
        
        self.sim_g.valueChanged.connect(lambda val: self.run_simulation())
        self.sim_friction.valueChanged.connect(lambda val: self.run_simulation())
        self.sim_l.valueChanged.connect(lambda val: self.run_simulation())
        self.sim_dt.valueChanged.connect(lambda val: self.run_simulation())
        self.sim_steps.valueChanged.connect(lambda val: self.run_simulation())
        self.initial_angle.valueChanged.connect(lambda val: self.run_simulation())
        self.q0.valueChanged.connect(lambda val: self.run_simulation())
        self.q1.valueChanged.connect(lambda val: self.run_simulation())
        self.q2.valueChanged.connect(lambda val: self.run_simulation())
        self.q3.valueChanged.connect(lambda val: self.run_simulation())
        self.r1.valueChanged.connect(lambda val: self.run_simulation())

        self.is_initialized = True
        self.update_limit_lines()
        self.run_simulation()

    # ---------------------------------------------------------
    def create_gain_slider(self, label_text, min_val, max_val, default_val):
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0); layout.setSpacing(2)
        
        top_layout = QtWidgets.QHBoxLayout()
        label = QtWidgets.QLabel(label_text)
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(-10000.0, 10000.0); spin.setDecimals(4); spin.setValue(default_val)
        spin.setStyleSheet("border: none; font-weight: bold; color: #1f66ff; background: transparent;")
        top_layout.addWidget(label); top_layout.addWidget(spin); top_layout.addStretch()
        layout.addLayout(top_layout)
        
        bot_layout = QtWidgets.QHBoxLayout()
        min_label = QtWidgets.QLabel(str(min_val))
        min_label.setStyleSheet("font-size: 10px; color: gray;")
        max_label = QtWidgets.QLabel(str(max_val))
        max_label.setStyleSheet("font-size: 10px; color: gray;")
        
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        scale = 100.0
        slider.setRange(int(min_val * scale), int(max_val * scale)); slider.setValue(int(default_val * scale))
        slider.setStyleSheet("""
            QSlider::groove:horizontal { border: 1px solid #bbb; background: white; height: 6px; border-radius: 3px; }
            QSlider::sub-page:horizontal { background: #ff4c4c; border: 1px solid #777; height: 6px; border-radius: 3px; }
            QSlider::add-page:horizontal { background: #e0e0e0; border: 1px solid #777; height: 6px; border-radius: 3px; }
            QSlider::handle:horizontal { background: white; border: 1px solid #777; width: 14px; margin-top: -4px; margin-bottom: -4px; border-radius: 7px; }
        """)
        bot_layout.addWidget(min_label); bot_layout.addWidget(slider); bot_layout.addWidget(max_label)
        layout.addLayout(bot_layout)
        
        def spin_changed(val):
            slider.blockSignals(True); slider.setValue(int(val * scale)); slider.blockSignals(False)
            self.run_simulation()
        def slider_changed(val):
            spin.blockSignals(True); spin.setValue(val / scale); spin.blockSignals(False)
            self.run_simulation()
            
        spin.valueChanged.connect(spin_changed); slider.valueChanged.connect(slider_changed)
        return container, spin, slider

    # ---------------------------------------------------------
    def run_simulation(self):
        if not self.is_initialized: return
        
        g = float(self.sim_g.value()); friction = float(self.sim_friction.value())
        length_mm = float(self.sim_l.value()); dt = float(self.sim_dt.value())
        steps = int(self.sim_steps.value())

        Ac, Bc = simulator.model_matrices(g=g, friction=friction, length_mm=length_mm)
        Ad, Bd = simulator.discretize(Ac, Bc, dt)

        k_use = np.array([[
            float(self.kpa_spin.value()), float(self.kda_spin.value()),
            float(self.kpm_spin.value()), float(self.kdm_spin.value())
        ]], dtype=float)

        ia_deg = float(self.initial_angle.value())
        ia_rad = ia_deg * math.pi / 180.0
        x0 = [ia_rad, 0.0, 0.0, 0.0]
        
        # ★ 롤백됨: 시뮬레이터 자체는 한계를 적용하지 않은 '이상적(4000.0)' 상태로 궤적을 생성합니다.
        traj = simulator.simulate(Ad, Bd, -k_use, x0, steps, u_limit=4000.0)

        angle_deg = 180.0 + traj[:, 0] * 180.0 / math.pi
        pos_mm = traj[:, 2] * 1000.0
        vel_mm = traj[:, 3] * 1000.0
        
        u_array = np.dot(traj, k_use.flatten()) 
        t_ms = np.arange(len(angle_deg), dtype=float) * dt * 1000.0

        self.sim_angle_curve.setData(t_ms, angle_deg)
        self.sim_pos_curve.setData(t_ms, pos_mm)
        self.sim_vel_curve.setData(t_ms, vel_mm)
        self.sim_acc_curve.setData(t_ms, u_array)

        a_min, a_max = float(np.min(angle_deg)), float(np.max(angle_deg))
        p_min, p_max = float(np.min(pos_mm)), float(np.max(pos_mm))
        v_min, v_max = float(np.min(vel_mm)), float(np.max(vel_mm))
        u_min, u_max = float(np.min(u_array)), float(np.max(u_array))
        
        pos_limit = float(self.max_magnitude.value())
        vel_limit = float(self.max_speed.value())
        max_acc = float(self.max_acc.value())
        
        if a_max > 1e5 or a_min < -1e5:
            self.plot_angle.setYRange(0, 360)
            self.plot_pos.setYRange(-pos_limit*2, pos_limit*2)
            self.plot_vel.setYRange(-vel_limit*2, vel_limit*2)
            self.plot_acc.setYRange(-max_acc*2, max_acc*2)
        else:
            self.plot_angle.setYRange(min(90.0, a_min - 10), max(270.0, a_max + 10))
            self.plot_pos.setYRange(-max(pos_limit, abs(p_min), abs(p_max)) * 1.15, max(pos_limit, abs(p_min), abs(p_max)) * 1.15)
            self.plot_vel.setYRange(-max(vel_limit, abs(v_min), abs(v_max)) * 1.15, max(vel_limit, abs(v_min), abs(v_max)) * 1.15)
            self.plot_acc.setYRange(-max(max_acc, abs(u_min), abs(u_max)) * 1.15, max(max_acc, abs(u_min), abs(u_max)) * 1.15)

        # ★ 롤백됨: Riccati 계산에도 클리핑을 제거하여 '이상적 비용 점수'를 표기합니다.
        Q = np.diag([float(self.q0.value()), float(self.q1.value()), float(self.q2.value()), float(self.q3.value())])
        R_val = float(self.r1.value())
        
        cost_integral = 0.0
        for i in range(steps):
            x_i = traj[i] 
            u_i = u_array[i]
            
            stage_cost = np.dot(x_i, np.dot(Q, x_i)) + (u_i * R_val * u_i)
            cost_integral += stage_cost * dt
            if cost_integral > 1e9 or np.isnan(cost_integral): break
                
        if cost_integral > 1e9 or np.isnan(cost_integral):
            self.riccati_label.setText('Ricatti Integration: DIVERGED')
            self.riccati_label.setStyleSheet("background-color: #ff3b30; color: white; font-size: 16px; font-weight: bold; padding: 8px; border-radius: 5px;")
        else:
            self.riccati_label.setText(f'Ricatti Integration: {cost_integral:.6f}')
            self.riccati_label.setStyleSheet("background-color: #4a90e2; color: white; font-size: 16px; font-weight: bold; padding: 8px; border-radius: 5px;")

    def calculate_lqr(self):
        # ── 이미 실행 중이면 무시 ──────────────────────────────────────────
        if hasattr(self, '_opt_worker') and self._opt_worker is not None:
            return

        # ── 파라미터 수집 ─────────────────────────────────────────────────
        g         = float(self.sim_g.value())
        friction  = float(self.sim_friction.value())
        length_mm = float(self.sim_l.value())
        dt        = float(self.sim_dt.value())
        steps     = int(self.sim_steps.value())

        Ac, Bc = simulator.model_matrices(g=g, friction=friction, length_mm=length_mm)
        Ad, Bd = simulator.discretize(Ac, Bc, dt)

        lim_pos = float(self.max_magnitude.value()) / 1000.0
        lim_vel = float(self.max_speed.value())     / 1000.0
        lim_acc = float(self.max_acc.value())

        ia_rad = float(self.initial_angle.value()) * math.pi / 180.0
        x0     = np.array([ia_rad, 0.0, 0.0, 0.0])

        Q_ui = np.diag([float(self.q0.value()), float(self.q1.value()),
                        float(self.q2.value()), float(self.q3.value())])
        r_ui = float(self.r1.value())

        init_k = np.array([
            float(self.kpa_spin.value()), float(self.kda_spin.value()),
            float(self.kpm_spin.value()), float(self.kdm_spin.value()),
        ])

        # ── 워커 스레드 생성 & 시작 ──────────────────────────────────────
        self._opt_worker = OptimizeWorker(
            Ad, Bd, x0, steps, dt,
            lim_acc, lim_pos, lim_vel,
            Q_ui, r_ui, init_k, parent=self
        )
        self._opt_lim = (lim_acc, lim_pos, lim_vel)   # 결과 표시용
        self._opt_worker.finished.connect(self._on_opt_finished)

        self.calc_lqr_btn.setEnabled(False)
        self.stop_opt_btn.setVisible(True)
        self.riccati_label.setText('Optimizing K...  (■ Stop으로 중단)')
        self.riccati_label.setStyleSheet(
            "background-color: #f0a500; color: white; font-size: 14px;"
            " font-weight: bold; padding: 8px; border-radius: 5px;")

        self._opt_worker.start()

    def _stop_optimization(self):
        if hasattr(self, '_opt_worker') and self._opt_worker is not None:
            self._opt_worker.request_stop()

    def _on_opt_finished(self, best):
        # ── UI 버튼 복원 ─────────────────────────────────────────────────
        self.calc_lqr_btn.setEnabled(True)
        self.stop_opt_btn.setVisible(False)
        self._opt_worker = None

        ev = best.get('ev')
        if ev is None:
            QtWidgets.QMessageBox.critical(self, 'K 최적화 오류',
                                           '유효한 K를 찾지 못했습니다.\n'
                                           '초기 K값이 너무 불안정하거나 한계치가 너무 좁습니다.')
            self.riccati_label.setText('Ricatti Integration: N/A')
            self.riccati_label.setStyleSheet(
                "background-color: #ff3b30; color: white; font-size: 16px;"
                " font-weight: bold; padding: 8px; border-radius: 5px;")
            return

        lim_acc, lim_pos, lim_vel = self._opt_lim
        acc_ok = ev['peak_acc'] <= lim_acc
        pos_ok = ev['peak_pos'] <= lim_pos
        vel_ok = ev['peak_vel'] <= lim_vel
        all_ok = acc_ok and pos_ok and vel_ok

        kpa, kda, kpm, kdm = best['k']
        status = '✅ 모든 한계치 만족!' if all_ok else '⚠️ 최선의 결과 (일부 초과)'
        msg  = f"{status}\n\n"
        msg += f"가속도 : {ev['peak_acc']:.3f}  /  {lim_acc:.3f} m/s²    {'✓' if acc_ok else '✗'}\n"
        msg += f"위치   : {ev['peak_pos']*1000:.2f}  /  {lim_pos*1000:.2f} mm    {'✓' if pos_ok else '✗'}\n"
        msg += f"속도   : {ev['peak_vel']*1000:.2f}  /  {lim_vel*1000:.2f} mm/s  {'✓' if vel_ok else '✗'}\n"
        msg += f"\nkpa={kpa:.4f}  kda={kda:.4f}\nkpm={kpm:.4f}  kdm={kdm:.4f}\n"
        msg += f"\n탐색 반복: {best['nit']}회"
        QtWidgets.QMessageBox.information(self, 'K 최적화 결과', msg)

        # K 스핀박스 업데이트 → run_simulation 자동 트리거 (Q/R 건드리지 않음)
        self.kpa_spin.setValue(kpa); self.kda_spin.setValue(kda)
        self.kpm_spin.setValue(kpm); self.kdm_spin.setValue(kdm)


    def update_limit_lines(self):
        pos_limit = float(self.max_magnitude.value())
        self.pos_limit_high.setValue(pos_limit); self.pos_limit_low.setValue(-pos_limit)
        
        acc_limit = float(self.max_acc.value())
        self.acc_limit_high.setValue(acc_limit); self.acc_limit_low.setValue(-acc_limit)
        
        vel_limit = float(self.max_speed.value())
        self.vel_limit_high.setValue(vel_limit); self.vel_limit_low.setValue(-vel_limit)

    def on_connect(self):
        port = self.port_edit.text().strip()
        if self.serial is None:
            try:
                self.serial = SerialProtocol(port, 115200, callback=self.on_float)
                self.serial.open()
                self.connect_btn.setText('Disconnect')
            except Exception as e: QtWidgets.QMessageBox.critical(self, 'Serial error', str(e))
        else:
            try: self.serial.close()
            finally: self.serial = None; self.connect_btn.setText('Connect')

    def on_start(self):
        if self.serial:
            try: self.serial.set_move_mode(1)
            except Exception as e: print('send error', e)

    def on_balance(self):
        if self.serial:
            try: self.serial.set_move_mode(2)
            except Exception as e: print('send error', e)

    def on_stop(self):
        if self.serial:
            try: self.serial.set_move_mode(0)
            except Exception as e: print('send error', e)

    def on_float(self, addr, value): pass

    def send_manual_gains(self):
        if not self.serial:
            QtWidgets.QMessageBox.information(self, 'Serial', 'Connect to the device first.')
            return
        gains = [self.kpa_spin.value(), self.kda_spin.value(), self.kpm_spin.value(), self.kdm_spin.value()]
        addrs = [ADDR['kpa'], ADDR['kda'], ADDR['kpm'], ADDR['kdm']]
        try:
            for addr, gain in zip(addrs, gains): self.serial.send_float(addr, gain)
        except Exception as e: QtWidgets.QMessageBox.critical(self, 'Send K error', str(e))

def main():
    app = QtWidgets.QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()