import numpy as np
import scipy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
from collections import deque

# ==========================================
# 1. 시스템 및 제어 파라미터 (NEMA17 + 600 CPR 엔코더)
# ==========================================
params = {
    # 물리 모델
    "M_cart": 0.8, "m_enc": 0.15, "m_pend": 0.2, "L": 0.3, "I_pend": 0.006, "g": 9.81,
    "x0": 0.0,
    "theta0": 0.0,  # 초기: 위쪽 수직 (upright)

    "cart_damping": 1.2, "joint_damping": 0.01,

    # LQR 가중치
    "Q_x": 100.0, "Q_theta": 1000.0, "Q_xdot": 10.0, "Q_thetadot": 100.0,

    # 하드웨어 스펙 (NEMA17 스텝모터)
    "encoder_cpr": 600,         # 600 CPR 엔코더
    "motor_steps": 200,         # 스텝모터 200 steps/rev
    "microstepping": 16,        # TMC5160 마이크로스텝 (1/16)
    "pulley_circum": 0.04,      # 풀리 둘레 (m) - 40mm
    "control_dt": 0.01,         # 10ms 제어 주기
    "buffer_size": 5,

    # NEMA17 최대 토크 (~0.5 Nm = 50 N·cm)
    # 풀리 반지름 ~20mm이므로 최대 힘 = 0.5 / 0.02 = 25N
    "holding_force": 20.0,      # 정격 근력 (N)
    "max_rpm_speed": 2.0,       # 최고 속도 (rad/s)

    # 제어기 출력 한계
    "u_max": 15.0,

    # 외란 및 레일
    "dist_amp": 0.0, "dist_freq": 1.5,
    "rail_x_min": -1.0, "rail_x_max": 1.0,

    # Swing-Up 제어 파라미터
    "catch_angle": 0.35,
    "k_swing": 40.0,
    "k_center": 10.0,
    "kd_center": 8.0,
    # 교수님 제어 게인 (default 0 — GUI에서 입력)
    "kpa": 0.0, "kda": 0.0, "kpm": 0.0, "kdm": 0.0,

    # 하드웨어 시뮬레이션
    "sensor_noise_encoder": 0.001,   # 엔코더 노이즈 (rad)
    "sensor_noise_position": 0.0005, # 포지션 노이즈 (m)
    "latency_ms": 10.0,              # 통신 지연 (ms)
}

R = np.array([[1.0]])
t_span = (0, 7.0)
t_eval = np.linspace(t_span[0], t_span[1], int(7.0 / params["control_dt"]))

# ==========================================
# 2. 제어기 및 시뮬레이션 연산
# ==========================================
def compute_matrices(p):
    M_t, m, L, I, g = p["M_cart"] + p["m_enc"], p["m_pend"], p["L"], p["I_pend"], p["g"]
    bx, bt = p["cart_damping"], p["joint_damping"]

    M0 = np.array([[M_t + m, m * L], [m * L, I + m * L**2]])
    K_mat = np.array([[0, 0], [0, -m * g * L]])
    D_mat = np.array([[bx, 0], [0, bt]])

    Minv = la.inv(M0)
    A = np.zeros((4, 4))
    A[0:2, 2:4] = np.eye(2)
    A[2:4, 0:2] = -Minv @ K_mat
    A[2:4, 2:4] = -Minv @ D_mat

    B = np.zeros((4, 1))
    B[2:4, 0] = Minv[:, 0]

    Q = np.diag([p["Q_x"], p["Q_theta"], p["Q_xdot"], p["Q_thetadot"]])
    P = la.solve_continuous_are(A, B, Q, R)
    K = la.inv(R) @ B.T @ P
    return A, B, K

def compute_solution(p):
    A, B, K = compute_matrices(p)
    pos_res = p["pulley_circum"] / (p["motor_steps"] * p["microstepping"])
    angle_res = (2.0 * np.pi) / p["encoder_cpr"]
    u_quant = pos_res / (p["control_dt"]**2)

    # Match firmware's accelerationRatio mapping from Pendulum.cpp
    distanceRatio = p["pulley_circum"] / (float(p["motor_steps"]) * float(p["microstepping"]))
    accelerationRatio = 0.015 / distanceRatio

    def nonlinear_dynamics(t, X, u):
        x, theta, x_dot, theta_dot = X
        M_t, m, L, I, g = p["M_cart"] + p["m_enc"], p["m_pend"], p["L"], p["I_pend"], p["g"]

        # 모터 탈조 모델
        speed_factor = max(0.0, 1.0 - (abs(x_dot) / p["max_rpm_speed"]))
        if abs(u) > p["holding_force"] * speed_factor:
            u = 0.0

        # 가상 벽
        wall_force = 0.0
        if x < p["rail_x_min"]:
            wall_force = 5000.0 * (p["rail_x_min"] - x) - 100.0 * x_dot
            if u < 0: u = 0.0
        elif x > p["rail_x_max"]:
            wall_force = 5000.0 * (p["rail_x_max"] - x) - 100.0 * x_dot
            if u > 0: u = 0.0

        sin_t, cos_t = np.sin(theta), np.cos(theta)
        Mq = np.array([[M_t + m, m * L * cos_t], [m * L * cos_t, I + m * L**2]])
        nonlinear_terms = np.array([-m * L * sin_t * theta_dot**2, 0.0])
        gravity_terms = np.array([0.0, m * g * L * sin_t])
        damping_terms = np.array([p["cart_damping"] * x_dot, p["joint_damping"] * theta_dot])

        disturbance = p["dist_amp"] * np.sin(2.0 * np.pi * p["dist_freq"] * t)
        tau = np.array([u + disturbance + wall_force, 0.0])

        q_ddot = la.solve(Mq, tau - nonlinear_terms - damping_terms + gravity_terms)
        return np.array([x_dot, theta_dot, q_ddot[0], q_ddot[1]])

    controller = {
        "next_t": 0.0, "u_prev": 0.0, "mode": "SWING",
        "x_buffer": deque([p["x0"]] * int(p["buffer_size"]), maxlen=int(p["buffer_size"])),
        "theta_buffer": deque([p["theta0"]] * int(p["buffer_size"]), maxlen=int(p["buffer_size"]))
    }

    mode_history = []

    def closed_loop(t, X):
        if t + 1e-12 >= controller["next_t"]:
            x, theta, _, _ = X

            # 센서 양자화 + 노이즈
            q_x = np.round(x / pos_res) * pos_res + np.random.normal(0, p["sensor_noise_position"])
            q_theta_raw = np.round(theta / angle_res) * angle_res + np.random.normal(0, p["sensor_noise_encoder"])

            controller["x_buffer"].append(q_x)
            controller["theta_buffer"].append(q_theta_raw)

            dt_span = p["control_dt"] * (p["buffer_size"] - 1)
            est_x_dot = (controller["x_buffer"][-1] - controller["x_buffer"][0]) / dt_span
            est_theta_dot = (controller["theta_buffer"][-1] - controller["theta_buffer"][0]) / dt_span

            q_theta_wrap = (q_theta_raw + np.pi) % (2 * np.pi) - np.pi

            if abs(q_theta_wrap) < p["catch_angle"]:
                # 교수님 제어기 (근접 제어) — invertedBalance style
                controller["mode"] = "PROF"
                sign = 1 if q_theta_wrap >= 0 else -1
                # balancePos — use small offset (0 by default)
                balancePos = p.get('balancePos', 0.0)
                a = sign * (abs(q_theta_wrap) - balancePos)
                # professor control signal mapping (same form as Pendulum.cpp)
                controlSignal = (p.get('kpa', 0.0) * a + p.get('kda', 0.0) * est_theta_dot + p.get('kpm', 0.0) * q_x + p.get('kdm', 0.0) * est_x_dot) * accelerationRatio
                u_cmd = float(controlSignal)
            else:
                # Swing-up mode (unchanged)
                controller["mode"] = "SWING"
                J_total = p["I_pend"] + p["m_pend"] * p["L"]**2
                energy = 0.5 * J_total * est_theta_dot**2 + p["m_pend"] * p["g"] * p["L"] * (np.cos(q_theta_wrap) - 1.0)

                u_swing = p["k_swing"] * energy * est_theta_dot * np.cos(q_theta_wrap)
                u_center = -p["k_center"] * q_x - p["kd_center"] * est_x_dot
                u_cmd = u_swing + u_center

            u_cmd = np.clip(u_cmd, -p["u_max"], p["u_max"])
            u_hold = controller["u_prev"]
            controller["u_prev"] = float(u_quant * np.round(u_cmd / u_quant))
            controller["next_t"] += p["control_dt"]

        else:
            u_hold = controller["u_prev"]

        mode_history.append(controller["mode"])
        return nonlinear_dynamics(t, X, u_hold)

    X0 = np.array([p["x0"], p["theta0"], 0.0, 0.0])
    sol = solve_ivp(closed_loop, t_span, X0, t_eval=t_eval, max_step=0.01)

    final_modes = [mode_history[min(i, len(mode_history)-1)] for i in range(len(t_eval))]
    return sol, K, final_modes

# ==========================================
# 3. 시각화
# ==========================================
sol, K, modes = None, None, None
ani = None

fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=0.35)
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-0.4, 0.8)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Single Inverted Pendulum (NEMA17 + 600 CPR Encoder)")

cart_w, cart_h = 0.2, 0.1
rail_left = ax.axvline(x=params["rail_x_min"], color='gray', linestyle='--', lw=2)
rail_right = ax.axvline(x=params["rail_x_max"], color='gray', linestyle='--', lw=2)
ax.plot([params["rail_x_min"], params["rail_x_max"]], [-cart_h/2, -cart_h/2], 'k-', lw=3)

cart_rect = plt.Rectangle((-cart_w/2, -cart_h/2), cart_w, cart_h, fc='steelblue', ec='black')
ax.add_patch(cart_rect)
pendulum_line, = ax.plot([], [], 'o-', lw=4, markersize=10, color='firebrick')

axcolor = 'lightcyan'

# Text inputs for professor gains (type exact values)
gain_axes = {
    "kpa": plt.axes([0.08, 0.22, 0.35, 0.04], facecolor=axcolor),
    "kda": plt.axes([0.08, 0.16, 0.35, 0.04], facecolor=axcolor),
    "kpm": plt.axes([0.55, 0.22, 0.35, 0.04], facecolor=axcolor),
    "kdm": plt.axes([0.55, 0.16, 0.35, 0.04], facecolor=axcolor),
}

gain_boxes = {
    "kpa": TextBox(gain_axes["kpa"], 'kpa (angle P): ', initial=str(params.get('kpa', 0.0))),
    "kda": TextBox(gain_axes["kda"], 'kda (angle D): ', initial=str(params.get('kda', 0.0))),
    "kpm": TextBox(gain_axes["kpm"], 'kpm (pos P): ', initial=str(params.get('kpm', 0.0))),
    "kdm": TextBox(gain_axes["kdm"], 'kdm (pos D): ', initial=str(params.get('kdm', 0.0))),
}

# Initial angle input (degrees)
init_angle_axes = plt.axes([0.42, 0.02, 0.15, 0.04], facecolor='lightgrey')
init_angle_box = TextBox(init_angle_axes, 'Init Angle (deg): ', initial=str(np.degrees(params.get('theta0', 0.0))))

info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, va='top', fontsize=11, fontweight='bold')

def update_text(frame):
    if sol is None:
        info_text.set_text("Press Start to run simulation. Enter gains and click Start.")
        info_text.set_color('black')
        return
    mode = modes[frame]
    color = "green" if mode == "LQR" else ("green" if mode == "PROF" else "red")
    theta = (sol.y[1, frame] + np.pi) % (2 * np.pi) - np.pi
    info_text.set_text(f"MODE: {mode}  |  Angle: {np.degrees(theta):.1f}°\n"
                       f"Hardware: NEMA17 + 600CPR Encoder (16μ-step)\n"
                       f"Gains(kpa,kda,kpm,kdm): {params.get('kpa',0)},{params.get('kda',0)},{params.get('kpm',0)},{params.get('kdm',0)}")
    info_text.set_color(color)

def apply_gains():
    """Read gain TextBoxes and store into params."""
    for k, box in gain_boxes.items():
        try:
            params[k] = float(box.text.strip())
        except Exception:
            # ignore invalid input, keep previous
            pass


def on_init_angle_submit(text):
    try:
        deg = float(text.strip())
        params['theta0'] = np.deg2rad(deg)
        # if running, restart
        if globals().get('ani') is not None:
            start_simulation(None)
    except Exception:
        print('Invalid init angle')


def recompute(_=None):
    # kept for API compatibility: apply gains but don't auto-run
    apply_gains()
    fig.canvas.draw_idle()

def reset_clicked(event):
    """Stop the running simulation and reset cart/pendulum to initial state."""
    global sol, K, modes, ani
    # stop animation if running
    if globals().get('ani') is not None:
        try:
            ani.event_source.stop()
        except Exception:
            pass
    ani = None
    # clear solution so update shows initial pose
    sol, K, modes = None, None, None
    # reset cart/pendulum to initial params
    cart_rect.set_xy((params.get('x0', 0.0) - cart_w/2, -cart_h/2))
    theta0 = params.get('theta0', 0.0)
    px = params['x0'] + params['L'] * np.sin(theta0)
    py = params['L'] * np.cos(theta0)
    pendulum_line.set_data([params['x0'], px], [0, py])
    fig.canvas.draw_idle()

# bind TextBox submit handlers to apply immediately (do not auto-run)
for box in gain_boxes.values():
    box.on_submit(lambda text: apply_gains())

# initial angle submit handler
init_angle_box.on_submit(on_init_angle_submit)

def init():
    cart_rect.set_xy((-cart_w/2, -cart_h/2))
    pendulum_line.set_data([], [])
    return cart_rect, pendulum_line, info_text

def update(frame):
    x = sol.y[0, frame]
    theta = sol.y[1, frame]

    cart_rect.set_xy((x - cart_w/2, -cart_h/2))
    px = x + params["L"] * np.sin(theta)
    py = params["L"] * np.cos(theta)

    pendulum_line.set_data([x, px], [0, py])
    update_text(frame)
    return cart_rect, pendulum_line, info_text

# Start and Reset buttons
start_ax = plt.axes([0.78, 0.02, 0.12, 0.06])
start_button = Button(start_ax, 'Start', color='lightgreen', hovercolor='green')

stop_ax = plt.axes([0.64, 0.02, 0.12, 0.06])
stop_button = Button(stop_ax, 'Stop', color='lightcoral', hovercolor='red')

def start_simulation(event):
    global sol, K, modes, ani
    apply_gains()
    sol, K, modes = compute_solution(params)
    # stop previous animation
    if ani is not None:
        try:
            ani.event_source.stop()
        except Exception:
            pass
    ani = animation.FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=10)
    fig.canvas.draw_idle()

start_button.on_clicked(start_simulation)
stop_button.on_clicked(reset_clicked)

plt.show()
