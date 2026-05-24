import numpy as np
import scipy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, TextBox
from collections import deque

# ==========================================
# 1. 시스템 및 제어 파라미터 
# ==========================================
params = {
    # 물리 모델
    "M_cart": 0.8, "m_enc": 0.15, "m_pend": 0.2, "L": 0.3, "I_pend": 0.006, "g": 9.81,
    "x0": 0.0, 
    "theta0": 3.10,  # 바닥(약 177도)에서 시작
    
    "cart_damping": 1.2, "joint_damping": 0.01,
    
    # LQR 가중치
    "Q_x": 100.0, "Q_theta": 1000.0, "Q_xdot": 10.0, "Q_thetadot": 100.0,
    
    # 하드웨어 스펙
    "encoder_cpr": 2400, "control_dt": 0.01, "buffer_size": 5,
    "holding_force": 18.0,  # 모터의 물리적 최대 근력 (N)
    "max_rpm_speed": 1.5,   # 모터 토크가 0이 되는 최고 속도
    "motor_steps": 200, "microstepping": 128, "pulley_circum": 0.04,
    
    # 제어기 출력 한계 (모터 근력보다 작아야 탈조가 안 남)
    "u_max": 12.0,
    
    # 외란 및 레일
    "dist_amp": 0.0, "dist_freq": 1.5,
    "rail_x_min": -1.0, "rail_x_max": 1.0,
    
    # Swing-Up 제어 파라미터
    "catch_angle": 0.35,  # 꼭대기 기준 약 20도 이내면 LQR 발동
    "k_swing": 40.0,      # 에너지 펌핑 게인
    "k_center": 10.0,     # 중앙 복귀 탄성
    "kd_center": 8.0      # 중앙 복귀 댐핑
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

    def nonlinear_dynamics(t, X, u):
        x, theta, x_dot, theta_dot = X
        M_t, m, L, I, g = p["M_cart"] + p["m_enc"], p["m_pend"], p["L"], p["I_pend"], p["g"]

        # 물리적 한계 1: 탈조 (Stall)
        speed_factor = max(0.0, 1.0 - (abs(x_dot) / p["max_rpm_speed"]))
        if abs(u) > p["holding_force"] * speed_factor:
            u = 0.0  # 요구 힘이 모터 한계치 초과 시 즉시 멈춤

        # 물리적 한계 2: 가상 벽
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
        "theta_buffer": deque([p["theta0"]] * int(p["buffer_size"]), maxlen=int(p["buffer_size"])) # Wrapping 안 된 값 저장
    }

    mode_history = []

    def closed_loop(t, X):
        if t + 1e-12 >= controller["next_t"]:
            x, theta, _, _ = X
            
            # 센서 데이터 (각도는 Wrapping 하지 않고 그대로 저장하여 미분 튐 방지)
            q_x = np.round(x / pos_res) * pos_res
            q_theta_raw = np.round(theta / angle_res) * angle_res
            
            controller["x_buffer"].append(q_x)
            controller["theta_buffer"].append(q_theta_raw)
            
            dt_span = p["control_dt"] * (p["buffer_size"] - 1)
            est_x_dot = (controller["x_buffer"][-1] - controller["x_buffer"][0]) / dt_span
            est_theta_dot = (controller["theta_buffer"][-1] - controller["theta_buffer"][0]) / dt_span
            
            # LQR과 에너지를 위한 각도 Wrapping (-pi ~ pi)
            q_theta_wrap = (q_theta_raw + np.pi) % (2 * np.pi) - np.pi
            
            if abs(q_theta_wrap) < p["catch_angle"]:
                # 1. LQR CATCH 모드
                controller["mode"] = "LQR"
                X_obs = np.array([q_x, q_theta_wrap, est_x_dot, est_theta_dot])
                u_cmd = float((-K @ X_obs)[0])
            else:
                # 2. SWING-UP 모드
                controller["mode"] = "SWING"
                J_total = p["I_pend"] + p["m_pend"] * p["L"]**2
                energy = 0.5 * J_total * est_theta_dot**2 + p["m_pend"] * p["g"] * p["L"] * (np.cos(q_theta_wrap) - 1.0)
                
                # 부드러운 스윙업 에너지 펌핑
                u_swing = p["k_swing"] * energy * est_theta_dot * np.cos(q_theta_wrap)
                # 카트를 중앙으로 유지하려는 탄성력
                u_center = -p["k_center"] * q_x - p["kd_center"] * est_x_dot
                u_cmd = u_swing + u_center

            # 🔥 중요: 컨트롤러 스스로 u_max 이내로 제한하여 모터 탈조 방지
            u_cmd = np.clip(u_cmd, -p["u_max"], p["u_max"])
            
            u_hold = controller["u_prev"] # 1스텝 지연(Latency) 반영
            controller["u_prev"] = float(u_quant * np.round(u_cmd / u_quant))
            controller["next_t"] += p["control_dt"]
            
        else:
            u_hold = controller["u_prev"]

        mode_history.append(controller["mode"])
        return nonlinear_dynamics(t, X, u_hold)

    X0 = np.array([p["x0"], p["theta0"], 0.0, 0.0])
    sol = solve_ivp(closed_loop, t_span, X0, t_eval=t_eval, max_step=0.01)
    
    # 시간에 따른 모드를 맞추기 위해 샘플링 보정
    final_modes = [mode_history[min(i, len(mode_history)-1)] for i in range(len(t_eval))]
    return sol, K, final_modes

# ==========================================
# 3. 시각화 및 전체 UI 복구
# ==========================================
sol, K, modes = compute_solution(params)

fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=0.35)
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-0.4, 0.8)
ax.set_aspect('equal')
ax.grid(True)

cart_w, cart_h = 0.2, 0.1
rail_left = ax.axvline(x=params["rail_x_min"], color='gray', linestyle='--', lw=2)
rail_right = ax.axvline(x=params["rail_x_max"], color='gray', linestyle='--', lw=2)
ax.plot([params["rail_x_min"], params["rail_x_max"]], [-cart_h/2, -cart_h/2], 'k-', lw=3)

cart_rect = plt.Rectangle((-cart_w/2, -cart_h/2), cart_w, cart_h, fc='steelblue', ec='black')
ax.add_patch(cart_rect)
pendulum_line, = ax.plot([], [], 'o-', lw=4, markersize=10, color='firebrick')

# ---- 파라미터 슬라이더 UI 10개 ----
axcolor_phy = 'lightcyan'
axcolor_lqr = 'lightgoldenrodyellow'

slider_axes = {
    # 좌측 (물리 및 외란)
    "m_pend": plt.axes([0.08, 0.25, 0.35, 0.02], facecolor=axcolor_phy),
    "theta0": plt.axes([0.08, 0.21, 0.35, 0.02], facecolor=axcolor_phy),
    "encoder_cpr": plt.axes([0.08, 0.17, 0.35, 0.02], facecolor=axcolor_phy),
    "dist_amp": plt.axes([0.08, 0.13, 0.35, 0.02], facecolor=axcolor_phy),
    "u_max": plt.axes([0.08, 0.09, 0.35, 0.02], facecolor=axcolor_phy),
    
    # 우측 (LQR 가중치 및 모터 근력)
    "Q_x": plt.axes([0.55, 0.25, 0.35, 0.02], facecolor=axcolor_lqr),
    "Q_theta": plt.axes([0.55, 0.21, 0.35, 0.02], facecolor=axcolor_lqr),
    "Q_xdot": plt.axes([0.55, 0.17, 0.35, 0.02], facecolor=axcolor_lqr),
    "Q_thetadot": plt.axes([0.55, 0.13, 0.35, 0.02], facecolor=axcolor_lqr),
    "holding_force": plt.axes([0.55, 0.09, 0.35, 0.02], facecolor=axcolor_lqr),
}

sliders = {
    "m_pend": Slider(slider_axes["m_pend"], 'm_pend', 0.05, 1.0, valinit=params["m_pend"]),
    "theta0": Slider(slider_axes["theta0"], 'Init Angle', -3.14, 3.14, valinit=params["theta0"]),
    "encoder_cpr": Slider(slider_axes["encoder_cpr"], 'Enc CPR', 400, 8000, valinit=params["encoder_cpr"], valstep=400),
    "dist_amp": Slider(slider_axes["dist_amp"], 'Disturb', 0.0, 3.0, valinit=params["dist_amp"]),
    "u_max": Slider(slider_axes["u_max"], 'Ctrl Max u', 5.0, 25.0, valinit=params["u_max"]),
    
    "Q_x": Slider(slider_axes["Q_x"], 'Q(x)', 1, 1000, valinit=params["Q_x"]),
    "Q_theta": Slider(slider_axes["Q_theta"], 'Q(θ)', 1, 5000, valinit=params["Q_theta"]),
    "Q_xdot": Slider(slider_axes["Q_xdot"], 'Q(v)', 1, 500, valinit=params["Q_xdot"]),
    "Q_thetadot": Slider(slider_axes["Q_thetadot"], 'Q(ω)', 1, 1000, valinit=params["Q_thetadot"]),
    "holding_force": Slider(slider_axes["holding_force"], 'Motor Force', 5.0, 30.0, valinit=params["holding_force"]),
}

text_axes = plt.axes([0.42, 0.02, 0.15, 0.04], facecolor='lightgrey')
text_box = TextBox(text_axes, 'Reset: ', initial='')

info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, va='top', fontsize=11, fontweight='bold')

def update_text(frame):
    mode = modes[frame]
    color = "green" if mode == "LQR" else "red"
    theta = (sol.y[1, frame] + np.pi) % (2 * np.pi) - np.pi
    info_text.set_text(f"MODE: {mode}  |  Angle: {np.degrees(theta):.1f}°\n"
                       f"Kx: {K[0,0]:.1f} | Kθ: {K[0,1]:.1f} | Kv: {K[0,2]:.1f} | Kω: {K[0,3]:.1f}")
    info_text.set_color(color)

def recompute(_=None):
    global sol, K, modes
    for key, slider in sliders.items():
        params[key] = slider.val
    sol, K, modes = compute_solution(params)
    fig.canvas.draw_idle()

def reset_text(text):
    if text.strip().lower() == 'reset':
        defaults = {
            "m_pend": 0.2, "theta0": 3.10, "encoder_cpr": 2400, "dist_amp": 0.0, "u_max": 12.0,
            "Q_x": 100, "Q_theta": 1000, "Q_xdot": 10, "Q_thetadot": 100, "holding_force": 18.0
        }
        for key, val in defaults.items():
            sliders[key].set_val(val)
        recompute()
        text_box.set_val('')

for slider in sliders.values():
    slider.on_changed(recompute)
text_box.on_submit(reset_text)

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

ani = animation.FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=10)
plt.show()