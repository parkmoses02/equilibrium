import numpy as np
import scipy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox
from collections import deque

# ==========================================
# 1. 하드웨어 기반 시스템 파라미터 (초기값)
# ==========================================
params = {
    # 물리 모델 (카트 및 진자)
    "M_cart": 0.8,
    "m_enc": 0.15,         # 엔코더 무게 150g
    "m_pend": 0.2,
    "L": 0.3,
    "I_pend": 0.006,
    "g": 9.81,
    "x0": 0.0,
    "theta0": 0.15,
    "threshold": 0.1,      # 안정화 판정 기준 (rad)
    
    # 마찰 (Damping)
    "cart_damping": 1.2,
    "joint_damping": 0.01,
    
    # LQR Q-Matrix 가중치 (튜닝 대상)
    "Q_x": 100.0,
    "Q_theta": 1000.0,
    "Q_xdot": 10.0,
    "Q_thetadot": 100.0,
    
    # 엔코더 및 제어기 스펙
    "encoder_cpr": 2400,   # 600 PPR * 4체배
    "control_dt": 0.01,    # 제어 루프 100Hz
    "buffer_size": 5,      # 각속도 필터용 과거 데이터 버퍼 크기
    
    # NEMA17 Actuator
    "u_max": 15.0,
    "motor_steps": 200,
    "microstepping": 128,
    "pulley_circum": 0.04,
    
    # 외란 및 물리적 한계
    "dist_amp": 0.5,
    "dist_freq": 1.5,
    "rail_x_min": -0.8,
    "rail_x_max": 0.8,
}

R = np.array([[1.0]])
t_span = (0, 5.0)
t_eval = np.linspace(t_span[0], t_span[1], int(5.0 / params["control_dt"]))

# ==========================================
# 2. 시스템 행렬 및 LQR 해 계산
# ==========================================
def compute_matrices(p):
    M_t = p["M_cart"] + p["m_enc"]
    m, L, I, g = p["m_pend"], p["L"], p["I_pend"], p["g"]
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

    # 슬라이더로 변경된 Q 행렬 적용
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
        M_t = p["M_cart"] + p["m_enc"]
        m, L, I, g = p["m_pend"], p["L"], p["I_pend"], p["g"]

        sin_t, cos_t = np.sin(theta), np.cos(theta)
        Mq = np.array([[M_t + m, m * L * cos_t], [m * L * cos_t, I + m * L**2]])
        
        nonlinear_terms = np.array([-m * L * sin_t * theta_dot**2, 0.0])
        gravity_terms = np.array([0.0, m * g * L * sin_t])
        damping_terms = np.array([p["cart_damping"] * x_dot, p["joint_damping"] * theta_dot])
        
        disturbance = p["dist_amp"] * np.sin(2.0 * np.pi * p["dist_freq"] * t)
        tau = np.array([u + disturbance, 0.0])

        q_ddot = la.solve(Mq, tau - nonlinear_terms - damping_terms + gravity_terms)
        return np.array([x_dot, theta_dot, q_ddot[0], q_ddot[1]])

    controller = {
        "next_t": 0.0, 
        "u_hold": 0.0,
        "x_buffer": deque([p["x0"]] * int(p["buffer_size"]), maxlen=int(p["buffer_size"])),
        "theta_buffer": deque([p["theta0"]] * int(p["buffer_size"]), maxlen=int(p["buffer_size"]))
    }

    def closed_loop(t, X):
        if t + 1e-12 >= controller["next_t"]:
            x, theta, _, _ = X
            
            # 센서 양자화
            q_x = np.round(x / pos_res) * pos_res
            q_theta = np.round(theta / angle_res) * angle_res
            
            # 필터를 통한 속도 추정
            controller["x_buffer"].append(q_x)
            controller["theta_buffer"].append(q_theta)
            
            dt_span = p["control_dt"] * (p["buffer_size"] - 1)
            est_x_dot = (controller["x_buffer"][-1] - controller["x_buffer"][0]) / dt_span
            est_theta_dot = (controller["theta_buffer"][-1] - controller["theta_buffer"][0]) / dt_span
            
            X_obs = np.array([q_x, q_theta, est_x_dot, est_theta_dot])
            u_cmd = float((-K @ X_obs)[0])
            u_cmd = float(np.clip(u_cmd, -p["u_max"], p["u_max"]))
            
            controller["u_hold"] = float(u_quant * np.round(u_cmd / u_quant))
            controller["next_t"] += p["control_dt"]

        return nonlinear_dynamics(t, X, controller["u_hold"])

    X0 = np.array([p["x0"], p["theta0"], 0.0, 0.0])
    sol = solve_ivp(closed_loop, t_span, X0, t_eval=t_eval, max_step=0.01)
    return sol, K

# ==========================================
# 3. 초기 계산 및 애니메이션 설정
# ==========================================
sol, K = compute_solution(params)

fig, ax = plt.subplots(figsize=(12, 8))
adjust_bottom = 0.35
plt.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=adjust_bottom)
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-0.3, 0.8)
ax.set_aspect('equal')
ax.grid(True)

cart_w, cart_h = 0.2, 0.1

rail_left = ax.axvline(x=params["rail_x_min"], color='gray', linestyle='--', lw=2)
rail_right = ax.axvline(x=params["rail_x_max"], color='gray', linestyle='--', lw=2)
rail_surface, = ax.plot([params["rail_x_min"], params["rail_x_max"]], [-cart_h/2, -cart_h/2], 'k-', lw=3)

cart_rect = plt.Rectangle((-cart_w/2, -cart_h/2), cart_w, cart_h, fc='steelblue', ec='black')
ax.add_patch(cart_rect)
pendulum_line, = ax.plot([], [], 'o-', lw=4, markersize=10, color='firebrick')

# ==========================================
# 4. 인터랙티브 UI 구성
# ==========================================
axcolor_phy = 'lightcyan'
axcolor_lqr = 'lightgoldenrodyellow'

slider_axes = {
    # 물리 & 센서 파라미터 (좌측)
    "m_pend": plt.axes([0.08, 0.25, 0.35, 0.02], facecolor=axcolor_phy),
    "cart_damping": plt.axes([0.08, 0.21, 0.35, 0.02], facecolor=axcolor_phy),
    "theta0": plt.axes([0.08, 0.17, 0.35, 0.02], facecolor=axcolor_phy),
    "encoder_cpr": plt.axes([0.08, 0.13, 0.35, 0.02], facecolor=axcolor_phy),
    "dist_amp": plt.axes([0.08, 0.09, 0.35, 0.02], facecolor=axcolor_phy),
    
    # LQR 가중치 파라미터 (우측)
    "Q_x": plt.axes([0.55, 0.25, 0.35, 0.02], facecolor=axcolor_lqr),
    "Q_theta": plt.axes([0.55, 0.21, 0.35, 0.02], facecolor=axcolor_lqr),
    "Q_xdot": plt.axes([0.55, 0.17, 0.35, 0.02], facecolor=axcolor_lqr),
    "Q_thetadot": plt.axes([0.55, 0.13, 0.35, 0.02], facecolor=axcolor_lqr),
    "u_max": plt.axes([0.55, 0.09, 0.35, 0.02], facecolor=axcolor_lqr),
}

sliders = {
    "m_pend": Slider(slider_axes["m_pend"], 'm_pend (kg)', 0.05, 1.0, valinit=params["m_pend"]),
    "cart_damping": Slider(slider_axes["cart_damping"], 'Friction', 0.0, 3.0, valinit=params["cart_damping"]),
    "theta0": Slider(slider_axes["theta0"], 'Init Angle', -0.5, 0.5, valinit=params["theta0"]),
    "encoder_cpr": Slider(slider_axes["encoder_cpr"], 'Enc CPR', 400, 8000, valinit=params["encoder_cpr"], valstep=400),
    "dist_amp": Slider(slider_axes["dist_amp"], 'Disturb Amp', 0.0, 2.0, valinit=params["dist_amp"]),
    
    "Q_x": Slider(slider_axes["Q_x"], 'Q(x)', 1, 1000, valinit=params["Q_x"]),
    "Q_theta": Slider(slider_axes["Q_theta"], 'Q(θ)', 1, 5000, valinit=params["Q_theta"]),
    "Q_xdot": Slider(slider_axes["Q_xdot"], 'Q(v)', 1, 500, valinit=params["Q_xdot"]),
    "Q_thetadot": Slider(slider_axes["Q_thetadot"], 'Q(ω)', 1, 1000, valinit=params["Q_thetadot"]),
    "u_max": Slider(slider_axes["u_max"], 'Max Accel', 5.0, 30.0, valinit=params["u_max"]),
}

text_axes = plt.axes([0.42, 0.02, 0.15, 0.04], facecolor='lightgrey')
text_box = TextBox(text_axes, 'Reset: ', initial='')

info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, va='top', fontsize=10)

def update_text():
    max_theta = np.max(np.abs(sol.y[1, :]))
    fell_over = max_theta >= np.pi / 2
    status = 'FALL' if fell_over else 'STABLE'
    info_text.set_text(
        f"[LQR Gain K]  Kx: {K[0,0]:.2f}  |  Kθ: {K[0,1]:.2f}  |  Kv: {K[0,2]:.2f}  |  Kω: {K[0,3]:.2f}\n"
        f"Max Angle: {np.degrees(max_theta):.1f}°  |  State: {status}\n"
        f"Quantization: {360/params['encoder_cpr']: .3f}°"
    )

def recompute(_=None):
    global sol, K
    for key, slider in sliders.items():
        params[key] = slider.val

    sol, K = compute_solution(params)
    
    ax.set_title(f"Hardware-In-The-Loop: Single Pendulum (CPR: {int(params['encoder_cpr'])}, u_max: {params['u_max']:.1f})")
    update_text()
    fig.canvas.draw_idle()

def reset_text(text):
    if text.strip().lower() == 'reset':
        defaults = {
            "m_pend": 0.2, "cart_damping": 1.2, "theta0": 0.15, "encoder_cpr": 2400, "dist_amp": 0.5,
            "Q_x": 100, "Q_theta": 1000, "Q_xdot": 10, "Q_thetadot": 100, "u_max": 15.0
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
    update_text()
    return cart_rect, pendulum_line, info_text

def update(frame):
    x = sol.y[0, frame]
    theta = sol.y[1, frame]
    
    # 레일 제한 적용
    x = np.clip(x, params["rail_x_min"] + cart_w/2, params["rail_x_max"] - cart_w/2)
    
    cart_rect.set_xy((x - cart_w/2, -cart_h/2))
    px = x + params["L"] * np.sin(theta)
    py = params["L"] * np.cos(theta)
    
    pendulum_line.set_data([x, px], [0, py])
    return cart_rect, pendulum_line, info_text

ani = animation.FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=10)
ax.set_title(f"Hardware-In-The-Loop: Single Pendulum (CPR: {int(params['encoder_cpr'])}, u_max: {params['u_max']:.1f})")
plt.show()