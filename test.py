import numpy as np
import scipy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox

from dynamics_template import Params, state_space_linearized, state_derivative
from lqr_controller import LQRController

# ==========================================
# 1. 시스템 파라미터 (임의의 초기값 설정)
# ==========================================
params = {
    "M": 1.0,
    "m1": 0.5,
    "m2": 0.5,
    "L1": 0.2,
    "L2": 0.15,
    "g": 9.81,
    "x0": 0.0,
    "theta1": 0.0,
    "theta2": -0.0,
    "threshold": 0.05,
    # Realistic actuator/controller limits (rough NEMA17-driven axis model).
    "u_max": 12.0,
    "u_quant": 0.2,
    "control_dt": 0.01,
    # Viscous damping coefficients.
    "cart_damping": 0.8,
    "joint1_damping": 0.05,
    "joint2_damping": 0.04,
    # External disturbance force (N): amp * sin(2*pi*freq*t)
    "dist_amp": 0.8,
    "dist_freq": 1.8,
}

# LQR 비용 행렬
Q = np.diag([100, 500, 500, 10, 50, 50])
R = np.array([[1.0]])

t_span = (0, 5.0)
t_eval = np.linspace(t_span[0], t_span[1], 500)

# ==========================================
# 2. 행렬/해 결산 함수
# ==========================================
def get_dynamics_params(p):
    return Params(
        m_total=p["M"] + p["m1"] + p["m2"],
        A=(0.5 * p["m1"] + p["m2"]) * p["L1"],
        B=0.5 * p["m2"] * p["L2"],
        Cc=0.5 * p["m2"] * p["L1"] * p["L2"],
        I_link1=(1.0 / 3.0 * p["m1"] + p["m2"]) * p["L1"]**2,
        I_link2=1.0 / 3.0 * p["m2"] * p["L2"]**2,
        k1=(0.5 * p["m1"] + p["m2"]) * p["g"] * p["L1"],
        k2=0.5 * p["m2"] * p["g"] * p["L2"]
    )


def compute_matrices(p):
    dp = get_dynamics_params(p)
    A, B = state_space_linearized(dp)
    
    # dynamics_template.py의 state_space_linearized는 매달린 진자(downright) 기준으로
    # A21 = -M_inv @ K를 구하지만, 역진자(upright)로 안정화하려면 부호가 반대(+M_inv @ K)가
    # 되어야 하므로 해당 부분의 부호를 반전시킵니다.
    A[3:6, 0:3] = -A[3:6, 0:3]
    
    # B는 3자유도 입력 [F_cart, tau_1, tau_2]^T 이므로 카트에만 힘이 들어가는 첫 번째 열(index 0) 추출
    B_cart = B[:, [0]]
    
    # LQRController를 활용하여 최적 피드백 게인 K 도출
    lqr = LQRController(A, B_cart, Q, R, u_max=p["u_max"], u_quant=p["u_quant"])
    
    return A, B_cart, lqr.K


def compute_solution(p):
    A, B_cart, K = compute_matrices(p)
    dp = get_dynamics_params(p)
    lqr = LQRController(A, B_cart, Q, R, u_max=p["u_max"], u_quant=p["u_quant"])

    dt = p["control_dt"]
    t0, tf = t_span
    t_steps = np.arange(t0, tf + 1e-12, dt)
    
    t_all = []
    y_all = []
    
    X = np.array([p["x0"], p["theta1"], p["theta2"], 0.0, 0.0, 0.0])
    
    for i in range(len(t_steps) - 1):
        t_start = t_steps[i]
        t_end = t_steps[i+1]
        
        # 현재 스텝 시작 시점의 상태를 기준으로 제어 입력 계산 후 고정 (ZOH)
        u_hold = lqr.compute_input(X)
        
        def dynamics_interval(t, state_val):
            x, theta1, theta2, x_dot, theta1_dot, theta2_dot = state_val
            damping_vector = np.array([
                p["cart_damping"] * x_dot,
                p["joint1_damping"] * theta1_dot,
                p["joint2_damping"] * theta2_dot,
            ])
            disturbance = p["dist_amp"] * np.sin(2.0 * np.pi * p["dist_freq"] * t)
            tau_effective = np.array([u_hold + disturbance, 0.0, 0.0]) - damping_vector
            return state_derivative(state_val, tau_effective, dp)
            
        sol_interval = solve_ivp(dynamics_interval, (t_start, t_end), X, max_step=dt/2.0)
        X = sol_interval.y[:, -1]
        
        # 중복 방지를 위해 마지막 포인트 제외하고 수집
        t_all.extend(sol_interval.t[:-1])
        y_all.extend(sol_interval.y[:, :-1].T)
        
    t_all.append(t_steps[-1])
    y_all.append(X)
    
    # 500프레임 애니메이션 요구 조건에 맞춰 시간 보간 수행
    y_interpolated = np.zeros((6, len(t_eval)))
    for idx in range(6):
        y_interpolated[idx, :] = np.interp(t_eval, t_all, np.array(y_all)[:, idx])
        
    class ODESolutionWrapper:
        def __init__(self, t, y):
            self.t = t
            self.y = y
            
    sol = ODESolutionWrapper(t_eval, y_interpolated)
    return sol, K


# ==========================================
# 3. 초기 계산 및 애니메이션 설정
# ==========================================
sol, K = compute_solution(params)

fig, ax = plt.subplots(figsize=(10, 7))
adjust_bottom = 0.32
plt.subplots_adjust(left=0.10, right=0.95, top=0.90, bottom=adjust_bottom)
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-0.5, 1.5)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Double Inverted Pendulum LQR Control")

cart_width = 0.2
cart_height = 0.1

cart_rect = plt.Rectangle((-cart_width / 2, -cart_height / 2), cart_width, cart_height, fc='blue')
ax.add_patch(cart_rect)
line1, = ax.plot([], [], 'o-', lw=3, markersize=8, color='red')
line2, = ax.plot([], [], 'o-', lw=3, markersize=8, color='green')

# 슬라이더 영역
slider_axes = {
    "M": plt.axes([0.10, 0.24, 0.35, 0.03], facecolor='lightgoldenrodyellow'),
    "m1": plt.axes([0.10, 0.20, 0.35, 0.03], facecolor='lightgoldenrodyellow'),
    "m2": plt.axes([0.10, 0.16, 0.35, 0.03], facecolor='lightgoldenrodyellow'),
    "L1": plt.axes([0.10, 0.12, 0.35, 0.03], facecolor='lightgoldenrodyellow'),
    "L2": plt.axes([0.10, 0.08, 0.35, 0.03], facecolor='lightgoldenrodyellow'),
    "x0": plt.axes([0.60, 0.28, 0.30, 0.03], facecolor='lightgoldenrodyellow'),
    "theta1": plt.axes([0.60, 0.24, 0.30, 0.03], facecolor='lightgoldenrodyellow'),
    "theta2": plt.axes([0.60, 0.20, 0.30, 0.03], facecolor='lightgoldenrodyellow'),
    "threshold": plt.axes([0.60, 0.16, 0.30, 0.03], facecolor='lightgoldenrodyellow'),
}

sliders = {
    "M": Slider(slider_axes["M"], 'M', 0.1, 5.0, valinit=params["M"]),
    "m1": Slider(slider_axes["m1"], 'm1', 0.1, 2.0, valinit=params["m1"]),
    "m2": Slider(slider_axes["m2"], 'm2', 0.1, 2.0, valinit=params["m2"]),
    "L1": Slider(slider_axes["L1"], 'L1', 0.05, 1.0, valinit=params["L1"]),
    "L2": Slider(slider_axes["L2"], 'L2', 0.05, 1.0, valinit=params["L2"]),
    "x0": Slider(slider_axes["x0"], 'x0', -1.0, 1.0, valinit=params["x0"]),
    "theta1": Slider(slider_axes["theta1"], 'theta1', -1.5, 1.5, valinit=params["theta1"]),
    "theta2": Slider(slider_axes["theta2"], 'theta2', -1.5, 1.5, valinit=params["theta2"]),
    "threshold": Slider(slider_axes["threshold"], 'threshold', 0.05, 0.5, valinit=params["threshold"]),
}

# 텍스트 박스 영역
text_axes = plt.axes([0.60, 0.04, 0.30, 0.05], facecolor='lightgrey')
text_box = TextBox(text_axes, '리셋 입력 (reset)', initial='')

info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, va='top')
frame_text = ax.text(0.02, 0.80, '', transform=ax.transAxes, va='top')


def get_stability_info(sol, threshold):
    final_angles = np.array([sol.y[1, -1], sol.y[2, -1]])
    max_angles = np.max(np.abs(sol.y[1:3, :]), axis=1)
    fell_over = np.any(np.abs(sol.y[1:3, :]) >= np.pi / 2)
    stable = (not fell_over) and np.all(np.abs(final_angles) <= threshold)
    return final_angles, max_angles, stable, fell_over


def update_text():
    final_angles, max_angles, stable, fell_over = get_stability_info(sol, params["threshold"])
    status = 'FALL' if fell_over else ('YES' if stable else 'NO')
    info_text.set_text(
        f"K[0,0]={K[0,0]:.2f}  K[0,1]={K[0,1]:.2f}  K[0,2]={K[0,2]:.2f}\n"
        f"x0={params['x0']:.2f}  theta1={params['theta1']:.2f}  theta2={params['theta2']:.2f}  thr={params['threshold']:.2f}\n"
        f"final θ1={final_angles[0]:.2f}  final θ2={final_angles[1]:.2f}  max|θ1|={max_angles[0]:.2f}  max|θ2|={max_angles[1]:.2f}  stable={status}"
    )


def recompute(_=None):
    global sol, K
    for key, slider in sliders.items():
        params[key] = slider.val

    sol, K = compute_solution(params)
    ax.set_title(
        f"Double Inverted Pendulum LQR Control  |  x0={params['x0']:.2f}  theta1={params['theta1']:.2f}  theta2={params['theta2']:.2f}"
    )
    update_text()
    fig.canvas.draw_idle()


def reset_text(text):
    if text.strip().lower() == 'reset':
        defaults = {
            "M": 1.0,
            "m1": 0.5,
            "m2": 0.5,
            "L1": 0.2,
            "L2": 0.15,
            "g": 9.81,
            "x0": 0.0,
            "theta1": 0.1,
            "theta2": -0.1,
            "threshold": 0.2,
            "u_max": 12.0,
            "u_quant": 0.2,
            "control_dt": 0.01,
            "cart_damping": 0.8,
            "joint1_damping": 0.05,
            "joint2_damping": 0.04,
            "dist_amp": 0.8,
            "dist_freq": 1.8,
        }
        for key, val in defaults.items():
            if key in sliders:
                sliders[key].set_val(val)
        recompute()
        text_box.set_val('')

for slider in sliders.values():
    slider.on_changed(recompute)
text_box.on_submit(reset_text)


def init():
    cart_rect.set_xy((-cart_width / 2, -cart_height / 2))
    line1.set_data([], [])
    line2.set_data([], [])
    frame_text.set_text('')
    update_text()
    return cart_rect, line1, line2, frame_text


def update(frame):
    x = sol.y[0, frame]
    theta1 = sol.y[1, frame]
    theta2 = sol.y[2, frame]

    cart_rect.set_xy((x - cart_width / 2, -cart_height / 2))

    x1 = x + params["L1"] * np.sin(theta1)
    y1 = params["L1"] * np.cos(theta1)
    x2 = x1 + params["L2"] * np.sin(theta2)
    y2 = y1 + params["L2"] * np.cos(theta2)

    line1.set_data([x, x1], [0, y1])
    line2.set_data([x1, x2], [y1, y2])
    frame_text.set_text(
        f"frame={frame}  θ1={theta1:.2f}  θ2={theta2:.2f}  x={x:.2f}"
    )

    return cart_rect, line1, line2, frame_text

ani = animation.FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=10)
plt.show()
