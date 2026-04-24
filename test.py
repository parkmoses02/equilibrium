import numpy as np
import scipy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox

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
    "theta1": 0.1,
    "theta2": -0.1,
    "threshold": 0.2,
}

# LQR 비용 행렬
Q = np.diag([100, 500, 500, 10, 50, 50])
R = np.array([[1.0]])

t_span = (0, 5.0)
t_eval = np.linspace(t_span[0], t_span[1], 500)

# ==========================================
# 2. 행렬/해 결산 함수
# ==========================================
def compute_matrices(p):
    M = p["M"]
    m1 = p["m1"]
    m2 = p["m2"]
    L1 = p["L1"]
    L2 = p["L2"]
    g = p["g"]

    M0 = np.array([
        [M + m1 + m2, (0.5 * m1 + m2) * L1, 0.5 * m2 * L2],
        [(0.5 * m1 + m2) * L1, (1 / 3 * m1 + m2) * L1**2, 0.5 * m2 * L1 * L2],
        [0.5 * m2 * L2, 0.5 * m2 * L1 * L2, 1 / 3 * m2 * L2**2],
    ])

    K_mat = np.array([
        [0, 0, 0],
        [0, -(0.5 * m1 + m2) * g * L1, 0],
        [0, 0, -0.5 * m2 * g * L2],
    ])

    Minv = la.inv(M0)
    A21 = -Minv @ K_mat

    A = np.zeros((6, 6))
    A[0:3, 3:6] = np.eye(3)
    A[3:6, 0:3] = A21

    B = np.zeros((6, 1))
    B[3:6, 0] = Minv[:, 0]

    P = la.solve_continuous_are(A, B, Q, R)
    K = la.inv(R) @ B.T @ P

    return A, B, K


def compute_solution(p):
    A, B, K = compute_matrices(p)

    def closed_loop_dynamics(t, X):
        u = -K @ X
        u = u[0]
        return A @ X + B.flatten() * u

    X0 = np.array([p["x0"], p["theta1"], p["theta2"], 0.0, 0.0, 0.0])
    sol = solve_ivp(closed_loop_dynamics, t_span, X0, t_eval=t_eval, max_step=0.02)
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
