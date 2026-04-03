import numpy as np
import scipy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 1. 시스템 파라미터 (임의의 초기값 설정)
# 실제 하드웨어 제작 후 측정한 파라미터로 수정해야 합니다!
# ==========================================
M = 1.0     # 카트 질량 (kg)
m1 = 0.5    # 첫 번째 진자 질량 (kg)
m2 = 0.5    # 두 번째 진자 질량 (kg)
L1 = 0.3    # 첫 번째 진자 길이 (m)
L2 = 0.3    # 두 번째 진자 길이 (m)
g = 9.81    # 중력가속도 (m/s^2)

# ==========================================
# 2. 선형화된 상태 공간 모델 행렬 (A, B)
# 업로드하신 자료의 M0 * q_ddot + K_mat * q = tau 형태에서 유도된 선형화 행렬 예시
# ==========================================
# (참고: 아래 A, B 행렬은 일반적인 이중역진자의 선형화된 근사 모델입니다. 
# 수기 노트에서 유도하신 정확한 A, B 행렬을 여기에 대입하시면 됩니다.)

# 질량/관성 행렬 M0 및 강성 행렬 K_mat 근사
M0 = np.array([
    [M + m1 + m2, (0.5*m1 + m2)*L1, 0.5*m2*L2],
    [(0.5*m1 + m2)*L1, (1/3*m1 + m2)*L1**2, 0.5*m2*L1*L2],
    [0.5*m2*L2, 0.5*m2*L1*L2, 1/3*m2*L2**2]
])

K_mat = np.array([
    [0, 0, 0],
    [0, -(0.5*m1 + m2)*g*L1, 0],
    [0, 0, -0.5*m2*g*L2]
])

Minv = la.inv(M0)
A21 = -Minv @ K_mat

# 상태 공간 모델 X = [x, theta1, theta2, x_dot, theta1_dot, theta2_dot]^T
A = np.zeros((6, 6))
A[0:3, 3:6] = np.eye(3)
A[3:6, 0:3] = A21

B = np.zeros((6, 1))
B[3:6, 0] = Minv[:, 0] # 카트에 가해지는 힘 tau

# ==========================================
# 3. LQR 제어기 설계
# ==========================================
# Q 행렬: 상태 변수의 오차에 대한 가중치 (수기 노트 내용 반영)
Q = np.diag([100, 500, 500, 10, 50, 50]) 
# R 행렬: 제어 입력(모터 토크/힘)에 대한 가중치
R = np.array([[1.0]])

# 연속시간 대수적 리카티 방정식(ARE) 풀이
P = la.solve_continuous_are(A, B, Q, R)
# 최적 피드백 게인 K 계산
K = la.inv(R) @ B.T @ P

print("계산된 LQR 게인 K:", K)

# ==========================================
# 4. 시뮬레이션 설정 및 적분 (폐루프 시스템)
# ==========================================
def closed_loop_dynamics(t, X):
    u = -K @ X  # LQR 제어 입력
    u = u[0]
    dX = A @ X + B.flatten() * u
    return dX

# 초기 상태: 진자들이 살짝 기울어진 상태에서 시작
# X0 = [카트위치, 진자1각도, 진자2각도, 카트속도, 진자1각속도, 진자2각속도]
X0 = np.array([0.0, 0.1, -0.1, 0.0, 0.0, 0.0]) # 라디안 단위
t_span = (0, 5.0)
t_eval = np.linspace(t_span[0], t_span[1], 500)

sol = solve_ivp(closed_loop_dynamics, t_span, X0, t_eval=t_eval)

# ==========================================
# 5. Matplotlib 애니메이션 시각화
# ==========================================
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-0.5, 1.5)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Double Inverted Pendulum LQR Control")

cart_width = 0.2
cart_height = 0.1

# 그래픽 요소 초기화
cart_rect = plt.Rectangle((-cart_width/2, -cart_height/2), cart_width, cart_height, fc='blue')
ax.add_patch(cart_rect)
line1, = ax.plot([], [], 'o-', lw=3, markersize=8, color='red')
line2, = ax.plot([], [], 'o-', lw=3, markersize=8, color='green')

def init():
    cart_rect.set_xy((-cart_width/2, -cart_height/2))
    line1.set_data([], [])
    line2.set_data([], [])
    return cart_rect, line1, line2

def update(frame):
    # 상태 변수 추출
    x = sol.y[0, frame]
    theta1 = sol.y[1, frame]
    theta2 = sol.y[2, frame]
    
    # 카트 위치 업데이트
    cart_rect.set_xy((x - cart_width/2, -cart_height/2))
    
    # 조인트 좌표 계산 (수직 상단을 0도로 기준)
    x1 = x + L1 * np.sin(theta1)
    y1 = L1 * np.cos(theta1)
    
    x2 = x1 + L2 * np.sin(theta2)
    y2 = y1 + L2 * np.cos(theta2)
    
    line1.set_data([x, x1], [0, y1])
    line2.set_data([x1, x2], [y1, y2])
    
    return cart_rect, line1, line2

ani = animation.FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=10)
plt.show()