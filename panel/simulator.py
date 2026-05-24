import numpy as np

def model_matrices(length_mm=305.0, friction=0.05, g=9.81):
    """
    실제 하드웨어(가속도 제어 스텝 모터) 기반의 물리적 A, B 행렬을 계산합니다.
    - length_mm: 진자의 회전축부터 상단 끝까지의 길이
    - friction: 회전축(베어링/엔코더)의 마찰 계수 (0.01 ~ 0.1 사이로 튜닝)
    - g: 중력 가속도
    """
    # 길이를 미터 단위로 변환
    L_m = max(1e-6, float(length_mm) / 1000.0)
    
    # [물리 모델 가정]
    # 진자가 균일한 막대(Uniform Rod)라고 가정했을 때의 관성 모멘트를 적용.
    # 만약 진자 끝에 무거운 추가 달려있는 형태라면 (3.0 * g) / (2.0 * L_m) 대신 g / L_m 을 사용하세요.
    
    # 상태 변수: x = [각도(theta), 각속도(theta_dot), 위치(pos), 속도(pos_dot)]
    # 제어 입력: u = 카트의 가속도 (ddot{pos})

    # A 행렬: 외력이 없을 때 시스템이 스스로 움직이는 성향
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [(3.0 * g) / (2.0 * L_m), -friction, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]
    ], dtype=float)

    # B 행렬: 카트의 가속도(u)가 시스템에 미치는 영향
    # 카트가 오른쪽(+u)으로 가속하면, 진자는 관성에 의해 왼쪽(-)으로 꺾이려 하므로 음수.
    # (주의: 만약 하드웨어 코드에서 모터 CW/CCW 방향 정의가 반대라면 B[1,0]을 + 로 바꿔주세요)
    B = np.array([
        [0.0],
        [-3.0 / (2.0 * L_m)],
        [0.0],
        [1.0]
    ], dtype=float)
    
    return A, B


def discretize(Ac, Bc, dt):
    """Euler discretization"""
    Ad = np.eye(Ac.shape[0]) + Ac * dt
    Bd = Bc * dt
    return Ad, Bd


def dare_lqr(Ad, Bd, Q, R, max_iters=50000, tol=1e-9):
    """
    이산 리카티 방정식(DARE)을 풀어 최적의 게인 K를 반환합니다.
    물리 모델이 현실적이므로 반복 횟수를 늘려 완전히 수렴하도록 보장합니다.
    """
    n = Ad.shape[0]
    P = Q.copy()
    for i in range(max_iters):
        BtP = Bd.T @ P
        S = R + BtP @ Bd
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
            
        P_next = Ad.T @ P @ Ad - Ad.T @ P @ Bd @ S_inv @ BtP @ Ad + Q
        
        if np.linalg.norm(P_next - P) < tol:
            P = P_next
            break
        P = P_next

    S = R + Bd.T @ P @ Bd
    try:
        K = np.linalg.inv(S) @ (Bd.T @ P @ Ad)
    except np.linalg.LinAlgError:
        K = np.linalg.pinv(S) @ (Bd.T @ P @ Ad)
        
    return P, np.atleast_2d(K)


def simulate(Ad, Bd, K, x0_rad, steps, u_limit=4000.0):
    """이산 시간 시스템 시뮬레이션"""
    x = np.array(x0_rad, dtype=float).reshape(-1, 1)
    n = x.shape[0]
    traj = np.zeros((steps, n))
    
    for k in range(steps):
        traj[k, :] = x.flatten()
        u = float(-(K @ x).item())
        
        if u_limit is not None:
            u = float(np.clip(u, -abs(u_limit), abs(u_limit)))
            
        x = Ad @ x + Bd * u
        
    return traj


# ==========================================
# 실행 및 시뮬레이션 블록 (UI에서 실행되는 로직)
# ==========================================
if __name__ == "__main__":
    # 1. 하드웨어 세팅 및 모델 생성
    dt = 0.001
    Ac, Bc = model_matrices(length_mm=305.0, friction=0.04, g=9.81)
    Ad, Bd = discretize(Ac, Bc, dt)

    # 2. LQR 가중치 설정 (Q, R)
    # 위치(pos)를 무시하고 각도(theta) 최우선 제어 세팅
    Q = np.diag([100.0, 1.0, 1.0, 1.0]) 
    R = np.array([[1.0]])

    # 3. K값 계산 (LQR)
    # 수동 테스트를 원하실 경우 이 부분을 주석 처리하고 K = np.array([[202, 22, 224, 62]]) 을 넣으시면 됩니다.
    P, K = dare_lqr(Ad, Bd, Q, R)
    print(f"Calculated Optimal K: {np.round(K[0], 2)}")

    # 4. ★ 필수: Degree -> Radian 변환 ★
    initial_angle_deg = 12.0
    initial_angle_rad = initial_angle_deg * np.pi / 180.0
    
    # [각도(rad), 각속도(rad/s), 위치(m), 속도(m/s)]
    x0 = [initial_angle_rad, 0.0, 0.0, 0.0]

    # 5. 시뮬레이션 실행
    traj = simulate(Ad, Bd, K, x0, steps=6000)

    # 6. UI 그래프에 데이터를 그리기 전 복원
    # 출력 데이터를 얻을 때 각도는 다시 Degree로, 위치는 mm로 변환하여 넘겨줍니다.
    plot_angle_deg = (traj[:, 0] * 180.0 / np.pi) + 180.0  # 0도를 중앙(180)으로 시프트
    plot_position_mm = traj[:, 2] * 1000.0
    
    print(f"Final Angle (deg): {plot_angle_deg[-1]:.2f}")
    print(f"Final Position (mm): {plot_position_mm[-1]:.2f}")