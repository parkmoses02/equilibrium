"""
upright_balance_test.cpp / pipeline_energy_lqr.cpp 의 5단계 스윙업 상태머신
(PREPOSITION -> SETTLE -> PUMP -> COAST -> RISE, 정점 에너지 부족 재시도 포함)을
그대로 파이썬으로 옮겨서, panel/simulator.py 의 실제 model_matrices/dare_lqr 로
계산한 K를 가지고 스윙업 -> 캐치 -> LQR 밸런스 전환이 되는지 확인하는 스크립트.

사용법:
  1) 이 파일을 equilibrium/panel/ 폴더 안에 둔다 (simulator.py 와 같은 위치).
  2) python test_swingup_statemachine.py

주의:
  - 하드웨어 요소(모터 응답 지연, 센서 노이즈, 500Hz 타이밍, 각속도 필터)는 없고
    이상적인 카트 모델이다. 그래서 "스윙업 상태머신의 논리가 캐치까지 이어지는가"를
    확인하는 용도이며, 센 킥에서 하드웨어가 뻗는 현상 등은 재현되지 않는다.
  - control_polarity 기본값 +1 은 이 파이썬 플랜트(-u*cos(theta))에 맞춘 값이다.
    펌웨어(하드웨어)는 -1 이다. 극성을 -1 로 두면 킥이 에너지를 빼서 캐치 실패한다.
  - 캐치 이후 밸런스는 펌웨어의 수동 게인이 아니라 simulator.dare_lqr 로 계산한
    리카티 K (u = -K @ x) 를 쓴다. 트립 각/350mm 리밋 disarm 로직은 없다.
  - 기본 설정에서 약 3.1초에 캐치되는 것을 확인했다.
"""
import math
import numpy as np
import simulator

PHASE_PREPOSITION, PHASE_SETTLE, PHASE_PUMP, PHASE_COAST, PHASE_RISE = range(5)
PHASE_NAMES = {PHASE_PREPOSITION: "PREPOSITION", PHASE_SETTLE: "SETTLE",
               PHASE_PUMP: "PUMP", PHASE_COAST: "COAST", PHASE_RISE: "RISE"}


def simulate_with_statemachine(length_m=0.20, friction=0.04, g=9.81,
                                K_lqr=None,
                                swing_speed=1.20, swing_accel=16.0,
                                cart_limit=0.35, max_speed=0.80, max_accel=12.0,
                                dt=0.002, total_time_s=30.0,
                                initial_angle_deg=175.0,
                                swing_start_offset=0.10,
                                swing_direction=-1.0,
                                control_polarity=1.0,  # 이 파이썬 플랜트(-u*cos)와 맞는 극성. 펌웨어 하드웨어는 -1 (thetaDD=(g sin - pol*a cos)/L)
                                swing_energy_margin=0.0,
                                swing_retry_gain=0.3,
                                catch_window_rad=math.radians(25.0),
                                swing_start_angle_rad=math.radians(5.0),
                                swing_start_rate=0.5,
                                swing_settle_timeout_s=3.0,
                                swing_timeout_s=10.0,
                                verbose=False):
    """upright_balance_test.cpp / pipeline_energy_lqr.cpp 의 상태머신을 그대로 이식."""
    if K_lqr is None:
        Ac, Bc = simulator.model_matrices(length_mm=length_m * 1000.0, friction=friction, g=g)
        Ad, Bd = simulator.discretize(Ac, Bc, dt)
        Q = np.diag([100.0, 10.0, 5.0, 2.0])
        R = np.array([[1.0]])
        _, K_lqr = simulator.dare_lqr(Ad, Bd, Q, R)
        K_lqr = K_lqr.flatten()

    total_steps = int(total_time_s / dt)
    t_arr = np.linspace(0, total_time_s, total_steps)

    theta = (math.radians(initial_angle_deg) + np.pi) % (2 * np.pi) - np.pi
    theta_dot = 0.0
    pos = 0.0
    vel = 0.0

    mode = "SWING"
    w2 = g / length_m

    swing_phase = PHASE_PREPOSITION
    swing_target_velocity = 0.0
    swing_energy_bias = 0.0
    previous_swing_rate = 0.0
    swing_kick_pending = True
    pushed_this_pass = False
    apex_handled = False
    t_swing_start = 0.0
    t_phase_start = 0.0

    traj_theta = np.zeros(total_steps)
    traj_pos = np.zeros(total_steps)
    traj_energy = np.zeros(total_steps)
    traj_mode = np.zeros(total_steps)
    traj_phase = np.zeros(total_steps)
    traj_energy_bias = np.zeros(total_steps)

    catch_step = -1

    def pendulum_energy(angle, rate):
        return 0.5 * rate * rate * length_m / g + math.cos(angle) - 1.0

    def wrap_pi(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def swing_start_x():
        return -control_polarity * swing_direction * swing_start_offset

    for i in range(total_steps):
        t_now = t_arr[i]
        energy = pendulum_energy(theta, theta_dot)
        traj_theta[i] = theta
        traj_pos[i] = pos
        traj_energy[i] = energy
        traj_mode[i] = 0 if mode == "SWING" else 1
        traj_phase[i] = swing_phase if mode == "SWING" else -1
        traj_energy_bias[i] = swing_energy_bias

        if mode == "SWING":
            L = length_m
            psi = wrap_pi(theta - np.pi)
            v = vel
            toward_bottom = (psi * theta_dot < 0.0)
            upper_half = (abs(theta) < 0.5 * np.pi)
            u = 0.0
            caught = False

            if swing_phase == PHASE_PREPOSITION:
                period = 2.0 * np.pi * math.sqrt(L / g)
                t = t_now - t_phase_start
                a = swing_start_x() / (period * period)
                if t < period:
                    u = a
                elif t < 2 * period:
                    u = -a
                else:
                    swing_target_velocity = 0.0
                    swing_phase = PHASE_SETTLE
                    t_phase_start = t_now
                    u = 0.0
            else:
                if swing_phase == PHASE_SETTLE:
                    still = (abs(psi) < swing_start_angle_rad and abs(theta_dot) < swing_start_rate)
                    if not still and (t_now - t_phase_start < swing_settle_timeout_s):
                        u = 0.0
                    else:
                        swing_phase = PHASE_PUMP
                        t_swing_start = t_now
                        previous_swing_rate = theta_dot

                if swing_phase in (PHASE_PUMP, PHASE_COAST, PHASE_RISE):
                    if not upper_half:
                        apex_handled = False
                    if upper_half and not apex_handled and (previous_swing_rate * theta_dot < 0.0) and (abs(theta) >= catch_window_rad):
                        apex_handled = True
                        swing_energy_bias += swing_retry_gain * (1.0 - math.cos(theta))
                        swing_phase = PHASE_PUMP
                        pushed_this_pass = False
                    if swing_phase == PHASE_COAST and not upper_half and (previous_swing_rate * theta_dot < 0.0):
                        swing_phase = PHASE_PUMP
                        pushed_this_pass = False
                    previous_swing_rate = theta_dot

                    if abs(theta) < catch_window_rad:
                        mode = "LQR_BALANCE"
                        caught = True
                        catch_step = i
                    else:
                        if swing_phase == PHASE_RISE and not upper_half and toward_bottom:
                            swing_phase = PHASE_PUMP
                            pushed_this_pass = False

                        if swing_phase != PHASE_RISE:
                            if not toward_bottom:
                                pushed_this_pass = False
                            if swing_phase == PHASE_PUMP and (swing_kick_pending or not pushed_this_pass):
                                bottom_rate = math.sqrt(max(0.0, 2.0 * w2 * (energy + 2.0)))
                                target_rate = math.sqrt(2.0 * w2 * (swing_energy_margin + swing_energy_bias + 2.0))
                                dv_mag = max(0.0, target_rate - bottom_rate) * L
                                direction = swing_direction if swing_kick_pending else (1.0 if theta_dot >= 0 else -1.0)
                                desired = v + control_polarity * direction * dv_mag
                                nxt = float(np.clip(desired, -swing_speed, swing_speed))
                                time_to_bottom = abs(psi) / max(abs(theta_dot), 1e-3)
                                lead_time = abs(nxt - v) / (2.0 * swing_accel) + dt
                                if swing_kick_pending or (toward_bottom and time_to_bottom < lead_time):
                                    swing_target_velocity = nxt
                                    swing_kick_pending = False
                                    pushed_this_pass = True
                                    if abs(desired - nxt) < 1e-4:
                                        swing_phase = PHASE_COAST

                        brake_lead = abs(theta_dot) * (abs(v) / swing_accel) * 0.5
                        if (not toward_bottom) and abs(theta) < 0.5 * np.pi + brake_lead:
                            swing_target_velocity = 0.0
                            if swing_phase == PHASE_COAST:
                                swing_phase = PHASE_RISE

                    stop_x = pos + v * abs(v) / (2.0 * swing_accel)
                    if abs(stop_x) > cart_limit - 0.03 and swing_target_velocity * stop_x > 0.0:
                        swing_target_velocity = 0.0

                    if not caught:
                        u = float(np.clip((swing_target_velocity - v) / dt, -swing_accel, swing_accel))

            if not caught:
                u = float(np.clip(u, -swing_accel, swing_accel))

        if mode == "LQR_BALANCE":
            u_lqr = -(K_lqr[0] * theta + K_lqr[1] * theta_dot + K_lqr[2] * pos + K_lqr[3] * vel)
            u = float(np.clip(u_lqr, -max_accel, max_accel))

        vel_next = float(np.clip(vel + u * dt, -max_speed, max_speed))
        u_actual = (vel_next - vel) / dt
        pos_next = pos + vel_next * dt
        theta_ddot = (g * math.sin(theta) - u_actual * math.cos(theta) - friction * theta_dot) / length_m
        theta_dot_next = theta_dot + theta_ddot * dt
        theta_next = wrap_pi(theta + theta_dot_next * dt)
        theta, theta_dot, pos, vel = theta_next, theta_dot_next, pos_next, vel_next

        if verbose and i % int(0.5 / dt) == 0:
            ph = PHASE_NAMES.get(int(traj_phase[i]), "BALANCE")
            print(f"t={t_now:6.2f}s  theta={math.degrees(theta):8.2f}deg  pos={pos*1000:8.2f}mm  "
                  f"E={energy:6.3f}  bias={swing_energy_bias:5.3f}  phase={ph}")

    return {'time': t_arr, 'theta_deg': np.degrees(traj_theta), 'pos_mm': traj_pos * 1000.0,
            'energy': traj_energy, 'mode': traj_mode, 'phase': traj_phase,
            'energy_bias': traj_energy_bias, 'catch_step': catch_step}


if __name__ == "__main__":
    res = simulate_with_statemachine(total_time_s=30.0, verbose=True)
    print()
    if res['catch_step'] >= 0:
        idx = res['catch_step']
        print(f"CATCH at t={res['time'][idx]:.2f}s, theta={res['theta_deg'][idx]:.2f}deg, pos={res['pos_mm'][idx]:.1f}mm")
        print(f"이후 |theta| max = {np.max(np.abs(res['theta_deg'][idx:])):.2f}deg")
        print(f"이후 |pos| max   = {np.max(np.abs(res['pos_mm'][idx:])):.1f}mm")
    else:
        print("캐치 실패 (지정한 total_time_s 안에 LQR_BALANCE 모드로 전환 못 함)")
