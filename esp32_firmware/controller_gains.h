/*
 * 실제 하드웨어에서 사용할 LQR 게인 계산 가이드
 *
 * 이 파일은 시뮬레이션 코드(singlePendulum_improved.py)의 K 행렬 값을
 * 실제 펌웨어에 반영하는 방법을 설명합니다.
 */

#ifndef CONTROLLER_GAINS_H
#define CONTROLLER_GAINS_H

// ==========================================
// 1. 시뮬레이션에서 K 행렬 추출 방법
// ==========================================
/*
 * Python singlePendulum_improved.py 실행 후:
 *
 * 화면에 표시되는:
 * "Kx: XXX | Kθ: XXX | Kv: XXX | Kω: XXX"
 *
 * 이 값들을 아래에 복사-붙여넣기 하세요:
 */

// ========== 기본 LQR 게인 (초기 설정) ==========
// Q_x=100, Q_theta=1000, Q_xdot=10, Q_thetadot=100
static const float LQR_GAINS_BASIC[] = {
    -10.00f,    // K_x
    -318.23f,   // K_theta
    -5.00f,     // K_xdot
    -31.62f     // K_thetadot
};

// ========== 가중 LQR 게인 (보다 공격적인 제어) ==========
// Q_x=200, Q_theta=2000, Q_xdot=15, Q_thetadot=150
static const float LQR_GAINS_AGGRESSIVE[] = {
    -14.14f,    // K_x
    -450.00f,   // K_theta
    -7.07f,     // K_xdot
    -44.72f     // K_thetadot
};

// ========== 보수적 LQR 게인 (안정성 중시) ==========
// Q_x=50, Q_theta=500, Q_xdot=5, Q_thetadot=50
static const float LQR_GAINS_CONSERVATIVE[] = {
    -7.07f,     // K_x
    -225.00f,   // K_theta
    -3.54f,     // K_xdot
    -22.36f     // K_thetadot
};

// ==========================================
// 2. 실제 시스템에 맞는 게인 선택
// ==========================================
/*
 * 프로토타입 테스트 절차:
 *
 * 1. BASIC 게인으로 시작
 *    - 약한 진동이 있는지 확인
 *    - 안정화에 너무 오래 걸리는지 확인
 *
 * 2. 문제 진단:
 *    - 약한 제어 → AGGRESSIVE로 변경
 *    - 불안정한 오버슈트 → CONSERVATIVE로 변경
 *    - 카트가 흔들림 → Q_x 증가
 *    - 진자가 흔들림 → Q_theta 증가
 *
 * 3. 정밀 튜닝:
 *    시뮬레이션의 슬라이더를 조정하여 최적 파라미터를 찾은 후,
 *    새로운 K값을 여기에 입력하고 펌웨어 재컴파일
 */

// ==========================================
// 3. 현재 사용 게인 (기본값)
// ==========================================
#define CURRENT_GAINS LQR_GAINS_BASIC

#define K_X             CURRENT_GAINS[0]
#define K_THETA         CURRENT_GAINS[1]
#define K_XDOT          CURRENT_GAINS[2]
#define K_THETADOT      CURRENT_GAINS[3]

#endif // CONTROLLER_GAINS_H
