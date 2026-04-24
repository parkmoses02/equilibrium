# Double Inverted Pendulum 동역학 유도 설명

이 문서는 첨부된 필기(라그랑주 기반 유도)를 읽기 쉽게 정리한 설명서입니다.
핵심은 비선형 모델

\[
M(q)\ddot q + C(q,\dot q)\dot q + G(q) = \tau
\]

를 얻고, 평형점 근처에서 선형화해

\[
\dot X = AX + B\tau
\]

형태로 바꾸는 것입니다.

## 1) 일반화 좌표 정의

필기에서 사용한 상태를 다음처럼 둡니다.

\[
q = \begin{bmatrix}x \\ \theta_1 \\ \theta_2\end{bmatrix},
\quad
\dot q = \begin{bmatrix}\dot x \\ \dot\theta_1 \\ \dot\theta_2\end{bmatrix}
\]

- \(x\): 카트 위치
- \(\theta_1\): 첫 번째 링크 각도
- \(\theta_2\): 두 번째 링크 각도

## 2) 운동에너지와 위치에너지

필기에서는 카트 + 각 링크의 질점/회전 성분을 모두 합쳐 총 운동에너지 \(T\)를 만들고,
중력항으로 위치에너지 \(V\)를 둡니다.

\[
L(q,\dot q) = T(q,\dot q) - V(q)
\]

이때 \(T\)는 최종적으로

\[
T = \frac12 \dot q^T M(q) \dot q
\]

꼴로 정리됩니다.

## 3) 질량행렬 \(M(q)\)

필기의 파란 박스(A, B, C) 항은 관성 결합 항으로,
\(M(q)\)의 비대각 성분에 대응됩니다.

정리된 형태는 다음과 같습니다.

\[
M(q)=
\begin{bmatrix}
M_{\text{total}} & A\cos\theta_1 & B\cos\theta_2 \\
A\cos\theta_1 & I_{\text{link1}} & C\cos(\theta_1-\theta_2) \\
B\cos\theta_2 & C\cos(\theta_1-\theta_2) & I_{\text{link2}}
\end{bmatrix}
\]

- \(M_{\text{total}}\): 카트 및 링크를 포함한 등가 질량
- \(I_{\text{link1}}, I_{\text{link2}}\): 각 링크의 등가 관성
- \(A, B, C\): 링크 길이/질량으로 만들어지는 결합 계수

## 4) 오일러-라그랑주 방정식

각 일반화 좌표 \(q_i\)에 대해

\[
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot q_i}\right) - \frac{\partial L}{\partial q_i} = \tau_i
\]

를 적용하면 최종적으로

\[
M(q)\ddot q + C(q,\dot q)\dot q + G(q) = \tau
\]

를 얻습니다.

- \(C(q,\dot q)\dot q\): 코리올리/원심 항
- \(G(q)\): 중력 항
- \(\tau = [\tau_x,\tau_{\theta_1},\tau_{\theta_2}]^T\): 일반화 힘/토크

필기에서 표시한 것처럼 중력은 각도에 대해 사인항으로 나타나며,
\(x\)방향 중력항은 0입니다.

## 5) 평형점 주변 선형화

직립 평형점 근처에서

\[
\theta_1 \approx 0,\; \theta_2 \approx 0,
\quad
\sin\theta \approx \theta,
\quad
\cos\theta \approx 1,
\quad
\dot\theta_1^2,\dot\theta_2^2 \approx 0
\]

를 적용하면

1. \(M(q) \to M_0\) (상수 행렬)
2. \(C(q,\dot q)\dot q \to 0\)
3. \(G(q) \to Kq\) (소각 근사로 선형 복원항)

따라서

\[
M_0\ddot q + Kq = \tau
\]

가 됩니다.

## 6) 상태공간 형태

상태를

\[
X = \begin{bmatrix}q \\ \dot q\end{bmatrix}
\]

로 두면

\[
\dot X =
\begin{bmatrix}
0_{3\times3} & I_{3\times3} \\
-M_0^{-1}K & 0_{3\times3}
\end{bmatrix}X
+
\begin{bmatrix}
0_{3\times3} \\
M_0^{-1}
\end{bmatrix}\tau
\]

즉,

\[
\dot X = AX + B\tau
\]

형태를 얻고, 필기 메모처럼 LQR 같은 선형 제어기 설계에 바로 사용할 수 있습니다.

## 7) 해석 포인트

- 비선형 모델 \(M,C,G\)는 시뮬레이션 정확도에 중요합니다.
- 선형화 모델 \((A,B)\)는 제어기 설계/안정도 해석에 유리합니다.
- 구현 시에는 비선형 시뮬레이터로 검증하고, 선형 제어기의 유효 범위를 반드시 확인해야 합니다.

---
필요하면 다음 단계로, 이 문서에 맞춰 Python 또는 MATLAB용 \(M(q), C(q,\dot q), G(q)\) 계산 코드 템플릿도 추가할 수 있습니다.
