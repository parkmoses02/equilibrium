# 시스템 식별 (System identification)

실제 하드웨어 데이터로 진자 모델(관성, 결합항, 마찰, 입력 지연)을 맞춥니다.
결과는 칼만필터(`src/double_pendulum/state_estimator.h`)가 쓰는 `model_params.h`에 들어가고,
궤적 설계 도구(check_model.py, dd-du/model.js, dd-uu/design.py)에 넣을 값도 함께 출력됩니다.

## 1. 기록 (펌웨어 `double_sysid`)

PlatformIO에서 `double_sysid` 환경을 업로드하고 모니터를 엽니다(115200).
`log2file` 필터가 켜져 있어서 프로젝트 폴더의 `logs/`에 로그가 저장됩니다.

| 명령 | 동작 |
|---|---|
| `z` | 링크를 매달린 채(DD) 정지시키고 카트를 가운데 둔 다음, 원점 저장 (모터 OFF) |
| `f` | 자유 감쇠 20 s, **모터 OFF**. 링크를 손으로 흔들고, 손을 뗀 다음 `f` |
| `c`, `c1`~`c9` | 처프: 카트가 0.3→5 Hz로 16 s 흔들고 4 s 정지 유지. 가속 진폭 = 숫자 × 0.4 m/s² (`c` = `c3`) |
| `d` | 마지막 기록을 `SYSID_CSV`로 출력 |
| `x` | 즉시 정지 (모터 OFF) |

권장 순서 (각도는 매달린 자세 기준, base = 카트에 붙은 1번 링크, tip = 2번 링크.
"일직선" = 팔꿈치를 펴서 tip이 base와 같은 방향을 가리킴(상대각 0), 수직으로 늘어뜨리는 것이 아님):

| 실험 | 시작 자세 | 주로 정해지는 값 |
|---|---|---|
| f1 | base +60°, tip 일직선 (tip 절대각도 +60°) | 느린 모드, 큰 진폭 (J1, d1, b1) |
| f2 | base +15°, tip 일직선 (tip 절대각도 +15°) | 작은 진폭: 점성 마찰과 쿨롱 마찰 구분 |
| f3 | base +30°, tip을 반대로 꺾어 절대각 -30° (상대각 -60°) | 빠른 모드 (J2, C) |
| f4 | base 매달린 채(0°), tip만 +45° | 팔꿈치 (J2, b2, fc2) |

1. `z` → 시작 자세로 잡고 있다가 밀지 말고 손을 뗀 뒤 바로 `f` → 끝나면 `d`. 이것을 f1~f4 각각 실행
   - 모든 각도는 70° 이하로. tip이 위로 넘어갈 수 있습니다.
   - **카트를 클램프나 손으로 고정하세요.** 모터 OFF에서는 XACTUAL이 실제 카트 움직임을 모르고,
     fit.py는 카트가 정지해 있다고 가정합니다. 흔들림에 카트가 밀리면 피팅이 틀어집니다.
2. 카트 가운데, DD 정지 → `z` → `c` → `d`
3. 링크 흔들림이 작으면 `c5`, 너무 크면(ANGLE_LIMIT으로 멈추면) `c1`~`c2`

안전장치: 처프 중 링크가 매달린 자세에서 80° 이상 벗어나거나, 레일 가드가 걸리거나,
드라이버 이상이 생기면 감속 후 정지합니다. 카트 진폭은 z 기준 ±80 mm로 제한됩니다.
처프가 끝난 뒤 카트가 가운데가 아닐 수 있으니, 다음 `c` 전에 가운데로 옮기고 `z`를 다시 하세요.

## 2. 피팅

```bash
python simulation/sysid/fit.py logs/*.log              # 결과 확인
python simulation/sysid/fit.py logs/*.log --refine     # 시뮬레이션 오차로 보정 (더 정확, 수십 초)
python simulation/sysid/fit.py logs/*.log --refine --write   # model_params.h 갱신
python simulation/sysid/fit.py --selftest              # 하드웨어 없이 합성 데이터로 검증
```

출력:
- 공칭값 대비 피팅값 (J1, J2, C, d1, 마찰 b1/b2/fc1/fc2) 과 입력 지연
- 0.6 s 개루프 시뮬레이션 각도 RMS: 공칭 모델 vs 피팅 모델 (작을수록 시뮬이 실제와 비슷)
- check_model.py / model.js / design.py 에 넣을 `B, S, length, T2, Tlocked` 와 `b1, b2`

방법: 2차 미분(각가속도)은 양자화 잡음 때문에 관성을 낮게 편향시키므로 쓰지 않고,
짧은 구간(0.1 s)마다 "일반화 운동량 변화 = 일반화 힘의 적분" 식을 최소제곱으로 풉니다.
`--refine`은 그 결과에서 출발해 0.6 s 개루프 시뮬레이션 오차를 최소화합니다.
엔코더 오류 플래그가 있는 샘플 주변은 자동으로 제외됩니다.

## 3. 이후

1. `--write`로 `model_params.h` 갱신 → 칼만필터가 새 모델을 씀 (펌웨어 재빌드)
2. 출력된 `B, S, length, T2, Tlocked`로 check_model.py 상수를 바꾸고 게인 재계산
3. model.js / design.py 파라미터와 마찰을 바꿔 궤적 재설계
4. 입력 지연이 크면(> 10 ms) 궤적 설계에 지연을 반영할지 검토

## 검증

```bash
g++ -std=c++17 -O2 simulation/sysid/test-estimator.cpp -o test-estimator && ./test-estimator
```
EKF 각속도 정확도(기존 차분 필터 대비), 엔코더 슬립 복구, 엔코더 고장 시 정지를 확인합니다.
