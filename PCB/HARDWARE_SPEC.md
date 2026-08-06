# 이중역진자 제어 PCB 하드웨어 사양

작성 기준: 2026-08-06  
상태: Rev.C WROOM 배선 완료 사양

## 1. 시스템 구성

- 제어기: Freenove ESP32 WROOM 보드
- 스테퍼 드라이버: TMC5160T Pro V1.0 모듈
- 스테퍼 모터: NEMA17, 17HS4401, 1.8° bipolar
- 엔코더: 광전 증분형 로터리 엔코더 38S6G5-B-G24N, 600 PPR
- 주 전원: DC 36 V
- 용도: 이중역진자 구동 및 위치/속도 계측

## 2. ESP32 WROOM 보드

- 보드: Freenove ESP32 WROOM
- 로직 전압: 3.3 V
- 보드 전원 입력: 5 V 핀 사용 예정
- 핀 피치: 2.54 mm
- GPIO6~GPIO11: Flash 연결 핀이므로 사용 금지
- GPIO16~GPIO17: WROOM 보드에서 사용 가능하지만 Rev.C PCB에서는 NC로 유지
- 카메라가 없는 WROOM 보드이므로 WROVER-CAM 카메라 핀 충돌 조건은 해당 없음
- ESP32 GPIO에는 3.3 V를 초과하는 신호를 직접 입력하지 않음

## 3. 스테퍼 모터

- 모델: 17HS4401
- 형식: 2상 bipolar hybrid stepper motor
- 프레임: NEMA17
- 스텝각: 1.8°/step
- 기본 스텝 수: 200 full steps/revolution
- 정격 상전류: 1.68 A/phase
- 정격 상전압: 3.27 V
- 상저항: 1.95 Ω ±10%
- 상인덕턴스: 4.4 mH ±20%
- 유지토크: 0.45 N·m
- 절연 등급: Class B
- 절연저항: 100 MΩ min (500 V DC)
- 중량: 0.3 kg
- 모터 길이: 40 mm
- 데이터시트 표기 동작환경: -20~+50 °C, 15~95% RH
- 결선: A+, A-, B+, B-의 4선
- 구동 방식: TMC5160의 전류 제한형 chopper 구동
- 주의: 36 V는 드라이버의 VMOT 전압이며 모터 코일에 36 V를 직접 연속 인가하는 방식이 아님

## 4. TMC5160 드라이버 모듈

- 모듈: TMC5160T Pro V1.0
- 모터 전원 VMOT: 36 V 입력 예정
- 로직 전원 VIO: ESP32와 동일한 3.3 V
- 제어: STEP/DIR + SPI
- 필수 신호: EN, STEP, DIR, MOSI, MISO, SCK, CS
- 모터 출력: A1, A2, B1, B2
- DIAG0/DIAG1: 현재 미사용
- ENCA/ENCB/ENCN: 현재 미사용
- 모듈 상단 보조 핀은 실제 모듈에 헤더가 납땜되지 않았으므로 캐리어 PCB에서도 사용하지 않는 방향
- 실제 연속 허용전류는 모듈의 방열판, 냉각 조건 및 제조사 사양을 기준으로 최종 확인 필요
- TMC5160 모듈 방열판 보유 및 사용 예정
- PCB에는 냉각팬 커넥터와 팬 전원회로를 넣지 않음
- 시운전 중 과열 시 외부 송풍으로 보조하고 SPI의 과열 사전 경고 상태를 확인

## 5. 엔코더

- 모델: 38S6G5-B-G24N
- 분해능: 600 PPR
- 케이블 길이: 약 1.5 m
- 형식: 광전 증분형, A/B 2상
- 출력: NPN open-collector로 알려져 있으나 제품 라벨/판매 사양으로 최종 확인 필요
- 전원 범위: 판매 사양에 따라 DC 5~24 V 또는 8~24 V로 표기되므로 실물 라벨 확인 필요
- 권장 PCB 인터페이스:
  - 엔코더를 5 V로 구동할 수 있는지 실물 사양 확인
  - A/B 출력에 각각 3.3 V 기준 외부 pull-up 적용
  - 초기값: 2.2~4.7 kΩ pull-up, 최고 RPM과 실측 파형에 따라 조정
  - 약 1.5 m 케이블의 노이즈 대책으로 입력 직렬 저항, ESD 보호 및 3.3 V Schmitt-trigger buffer 권장
  - 엔코더 케이블은 모터 배선과 분리하고 가능하면 shielded twisted-pair 사용
  - 엔코더 GND와 PCB GND 공통 연결
- 600 PPR는 A/B 4체배 시 최대 2400 counts/revolution으로 처리 가능

## 6. 전원 구조

- 입력 전원: DC 36 V
- 36 V 직접 공급 대상: TMC5160 VMOT
- 5 V 필요 대상: ESP32 보드, 엔코더(5 V 구동 가능 모델인 경우)
- 3.3 V 대상: TMC5160 VIO 및 ESP32 신호 pull-up
- 시스템 전원은 외부 power supply에서 36 V와 필요한 저전압을 공급하는 방향
- 36 V는 VMOT에만 연결하며 ESP32에는 직접 연결하지 않음
- ESP32용 5 V를 외부에서 별도로 공급하거나, 외부 DC-DC를 사용하는 것으로 잠정 결정
- 36 V와 5 V 입력은 분리하고 GND는 공통으로 구성
- 전원 입력부에 필요한 항목:
  - 역극성 보호
  - 퓨즈 또는 resettable fuse
  - 36 V 이상의 충분한 정격을 가진 TVS 검토
  - TMC5160 가까이에 bulk capacitor 및 ceramic bypass capacitor
  - 5 V regulator 입력/출력 capacitor

## 7. PCB 배선 설계 기준(잠정)

- PCB: 2 layer 가정
- 동박: 1 oz 가정
- 외층 허용 온도 상승: 10 °C 목표
- 모터 상전류 계산 기준: 1.68 A/phase 정격, 설계 여유를 위해 2.0 A 연속 기준 적용 예정
- VMOT 및 모터 A/B 배선: 전류 기준으로 폭 계산 후 적용
- 신호선: 0.25~0.30 mm 이상 권장
- GND: 양면 ground plane 및 stitching via 적용
- 모터 전류 루프는 짧고 굵게 구성
- SPI/엔코더 신호는 모터 출력 및 VMOT 배선과 이격
- 전원·모터 커넥터와 핀헤더의 정격전류가 PCB 패턴보다 낮지 않은지 확인

### 7.1 전류별 패턴 폭 계산 및 최종 적용값

계산 가정:

- 2층 PCB, 외층 배선
- 1 oz copper, 약 35 µm 두께
- 허용 온도 상승 10 °C
- IPC-2221 외층 경험식 기준의 보수적 1차 계산
- 실제 적용 폭은 계산 최소값보다 넓게 선정

| 배선 종류 | 설계전류 | 계산 최소 폭 | PCB 적용 폭 | 비고 |
|---|---:|---:|---:|---|
| VMOT 36 V | 2.0 A 기준 | 약 0.78 mm | 2.00 mm | bulk capacitor와 TMC 사이를 짧게 구성 |
| 모터 A1/A2/B1/B2 | 2.0 A 연속 기준 | 약 0.78 mm | 1.50 mm | 모터 정격 1.68 A/phase에 여유 적용 |
| 외부 5 V 전원 | 1.0 A 잠정 | 약 0.30 mm | 1.00 mm | ESP32와 엔코더 공급 |
| 3.3 V 로직 | 0.5 A 미만 | 0.30 mm 미만 | 0.50 mm | TMC VIO 및 pull-up |
| SPI/STEP/DIR/EN | 수 mA | 제조 최소폭 이하 | 0.30 mm | 모터선과 이격 |
| 엔코더 A/B | 수 mA | 제조 최소폭 이하 | 0.30 mm | 1.5 m 케이블, 4.7 kΩ pull-up 및 100 Ω 직렬저항 |

패턴 폭보다 핀헤더, 단자대, 납땜 상태 및 TMC5160 모듈의 방열 조건이 먼저 전류 한계가 될 수 있으므로 이 부품들의 정격도 함께 확인한다.

### 7.2 Rev.C WROOM에 적용한 배선 구조

- F.Cu/B.Cu 양면을 사용해 신호 교차를 분리
- F.Cu와 B.Cu에 GND copper zone 적용
- VMOT 및 모터 출력은 신호선보다 넓게 배선
- 엔코더 A/B는 GPIO34/GPIO35에 연결
- 엔코더 입력마다 4.7 kΩ 3.3 V pull-up과 100 Ω 직렬저항 적용
- TMC5160 상단 DIAG0/DIAG1/ENCA/ENCB/ENCN은 미사용 및 캐리어 풋프린트에서 제외
- 냉각팬 회로와 팬 커넥터는 제외
- Freenove ESP32 WROOM 2×20 핀맵 적용
- WROVER-CAM에서 GND였던 위치 중 WROOM의 GPIO17/GPIO16 위치는 GND 연결을 제거하고 NC 처리
- KiCad 10 DRC 결과는 Rev.C 생성 후 별도 보고서 기준
- 잔여 DRC 항목은 사용자 정의 풋프린트 라이브러리 및 silkscreen 겹침 관련 비전기적 경고
- GND zone은 F.Cu/B.Cu 양면 모두 채운 상태로 저장

## 8. 배선 후 제작 전 확인 필요 항목

1. 외부 power supply의 5 V 출력 제공 여부와 최대 전류
2. 36 V 전원공급기의 최대 출력전류
3. TMC5160T Pro 모듈의 정확한 구매 링크 또는 뒷면 사진
4. 엔코더 실물 라벨 및 케이블 색상별 기능(V+, GND, A, B, shield)
5. 모터 예상 최고 회전속도와 연속 운전시간
6. 사용할 전원·모터·엔코더 커넥터 종류와 정격
7. PCB 동박 1 oz/2 oz 선택과 보드 제작업체 최소 선폭/간격
8. Freenove WROOM 실물의 핀열 간격과 방향 1:1 출력 확인

## 9. 참고 자료

- Analog Devices TMC5160 제품 정보: https://www.analog.com/en/products/tmc5160.html
- TMC5160 데이터시트: https://www.analog.com/media/en/technical-documentation/data-sheets/tmc5160a_datasheet_rev1.18.pdf
- Freenove ESP32 WROOM 제품 핀맵: 사용자 제공 이미지 기준

> 이 문서는 PCB 설계를 위한 작업 사양이다. 모델명이 같은 모터와 엔코더도 판매처에 따라 전기 사양이 달라질 수 있으므로, 최종 배선 및 부품 선정 전에 실물 라벨과 구매 사양을 우선한다.
