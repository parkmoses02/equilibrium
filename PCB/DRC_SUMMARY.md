# Rev.C WROOM DRC 요약

대상: `esp32_wroom_tmc5160_carrier_revC_routed.kicad_pcb`  
검사 도구: KiCad 10 `kicad-cli pcb drc`

## 전기적 검사 결과

- Shorting items: 0
- Tracks crossing: 0
- Copper clearance: 0
- Unconnected items: 0
- Copper edge clearance: 0
- Solder-mask bridge caused by different nets: 0

## 잔여 경고

- Custom footprint library reference: 14
- Silkscreen overlap/over copper: 36
- Silkscreen edge clearance: 3

잔여 항목은 실크 인쇄와 생성형 custom footprint의 라이브러리 참조에 관한 경고이며 copper connectivity 오류는 아니다. 제작 전에는 실제 모듈을 1:1 출력물에 올려 핀열 간격, 모듈 방향 및 커넥터 방향을 확인해야 한다.

WROOM 핀맵 변경 검증:

- 우측 R11 위치(GPIO17): NC, GND 연결 없음
- 우측 R12 위치(GPIO16): NC, GND 연결 없음
- 우측 R17~R20: GND
- VSPI: MOSI 23, MISO 19, SCK 18, CS 5
