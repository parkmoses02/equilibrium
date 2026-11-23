# Rev.L PCBA 업로드

- Add BOM File: RevL_BOM.xlsx
- Add CPL File: RevL_CPL.xlsx
- 대상: 21개 부품, 15개 BOM 그룹. H1~H4는 구멍이라 제외. R1~R4는 이전 요청대로 수납땜: R1/R2=4.7k, R3/R4=100Ω. ESP32/TMC 모듈 자체는 제외하고 해당 암소켓을 포함.
- BOM의 빈 LCSC 번호는 업로드 후 선택 필요. 기존 소켓/단자/JST 번호는 과거 선택 이력에서 가져온 후보이며 현재 재고/조립 가능 여부는 사이트에서 확인. U3/U4는 SN74LVC14ADR SOIC-14 (C133541); HC14/HC574 또는 TSSOP 패키지로 대체 금지.
- BOB용 암소켓은 1x11, 피치 2.54mm. ESP용은 1x20. 수헤더/암소켓을 혼동하지 말 것.
- 좌표는 Gerber와 같은 절대좌표계 X, -Y(mm). 음수 Y는 오류가 아니며 임의로 양수로 바꾸거나 보드 원점을 다시 빼지 말 것. 소켓/헤더는 핀열 중앙으로 보정.
- 회전은 업로드 시작값이다. 소켓/헤더는 이전 프로젝트의 수평/수직 보정 +90도를 반영했지만, 현재 선택한 공급사 모델로 웹 검증하지 않았다. U3/U4 pin1은 각각 (15.525,-13.19), (15.525,-26.19). IC pin1, J4 핀1/입구 방향, J1 전선 입구 방향과 모든 핀열을 미리보기에서 확인한 후 주문. 비대칭 부품의 부품몸체 중심 오프셋은 공급사 모델에 따라 조정이 필요할 수 있다.
- 이 파일 생성은 보드/배선을 수정하지 않음. 모듈 SD_MODE=0 스트랩 및 SPI_MODE=1 설정, 펌웨어 설정은 별도.

형식: https://jlcpcb.com/help/article/how-to-generate-the-bom-and-centroid-file-from-kicad
IC 확인: https://www.ti.com/product/SN74LVC14A/part-details/SN74LVC14ADR
부품 번호: https://www.lcsc.com/product-detail/C133541.html
