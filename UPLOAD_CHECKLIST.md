# 업로드 및 하드웨어 점검 체크리스트

다음은 내일 하드웨어에 펌웨어 업로드하고 테스트할 때 따라할 체크리스트입니다.
필요 시 하나씩 체크하면서 진행하세요.

---

## 1. 준비물
- PC (Windows)
- USB 케이블 (ESP 보드 연결용)
- ESP32 보드 (esp-wroom-32 또는 Adafruit QT Py ESP32-C3)
- TMC5160 드라이버(전원 별도 공급권장) + 스테퍼 모터
- 엔코더(증분형) 연결 완료
- 멀티미터, 전원 스위치
- 이 저장소의 코드 (로컬 복사)

## 2. 배선 확인 (기본값: `include/MyData.h`)
- CHA (엔코더 A): 핀 5
- CHB (엔코더 B): 핀 6
- EN  (TMC Enable): 핀 0
- CS  (TMC CS): 핀 4
- MISO (TMC MISO / SDO): 핀 1
- MOSI (TMC MOSI / SDI): 핀 2
- SCK (TMC SCK / CLK): 핀 3

> 주의: 사용 보드에 따라 핀 번호가 다를 수 있습니다. `include/MyData.h`의 값과 물리 배선을 반드시 일치시키세요.

## 3. 전원 관련 안전 확인
- TMC5160과 모터는 별도 모터 전원(일반적으로 12V~36V)을 필요로 합니다. 업로드 전엔 보드(ESP)만 USB 전원으로 연결해도 됨.
- 모터 전원은 테스트 모드에서만 켜고, 초기에는 전류를 낮게 설정하세요.
- 전원 연결/분리 시 모터 전원은 항상 꺼진 상태에서 작업.
- TMC의 `setCurrent`/`setGlobalScaler` 파라미터로 전류 한계를 확인하세요.

## 4. 빌드 및 업로드 (권장: VS Code + PlatformIO)
- VS Code -> PlatformIO 아이콘 -> Project Tasks -> `env:esp32dev` (또는 사용하는 보드) -> `Upload` 클릭

대체(터미널) 명령 (PowerShell 예):

```powershell
# esp32dev 환경으로 업로드
C:\Users\phs02\.platformio\penv\Scripts\platformio.exe run --environment esp32dev --target upload

# adafruit qtpy esp32-c3 쓰는 경우
C:\Users\phs02\.platformio\penv\Scripts\platformio.exe run --environment adafruit_qtpy_esp32c3 --target upload
```

- 업로드가 실패하면 포트 확인(장치 관리자 / VS Code status bar) 후 다시 시도.

## 5. 시리얼 모니터로 확인
- 업로드 후 시리얼 모니터를 열어 보드가 출력하는 상태 메시지를 확인합니다.

```powershell
# example: esp32dev 환경의 시리얼 모니터
C:\Users\phs02\.platformio\penv\Scripts\platformio.exe device monitor --environment esp32dev
```
- 보레이트는 펌웨어에 정의된 `Serial.begin(115200);`이므로 `115200`으로 설정하세요.
- 주기적으로 전송되는 값(각도, 각속도, 위치, 속도)들이 보이는지 확인.

## 6. 기본 동작 테스트 순서
1. 엔코더 리셋: 전원 및 리셋 후 `resetEncoder()` 호출(Serial 명령 또는 전용 버튼).
2. TMC 리셋/초기화: `resetTMC()` 호출로 기본 모드 진입.
3. STANDBY 모드 확인: 모터가 위치 0을 목표로 하는지 확인.
4. OSCILLATION 또는 BALANCE 모드로 전환(Serial 명령/PC 패널 사용)
   - 각 모드에서 엔코더 값과 모터 반응을 관찰.
5. 안전 테스트: 전류가 과도하게 상승하면 즉시 정지.

## 7. 시리얼 명령(간단 요약)
- 펌웨어는 5바이트 패킷으로 설정을 받습니다. (첫 바이트: 주소, 뒤 4바이트: float)
- 주요 주소 예시:
  - `0x00` : magnitude (진폭)
  - `0x01` : speed
  - `0x02` : acceleration
  - `0x03` : threshold
  - `0x20`~`0x23` : PID 계수 (`kpa`, `kda`, `kpm`, `kdm`)
  - `0x50` : setMoveMode (data[1]에 모드 값)
  - `0x51` : resetTMC
  - `0x52` : resetEncoder

(더 정확한 매핑은 `src/main.cpp`의 `checkSerial()`를 참조)

## 8. 문제 발생 시 확인 포인트
- 빌드 실패: `platformio` 에러 로그(종속성/툴체인) 확인
- 업로드 실패: 포트 충돌, 드라이버(USB-to-UART) 설치 확인
- 엔코더값 이상: 엔코더 배선(풀업/풀다운), 인터럽트 핀 변경 필요 여부
- 모터 미동작: 모터 전원 유무 / TMC Enable 핀(EN) 신호 / CS, SPI 연결 확인
- 과열/소음: TMC 전류 설정 낮추기

## 9. 테스트 기록(현장에 적을 항목)
- 업로드 성공 여부: ( )
- 시리얼 출력 포맷 정상: ( )
- 엔코더 값 정상: ( )
- 모터 응답 정상: ( )
- 각 모드별 동작 확인: STANDBY ( ), OSCILLATION ( ), BALANCE ( ), DUMPING ( )
- 특이사항 / 로그 파일 위치:

---

파일 위치: `UPLOAD_CHECKLIST.md` (프로젝트 루트)

필요하면 이 체크리스트를 기반으로 `CHECKLIST.pdf`나 출력용 버전을 만들어 드리겠습니다.
