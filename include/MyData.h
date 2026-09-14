#pragma once
#include <Encoder.h>

#include <TMC.h>
#include <TMCStepDir.h>

// ===========================================================================
//  모터 구동 백엔드 선택
// ---------------------------------------------------------------------------
//  1 = SD_MODE 1 : STEP/DIR + 소프트웨어 램프 (TMCStepDir)
//      BIGTREETECH TMC5160T Pro 등 3D프린터용 스텝스틱. 이 보드들은 SD_MODE 를
//      기판에서 HIGH 로 묶어 출하해서 칩 내장 램프로는 모터가 돌지 않는다.
//
//  0 = SD_MODE 0 : 칩 내장 모션 컨트롤러 (TMC)
//      TMC5160_BOB_V1.0 등 SPI 모션 컨트롤용 보드. 보드를 교체하면 이 값을
//      0 으로만 바꾸면 되고, 다른 코드는 손댈 필요가 없다.
//
//  두 백엔드는 같은 API 와 같은 단위(TMC5160 내부 단위)를 쓰므로 Pendulum.cpp
//  의 변환 비율과 LQR 게인, 캘리브레이션 값은 양쪽에서 그대로 유효하다.
// ===========================================================================
#define USE_STEPDIR_BACKEND 1

// Freenove ESP32 WROOM + TMC5160 carrier Rev.C pin map.
#define CHA 34   // Encoder channel A (input-only GPIO)
#define CHB 35   // Encoder channel B (input-only GPIO)
#define EN 13    // TMC5160 Enable (active low)
#define CS 5     // TMC5160 CSN / VSPI CS; keep high while booting
#define MISO 19  // TMC5160 SDO / VSPI MISO
#define MOSI 23  // TMC5160 SDI / VSPI MOSI
#define SCK 18   // TMC5160 SCK / VSPI clock

// These pins are physically routed on Rev.C / Rev.J.
// USE_STEPDIR_BACKEND=1 이면 TMCStepDir 이 이 두 핀으로 직접 펄스를 만든다.
// USE_STEPDIR_BACKEND=0 이면 칩 내장 램프를 쓰므로 사용되지 않는다.
#define TMC_STEP 14
#define TMC_DIR 27
#define CW 1     // Clockwise direction
#define CCW 2    // Counter-clockwise direction
#define MSTEPS 1 // Microsteps for TMC5160: 128
// 0: 256, 1: 128, 2: 64, 3: 32, 4: 16, 5: 8, 6: 4, 7: 2, 8: 1
enum MoveMode
{
  STANDBY = 0,
  OSCILLATION = 1,
  BALANCE = 2,
  INVERTEDBALANCE = 3,
  DUMPING = 4,
  TEST = 5,
  LOOPINGCW = 6,
  LOOPINGCCW = 7
};
union FloatBytes
{
  float value;
  uint8_t bytes[4];
};
