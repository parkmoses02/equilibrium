#pragma once
#include <Encoder.h>

#include <TMCStepDir.h>   // 이 브랜치의 유일한 구동 백엔드 (SD_MODE = 1)

// Freenove ESP32 WROOM + TMC5160 carrier Rev.C pin map.
#define CHA 34   // Encoder channel A (input-only GPIO)
#define CHB 35   // Encoder channel B (input-only GPIO)
#define EN 13    // TMC5160 Enable (active low)
#define CS 5     // TMC5160 CSN / VSPI CS; keep high while booting
#define MISO 19  // TMC5160 SDO / VSPI MISO
#define MOSI 23  // TMC5160 SDI / VSPI MOSI
#define SCK 18   // TMC5160 SCK / VSPI clock

// These pins are physically routed on Rev.C / Rev.J.
// TMCStepDir 이 이 두 핀으로 직접 펄스를 만든다 (STEP 은 LEDC 출력).
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
