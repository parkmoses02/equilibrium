#include <Encoder.h>

#include <TMC.h>

#define CHA GPIO_NUM_5 //Encoder channel A
#define CHB GPIO_NUM_6 //Encoder channel B
#define EN GPIO_NUM_0 //TMC5160 Enable
#define CS GPIO_NUM_4 //TMC5160 Chip select
#define MISO GPIO_NUM_1 //TMC5160 MISO
#define MOSI GPIO_NUM_2 //TMC5160 MOSI
#define SCK GPIO_NUM_3 //TMC5160 Clock
#define CW 1 //Clockwise direction
#define CCW 2 //Counter-clockwise direction
#define MSTEPS 1 //Microsteps for TMC5160: 128
// 0: 256, 1: 128, 2: 64, 3: 32, 4: 16, 5: 8, 6: 4, 7: 2, 8: 1
enum MoveMode {
  STANDBY = 0,
  OSCILLATION = 1,
  BALANCE = 2,
  INVERTEDBALANCE = 3,
  DUMPING = 4,
  TEST = 5,
  LOOPINGCW = 6,
  LOOPINGCCW = 7
};
union FloatBytes {
    float value;
    uint8_t bytes[4];
};