#include <Arduino.h>

// >>> 여기가 엔코더 캘리브레이션 코드입니다 (src/tests/encoder_calibration.cpp) <<<

// Carrier Rev.J encoder inputs. GPIO34/35 do not provide internal pull-ups;
// the PCB's external pull-up resistors must therefore be fitted/enabled.
constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr uint32_t REPORT_INTERVAL_MS = 100;

volatile int32_t encoderCount = 0;
volatile uint8_t previousAB = 0;

// Quadrature transition table. Invalid/no-change transitions contribute zero.
constexpr int8_t QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

void IRAM_ATTR updateEncoder()
{
    const uint8_t currentAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    const uint8_t transition = (previousAB << 2) | currentAB;
    encoderCount += QUADRATURE_TABLE[transition];
    previousAB = currentAB;
}

int32_t readEncoderCount()
{
    noInterrupts();
    const int32_t value = encoderCount;
    interrupts();
    return value;
}

void zeroEncoder()
{
    noInterrupts();
    encoderCount = 0;
    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    interrupts();
}

void setup()
{
    Serial.begin(115200);
    // ESP-IDF's GPIO ISR service is installed via an inter-processor call to
    // core 1. Calling attachInterrupt() before that IPC task is ready causes
    // "esp_ipc_call_blocking failed" -> "GPIO ISR Service Failed To Start"
    // -> a null-pointer LoadProhibited panic and a boot loop. Give it time.
    delay(100);
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);

    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);

    delay(300);
    Serial.println();
    Serial.println("=== Incremental encoder calibration ===");
    Serial.println("1) Let the pendulum hang vertically downward.");
    Serial.println("2) Send Z (or R) to set that position to count 0.");
    Serial.println("3) Rotate exactly one full revolution and note COUNT.");
    Serial.println("Typical 600-PPR x4 result is about +/-2400 counts/rev.");
    Serial.println("GPIO34/35 require external pull-ups; INPUT_PULLUP is unavailable.");
}

void loop()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 'z' || command == 'Z' ||
            command == 'r' || command == 'R')
        {
            zeroEncoder();
            Serial.println("ZERO SET: current position is now count 0.");
        }
    }

    static uint32_t lastReportMs = 0;
    static int32_t previousCount = 0;
    const uint32_t now = millis();
    if (now - lastReportMs >= REPORT_INTERVAL_MS)
    {
        const int32_t count = readEncoderCount();
        const int32_t delta = count - previousCount;
        Serial.printf("COUNT=%ld  DELTA/100ms=%ld  DIR=%s  A=%d B=%d\n",
                      static_cast<long>(count),
                      static_cast<long>(delta),
                      delta > 0 ? "+" : (delta < 0 ? "-" : "STOP"),
                      digitalRead(PIN_ENCODER_A),
                      digitalRead(PIN_ENCODER_B));
        previousCount = count;
        lastReportMs = now;
    }
}
