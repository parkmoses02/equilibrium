#include <Arduino.h>

// Hand-turn signal check (not a high-speed position controller).
constexpr int8_t steps[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
constexpr int8_t transition(unsigned prev, unsigned now) {
    return steps[prev * 4 + now];
}
static_assert(transition(0, 1) + transition(1, 3) + transition(3, 2) +
              transition(2, 0) == 4, "forward quadrature cycle");
static_assert(transition(0, 2) + transition(2, 3) + transition(3, 1) +
              transition(1, 0) == -4, "reverse quadrature cycle");
static_assert(transition(1, 1) == 0 && transition(0, 3) == 0, "no false steps");

struct Encoder {
    uint8_t a, b, ab;
    long count;
    unsigned sample() const { return (digitalRead(a) << 1) | digitalRead(b); }
};
Encoder enc[] = {{36, 39, 0, 0}, {32, 33, 0, 0}};

void setup() {
    digitalWrite(13, HIGH); // TMC DRV_ENN: keep motor output disabled
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);
    for (auto &e : enc) {
        pinMode(e.a, INPUT);
        pinMode(e.b, INPUT);
        e.ab = e.sample();
    }
    Serial.println("Rev.L HEDS: turn J5 encoders slowly by hand.");
}

void loop() {
    for (auto &e : enc) {
        const unsigned next = e.sample();
        e.count += transition(e.ab, next);
        e.ab = next;
    }
    static uint32_t last = 0;
    if (millis() - last >= 250) {
        last = millis();
        for (unsigned i = 0; i < 2; ++i) {
            const auto &e = enc[i];
            Serial.printf("HEDS%u A=%d B=%d COUNT=%ld  ", i+1, e.ab >> 1, e.ab & 1, e.count);
        }
        Serial.println();
    }
}
