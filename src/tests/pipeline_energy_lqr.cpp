#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>

/**
 * ============================================================================
 * Pipeline: Actual Hardware + Real Riccati LQR + Acceleration Control + Energy Swing-up
 * ============================================================================
 * 
 * 1. Hardware: ESP32 + TMC5160 (TMCStepDir / LEDC hardware pulse) + AB Quadrature Encoder
 * 2. Acceleration Control:
 *    - The control output is cart acceleration u = ddot{x} [m/s^2].
 *    - Firmware integrates u into commandedVelocity [m/s] each control tick (500Hz),
 *      clamped by MAX_CART_ACCEL and MAX_CART_SPEED, and streams step pulses.
 * 3. Energy-based Swing-up:
 *    - Reused from upright_balance_test.cpp.
 *    - Normalized mechanical energy E = 0.5 * theta_dot^2 * (L/g) + cos(theta) - 1.
 *    - Bottom velocity kicks until E reaches 0 (upright), horizontal braking.
 * 4. Riccati LQR Balancing:
 *    - Gains K = [k_angle, k_rate, k_cartPos, k_cartVel] can be calculated via Riccati (DARE)
 *      on the PC panel and received over serial in real time, or use default calibrated gains.
 * 5. Serial Protocol:
 *    - ESP32 -> PC telemetry: 0xAA + addr + float32 (6 bytes).
 *    - PC -> ESP32 commands use the SAME framing: 0xAA + addr + 4 payload bytes (6 bytes).
 *      The header keeps binary frames unambiguous against the single-character ASCII
 *      commands (address 0x20 is ' ' = disarm and 0x50 is 'P' = flip polarity, which
 *      used to be misread as ASCII and desynchronised the stream).
 *    - Also accepts ASCII serial commands for testing (Z, A, S, X, P, D, O).
 *      O (or frame 0x53) re-defines the CURRENT cart position as 0 without touching the
 *      encoder zero: used to re-centre after the open-loop step counter drifted (lost steps).
 */

// --- Pin Map (ESP32 WROOM + carrier Rev.J) ---
constexpr uint8_t PIN_ENCODER_A = 34;
constexpr uint8_t PIN_ENCODER_B = 35;
constexpr uint8_t PIN_TMC_EN   = 13; // active LOW
constexpr uint8_t PIN_TMC_STEP = 14;
constexpr uint8_t PIN_TMC_DIR  = 27;
constexpr uint8_t PIN_TMC_CS   = 5;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_SCK  = 18;

// --- Physical & Mechanical Constants ---
constexpr int32_t ENCODER_COUNTS_PER_REV = 2400;
constexpr float RAD_PER_COUNT = 2.0f * PI / ENCODER_COUNTS_PER_REV;
constexpr float STEPS_PER_METRE = (200.0f * 16.0f) / 0.040f; // 80,000 steps/m
constexpr float GRAVITY = 9.81f;

// --- Safety & Kinematic Limits (Defaults, can be updated via Serial) ---
float gMaxCartSpeed = 0.80f;     // m/s
float gMaxCartAccel = 12.0f;    // m/s^2
float gCartSoftLimit = 0.35f;   // m (rail soft limit from origin)
// Direct balance ('A' / panel "Direct LQR Balance"): the pendulum must be within +-gArmWindowRad of
// upright when arming. Adjustable from the panel (frame 0x27, degrees, 1..90). The balance does not
// have to be catchable from there; the window only decides whether arming is allowed.
float gArmWindowRad = 45.0f * PI / 180.0f;
constexpr float ARM_TRIP_MARGIN_RAD = 10.0f * PI / 180.0f;   // trip angle = window + margin (directly armed)
constexpr float TRIP_ANGLE_RAD = 25.0f * PI / 180.0f;        // normal trip angle (swing-up -> catch path)
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz control loop
constexpr uint32_t TELEMETRY_PERIOD_MS = 20; // 50 Hz telemetry

// --- Energy Swing-up Constants (reused from upright_balance_test.cpp) ---
float gSwingSpeed = 1.20f;      // m/s
float gSwingAccel = 16.0f;     // m/s^2
constexpr float SWING_START_OFFSET_M = 0.10f;
constexpr float SWING_DIRECTION = -1.0f;
constexpr uint32_t SWING_SETTLE_TIMEOUT_MS = 3000;
constexpr uint32_t SWING_TIMEOUT_MS = 10000;
constexpr float SWING_START_ANGLE_RAD = 5.0f * PI / 180.0f;
constexpr float SWING_START_RATE = 0.5f; // rad/s
float gSwingEnergyMargin = 0.0f;
constexpr float SWING_RETRY_GAIN = 0.3f;
constexpr float CATCH_WINDOW_RAD = 25.0f * PI / 180.0f;
constexpr float CATCH_TRIP_RAD   = 40.0f * PI / 180.0f;
constexpr uint32_t CATCH_SETTLE_MS = 700;
constexpr float CATCH_KCARTPOS = 20.0f;
constexpr float CATCH_KCARTVEL = 14.0f;
constexpr uint32_t CATCH_STIFF_MS = 1000;
constexpr uint32_t CATCH_BLEND_MS = 1000;

// --- Riccati LQR Gains ---
// Acceleration control law: u = - (K_angle * angle + K_rate * angularRate + K_cartPos * x + K_cartVel * v)
// Calibrated baseline values; can be overwritten live via serial packet from Riccati solver.
float gKangle   = 50.0f;   // [m/rad/s^2]
float gKrate    = 8.0f;    // [m/rad/s]
float gKcartPos = 3.0f;    // [1/s^2]
float gKcartVel = 5.0f;    // [1/s]
float gAngleTrim = 0.0f;   // [rad]
float gPendulumLengthM = 0.20f; // [m]

// Soft (demo) balance gains, toggled with 'D' (ported from upright_balance_test.cpp).
// Lower pendulum-loop gains make the tilt visible before the cart catches it. Below roughly
// (28, 3) the damping ratio at L = 0.2 m gets small and, with real motor lag, the pendulum can
// oscillate at 1~1.5 Hz and fall. If it wobbles, raise gKrateSoft first.
float gKangleSoft = 15.0f; // [m/rad/s^2]
float gKrateSoft  = 2.0f;  // [m/rad/s]
bool softBalance = false;  // off at boot

int8_t controlPolarity = -1; // Polarity verified on assembled hardware
constexpr int8_t cartFeedbackSign = 1;

// --- TMC5160 SPI Registers ---
constexpr uint8_t REG_GCONF        = 0x00;
constexpr uint8_t REG_IOIN         = 0x04;
constexpr uint8_t REG_GLOBALSCALER = 0x0B;
constexpr uint8_t REG_IHOLD_IRUN   = 0x10;
constexpr uint8_t REG_TPOWERDOWN   = 0x11;
constexpr uint8_t REG_CHOPCONF     = 0x6C;
constexpr uint8_t WRITE_FLAG       = 0x80;

SPISettings tmcSpi(1000000, MSBFIRST, SPI_MODE3);
TMCStepDir motor(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_TMC_CS,
                 PIN_TMC_EN, PIN_TMC_STEP, PIN_TMC_DIR);

constexpr float VEL_UNIT_TO_SPS = 12000000.0f / 16777216.0f;
uint32_t spsToUnit(float sps) { return (uint32_t)(sps / VEL_UNIT_TO_SPS); }

// --- Encoder Quadrature Decoding ---
volatile int32_t encoderCount = 0;
volatile uint8_t previousAB = 0;
constexpr int8_t QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

void IRAM_ATTR updateEncoder() {
    const uint8_t currentAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    encoderCount += QUADRATURE_TABLE[(previousAB << 2) | currentAB];
    previousAB = currentAB;
}

int32_t readEncoderCount() {
    noInterrupts();
    const int32_t val = encoderCount;
    interrupts();
    return val;
}

void zeroEncoderDownward() {
    noInterrupts();
    encoderCount = 0;
    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    interrupts();
}

float wrapPi(float angle) {
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle <= -PI) angle += 2.0f * PI;
    return angle;
}

// Hanging downward is zero count; upright is half a revolution away
float uprightAngleFromCount(int32_t count) {
    return wrapPi(count * RAD_PER_COUNT - PI);
}

// --- Modes and State Variables ---
enum PipelineMode : uint8_t {
    PIPELINE_SAFE = 0,
    PIPELINE_SWING = 1,
    PIPELINE_BALANCE = 2,
    PIPELINE_MOTION_TEST = 3
};

enum SwingPhase : uint8_t {
    PHASE_PREPOSITION,
    PHASE_SETTLE,
    PHASE_PUMP,
    PHASE_COAST,
    PHASE_RISE
};

PipelineMode pipelineMode = PIPELINE_SAFE;
bool armed = false;
bool calibrated = false;

SwingPhase swingPhase = PHASE_PUMP;
float swingTargetVelocity = 0.0f;
float swingEnergyBias = 0.0f;
float previousSwingRate = 0.0f;
bool swingKickPending = false;
bool pushedThisPass = false;
bool apexHandled = false;
uint32_t swingStartMs = 0;
uint32_t phaseStartMs = 0;
uint32_t relaxedTripUntilMs = 0;
float balanceTripRad = TRIP_ANGLE_RAD;   // trip angle used while balancing (widened for a direct arm)
uint32_t catchMs = 0;

float angularRate = 0.0f;
float commandedVelocity = 0.0f;
float commandedAcceleration = 0.0f;
float cartPosition = 0.0f;
uint32_t lastControlUs = 0;

// --- SPI Driver Helpers ---
void writeRegister(uint8_t address, uint32_t value) {
    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address | WRITE_FLAG);
    SPI.transfer(static_cast<uint8_t>(value >> 24));
    SPI.transfer(static_cast<uint8_t>(value >> 16));
    SPI.transfer(static_cast<uint8_t>(value >> 8));
    SPI.transfer(static_cast<uint8_t>(value));
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();
}

uint32_t readRegister(uint8_t address) {
    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address & 0x7F);
    for (uint8_t i = 0; i < 4; ++i) SPI.transfer(0);
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();

    SPI.beginTransaction(tmcSpi);
    digitalWrite(PIN_TMC_CS, LOW);
    SPI.transfer(address & 0x7F);
    uint32_t value = static_cast<uint32_t>(SPI.transfer(0)) << 24;
    value |= static_cast<uint32_t>(SPI.transfer(0)) << 16;
    value |= static_cast<uint32_t>(SPI.transfer(0)) << 8;
    value |= static_cast<uint32_t>(SPI.transfer(0));
    digitalWrite(PIN_TMC_CS, HIGH);
    SPI.endTransaction();
    return value;
}

void configureDriver() {
    motor.init(0.4f, 1.0f, 4, 192); // 1/16 microsteps, spreadCycle
    writeRegister(REG_GCONF, 0x00000000);
    writeRegister(0x13, 0); // TPWMTHRS = 0
    motor.setSpeed(spsToUnit(max(gMaxCartSpeed, gSwingSpeed) * STEPS_PER_METRE));
    motor.setAcceleration(0xFFFF);
}

void setPipelineMode(PipelineMode next) {
    pipelineMode = next;
    armed = (next != PIPELINE_SAFE);
}

// Normalized mechanical energy: bottom still = -2, upright still = 0
float pendulumEnergy(float angle, float rate) {
    return 0.5f * rate * rate * gPendulumLengthM / GRAVITY + cos(angle) - 1.0f;
}

void disarm(const char *reason) {
    setPipelineMode(PIPELINE_SAFE);
    commandedVelocity = 0.0f;
    commandedAcceleration = 0.0f;
    motor.setVelocityDirect(0.0f);
    digitalWrite(PIN_TMC_EN, HIGH);
    Serial.print("DISARMED: ");
    Serial.println(reason);
}

void armBalanceController() {
    if (!calibrated) {
        Serial.println("ARM REFUSED: zero downward with Z first.");
        return;
    }
    const float angle = uprightAngleFromCount(readEncoderCount());
    if (fabsf(angle) > gArmWindowRad) {
        Serial.printf("ARM REFUSED: hold within %.0f deg (now %.2f deg).\n",
                      gArmWindowRad * 180.0f / PI, angle * 180.0f / PI);
        return;
    }
    motor.actualPosition(0);
    cartPosition = 0.0f;
    commandedVelocity = 0.0f;
    commandedAcceleration = 0.0f;
    angularRate = 0.0f;
    lastControlUs = micros();
    relaxedTripUntilMs = millis();
    // Armed from a tilted start: the trip angle must be wider than the arm window, otherwise the
    // very first control tick would disarm again.
    balanceTripRad = max(TRIP_ANGLE_RAD, gArmWindowRad + ARM_TRIP_MARGIN_RAD);
    catchMs = millis() - CATCH_STIFF_MS - CATCH_BLEND_MS;
    digitalWrite(PIN_TMC_EN, LOW);
    setPipelineMode(PIPELINE_BALANCE);
    Serial.println("ARMED: Riccati LQR Balance active.");
}

void startSwingUp() {
    if (!calibrated) {
        Serial.println("SWING REFUSED: zero downward with Z first.");
        return;
    }
    if (armed) {
        Serial.println("SWING REFUSED: stop first.");
        return;
    }
    const float psi = wrapPi(uprightAngleFromCount(readEncoderCount()) - PI);
    if (fabsf(psi) > SWING_START_ANGLE_RAD || fabsf(angularRate) > SWING_START_RATE) {
        Serial.printf("SWING REFUSED: let it hang still (now %.1f deg, %.2f rad/s).\n",
                      psi * 180.0f / PI, angularRate);
        return;
    }
    balanceTripRad = TRIP_ANGLE_RAD;   // the swing-up -> catch path keeps the normal trip angle
    motor.actualPosition(0);
    cartPosition = 0.0f;
    commandedVelocity = 0.0f;
    commandedAcceleration = 0.0f;
    swingTargetVelocity = 0.0f;
    swingEnergyBias = 0.0f;
    swingPhase = PHASE_PREPOSITION;
    swingKickPending = true;
    pushedThisPass = false;
    apexHandled = false;
    previousSwingRate = angularRate;
    lastControlUs = micros();
    swingStartMs = millis();
    phaseStartMs = swingStartMs;
    digitalWrite(PIN_TMC_EN, LOW);
    setPipelineMode(PIPELINE_SWING);
    Serial.printf("ARMED: Energy Swing-up started (L=%.3f m). Handover to LQR on catch.\n", gPendulumLengthM);
}

float swingStartX() {
    return -controlPolarity * SWING_DIRECTION * SWING_START_OFFSET_M;
}

float updatePreposition(float dt) {
    const float period = 2.0f * PI * sqrt(gPendulumLengthM / GRAVITY);
    const float t = (millis() - phaseStartMs) * 1.0e-3f;
    const float a = swingStartX() / (period * period);
    if (t < period) return a;
    if (t < 2.0f * period) return -a;
    commandedVelocity = 0.0f;
    swingPhase = PHASE_SETTLE;
    phaseStartMs = millis();
    return 0.0f;
}

// --- Energy Swing-up Step (returns acceleration [m/s^2]) ---
float updateSwingUp(float angle, float rate, float dt) {
    const float L = gPendulumLengthM;
    const float w2 = GRAVITY / L;
    const float psi = wrapPi(angle - PI);
    const float energy = pendulumEnergy(angle, rate);
    const float v = commandedVelocity;
    const bool towardBottom = (psi * rate < 0.0f);
    const bool upperHalf = (fabsf(angle) < 0.5f * PI);

    if (swingPhase == PHASE_PREPOSITION) return updatePreposition(dt);
    if (swingPhase == PHASE_SETTLE) {
        const bool still = (fabsf(psi) < SWING_START_ANGLE_RAD && fabsf(rate) < SWING_START_RATE);
        if (!still && (millis() - phaseStartMs < SWING_SETTLE_TIMEOUT_MS)) return 0.0f;
        swingPhase = PHASE_PUMP;
        swingStartMs = millis();
        previousSwingRate = rate;
        Serial.printf("KICK: from X=%+.3f m\n", cartPosition);
    }

    if (!upperHalf) apexHandled = false;
    if (upperHalf && !apexHandled && (previousSwingRate * rate < 0.0f) && (fabsf(angle) >= CATCH_WINDOW_RAD)) {
        apexHandled = true;
        swingEnergyBias += SWING_RETRY_GAIN * (1.0f - cos(angle));
        swingPhase = PHASE_PUMP;
        pushedThisPass = false;
    }
    if (swingPhase == PHASE_COAST && !upperHalf && (previousSwingRate * rate < 0.0f)) {
        swingPhase = PHASE_PUMP;
        pushedThisPass = false;
    }
    previousSwingRate = rate;

    // Catch condition -> Handover to Riccati LQR
    if (fabsf(angle) < CATCH_WINDOW_RAD) {
        setPipelineMode(PIPELINE_BALANCE);
        relaxedTripUntilMs = millis() + CATCH_SETTLE_MS;
        catchMs = millis();
        Serial.printf("CATCH: Handover to Riccati LQR (angle=%.1f deg, rate=%.2f rad/s, E=%+.2f)\n",
                      angle * 180.0f / PI, rate, energy);
        return 0.0f;
    }

    if (swingPhase == PHASE_RISE && !upperHalf && towardBottom) {
        swingPhase = PHASE_PUMP;
        pushedThisPass = false;
    }

    if (swingPhase != PHASE_RISE) {
        if (!towardBottom) pushedThisPass = false;
        if (swingPhase == PHASE_PUMP && (swingKickPending || !pushedThisPass)) {
            const float bottomRate = sqrt(max(0.0f, 2.0f * w2 * (energy + 2.0f)));
            const float targetRate = sqrt(2.0f * w2 * (gSwingEnergyMargin + swingEnergyBias + 2.0f));
            const float dvMagnitude = max(0.0f, targetRate - bottomRate) * L;
            const float direction = swingKickPending ? SWING_DIRECTION : (rate >= 0.0f ? 1.0f : -1.0f);
            const float desired = v + controlPolarity * direction * dvMagnitude;
            const float next = constrain(desired, -gSwingSpeed, gSwingSpeed);

            const float timeToBottom = fabsf(psi) / max(fabsf(rate), 1e-3f);
            const float leadTime = fabsf(next - v) / (2.0f * gSwingAccel) + dt;
            if (swingKickPending || (towardBottom && timeToBottom < leadTime)) {
                swingTargetVelocity = next;
                swingKickPending = false;
                pushedThisPass = true;
                if (fabsf(desired - next) < 1e-4f) {
                    swingPhase = PHASE_COAST;
                }
            }
        }

        // Horizontal brake
        const float brakeLead = fabsf(rate) * (fabsf(v) / gSwingAccel) * 0.5f;
        if (!towardBottom && fabsf(angle) < 0.5f * PI + brakeLead) {
            swingTargetVelocity = 0.0f;
            if (swingPhase == PHASE_COAST) swingPhase = PHASE_RISE;
        }
    }

    // Software rail limit protection
    const float stopX = cartPosition + v * fabsf(v) / (2.0f * gSwingAccel);
    if (fabsf(stopX) > gCartSoftLimit - 0.03f && swingTargetVelocity * stopX > 0.0f) {
        swingTargetVelocity = 0.0f;
    }

    return constrain((swingTargetVelocity - v) / dt, -gSwingAccel, gSwingAccel);
}

// Re-define the current cart position as 0 (step counter origin). The cart position is an
// open-loop step count, so after lost steps it no longer matches the real cart; the operator
// moves the cart to the centre by hand and calls this. It only moves an offset (atomic inside
// the TMC driver), keeps commandedVelocity untouched, and is safe in any mode.
void zeroCartPosition() {
    motor.actualPosition(0);
    cartPosition = 0.0f;
    Serial.println("OK: Cart position zeroed");
}

// --- Real-time Controller Loop (500 Hz) ---
void updatePipelineController() {
    const uint32_t now = micros();
    if (now - lastControlUs < CONTROL_PERIOD_US) return;
    const float dt = (now - lastControlUs) * 1.0e-6f;
    lastControlUs = now;

    static int32_t previousEncoderCount = 0;
    const int32_t count = readEncoderCount();
    const float angle = uprightAngleFromCount(count);
    const float rawRate = (count - previousEncoderCount) * RAD_PER_COUNT / dt;
    previousEncoderCount = count;
    angularRate += 0.25f * (rawRate - angularRate); // Low-pass filter

    cartPosition = motor.getSPIPosition() / STEPS_PER_METRE;

    if (!armed) return;

    // Hard rail limit check
    if (fabsf(cartPosition) > gCartSoftLimit) {
        disarm("Cart exceeded soft limit");
        return;
    }

    // Mode: Swing-up
    if (pipelineMode == PIPELINE_SWING) {
        if (millis() - swingStartMs > SWING_TIMEOUT_MS) {
            disarm("Swing-up timed out");
            return;
        }
        commandedAcceleration = updateSwingUp(angle, angularRate, dt);
        if (pipelineMode == PIPELINE_SWING) {
            commandedVelocity += commandedAcceleration * dt;
            commandedVelocity = constrain(commandedVelocity, -gSwingSpeed, gSwingSpeed);
            return;
        }
    }

    // Mode: Riccati LQR Balance
    const float tripAngle = (millis() < relaxedTripUntilMs) ? CATCH_TRIP_RAD : balanceTripRad;
    if (fabsf(angle) > tripAngle) {
        disarm("Pendulum exceeded trip angle");
        return;
    }

    const uint32_t sinceCatchMs = millis() - catchMs;
    const float blend = sinceCatchMs < CATCH_STIFF_MS
        ? 0.0f
        : min(1.0f, (sinceCatchMs - CATCH_STIFF_MS) / static_cast<float>(CATCH_BLEND_MS));
    const float kCartPos = CATCH_KCARTPOS + (gKcartPos - CATCH_KCARTPOS) * blend;
    const float kCartVel = CATCH_KCARTVEL + (gKcartVel - CATCH_KCARTVEL) * blend;

    // Right after a catch the pendulum still swings a lot, so even in soft mode the stiff
    // pendulum gains are used and then blended to the soft ones with the same blend as the
    // cart gains. Arming with 'A' gives blend = 1.
    const float kAngle = softBalance ? gKangle + (gKangleSoft - gKangle) * blend : gKangle;
    const float kRate  = softBalance ? gKrate  + (gKrateSoft  - gKrate)  * blend : gKrate;

    // Riccati acceleration control law
    float u = controlPolarity * (kAngle * (angle - gAngleTrim) + kRate * angularRate) +
              cartFeedbackSign * (kCartPos * cartPosition + kCartVel * commandedVelocity);

    // Apply Acceleration Limit
    commandedAcceleration = constrain(u, -gMaxCartAccel, gMaxCartAccel);

    // Integrate Acceleration into Commanded Velocity
    commandedVelocity += commandedAcceleration * dt;

    // Apply Speed Limit
    commandedVelocity = constrain(commandedVelocity, -gMaxCartSpeed, gMaxCartSpeed);
}

void serviceStepper() {
    if (!armed) return;
    motor.setVelocityDirect(commandedVelocity * STEPS_PER_METRE);
}

// --- Serial Telemetry (0xAA Packet matching PC Panel) ---
void transmitTelemetry() {
    static uint32_t lastTxMs = 0;
    const uint32_t now = millis();
    if (now - lastTxMs < TELEMETRY_PERIOD_MS) return;
    lastTxMs = now;

    const float angle = uprightAngleFromCount(readEncoderCount());
    const float energy = pendulumEnergy(angle, angularRate);

    auto sendFloatPacket = [](uint8_t addr, float val) {
        uint8_t pkt[6];
        pkt[0] = 0xAA;
        pkt[1] = addr;
        memcpy(&pkt[2], &val, sizeof(float));
        Serial.write(pkt, sizeof(pkt));
    };

    sendFloatPacket(0x00, angle);                 // Angle (rad)
    sendFloatPacket(0x01, angularRate);           // Angular rate (rad/s)
    sendFloatPacket(0x02, cartPosition);          // Position (m)
    sendFloatPacket(0x03, commandedVelocity);     // Velocity (m/s)
    sendFloatPacket(0x04, energy);                // Energy E
    sendFloatPacket(0x05, commandedAcceleration); // Acceleration (m/s^2)
    sendFloatPacket(0x06, static_cast<float>(pipelineMode)); // Mode (0:safe, 1:swing, 2:balance)
}

// --- Packet & Command Receiver ---
constexpr uint8_t PKT_HEADER = 0xAA;
constexpr uint32_t PKT_PARTIAL_TIMEOUT_MS = 20;

void printGains() {
    Serial.printf("GAINS %s angle=%.2f rate=%.2f cartPos=%.2f cartVel=%.2f trim=%+.2fdeg pol=%d L=%.3fm\n",
                  softBalance ? "SOFT" : "STIFF",
                  softBalance ? gKangleSoft : gKangle,
                  softBalance ? gKrateSoft : gKrate,
                  gKcartPos, gKcartVel,
                  gAngleTrim * 180.0f / PI, controlPolarity, gPendulumLengthM);
}

// One complete PC -> ESP32 frame: [0xAA][addr][payload x4]. For floats the payload is a
// little-endian float32; for the mode command (0x50) payload[0] is the mode byte.
void applyFrame(const uint8_t* f) {
    const uint8_t addr = f[1];
    float val;
    memcpy(&val, &f[2], sizeof(float));

    switch (addr) {
        case 0x01: gMaxCartSpeed = val; break;
        case 0x02: gMaxCartAccel = val; break;
        case 0x07: gCartSoftLimit = val; break;
        case 0x0B: gPendulumLengthM = val; break;
        case 0x0C: gSwingEnergyMargin = val; break;
        case 0x20: gKangle = val; break;     // LQR K_theta
        case 0x21: gKrate = val; break;      // LQR K_theta_dot
        case 0x22: gKcartPos = val; break;   // LQR K_x
        case 0x23: gKcartVel = val; break;   // LQR K_x_dot
        case 0x24: gKangleSoft = val; break; // soft-balance K_theta
        case 0x25: gKrateSoft = val; break;  // soft-balance K_theta_dot
        case 0x26: softBalance = (val != 0.0f); break; // soft balance on/off (same as 'D')
        case 0x27: gArmWindowRad = constrain(val, 1.0f, 90.0f) * PI / 180.0f; break; // direct-balance arm window [deg]
        case 0x50: {
            const uint8_t m = f[2];
            if (m == 0) disarm("Panel Standby");
            else if (m == 1) startSwingUp();
            else if (m == 2) armBalanceController();
            break;
        }
        case 0x52:
            if (armed) disarm("Zero requested");
            zeroEncoderDownward();
            calibrated = true;
            break;
        case 0x53:
            zeroCartPosition(); // does NOT disarm and does NOT touch the encoder
            break;
        default:
            break;
    }
}

void handleSerial() {
    static uint32_t partialSinceMs = 0;
    while (Serial.available() > 0) {
        const int b = Serial.peek();
        if (b == PKT_HEADER) {
            if (Serial.available() < 6) {
                // Wait for the rest of the frame, but never block ASCII commands forever
                // behind a stray 0xAA.
                if (partialSinceMs == 0) {
                    partialSinceMs = millis();
                } else if (millis() - partialSinceMs > PKT_PARTIAL_TIMEOUT_MS) {
                    Serial.read();
                    partialSinceMs = 0;
                    continue;
                }
                break;
            }
            partialSinceMs = 0;
            uint8_t frame[6];
            Serial.readBytes(frame, sizeof(frame));
            applyFrame(frame);
        } else if (b == 'Z' || b == 'z') {
            Serial.read();
            if (armed) disarm("Zero requested");
            zeroEncoderDownward();
            calibrated = true;
            Serial.println("OK: Downward Zero Set");
        } else if (b == 'A' || b == 'a') {
            Serial.read();
            armBalanceController();
        } else if (b == 'S' || b == 's') {
            Serial.read();
            startSwingUp();
        } else if (b == 'X' || b == 'x' || b == ' ') {
            Serial.read();
            if (armed) disarm("User stopped");
        } else if (b == 'P' || b == 'p') {
            Serial.read();
            if (!armed) {
                controlPolarity = -controlPolarity;
                Serial.printf("OK: Control polarity = %d\n", controlPolarity);
            }
        } else if (b == 'O' || b == 'o') {
            Serial.read();
            zeroCartPosition();
        } else if (b == 'D' || b == 'd') {
            // Works while balancing too: only the pendulum-loop gains change, so the cart
            // does not jump.
            Serial.read();
            softBalance = !softBalance;
            printGains();
        } else {
            Serial.read(); // unknown byte: discard so the stream can resynchronise
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    pinMode(PIN_TMC_EN, OUTPUT);
    pinMode(PIN_TMC_STEP, OUTPUT);
    pinMode(PIN_TMC_DIR, OUTPUT);
    pinMode(PIN_TMC_CS, OUTPUT);
    digitalWrite(PIN_TMC_EN, HIGH);
    digitalWrite(PIN_TMC_STEP, LOW);
    digitalWrite(PIN_TMC_DIR, LOW);
    digitalWrite(PIN_TMC_CS, HIGH);

    previousAB =
        (static_cast<uint8_t>(digitalRead(PIN_ENCODER_A)) << 1) |
        static_cast<uint8_t>(digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TMC_CS);
    delay(300);

    const uint32_t ioin = readRegister(REG_IOIN);
    const uint8_t version = static_cast<uint8_t>(ioin >> 24);
    if (version == 0x30) {
        configureDriver();
    }
    digitalWrite(PIN_TMC_EN, HIGH);

    Serial.println("\n=== PIPELINE: ENERGY SWING-UP + RICCATI LQR + ACCEL CONTROL READY ===");
}

void loop() {
    handleSerial();
    updatePipelineController();
    serviceStepper();
    transmitTelemetry();
}
