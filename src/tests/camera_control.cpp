#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <MyData.h>
#include "camera_angle_calib.h"

// ============================================================================
// CAMERA CONTROL (step 5 of the camera pipeline): copy of camera_pipeline.cpp where the BALANCE loop can use the camera
// estimate (angle and rate of the fixed-lag EKF below) instead of the encoder. The encoder stays connected as a
// watchdog. Key 'C' (only while disarmed) toggles the source; the default is ENCODER, so the firmware behaves like
// camera_pipeline until you ask for the camera. Camera control is for 'A' (balance armed by hand) only: swing-up
// stays on the encoder ('S' is refused in camera mode, the markers blur in a fast swing).
// Guards in camera mode (all of them disarm and stop the cart):
//   - no valid camera packet for CAM_TIMEOUT_MS,
//   - |camera estimate - encoder| > CAM_MISMATCH_RAD for CAM_MISMATCH_TICKS control ticks in a row,
//   - the usual rail limit and trip angle (now on the camera angle).
// 'A' is refused unless the camera data is fresh and within CAM_ARM_AGREE_RAD of the encoder.
// ============================================================================

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
// COUNTED metres (STEP pulses / 80000; the real travel is 1.5x). The real rail allows +-0.35 m, i.e. +-0.23 counted m,
// so the default here is 0.20 (about 0.30 m real) instead of the original 0.35. The GUI can still change it (0x07).
float gCartSoftLimit = 0.20f;
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
// --- camera control source (the estimator itself lives in the shadow block below) ---
constexpr uint32_t CAM_TIMEOUT_MS = 60;                      // ~5 camera frames at 90 fps
constexpr float CAM_MISMATCH_RAD = 8.0f * PI / 180.0f;       // camera vs encoder, balancing errors are < 1.5 deg
constexpr int CAM_MISMATCH_TICKS = 10;                       // 20 ms
constexpr float CAM_ARM_AGREE_RAD = 3.0f * PI / 180.0f;
static int camMismatchTicks = 0;
// The camera angle depends on where the cart is (perspective): the apparent rod angle is off by a few degrees per metre
// of cart travel. Uncorrected, that bias acts like extra position feedback (Kangle 50 x c vs KcartPos 3) that cancels
// most of the position stiffness; camctl_n2 (no correction) drifted 0.2 m in 15 s. The balance angle is corrected:
//     angle = camera + camXSlopeDegPerM * cartPosition       (cartPosition: counted m, STEP count increasing = +)
// SIGN: in camctl_n3 the cart ran away to +0.200 m within 2-3 s for every slope from +4.5 to +9.0 deg/m, i.e. a
// POSITIVE slope makes it worse; the EKF - encoder bias is positive when the cart is at + (about +1.5..+3 deg/m), so the
// correction must be NEGATIVE. (The hanging-pendulum log cart_n1 gave -5.8 deg/m with the opposite sign: do not trust it.)
// Default -3.0 cancels the bias estimate; keys '.' / ',' change it by +-0.5 (stopped only).
float camXSlopeDegPerM = -3.0f;
bool cameraControl = false;                                  // balance source: false = encoder, true = camera
static bool camEstimate(float& phi, float& w, uint32_t& ageMs);   // false until the estimator has data

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
    if (cameraControl) {
        float camPhi = 0.0f, camW = 0.0f;
        uint32_t ageMs = 0;
        if (!camEstimate(camPhi, camW, ageMs) || ageMs > CAM_TIMEOUT_MS) {
            Serial.println("ARM REFUSED: no fresh camera data (is pend_cam running? markers visible?).");
            return;
        }
        if (fabsf(wrapPi(camPhi - angle)) > CAM_ARM_AGREE_RAD) {
            Serial.printf("ARM REFUSED: camera %.2f deg and encoder %.2f deg disagree.\n",
                          camPhi * 180.0f / PI, angle * 180.0f / PI);
            return;
        }
    }
    motor.actualPosition(0);
    cartPosition = 0.0f;
    commandedVelocity = 0.0f;
    commandedAcceleration = 0.0f;
    angularRate = 0.0f;
    lastControlUs = micros();
    camMismatchTicks = 0;
    relaxedTripUntilMs = millis();
    // Armed from a tilted start: the trip angle must be wider than the arm window, otherwise the
    // very first control tick would disarm again.
    balanceTripRad = max(TRIP_ANGLE_RAD, gArmWindowRad + ARM_TRIP_MARGIN_RAD);
    catchMs = millis() - CATCH_STIFF_MS - CATCH_BLEND_MS;
    digitalWrite(PIN_TMC_EN, LOW);
    setPipelineMode(PIPELINE_BALANCE);
    Serial.printf("ARMED: Riccati LQR Balance active. source=%s\n", cameraControl ? "CAMERA" : "ENCODER");
}

void startSwingUp() {
    if (cameraControl) {
        Serial.println("SWING REFUSED: camera control is for balance only (press C for the encoder).");
        return;
    }
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
    float angle = uprightAngleFromCount(count);
    const float encoderAngle = angle;
    const float rawRate = (count - previousEncoderCount) * RAD_PER_COUNT / dt;
    previousEncoderCount = count;
    angularRate += 0.25f * (rawRate - angularRate); // Low-pass filter

    cartPosition = motor.getSPIPosition() / STEPS_PER_METRE;

    if (!armed) return;

    // Hard rail limit check
    if (fabsf(cartPosition) > gCartSoftLimit) {
        Serial.printf("# cart position at the limit: %+.3f m (counted)\n", cartPosition);
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
    float rate = angularRate;
    if (cameraControl) {
        float camPhi = 0.0f, camW = 0.0f;
        uint32_t ageMs = 0;
        if (!camEstimate(camPhi, camW, ageMs) || ageMs > CAM_TIMEOUT_MS) {
            disarm("Camera data lost");
            return;
        }
        if (fabsf(wrapPi(camPhi + camXSlopeDegPerM * (PI / 180.0f) * cartPosition - encoderAngle)) > CAM_MISMATCH_RAD) {
            if (++camMismatchTicks >= CAM_MISMATCH_TICKS) {
                disarm("Camera and encoder disagree");
                return;
            }
        } else {
            camMismatchTicks = 0;
        }
        angle = wrapPi(camPhi + camXSlopeDegPerM * (PI / 180.0f) * cartPosition);
        rate = camW;
    }
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
    float u = controlPolarity * (kAngle * (angle - gAngleTrim) + kRate * rate) +
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

// ============================================================================
// SHADOW camera angle estimator (added to a copy of pipeline_energy_lqr.cpp)
// ----------------------------------------------------------------------------
// Reads the AI-G camera packets on Serial2 (460800, GPIO32 RX / GPIO33 TX) and runs the EKF with fixed-lag
// delay compensation (same model/parameters as camera_estimator_test.cpp). Its output is ONLY compared with
// the encoder: nothing here feeds the controller, the motor or the telemetry frames.
// Key 'E' toggles a '#' status line once per second (off by default so the GUI stream is untouched; it is
// skipped when the serial TX buffer is too full, so it can never block the control loop).
//   # SH m<mode> all <ekf>/<raw> b<bias> deg w<rate> | up <ekf>/<raw> b<bias> w<rate> max<ekf> n<ticks> |
//        ctr <ekf> b<bias> n<ticks> | <avg>/<max>us v<valid pkts>
//   all = the whole second, up = only the time the pendulum is within 30 deg of upright, ctr = within 5 deg
//   (the range balancing really uses). ekf/raw = angle rms error of the estimator / the raw camera angle against
//   the encoder [deg], b = MEAN signed error estimator - encoder [deg] (a constant bias, e.g. a wrong upright
//   reference, shows up here but not in the rms of a noisy signal), w = estimator rate rms error [rad/s].
// ============================================================================
constexpr uint32_t SH_UART_BAUD = 460800;
constexpr int SH_PIN_RX = 32;
constexpr int SH_PIN_TX = 33;
constexpr size_t SH_PKT_LEN = 14;
constexpr uint32_t SH_TICK_US = 2000;
constexpr float SH_DT = 0.002f;
constexpr float SH_K = 64.6f;                 // 1/s^2
constexpr float SH_C = 0.05f;                 // 1/s
constexpr float SH_B_MAG = 9.4f;              // 1/m, cart acceleration in the firmware's counted units
// Sign of B: +9.44 in the 2026-11-22 rerun (the earlier -9.37 had the wrong sign: the shadow rate error halved
// with +, see CLAUDE.md section 5). Key 'B' still flips it at run time (shadow estimator only, no effect on control).
static float SH_B = SH_B_MAG;
constexpr float SH_Q_RATE = 200.0f;
constexpr float SH_R_ANGLE = 0.017453f * 0.017453f;   // (1 deg)^2
constexpr float SH_CAM_SIGN = -1.0f;          // camera angle sign relative to the encoder
// (the camera zero offset is part of the calibration table in camera_angle_calib.h)
constexpr float SH_PRE_DRIVER_MS = 6.3f;
constexpr int SH_DEFAULT_LAG = 7;
constexpr int SH_MAX_LAG = 14;
constexpr int SH_HIST = 24;
constexpr float SH_UPRIGHT_RAD = 30.0f * 0.017453293f;

struct ShEst { float phi, w, p00, p01, p11; };
struct ShNode { ShEst e; float a; };
static ShNode shHist[SH_HIST];
static int shIdx = 0;
static bool shInit = false;

static ShEst shPredict(const ShEst& e, float a) {
    const float s = sinf(e.phi), c = cosf(e.phi);
    ShEst n;
    n.phi = e.phi + SH_DT * e.w;
    n.w = e.w + SH_DT * (SH_K * s - SH_C * e.w + SH_B * c * a);
    const float f10 = SH_DT * (SH_K * c - SH_B * s * a);
    const float f11 = 1.0f - SH_C * SH_DT;
    const float a00 = e.p00 + SH_DT * e.p01, a01 = e.p01 + SH_DT * e.p11;
    const float a10 = f10 * e.p00 + f11 * e.p01, a11 = f10 * e.p01 + f11 * e.p11;
    n.p00 = a00 + a01 * SH_DT;
    n.p01 = 0.5f * ((a00 * f10 + a01 * f11) + (a10 + a11 * SH_DT));
    n.p11 = a10 * f10 + a11 * f11 + SH_Q_RATE * SH_DT;
    return n;
}

static ShEst shUpdate(const ShEst& e, float z) {
    const float y = wrapPi(z - e.phi);
    const float S = e.p00 + SH_R_ANGLE;
    const float k0 = e.p00 / S, k1 = e.p01 / S;
    ShEst n;
    n.phi = wrapPi(e.phi + k0 * y);
    n.w = e.w + k1 * y;
    n.p00 = e.p00 - k0 * e.p00;
    n.p01 = e.p01 - k0 * e.p01;
    n.p11 = e.p11 - k1 * e.p01;
    return n;
}

// Outlier gate: a camera packet whose angle differs from the estimate at its own frame time by more than
// SH_GATE_RAD is rejected (a half-covered marker or a wrong marker gives a wrong angle; with R = (1 deg)^2 the filter
// would follow it). After SH_RESYNC_REJECTS rejected packets in a row the filter re-syncs on the camera instead.
// Balancing innovations are < 1.5 deg. The time of the last ACCEPTED packet drives the camera-lost timeout.
constexpr float SH_GATE_RAD = 6.0f * 0.017453293f;
constexpr int SH_RESYNC_REJECTS = 10;                  // ~0.11 s of packets at 90 fps
static int shRejectRun = 0;
static uint32_t shRejects = 0;                         // rejected packets in the current second
static uint32_t shLastValidUs = 0;                     // last accepted camera packet

static void shEstimatorTick(float a, bool hasZ, float z, int lag) {
    if (hasZ && !shInit) {
        const ShEst e0 = {z, 0.0f, 1.0f, 0.0f, 100.0f};
        for (int i = 0; i < SH_HIST; i++) { shHist[i].e = e0; shHist[i].a = 0.0f; }
        shInit = true;
        shLastValidUs = micros();
    }
    if (!shInit) return;
    const int prev = shIdx;
    shIdx = (shIdx + 1) % SH_HIST;
    shHist[shIdx].e = shPredict(shHist[prev].e, a);
    shHist[shIdx].a = a;
    if (!hasZ) return;
    if (lag < 0) lag = 0;
    if (lag > SH_MAX_LAG) lag = SH_MAX_LAG;
    int k = (shIdx - lag + SH_HIST) % SH_HIST;
    if (fabsf(wrapPi(z - shHist[k].e.phi)) > SH_GATE_RAD) {
        shRejects++;
        if (++shRejectRun < SH_RESYNC_REJECTS) return;         // ignore this packet
        shHist[k].e = {z, 0.0f, 1.0f, 0.0f, 100.0f};           // the estimate is lost: restart from the camera
    } else {
        shHist[k].e = shUpdate(shHist[k].e, z);
    }
    shRejectRun = 0;
    shLastValidUs = micros();
    while (k != shIdx) {
        const int nxt = (k + 1) % SH_HIST;
        shHist[nxt].e = shPredict(shHist[k].e, shHist[nxt].a);
        k = nxt;
    }
}

static bool camEstimate(float& phi, float& w, uint32_t& ageMs) {
    if (!shInit) return false;
    phi = shHist[shIdx].e.phi;
    w = shHist[shIdx].e.w;
    ageMs = (uint32_t)(micros() - shLastValidUs) / 1000u;
    return true;
}

static uint8_t shCrc8(const uint8_t* d, size_t n) {
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

static uint8_t shBuf[SH_PKT_LEN];
static size_t shBufLen = 0;
static uint32_t shPktOk, shPktValid;
static bool shPendingValid = false;
static float shPendingZ = 0.0f;
static int shPendingLag = SH_DEFAULT_LAG;
static float shZHold = 0.0f;
static bool shZHoldValid = false;
static bool shStatsOn = false;

static void shDropFirstByte() {
    memmove(shBuf, shBuf + 1, shBufLen - 1);
    shBufLen--;
}

static void shHandlePacket() {
    shPktOk++;
    const int16_t angle = (int16_t)(shBuf[4] | (shBuf[5] << 8));
    const uint8_t flags = shBuf[10];
    const uint16_t age = shBuf[11] | (shBuf[12] << 8);
    if (!(flags & 0x01) || angle == (int16_t)0x8000) return;
    shPktValid++;
    const float camRad = angle * 0.01f * 0.017453293f;
    shPendingZ = camCalibrate(wrapPi(SH_CAM_SIGN * camRad));
    shZHold = shPendingZ;
    shZHoldValid = true;
    shPendingValid = true;
    shPendingLag = (flags & 0x20) ? (int)lroundf((age * 0.1f + SH_PRE_DRIVER_MS) / (SH_DT * 1000.0f))
                                  : SH_DEFAULT_LAG;
}

static void shPollRx() {
    while (Serial2.available()) {
        const uint8_t b = (uint8_t)Serial2.read();
        shBuf[shBufLen++] = b;
        while (shBufLen >= 1 && shBuf[0] != 0xAA) shDropFirstByte();
        if (shBufLen >= 2 && shBuf[1] != 0x55) { shDropFirstByte(); continue; }
        if (shBufLen == SH_PKT_LEN) {
            if (shCrc8(shBuf + 2, 11) == shBuf[13]) {
                shHandlePacket();
                shBufLen = 0;
            } else {
                shDropFirstByte();
                while (shBufLen >= 1 && shBuf[0] != 0xAA) shDropFirstByte();
                if (shBufLen >= 2 && shBuf[1] != 0x55) shDropFirstByte();
            }
        }
    }
}

// error statistics against the encoder at the current time
struct ShStat { double e2 = 0, esum = 0, raw2 = 0, w2 = 0; float emax = 0; uint32_t n = 0, nRaw = 0, nW = 0; };
static ShStat shAll, shUp, shCtr;
constexpr float SH_CENTER_RAD = 5.0f * 0.017453293f;
static uint32_t shTickSumUs, shTickMaxUs, shTicks;
static int32_t shCntHist[11];
static float shWHist[11];
static int shCntIdx = 0;
static float shPrevCmdVel = 0.0f;

static void shTick() {
    static uint32_t lastUs = micros();
    const uint32_t now = micros();
    if ((uint32_t)(now - lastUs) < SH_TICK_US) return;
    lastUs += SH_TICK_US;
    const uint32_t t0 = micros();

    // cart acceleration actually applied (counted m/s^2, firmware sign): change of the commanded velocity
    const float accel = (commandedVelocity - shPrevCmdVel) / SH_DT;
    shPrevCmdVel = commandedVelocity;
    const bool hasZ = shPendingValid;
    const float z = shPendingZ;
    const int lag = shPendingLag;
    shPendingValid = false;
    shEstimatorTick(accel, hasZ, z, lag);

    const uint32_t dtUs = micros() - t0;
    shTickSumUs += dtUs;
    if (dtUs > shTickMaxUs) shTickMaxUs = dtUs;
    shTicks++;

    const int32_t count = readEncoderCount();
    shCntIdx = (shCntIdx + 1) % 11;
    shCntHist[shCntIdx] = count;
    shWHist[shCntIdx] = shInit ? shHist[shIdx].e.w : 0.0f;
    if (!shInit || !calibrated) return;

    const float phiEnc = uprightAngleFromCount(count);
    const float e = wrapPi(shHist[shIdx].e.phi - phiEnc);
    const int old = (shCntIdx + 1) % 11;
    const float wTrue = (count - shCntHist[old]) * RAD_PER_COUNT / (10.0f * SH_DT);
    const float ew = shWHist[(shCntIdx + 11 - 5) % 11] - wTrue;
    ShStat* buckets[3] = {&shAll, fabsf(phiEnc) < SH_UPRIGHT_RAD ? &shUp : nullptr,
                          fabsf(phiEnc) < SH_CENTER_RAD ? &shCtr : nullptr};
    for (ShStat* s : buckets) {
        if (!s) continue;
        s->e2 += (double)e * e;
        s->esum += e;
        if (fabsf(e) > s->emax) s->emax = fabsf(e);
        s->n++;
        if (shZHoldValid) {
            const float er = wrapPi(shZHold - phiEnc);
            s->raw2 += (double)er * er;
            s->nRaw++;
        }
        s->w2 += (double)ew * ew;
        s->nW++;
    }
}

static void shReport() {
    static uint32_t lastUs = micros();
    static bool pending = false;       // the line is built but the TX FIFO had no room yet: retry, do not drop it
    static char line[192];
    static int len = 0;
    const uint32_t now = micros();
    if (!pending && (uint32_t)(now - lastUs) < 1000000) return;
    if (shStatsOn) {
        if (!pending) {
            const float d = 57.29578f;
            auto rms = [](double s, uint32_t n) { return n ? sqrtf((float)(s / n)) : 0.0f; };
            auto mean = [](double s, uint32_t n) { return n ? (float)(s / n) : 0.0f; };
            len = snprintf(line, sizeof(line),
                "# SH m%d all %.2f/%.2f b%+.2f w%.2f | up %.2f/%.2f b%+.2f w%.2f max%.1f n%lu | ctr %.2f b%+.2f n%lu | %lu/%luus v%lu r%lu",
                (int)pipelineMode, rms(shAll.e2, shAll.n) * d, rms(shAll.raw2, shAll.nRaw) * d, mean(shAll.esum, shAll.n) * d,
                rms(shAll.w2, shAll.nW), rms(shUp.e2, shUp.n) * d, rms(shUp.raw2, shUp.nRaw) * d, mean(shUp.esum, shUp.n) * d,
                rms(shUp.w2, shUp.nW), shUp.emax * d, (unsigned long)shUp.n, rms(shCtr.e2, shCtr.n) * d,
                mean(shCtr.esum, shCtr.n) * d, (unsigned long)shCtr.n,
                shTicks ? (unsigned long)(shTickSumUs / shTicks) : 0UL, (unsigned long)shTickMaxUs, (unsigned long)shPktValid, (unsigned long)shRejects);
            // snprintf reports the length it WOULD have written: never read past the buffer
            if (len > 126) len = 126;    // longer than the 128-byte FIFO could ever take: cut it (keeps room for the newline)
            // cart position (counted m) at the end of the line if it still fits the 128-byte FIFO, then the newline
            const int xn = snprintf(line + len, sizeof(line) - len, " x%+.2f", (double)cartPosition);
            if (xn > 0 && len + xn <= 126) len += xn;
            line[len++] = '\n';
        }
        // never block the control loop: print only if the whole line fits in the TX FIFO. The 50 Hz binary telemetry
        // can occupy the FIFO at the moment the line is due (the phase between the two drifts), so wait a few
        // milliseconds for room instead of dropping the line.
        if (len > 0 && Serial.availableForWrite() < len) {
            if (len <= 127) {            // fits the 128-byte FIFO once the telemetry has drained: retry (length <= 127)
                pending = true;
                return;
            }
            len = 0;                     // can never fit: drop it like before
        }
        if (len > 0) Serial.write(reinterpret_cast<const uint8_t*>(line), (size_t)len);
    }
    pending = false;
    lastUs += 1000000;
    shAll = ShStat();
    shUp = ShStat();
    shCtr = ShStat();
    shTickSumUs = shTickMaxUs = shTicks = 0;
    shPktOk = shPktValid = 0;
    shRejects = 0;
}

static void shBegin() {
    Serial2.setRxBufferSize(1024);
    Serial2.begin(SH_UART_BAUD, SERIAL_8N1, SH_PIN_RX, SH_PIN_TX);
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
        } else if (b == 'E' || b == 'e') {
            // shadow camera estimator status line on/off (does not touch the controller)
            Serial.read();
            shStatsOn = !shStatsOn;
            Serial.println(shStatsOn ? "OK: shadow stats ON" : "OK: shadow stats OFF");
        } else if (b == 'C' || b == 'c') {
            Serial.read();
            if (armed) {
                Serial.println("C REFUSED: stop first (X).");
            } else {
                cameraControl = !cameraControl;
                Serial.printf("OK: balance source = %s\n", cameraControl ? "CAMERA" : "ENCODER");
            }
        } else if (b == '.' || b == ',') {
            Serial.read();
            if (armed) {
                Serial.println("slope change REFUSED: stop first (X).");
            } else {
                camXSlopeDegPerM += (b == '.') ? 0.5f : -0.5f;
                Serial.printf("OK: camera x slope = %.1f deg/m\n", (double)camXSlopeDegPerM);
            }
        } else if (b == 'B' || b == 'b') {
            // flip the sign of the shadow estimator's cart-acceleration gain B (does not touch the controller)
            Serial.read();
            SH_B = -SH_B;
            Serial.printf("OK: shadow B = %+.1f\n", (double)SH_B);
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

    shBegin();
    Serial.println("\n=== CAMERA CONTROL (balance source: C = camera / encoder, default encoder): ENERGY SWING-UP + RICCATI LQR + ACCEL CONTROL READY ===");
}

void loop() {
    handleSerial();
    shPollRx();                 // the camera estimate must be current when the controller reads it
    shTick();
    updatePipelineController();
    serviceStepper();
    transmitTelemetry();
    shReport();
}
