// #include <Arduino.h>
#include "Pendulum.h"

namespace
{
// Uniform rod plus a small attachment near its end. Replace the ratio with
// measured attachment_mass / bare_rod_mass. Zero gives a pure uniform rod.
constexpr float PENDULUM_LENGTH_M = 0.305f;
constexpr float TIP_TO_ROD_MASS_RATIO = 0.10f;
constexpr float TIP_DISTANCE_RATIO = 0.95f;

// Start bench tests at 0.5 m/s^2 and raise this gradually after direction and
// rail-limit checks. 2.0 m/s^2 is the intended initial tuning ceiling.
constexpr float SWING_ACCEL_MAX_MPS2 = 0.5f;
constexpr float CART_CENTER_KP = 4.0f;
constexpr float CART_CENTER_KD = 2.5f;
constexpr float FALLBACK_RAIL_LIMIT_M = 0.30f;
constexpr float SOFT_LIMIT_RATIO = 0.80f;

constexpr float CATCH_ANGLE_RAD = 12.0f * PI / 180.0f;
constexpr float CATCH_ANGULAR_RATE_RAD_S = 1.5f;
constexpr float CATCH_CART_SPEED_MPS = 0.30f;
constexpr float RELEASE_ANGLE_RAD = 25.0f * PI / 180.0f;

// Flip to -1 if a positive acceleration command makes measured cart position
// decrease, or if repeated swings lose rather than gain energy.
constexpr float SWING_DIRECTION_SIGN = 1.0f;
}

Pendulum::Pendulum(uint32_t encoderResolution_ = 10000, uint16_t motorMicrosteps_ = 200, uint32_t motorSteps_ = 128, float pulleyCircumference_ = 0.04f)
    : encoderSpd(50), motorSpd(50)
{
    encoderResolution = encoderResolution_;
    motorMicrosteps = motorMicrosteps_;
    motorSteps = motorSteps_;
    pulleyCircumference = pulleyCircumference_;
    updateRatios();
}
void Pendulum::updateRatios()
{
    angleRatio = (2.0f * PI) / static_cast<float>(encoderResolution);                                             // Radians per encoder step
    distanceRatio = pulleyCircumference / (static_cast<float>(motorSteps) * static_cast<float>(motorMicrosteps)); // Meters per motor step
    accelerationRatio = 0.015f / distanceRatio;                                                                   // Calculate acceleration unit for the motor
    speedRatio = 1.37f / distanceRatio;                                                                           // Calculate speed unit for the motor
}
void Pendulum::updateAngle(Encoder &encoder)
{
    long currentPosition = encoder.getPosition() - (rev * encoderResolution);
    if (currentPosition != currentPosition % int(encoderResolution))
    {
        rev += (currentPosition > 0) ? 1 : -1;
    }
    angle = float(currentPosition) * angleRatio;
    angularVelocity = float(encoderSpd.update(currentPosition)) * angleRatio;
}
void Pendulum::updatePosition(TMC &tmc)
{
    long currentPosition = tmc.getSPIPosition();
    position = float(currentPosition) * distanceRatio;
    velocity = float(motorSpd.update(currentPosition)) * distanceRatio;
}
void Pendulum::setMagnitude(float d0_) { d0 = d0_; }
void Pendulum::setThreshold(float threshold_) { threshold = threshold_; }
void Pendulum::setBalancePos(float balancePos_) { balancePos = balancePos_; }
void Pendulum::setLimit(float limit_) { limit = limit_; }
void Pendulum::setLoopImpulse1(float loopImpulse1_) { loopImpulse1 = loopImpulse1_; }
void Pendulum::setLoopImpulse2(float loopImpulse2_) { loopImpulse2 = loopImpulse2_; }
void Pendulum::setLoopAngle(float loopAngle_) { loopAngle = loopAngle_; }
void Pendulum::setSpeed(TMC &tmc, float speed_)
{
    speed = speed_;
    tmc.setSpeed(speed * speedRatio);
}
void Pendulum::setAcceleration(TMC &tmc, float acceleration_)
{
    acceleration = acceleration_;
    tmc.setAcceleration(acceleration * accelerationRatio);
}
void Pendulum::updateSpeed(TMC &tmc) { tmc.setSpeed(speed * speedRatio); }
void Pendulum::updateAcceleration(TMC &tmc) { tmc.setAcceleration(acceleration * accelerationRatio); }
void Pendulum::oscillate(TMC &tmc, uint8_t *counter)
{
    float target;
    float ratio = 1.0f;
    if (*counter > 5)
    {
        ratio = 0.8f;
    }
    if (*counter % 2 == 0)
    {
        target = d0 * ratio;
    }
    else
    {
        target = -d0 * ratio;
    }
    tmc.targetPosition(static_cast<int32_t>(target / distanceRatio));
    if (abs(angle) < (PI / 180.0f * 3.0f) && abs(position - target) < 0.005)
    { // If angle is less than 2 degrees) {
        (*counter)++;
    }
}

float Pendulum::getUprightError() const
{
    return atan2f(sinf(angle - PI), cosf(angle - PI));
}

bool Pendulum::readyForUpright() const
{
    const float activeLimit = (limit > 0.05f) ? limit : FALLBACK_RAIL_LIMIT_M;
    return fabsf(getUprightError()) < CATCH_ANGLE_RAD &&
           fabsf(angularVelocity) < CATCH_ANGULAR_RATE_RAD_S &&
           fabsf(velocity) < CATCH_CART_SPEED_MPS &&
           fabsf(position) < activeLimit * SOFT_LIMIT_RATIO;
}

bool Pendulum::uprightLost() const
{
    return fabsf(getUprightError()) > RELEASE_ANGLE_RAD;
}

void Pendulum::swingUp(TMC &tmc)
{
    constexpr float gravity = 9.81f;
    constexpr float rodMass = 1.0f; // normalized; only the ratio matters
    constexpr float tipMass = TIP_TO_ROD_MASS_RATIO * rodMass;
    constexpr float totalMass = rodMass + tipMass;
    constexpr float tipDistance = TIP_DISTANCE_RATIO * PENDULUM_LENGTH_M;

    // Compound-pendulum parameters per unit total mass.
    constexpr float centerOfMass =
        (rodMass * PENDULUM_LENGTH_M * 0.5f + tipMass * tipDistance) /
        totalMass;
    constexpr float inertiaPerMass =
        (rodMass * PENDULUM_LENGTH_M * PENDULUM_LENGTH_M / 3.0f +
         tipMass * tipDistance * tipDistance) /
        totalMass;

    const float kineticEnergy =
        0.5f * inertiaPerMass * angularVelocity * angularVelocity;
    const float potentialEnergy =
        gravity * centerOfMass * (1.0f - cosf(angle));
    const float targetEnergy = 2.0f * gravity * centerOfMass;
    const float energyError = targetEnergy - (kineticEnergy + potentialEnergy);

    // E_dot is proportional to cart acceleration * rate * cos(angle).
    float phase = angularVelocity * cosf(angle);
    if (fabsf(phase) < 0.05f)
        phase = (position <= 0.0f) ? 1.0f : -1.0f;

    const float energyScale = 0.30f * targetEnergy;
    float pumpAcceleration =
        SWING_DIRECTION_SIGN * SWING_ACCEL_MAX_MPS2 *
        tanhf(energyError / energyScale) *
        ((phase >= 0.0f) ? 1.0f : -1.0f);

    const float centerAcceleration =
        -CART_CENTER_KP * position - CART_CENTER_KD * velocity;
    float accelerationCommand = pumpAcceleration + centerAcceleration;

    // Finite rail safety takes priority over energy pumping.
    const float activeLimit = (limit > 0.05f) ? limit : FALLBACK_RAIL_LIMIT_M;
    const float softLimit = activeLimit * SOFT_LIMIT_RATIO;
    if (position > softLimit)
        accelerationCommand = -SWING_ACCEL_MAX_MPS2;
    else if (position < -softLimit)
        accelerationCommand = SWING_ACCEL_MAX_MPS2;

    accelerationCommand = constrain(accelerationCommand,
                                    -SWING_ACCEL_MAX_MPS2,
                                    SWING_ACCEL_MAX_MPS2);

    const uint32_t rawAcceleration = static_cast<uint32_t>(
        fabsf(accelerationCommand) * accelerationRatio);
    tmc.setAccelerationMax(rawAcceleration);
    tmc.setRampMode((accelerationCommand >= 0.0f) ? CW : CCW);
}

void Pendulum::invertedBalance(TMC &tmc)
{
    // Control algorithm for inverted balance
    int sign = angle >= 0 ? 1 : -1;
    float a = sign * (abs(angle) - balancePos);
    float controlSignal = (kpa * a + kda * angularVelocity + kpm * position + kdm * velocity) * accelerationRatio;
    if (controlSignal < 0)
        tmc.setRampMode(CCW);
    else
        tmc.setRampMode(CW);
    tmc.setAccelerationMax(abs(controlSignal));
}
void Pendulum::balance(TMC &tmc)
{
    float controlSignal = (kpa * angle + kda * angularVelocity + kpm * position / 2.0f + kdm * velocity * 2.0f) * accelerationRatio;
    if (controlSignal > 0)
        tmc.setRampMode(CCW);
    else
        tmc.setRampMode(CW);
    tmc.setAccelerationMax(abs(controlSignal));
}
bool Pendulum::overThreshold()
{
    return (abs(angle) > threshold && threshold > 0.0f);
}
bool Pendulum::overLimit() { return (abs(position) > limit && limit > 0.0f); }
bool Pendulum::underLowLimit() { return (abs(angle) < 5.01f * PI / 180.0f); }
void Pendulum::dumping(TMC &tmc)
{
    tmc.setAcceleration(acceleration * accelerationRatio * 1.26f);
    float x = -0.3f * sin(angle);
    if (x > limit)
    {
        x = limit;
    }
    if (x < -limit)
    {
        x = -limit;
    }
    tmc.targetPosition(static_cast<int32_t>(x / distanceRatio));
}
bool Pendulum::underLowLimit2()
{
    return (abs(angularVelocity) < 0.5 && abs(angle) < 2.0f * PI / 180.0f);
}
void Pendulum::impulse1(TMC &tmc, int8_t sign)
{
    tmc.setRampMode(0);
    tmc.setAcceleration(acceleration * accelerationRatio);
    float x = sign * loopImpulse1 / distanceRatio;
    tmc.targetPosition(static_cast<int32_t>(x));
}
bool Pendulum::loopingTest1()
{
    return (abs(abs(angle) - PI) > loopAngle);
}
void Pendulum::impulse2(TMC &tmc, int8_t sign)
{
    float x = sign * (loopImpulse1 - loopImpulse2) / distanceRatio;
    tmc.targetPosition(static_cast<int32_t>(x));
}
bool Pendulum::loopingTest2()
{
    return (abs(angle) < threshold);
}
bool Pendulum::loopingTest3()
{
    return (abs(angle) > threshold);
}
void Pendulum::setKpa(float kpa_) { kpa = kpa_; }
void Pendulum::setKda(float kda_) { kda = kda_; }
void Pendulum::setKpm(float kpm_) { kpm = kpm_; }
void Pendulum::setKdm(float kdm_) { kdm = kdm_; }