//#include <Arduino.h>
#include "Pendulum.h"

Pendulum::Pendulum(uint32_t encoderResolution_ = 10000, uint16_t motorMicrosteps_ = 200, uint32_t motorSteps_ = 128, float pulleyCircumference_ = 0.04f)
: encoderSpd(50), motorSpd(50) {
    encoderResolution = encoderResolution_ ;
    motorMicrosteps = motorMicrosteps_;
    motorSteps = motorSteps_;
    pulleyCircumference = pulleyCircumference_;
    updateRatios();
}
void Pendulum::updateRatios() {
    angleRatio = (2.0f * PI) / static_cast<float>(encoderResolution); // Radians per encoder step
    distanceRatio = pulleyCircumference / (static_cast<float>(motorSteps) * static_cast<float>(motorMicrosteps)); // Meters per motor step
    accelerationRatio = 0.015f / distanceRatio; //Calculate acceleration unit for the motor
    speedRatio = 1.37f / distanceRatio; //Calculate speed unit for the motor
}
void Pendulum::updateAngle(Encoder &encoder) {
    long currentPosition = encoder.getPosition() - (rev * encoderResolution);
    if (currentPosition != currentPosition % int(encoderResolution)) {
        rev += (currentPosition > 0) ? 1 : -1;
    }
    angle = float(currentPosition) * angleRatio;
    angularVelocity = float(encoderSpd.update(currentPosition)) * angleRatio;
}
void Pendulum::updatePosition(TMC &tmc) {
    long currentPosition = tmc.getSPIPosition();
    position = float(currentPosition) * distanceRatio;
    velocity = float(motorSpd.update(currentPosition)) * distanceRatio;
}
void Pendulum::setMagnitude(float d0_) {d0 = d0_;}
void Pendulum::setThreshold(float threshold_) {threshold = threshold_;}
void Pendulum::setBalancePos(float balancePos_) {balancePos = balancePos_;}
void Pendulum::setLimit(float limit_) {limit = limit_;}
void Pendulum::setLoopImpulse1(float loopImpulse1_) {loopImpulse1 = loopImpulse1_;}
void Pendulum::setLoopImpulse2(float loopImpulse2_) {loopImpulse2 = loopImpulse2_;}
void Pendulum::setLoopAngle(float loopAngle_) {loopAngle = loopAngle_;}
void Pendulum::setSpeed(TMC &tmc, float speed_) {
    speed = speed_;
    tmc.setSpeed(speed * speedRatio);
}
void Pendulum::setAcceleration(TMC &tmc, float acceleration_) {
    acceleration = acceleration_;
    tmc.setAcceleration(acceleration * accelerationRatio);
}
void Pendulum::updateSpeed(TMC &tmc) {tmc.setSpeed(speed * speedRatio);}
void Pendulum::updateAcceleration(TMC &tmc) {tmc.setAcceleration(acceleration * accelerationRatio);}
void Pendulum::oscillate(TMC &tmc, uint8_t *counter) {
    float target;
    float ratio = 1.0f;
    if (*counter > 5) {ratio = 0.8f;}
    if (*counter % 2 == 0) {target = d0 * ratio;} else {target = -d0 * ratio;}
    tmc.targetPosition(static_cast<int32_t>(target / distanceRatio));
    if (abs(angle) < (PI/180.0f * 3.0f) && abs(position - target) < 0.005 ){ //If angle is less than 2 degrees) {
        (*counter)++;
    }
}
void Pendulum::invertedBalance(TMC &tmc) {
    //Control algorithm for inverted balance
    int sign = angle>=0 ? 1 : -1;
    float a = sign*(abs(angle)-balancePos);
    float controlSignal = (kpa * a + kda * angularVelocity + kpm * position + kdm * velocity)*accelerationRatio;
    if (controlSignal<0) tmc.setRampMode(CCW); else tmc.setRampMode(CW);
    tmc.setAccelerationMax(abs(controlSignal));
}
void Pendulum::balance(TMC &tmc) {
    float controlSignal = (kpa * angle + kda * angularVelocity + kpm * position / 2.0f + kdm * velocity * 2.0f)*accelerationRatio;
    if (controlSignal>0) tmc.setRampMode(CCW); else tmc.setRampMode(CW);
    tmc.setAccelerationMax(abs(controlSignal));
}
bool Pendulum::overThreshold() {
    return (abs(angle) > threshold && threshold > 0.0f);
}
bool Pendulum::overLimit() {return (abs(position) > limit && limit > 0.0f);}
bool Pendulum::underLowLimit() {return (abs(angle) < 5.01f*PI/180.0f);}
void Pendulum::dumping(TMC &tmc) {
    tmc.setAcceleration(acceleration * accelerationRatio*1.26f);
    float x = -0.3f * sin(angle);
    if (x > limit) {x = limit;}
    if (x < -limit) {x = -limit;}
    tmc.targetPosition(static_cast<int32_t>(x / distanceRatio));
}
bool Pendulum::underLowLimit2() {
    return (abs(angularVelocity) < 0.5 && abs(angle) < 2.0f*PI/180.0f);
}
void Pendulum::impulse1(TMC &tmc, int8_t sign) {
    tmc.setRampMode(0);
    tmc.setAcceleration(acceleration * accelerationRatio);
    float x = sign * loopImpulse1 / distanceRatio;
    tmc.targetPosition(static_cast<int32_t>(x));
}
bool Pendulum::loopingTest1() {
    return (abs(abs(angle) - PI) > loopAngle);
}
void Pendulum::impulse2(TMC &tmc, int8_t sign) {
    float x = sign * (loopImpulse1 - loopImpulse2) / distanceRatio;
    tmc.targetPosition(static_cast<int32_t>(x));
}
bool Pendulum::loopingTest2() {
    return (abs(angle) < threshold);
}
bool Pendulum::loopingTest3() {
    return (abs(angle) > threshold);
}
void Pendulum::setKpa(float kpa_){kpa = kpa_;}
void Pendulum::setKda(float kda_){kda = kda_;}
void Pendulum::setKpm(float kpm_){kpm = kpm_;}
void Pendulum::setKdm(float kdm_){kdm = kdm_;}