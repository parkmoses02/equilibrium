#include "MyData.h"
#include <Speed.h>
class Pendulum {

    private:
        float angleRatio = 1.0f; // Conversion factor from encoder steps to radians
        float distanceRatio = 1.0f; // Conversion factor from motor steps to meters
        float accelerationRatio = 1.0f; // Conversion factor from motor steps/s^2 to m/s^2
        float speedRatio = 1.0f; // Conversion factor from motor steps/s to m/s
        float pulleyCircumference = 0.04f; // Circumference of the pulley in meters
        uint32_t motorSteps = 200; // Number of steps per revolution for the motor
        uint16_t motorMicrosteps = 128; // Number of microsteps per step for the motor
        uint32_t encoderResolution = 10000; // Encoder resolution in steps
        float kpa = 0.0f; // Proportional gain for encoder control
        float kda = 0.0f; // Derivative gain for balance control
        float kpm = 0.0f; // Proportional gain for motor
        float kdm = 0.0f; // Derivative gain for motor
        float d0 = 0.0f; // Magnitude of oscillation in meters
        float speed = 0.0f; // Speed in meters per second
        float acceleration = 0.0f; // Acceleration in meters per second squared of cart
        float threshold = 0.0f; // Threshold for angle in radians
        float balancePos = 0.0f; // Position for balance in radians
        float limit = 0.0f; // Limit for distance in meters
        float loopImpulse1 = 0.0f; // Impulse for looping in meters
        float loopImpulse2 = 0.0f; // Impulse for looping in meters
        float loopAngle = 0.0f; // Angle for looping in radians
        int rev = 0; // Revolution counter for the encoder
        Speed encoderSpd;
        Speed motorSpd;
        void updateRatios();
    public:
        float angle = 0.0f; // Angle in radians of pendulum
        float angularVelocity = 0.0f; // Angular velocity in radians per second
        float position = 0.0f; // Position in meters of cart
        float velocity = 0.0f; // Velocity in meters per second of cart
        Pendulum(uint32_t encoderResolution_, uint16_t motorMicrosteps_, uint32_t motorSteps_, float pulleyCircumference_);
        void updateAngle(Encoder &encoder);
        void updatePosition(TMC &tmc);
        void setMagnitude(float d0_);
        void setThreshold(float threshold_);
        void setBalancePos(float balancePos_);
        void setLimit(float limit_);
        void setLoopImpulse1(float loopImpulse1_);
        void setLoopImpulse2(float loopImpulse2_);
        void setLoopAngle(float loopAngle_);
        void setSpeed(TMC &tmc, float speed_);
        void setAcceleration(TMC &tmc, float acceleration_);
        void updateSpeed(TMC &tmc);
        void updateAcceleration(TMC &tmc);
        void oscillate(TMC &tmc, uint8_t *counter);
        void invertedBalance(TMC &tmc);
        void balance(TMC &tmc);
        bool overThreshold();
        bool overLimit();
        bool underLowLimit();
        void dumping(TMC &tmc);
        bool underLowLimit2();
        void impulse1(TMC &tmc, int8_t sign);
        bool loopingTest1();
        void impulse2(TMC &tmc, int8_t sign);
        bool loopingTest2();
        bool loopingTest3();
        void setKpa(float kpa_);
        void setKda(float kda_);
        void setKpm(float kpm_);
        void setKdm(float kdm_);
};