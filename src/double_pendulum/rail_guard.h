#pragma once
#include <cmath>

namespace RailGuard {
constexpr float brakeLimit = .28f;       // metres from z; 2 cm inside the hard trip
constexpr float reactionSeconds = .020f; // longest permitted control interval
constexpr float assumedDecel = 4.f;      // ramp model, not measured mechanical braking

inline bool needsBrake(float x, float v){
    const float stopX = x + v*reactionSeconds + v*std::fabs(v)/(2.f*assumedDecel);
    return std::fabs(x)>=brakeLimit || std::fabs(stopX)>=brakeLimit;
}
}
