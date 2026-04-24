#include "Speed.h"

Speed::Speed(uint16_t count_) {
    this->count = count_;
    index = 0;
    position = new int32_t[count];
    time = new uint64_t[count];
    for (int i = 0; i < count; i++) {
        position[i] = 0;
        time[i] = 0;
    }
}
float Speed::update(int32_t newValue_) {
    int32_t lastPosition = position[index];
    uint64_t lastTime = time[index];
    position[index] = newValue_;
    time[index] = micros();
    float speed = float(position[index] - lastPosition)/float(time[index] - lastTime) * 1000000.0f; // Calculate speed in steps per second
    index = (index + 1) % count;
    return speed;
}