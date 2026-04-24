
#include <Arduino.h>

class Speed {
    private:
        uint16_t count; // Number of samples for averaging
        int32_t sum;
        uint16_t index; // Current index in the circular buffer
        int32_t *position; // Circular buffer to store position
        uint64_t *time; // Circular buffer to store time
    public:
        Speed(uint16_t count_);
        float update(int32_t newValue_);
};