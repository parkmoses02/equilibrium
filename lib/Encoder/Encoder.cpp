/*
* Library for controlling 2 channels rotary encoder with interrupts.
* Created by Magdi Laoun, 19th July 2025.
* 20th July 2025: Added static instance for interrupt handling.
*/
#include <Encoder.h>
static Encoder* encoderInstance = nullptr;
Encoder::Encoder(uint8_t cha_, uint8_t chb_) {
    // Store configuration only. Touching GPIO or attaching interrupts from a
    // global object's constructor runs before the Arduino/ESP-IDF core has
    // finished starting, which makes attachInterrupt() fail with
    // "GPIO ISR Service Failed To Start" and panics the board into a boot
    // loop. The hardware setup lives in begin(), called from setup().
    cha = cha_;
    chb = chb_;
    position = 0;
    aState = LOW;
    bState = LOW;
    encoderInstance = this; //Set the static instance to this object
}
void Encoder::begin() {
    position = 0;
    aState = LOW;
    bState = LOW;
    // GPIO34/35 are input-only and have no internal pull-ups; the carrier
    // board provides external ones, so request a plain INPUT here.
    pinMode(cha, INPUT);
    pinMode(chb, INPUT);
    aState = digitalRead(cha);
    bState = digitalRead(chb);
    attachInterrupt(digitalPinToInterrupt(cha), isrHandleInterruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(chb), isrHandleInterruptB, CHANGE);
}
long Encoder::getPosition() {
    return position;
}
void Encoder::resetPosition() {
    position = 0;
}
void Encoder::handleInterruptA() {
    aState = digitalRead(cha);
    if (aState ^ bState) {
        position++;
    } else {
        position--;
    }
}
void Encoder::handleInterruptB() {
    bState = digitalRead(chb);
    if (aState ^ bState) {
        position--;
    } else {
        position++;
    }
}
void Encoder::setEncoderEnabled(bool enable) {
    if (enable) {
        attachInterrupt(digitalPinToInterrupt(cha), isrHandleInterruptA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(chb), isrHandleInterruptB, CHANGE);
    } else {
        detachInterrupt(digitalPinToInterrupt(cha));
        detachInterrupt(digitalPinToInterrupt(chb));
    }
}
void isrHandleInterruptA() {
    if (encoderInstance) encoderInstance->handleInterruptA();
}

void isrHandleInterruptB() {
    if (encoderInstance) encoderInstance->handleInterruptB();
}