/*
* Library for controlling 2 channels rotary encoder with interrupts.
* Created by Magdi Laoun, 19th July 2025.
* 20th July 2025: Added static instance for interrupt handling.
*/
#include <Encoder.h>
static Encoder* encoderInstance = nullptr;
Encoder::Encoder(uint8_t cha_, uint8_t chb_) {
    cha = cha_;
    chb = chb_;
    position = 0;
    aState = LOW;
    bState = LOW;
    pinMode(cha, INPUT_PULLUP);
    pinMode(chb, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(cha), isrHandleInterruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(chb), isrHandleInterruptB, CHANGE);  
    encoderInstance = this; //Set the static instance to this object
}
void Encoder::begin() {
    position = 0;
    aState = LOW;
    bState = LOW;
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