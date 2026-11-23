#pragma once
#include "soc/gpio_struct.h"

// Both encoder pairs are on GPIO32..39. Read A/B in one snapshot.
static DRAM_ATTR int8_t delta[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
struct Encoder { unsigned a,b; volatile int32_t pos=0; volatile unsigned prev=0, bad=0; };
static Encoder enc[2];
static portMUX_TYPE encoderLock = portMUX_INITIALIZER_UNLOCKED;
struct EncoderFrame { int32_t pos[2]; unsigned bad[2]; };
static EncoderFrame readEncoderFrame(){
    portENTER_CRITICAL(&encoderLock);
    EncoderFrame frame{{enc[0].pos,enc[1].pos},{enc[0].bad,enc[1].bad}};
    portEXIT_CRITICAL(&encoderLock);
    return frame;
}
static unsigned IRAM_ATTR pins(const Encoder &e) {
    uint32_t v=GPIO.in1.val;
    return (((v>>(e.a-32))&1)<<1) | ((v>>(e.b-32))&1);
}
static void IRAM_ATTR edge(void *arg) {
    auto &e=*static_cast<Encoder*>(arg);
    portENTER_CRITICAL_ISR(&encoderLock);
    unsigned n=pins(e), p=e.prev;
    if ((p^n)==3) ++e.bad;
    e.pos+=delta[4*p+n]; e.prev=n;
    portEXIT_CRITICAL_ISR(&encoderLock);
}
