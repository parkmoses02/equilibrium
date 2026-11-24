#pragma once
// Encoder wiring and angle conversion shared by upright.h and sysid.h.
#include "encoder_pair.h"

// Commission with motor OFF. SIGNED x4 counts/revolution; positive tilt
// at upright is toward positive cart travel. Zero deliberately blocks arming.
constexpr int BASE_CHANNEL=0; // 0=HEDS1, 1=HEDS2; verify with 'p'
// Down -> right decreases counts; upright -> right increases counts.
constexpr int BASE_CPR=4000, REL_CPR=4000; // nominal x4; verify at physical upright

static float encoderAngle(const EncoderFrame &frame,const long offsets[2],int i,int cpr){
    return remainderf((frame.pos[i]-offsets[i])*(2*PI)/cpr,2*PI);
}
// Absolute link angles from up; offsets are the counts saved at hanging DD.
static void absoluteAnglesFrom(const EncoderFrame &frame,const long offsets[2],float &base,float &tip){
    base=remainderf(encoderAngle(frame,offsets,BASE_CHANNEL,BASE_CPR)-PI,2*PI);
    tip=remainderf(base+encoderAngle(frame,offsets,1-BASE_CHANNEL,REL_CPR),2*PI);
}
static void setupEncoders(){
    enc[0].a=36;enc[0].b=39;enc[1].a=32;enc[1].b=33;
    for(auto &e:enc){
        pinMode(e.a,INPUT);pinMode(e.b,INPUT);e.prev=pins(e);
        attachInterruptArg(e.a,edge,&e,CHANGE);attachInterruptArg(e.b,edge,&e,CHANGE);
    }
}
