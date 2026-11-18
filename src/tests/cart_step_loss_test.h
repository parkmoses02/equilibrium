#pragma once
#include "../double_pendulum/revl_drive.h"

// Signed ruler coordinates, relative to the same physical centre throughout a run.
static const int route[]={100,0,-100,0};
enum class Phase {off,ready,moving,measure};
static Phase phase=Phase::off;
static const char *mode=nullptr;
static int speedMM=20,leg=0;
static float accel=.05f,peakMM=0;
static uint32_t since=0,lastPoll=0,moveTime=0;
static constexpr int repeatCycles=10;
static bool repeating=false;
static uint32_t runStart=0;
// Approach +100; traverse -100/+100 ten times; return to centre.
static constexpr int repeatTarget(int i){return i==2*repeatCycles+1?0:(i%2?-100:100);}
static_assert(repeatTarget(0)==100 && repeatTarget(1)==-100 && repeatTarget(20)==100 && repeatTarget(21)==0,"repeat route");
static int targetMM(){return repeating?repeatTarget(leg):route[leg];}

static void cartHelp(){
    Serial.println("STEP LOSS | Centre cart physically; clear BOTH sides. No endstops.");
    Serial.println("POS | ACCEL <2|4|6|8|12> | SPEED <250|400|600|800 mm/s> <2|4|6|8|12 m/s^2>");
    Serial.println("ARM at centre; NEXT for one leg; MEAS <signed ruler mm from centre> while holding. x=output OFF.");
    Serial.println("REPEAT <250|400|600|800|900|1000|1100|1200 mm/s> <2|4|6|8|12 m/s^2>: ARM then START for 10 automatic +/-100 mm cycles. MEAS only at final centre.");
    Serial.println("Extra acceleration trials at 800 mm/s only: REPEAT 800 14 | REPEAT 800 16");
}
static void stopTrial(const char *reason){
    digitalWrite(PIN_EN,HIGH);
    writeRegister(REG_VMAX,0);writeRegister(REG_RAMPMODE,1);
    phase=Phase::off;mode=nullptr;Serial.println(reason);
}
static void startCartLeg(){
    int32_t target=targetMM()*CART_STEPS_MM;
    writeRegister(REG_XTARGET,target);
    if(readSigned(REG_XTARGET)!=target){stopTrial("OFF: target mismatch");return;}
    phase=Phase::moving;since=lastPoll=millis();
}
static void cartCommand(const char *s){
    if(!strcmp(s,"x")||!strcmp(s,"X")){stopTrial("OFF: cancelled. Re-centre before next ARM.");return;}
    if(phase==Phase::off){
        if(!strcmp(s,"ARM")){
            if(!mode){cartHelp();return;}
            if(!prepareRevl()){stopTrial("BLOCK: preflight");return;}
            uint32_t ar=lround(accel*CART_STEPS_MM*1000.0*(2199023255552.0/144000000000000.0));
            writeRegister(REG_AMAX,ar);writeRegister(REG_DMAX,ar);
            writeRegister(REG_VMAX,velocityToInternal(speedMM*CART_STEPS_MM));
            digitalWrite(PIN_EN,LOW);leg=0;phase=Phase::ready;since=millis();
            Serial.println(repeating?"ARMED: START runs entire route. x + Enter aborts.":"ARMED at 0 mm. NEXT goes to +100 mm.");
            Serial.println("mode,leg,a_m_s2,v_mm_s,target_mm,ruler_mm,error_mm,peak_ramp_mm_s,move_ms");return;
        }
        // Match only enumerated, canonical commands before accepting any parameters.
        mode=nullptr;repeating=false;
        if(!strcmp(s,"POS")){mode="POS";speedMM=20;accel=.05f;}
        for(int a:{14,16}){
            char key[32];snprintf(key,sizeof(key),"REPEAT 800 %d",a);
            if(!strcmp(s,key)){mode="REPEAT";speedMM=800;accel=a;repeating=true;}
        }
        for(int a:{2,4,6,8,12}){
            char key[32];snprintf(key,sizeof(key),"ACCEL %d",a);
            if(!strcmp(s,key)){mode="ACCEL";accel=a;speedMM=250;}
            for(int v:{250,400,600,800}){
                snprintf(key,sizeof(key),"SPEED %d %d",v,a);
                if(!strcmp(s,key)){mode="SPEED";speedMM=v;accel=a;}
            }
            for(int v:{250,400,600,800,900,1000,1100,1200}){
                snprintf(key,sizeof(key),"REPEAT %d %d",v,a);
                if(!strcmp(s,key)){mode="REPEAT";speedMM=v;accel=a;repeating=true;}
            }
        }
        if(mode)Serial.printf("%s: %.2f m/s^2, %d mm/s; type ARM at physical centre.\n",mode,accel,speedMM);
        else cartHelp();
        return;
    }
    if(phase==Phase::ready && !strcmp(s,repeating?"START":"NEXT")){
        peakMM=0;runStart=millis();startCartLeg();return;
    }
    if(phase==Phase::measure && !strncmp(s,"MEAS ",5)){
        char *end;double mm=strtod(s+5,&end);
        if(end==s+5||*end||!isfinite(mm)||fabs(mm)>300){Serial.println("Need signed ruler coordinate in mm.");return;}
        Serial.printf("%s,%d,%.2f,%d,%d,%.3f,%+.3f,%.2f,%lu\n",mode,leg+1,accel,speedMM,targetMM(),mm,mm-targetMM(),peakMM,(unsigned long)moveTime);
        if(repeating){stopTrial("OFF: 10 cycles complete.");return;}
        if(++leg==sizeof(route)/sizeof(route[0])){stopTrial("OFF: route complete.");return;}
        phase=Phase::ready;since=millis();Serial.printf("Holding. NEXT goes to %+d mm, or x to abort.\n",route[leg]);return;
    }
    Serial.println("No move: follow NEXT / MEAS prompt, or x.");
}
static void cartLoop(){
    static char buf[48];static unsigned n=0;static bool overflow=false;
    while(Serial.available()){
        char c=Serial.read();
        if(c=='\n'||c=='\r'){
            if(overflow)stopTrial("OFF: command too long.");
            else if(n){buf[n]=0;cartCommand(buf);}
            n=0;overflow=false;
        }else if(n<sizeof(buf)-1)buf[n++]=c;else overflow=true;
    }
    uint32_t now=millis();
    if(phase==Phase::off || now-lastPoll<5)return;
    lastPoll=now;
    auto id=readRegister(REG_IOIN);
    if(!safeChip(id,readRegister(REG_DRV_STATUS))||(id&IOIN_DRV_ENN)||(readRegister(REG_GSTAT)&7)){
        stopTrial("OFF: driver fault/reset");return;
    }
    auto x=readSigned(REG_XACTUAL);
    if(x>110*CART_STEPS_MM||x< -110*CART_STEPS_MM){stopTrial("OFF: internal travel guard");return;}
    if(now-since>(phase==Phase::moving?10000u:60000u)){stopTrial("OFF: timeout");return;}
    if(phase!=Phase::moving)return;
    int32_t v=readRegister(REG_VACTUAL)&0xffffff;
    if(v&0x800000)v-=0x1000000;
    peakMM=fmaxf(peakMM,fabsf(v)*(12000000.f/16777216.f)/CART_STEPS_MM);
    if(x==targetMM()*CART_STEPS_MM && v==0){
        if(repeating && leg<2*repeatCycles+1){++leg;startCartLeg();return;}
        moveTime=now-runStart;phase=Phase::measure;since=now;
        Serial.printf("Holding %+d mm. MEAS <signed ruler mm>, or x. Peak internal speed %.1f mm/s.\n",targetMM(),peakMM);
    }
}
