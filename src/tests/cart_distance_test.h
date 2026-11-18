#pragma once
// Used only by the isolated cart_distance_test build of the SPI diagnostics.
#include "../double_pendulum/revl_drive.h"
constexpr uint32_t CART_VMAX = velocityToInternal(20 * CART_STEPS_MM); // 20 mm/s
// a[usteps/s^2] = AMAX * fclk^2 / 2^41, nominal 12 MHz internal clock.
constexpr uint32_t CART_AMAX = (50ULL*CART_STEPS_MM*(1ULL<<41)+72000000000000ULL)/144000000000000ULL;
static_assert(CART_AMAX==31 && CART_STEPS_MM*100==4000,"check calibrated ramp scale");
static bool moving=false,armed=false,measurable=false;
static int target=0;
static uint32_t armedAt,started,lastLog;
static void cartHelp(){
    Serial.println("CART DISTANCE | 1/16, calibrated 40 steps/mm. Output OFF. No homing/endstop.");
    Serial.println("STATUS = idle register snapshot; no motion, no register writes.");
    Serial.println("Check travel BOTH ways. ARM then +10/-10, +100/-100 or +500/-500 (mm), each with Enter.");
    Serial.println("x = disable. After internal completion type MEAS 99.5 (absolute ruler distance, mm).");
}
static void disableCart(){
    digitalWrite(PIN_EN,HIGH); writeRegister(REG_VMAX,0);writeRegister(REG_RAMPMODE,3);
    moving=armed=measurable=false;
}
static bool chipValuesOK(uint32_t id,uint32_t drv){
    return (id>>24)==0x30 && !(id & IOIN_SD_MODE) &&
        !(drv&(DRV_FAULT_MASK|DRV_OTPW));
}
static bool chipOK(){
    auto id=readRegister(REG_IOIN);auto drv=readRegister(REG_DRV_STATUS);
    return chipValuesOK(id,drv);
}
static bool inspectCart(const char *stage,bool checkZero,bool always=false){
    auto id=readRegister(REG_IOIN);auto drv=readRegister(REG_DRV_STATUS);
    auto x=readSigned(REG_XACTUAL);auto goal=readSigned(REG_XTARGET);
    bool chip=chipValuesOK(id,drv);
    // Do not add a GSTAT read on the successful preconfiguration path.
    if(chip && !checkZero && !always)return true;
    auto gs=readRegister(REG_GSTAT);
    bool ok=chip && (!checkZero || (!x && !goal && !(gs&6)));
    if(always || !ok)Serial.printf("%s IOIN=0x%08lX VERSION=0x%02lX SD_MODE=%u DRV_STATUS=0x%08lX drv_block=0x%08lX GSTAT=0x%08lX XACTUAL=%ld XTARGET=%ld\n",
        stage,(unsigned long)id,(unsigned long)(id>>24),unsigned(bool(id&IOIN_SD_MODE)),(unsigned long)drv,
        (unsigned long)(drv&(DRV_FAULT_MASK|DRV_OTPW)),(unsigned long)gs,(long)x,(long)goal);
    return ok;
}
static void cartCommand(const char *s){
    if(!strcmp(s,"x")){disableCart();Serial.println("Output OFF.");return;}
    if(moving)return;
    if(!strcmp(s,"STATUS")){armed=false;inspectCart("IDLE",false,true);return;}
    if(!strcmp(s,"ARM")){armed=true;armedAt=millis();Serial.println("Armed for one move, 15 s.");return;}
    if(!strncmp(s,"MEAS ",5)){
        armed=false;
        char *end;double mm=strtod(s+5,&end);
        if(!measurable || *end || !isfinite(mm) || mm<=0){Serial.println("Need completed move and positive ruler distance.");return;}
        int requested=abs(target)/CART_STEPS_MM;
        Serial.printf("cmd_mm=%d ruler_mm=%.3f error_mm=%+.3f error_pct=%+.2f inferred_steps_per_m=%.1f (not applied)\n",
            requested,mm,mm-requested,100*(mm-requested)/requested,1000.0*abs(target)/mm);return;
    }
    int mm=0;
    for(int d:{10,100,500}){char cmd[8];for(int sign:{-1,1}){
        snprintf(cmd,sizeof(cmd),"%+d",sign*d);if(!strcmp(cmd,s))mm=sign*d;
    }}
    bool permitted=armed && millis()-armedAt<15000;armed=false;
    if(!mm || !permitted){cartHelp();return;}
    digitalWrite(PIN_EN,HIGH);measurable=false;
    if(!prepareRevl()){Serial.println("Blocked: see startup checks.");return;}
    writeRegister(REG_AMAX,CART_AMAX);writeRegister(REG_DMAX,CART_AMAX);
    if(!inspectCart("AFTER_CONFIG",true)){Serial.println("Blocked: zeroing or driver fault.");return;}
    target=mm*CART_STEPS_MM;writeRegister(REG_VMAX,CART_VMAX);
    digitalWrite(PIN_EN,LOW);writeRegister(REG_XTARGET,target);
    if(readSigned(REG_XTARGET)!=target){disableCart();Serial.println("Target readback failed.");return;}
    moving=true;started=lastLog=millis();Serial.println("time_ms,internal_steps");
}
static void cartLoop(){
    static char line[48];static size_t n;static bool overflow;
    while(Serial.available()){
        char c=Serial.read();
        if(c=='\r'||c=='\n'){if(n&&!overflow){line[n]=0;cartCommand(line);}n=0;overflow=false;}
        else if(n<sizeof(line)-1)line[n++]=c;else {overflow=true;armed=false;}
    }
    if(!moving || millis()-lastLog<50)return;
    lastLog=millis();auto elapsed=lastLog-started;
    if(elapsed>uint32_t(abs(target)/CART_STEPS_MM*50+3000) || !chipOK() || (readRegister(REG_GSTAT)&7)){
        disableCart();Serial.println("Aborted: timeout, reset or driver fault.");return;
    }
    int32_t x=readSigned(REG_XACTUAL);Serial.printf("%lu,%ld\n",(unsigned long)elapsed,(long)x);
    if(x==target && readRegister(REG_VACTUAL)==0){
        disableCart();measurable=true;Serial.println("Internal ramp complete; output OFF. Measure cart, enter MEAS mm.");
    }
}
