#pragma once
// 200 mm measured for 8000 microsteps at MRES=4, in both directions.
constexpr int CART_STEPS_MM=40;
static_assert(CFG_MRES==4,"Recalibrate distance for different microstepping");
static bool safeChip(uint32_t id,uint32_t drv){
    return (id>>24)==0x30 && !(id&IOIN_SD_MODE) && !(drv&(DRV_FAULT_MASK|DRV_OTPW));
}
// Never enables coils. Record startup evidence, clear reset/UV history once,
// wait as in diagnostics, then reject any recurring fault. Never clear drv_err.
static bool prepareRevl(){
    digitalWrite(PIN_EN,HIGH);
    auto id=readRegister(REG_IOIN),drv=readRegister(REG_DRV_STATUS),g=readRegister(REG_GSTAT);
    Serial.printf("PRECHECK IOIN=%08lx DRV=%08lx GSTAT=%lx\n",(long)id,(long)drv,(long)g);
    if(!safeChip(id,drv)||!(id&IOIN_DRV_ENN)||(g&2))return false;
    writeRegister(REG_GSTAT,g&5);delay(200);
    id=readRegister(REG_IOIN);drv=readRegister(REG_DRV_STATUS);g=readRegister(REG_GSTAT);
    if(!safeChip(id,drv)||!(id&IOIN_DRV_ENN)||(g&7)){
        Serial.printf("RECHECK IOIN=%08lx DRV=%08lx GSTAT=%lx\n",(long)id,(long)drv,(long)g);
        return false;
    }
    configureDriver();
    // ENN disables the coils, not the ramp generator. Command zero velocity
    // and verify standstill before resetting its position or enabling outputs.
    writeRegister(REG_VMAX,0);writeRegister(REG_RAMPMODE,1);
    uint32_t since=millis();
    while(readRegister(REG_VACTUAL)&0xffffff){
        if(millis()-since>=500){Serial.println("BLOCK: ramp failed to reach zero speed");return false;}
        delay(2);
    }
    writeRegister(REG_XACTUAL,0);writeRegister(REG_XTARGET,0);writeRegister(REG_RAMPMODE,0);
    int32_t x=readSigned(REG_XACTUAL),target=readSigned(REG_XTARGET);
    id=readRegister(REG_IOIN);drv=readRegister(REG_DRV_STATUS);g=readRegister(REG_GSTAT);
    Serial.printf("AFTER_CONFIG X=%ld TARGET=%ld IOIN=%08lx DRV=%08lx GSTAT=%lx\n",(long)x,(long)target,(long)id,(long)drv,(long)g);
    return x==0 && target==0 && !(readRegister(REG_VACTUAL)&0xffffff) && safeChip(id,drv) && (id&IOIN_DRV_ENN) && !(g&7);
}
