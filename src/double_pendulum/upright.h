#pragma once
#include "encoder_pair.h"
#include "revl_drive.h"
#include "du_frames.h"
#include "uu_frames.h"
#include "re_learning.h"
#include "du_edges.h"
// Commission with motor OFF. SIGNED x4 counts/revolution; positive tilt
// at upright is toward positive cart travel. Zero deliberately blocks arming.
constexpr int BASE_CHANNEL=0; // 0=HEDS1, 1=HEDS2; verify with 'p'
// Down -> right decreases counts; upright -> right increases counts.
constexpr int BASE_CPR=4000, REL_CPR=4000; // nominal x4; verify at physical upright
constexpr float ARM_TILT=0.05f;
constexpr float DU_CATCH_ANGLE_RAD=0.10f;
constexpr float DU_CATCH_SPEED_MPS=0.30f; // inclusive; terminal ramp speed, not the target
constexpr float UU_CORE_ANGLE_RAD = 0.05f, UU_CORE_SPEED_MPS = 0.20f;
constexpr float UU_CATCH_ANGLE_RAD = 0.10f, UU_CATCH_SPEED_MPS = 0.30f;
constexpr float UU_CATCH_RATE_RAD_S = 0.50f;
constexpr float UU_EXTENDED_LINK_DIFF_RAD = 0.01f, UU_EXTENDED_RATE_RAD_S = 0.05f;
constexpr float MANUAL_HOLD_TILT_RAD = 0.17f;
constexpr float DU_POST_CATCH_TILT_RAD = radians(20.0f);
constexpr float SCALE=CART_STEPS_MM*1000.f;
constexpr float MAX_ACCEL_MPS2=12.0f, MAX_SPEED_MPS=0.80f;
constexpr float CART_TRAVEL_LIMIT_M=0.30f;
constexpr float K_q[]={3.464102f,-116.851324f,140.474145f,5.061349f,-4.628502f,15.246215f};
constexpr float K_w[]={-3.464102f,-52.422759f,17.225801f,-4.465170f,-4.803921f,-0.558010f};
constexpr float K_e[]={-14.832397f,153.628010f,-244.278154f,-17.451330f,11.412767f,-27.573131f};
struct Equilibrium {char key;const char *name;float base,tip;const float *gain;};
constexpr Equilibrium modes[]={{'q',"UU",0,0,K_q},{'w',"UD",0,PI,K_w},{'e',"DU",PI,0,K_e},{'r',"DD",PI,PI,nullptr}};
static const Equilibrium *poseFor(char key){
    for(const auto &m:modes)if(m.key==key)return &m;
    return nullptr;
}
static const Equilibrium *mode=&modes[0];
static bool referenced=false,active=false;
static uint32_t cartZeroSteps;
static float rampPositionFromZero(){
    return static_cast<int32_t>(readRegister(REG_XACTUAL)-cartZeroSteps)/SCALE;
}
static long offsets[2];static unsigned errors[2];
static DuEdges frameEdges{0,0}; // shared across swing and catch; z baseline stays untouched
static uint32_t tick;
static float prev1,prev2,w1,w2,vCmd;
static bool swinging=false;
static float holdTiltLimitRad = MANUAL_HOLD_TILT_RAD;
static unsigned swingStep=0;
static uint32_t swingStart=0;
static float lastErr[2],errRate[2];
static constexpr unsigned DU_COUNT=sizeof(duFrames)/sizeof(duFrames[0]);
static_assert(DU_COUNT==336,"5 ms DD->DU trajectory expected");
static constexpr unsigned UU_COUNT=sizeof(uuFrames)/sizeof(uuFrames[0]);
struct SwingPlan { const DuFrame *frames; unsigned count; char phase; };
static constexpr SwingPlan ddToDU{duFrames, DU_COUNT, 's'}, ddToUU{uuFrames, UU_COUNT, 'u'};
static const SwingPlan *plan = &ddToDU;
static ReLearning reLearn;
static_assert(ReLearning::bins*ReLearning::width==DU_COUNT,"RE learning frame count");
// One sample per swing frame plus the catch decision. No serial I/O here.
struct SwingSample { uint32_t us; float base,tip,wb,wt,x,v,priorCmd,nextCmd,trim; uint8_t estimatedHeds; };
static SwingSample swingLog[(UU_COUNT > DU_COUNT ? UU_COUNT : DU_COUNT)+1];
static unsigned swingUsed;
static void dumpSwing(){
    if(!swingUsed)return;
    Serial.println(plan == &ddToUU ? "RQ_CSV_BEGIN" : "RE_CSV_BEGIN");
    Serial.println("step,t_us,base_rad,tip_rad,base_rad_s,tip_rad_s,ramp_x_m,ramp_v_m_s,ref_v_m_s,prev_cmd_v_m_s,next_cmd_v_m_s,learned_trim_m_s2,estimated_heds");
    for(unsigned i=0;i<swingUsed;i++){
        const auto &s=swingLog[i];
        Serial.printf("%u,%lu,%.6f,%.6f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.5f,%u\n",i,
            (unsigned long)s.us,s.base,s.tip,s.wb,s.wt,s.x,s.v,i<plan->count?plan->frames[i].v:0.f,s.priorCmd,s.nextCmd,s.trim,unsigned(s.estimatedHeds));
    }
    Serial.println(plan == &ddToUU ? "RQ_CSV_END" : "RE_CSV_END");swingUsed=0;
}
static void rampLimit(float mps2){
    writeRegister(REG_AMAX,lround(mps2*SCALE*(2199023255552.0/144000000000000.0)));
}
#ifdef UPRIGHT_TRACE
struct TraceRow { uint32_t t,dt; float base,tip,x,v,u,a,vc; bool speedLimited; uint8_t estimatedHeds; };
constexpr uint32_t TRACE_US=1000000;
static TraceRow rows[TRACE_US/5000+1];
static unsigned used;
static uint32_t traceStart;
static void dumpTrace(){
    Serial.println("elapsed_us,dt_us,base_error_deg,tip_error_deg,ramp_x_mm,ramp_v_mm_s,lqr_u_m_s2,limited_a_m_s2,v_cmd_mm_s,speed_clipped,estimated_heds");
    for(unsigned i=0;i<used;i++){
        const auto &r=rows[i];
        Serial.printf("%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%d,%u\n",
            (unsigned long)r.t,(unsigned long)r.dt,degrees(r.base),degrees(r.tip),r.x*1000,r.v*1000,r.u,r.a,r.vc*1000,r.speedLimited,unsigned(r.estimatedHeds));
    }
}
#endif
static void reportEdges(const char *checkedAt){
    Serial.printf("ENC checked_at=%s invalid_since_z HEDS1=+%u HEDS2=+%u raw=%ld,%ld\n",
        checkedAt,enc[0].bad-errors[0],enc[1].bad-errors[1],(long)enc[0].pos,(long)enc[1].pos);
}
static void stopAndDump(float missScore=-1.f){
    digitalWrite(PIN_EN,HIGH);writeRegister(REG_VMAX,0);writeRegister(REG_RAMPMODE,1);
    bool wasActive=active;
    bool wasSwing=swinging;
    active=swinging=false;
    if(referenced && (enc[0].bad!=errors[0] || enc[1].bad!=errors[1])){
        reportEdges(wasSwing?"swing":wasActive?"hold":"motor_off");
        referenced=false;Serial.println("Encoder counts suspect: centre cart, hang DD, then z.");
    }else if(referenced)Serial.println("DD and cart references kept. If cart moved by hand, centre it, hang DD, then z.");
    if(wasSwing && plan == &ddToDU){
        if(missScore>=0 && referenced && swingUsed==DU_COUNT+1){
            bool updated=reLearn.miss(missScore);
            Serial.printf("RE_LEARN %s score=%.3f (lower better); manual restart only\n",updated?"updated":"rollback",missScore);
        }else{
            reLearn.abort();Serial.println("RE_LEARN skipped; using last accepted profile.");
        }
    }
    dumpSwing();
#ifdef UPRIGHT_TRACE
    if(wasActive)dumpTrace();
#endif
}
template<typename... Args>
static void stopDouble(const char *why, float missScore=-1.f,
                       const char *detailFormat=nullptr, Args... detailArgs){
    stopAndDump(missScore);
    if(detailFormat) Serial.printf(detailFormat, detailArgs...);
    Serial.println(why);
}
static float angle(const EncoderFrame &frame,int i,int cpr){
    return remainderf((frame.pos[i]-offsets[i])*(2*PI)/cpr,2*PI);
}
static void absoluteAngles(const EncoderFrame &frame,float &base,float &tip){
    base=remainderf(angle(frame,BASE_CHANNEL,BASE_CPR)-PI,2*PI);
    tip=remainderf(base+angle(frame,1-BASE_CHANNEL,REL_CPR),2*PI);
}
static void subtractTargetAngles(float &base,float &tip){
    base=remainderf(base-mode->base,2*PI);
    tip=remainderf(tip-mode->tip,2*PI);
}
static bool poseOK(float t){return fabsf(t)<=ARM_TILT;}
static bool uuCatchReady(float base, float tip, float baseRate, float tipRate,
                         float cartSpeed){
    if (!std::isfinite(base) || !std::isfinite(tip) || !std::isfinite(baseRate)
        || !std::isfinite(tipRate) || !std::isfinite(cartSpeed)) return false;
    const float angle = fmaxf(fabsf(base), fabsf(tip));
    const float rate = fmaxf(fabsf(baseRate), fabsf(tipRate));
    const float speed = fabsf(cartSpeed);
    if (angle <= UU_CORE_ANGLE_RAD && speed <= UU_CORE_SPEED_MPS)
        return rate < UU_CATCH_RATE_RAD_S;
    // Nearly parallel, slow links: reserve recovery travel as cart speed increases.
    return rate <= UU_EXTENDED_RATE_RAD_S && fabsf(base-tip) <= UU_EXTENDED_LINK_DIFF_RAD
        && angle / UU_CATCH_ANGLE_RAD + speed / UU_CATCH_SPEED_MPS <= 1.f;
}
static bool startPoseOK(bool dd){
    absoluteAngles(readEncoderFrame(),prev1,prev2);
    float base=remainderf(prev1-(dd?PI:mode->base),2*PI),tip=remainderf(prev2-(dd?PI:mode->tip),2*PI);
    if(poseOK(base)&&poseOK(tip))return true;
    Serial.printf("BLOCK %s: base=%+.2f [%s], tip=%+.2f [%s] deg; limit +/-%.2f deg\n",dd?"DD":mode->name,
        degrees(base),poseOK(base)?"PASS":"FAIL",degrees(tip),poseOK(tip)?"PASS":"FAIL",degrees(ARM_TILT));
    return false;
}
static void doubleSetup(){
    // Captured DD->DU trial, final ramp v=0.241113 m/s. Seed once per boot;
    // z preserves subsequent learning and lc still returns to zero trim.
    static constexpr float caughtTrim[]={
        0.00760f, 0.01300f, -0.01776f, -0.00395f, -0.00597f, 0.00034f, 0.00354f,
        0.01520f, -0.03064f, 0.00626f, 0.07502f, 0.00323f, -0.11395f, -0.06590f,
        -0.00484f, -0.02151f, -0.04316f, -0.07576f, 0.01023f, 0.15018f, 0.11125f,
        -0.10159f, 0.00922f, -0.02879f, 0.04660f, 0.07314f, -0.14613f, -0.20707f,
    };
    static_assert(sizeof(caughtTrim)/sizeof(*caughtTrim)==ReLearning::bins,"RE seed size");
    std::copy_n(caughtTrim,ReLearning::bins,reLearn.current);
    std::copy_n(caughtTrim,ReLearning::bins,reLearn.accepted);
    Serial.printf("Cart command limits: +/-%.1f m/s^2, +/-%.2f m/s\n",MAX_ACCEL_MPS2,MAX_SPEED_MPS);
    Serial.printf("Ramp position limit: +/-%.0f mm from z\n",CART_TRAVEL_LIMIT_M*1000.f);
    enc[0].a=36;enc[0].b=39;enc[1].a=32;enc[1].b=33;
    for(auto &e:enc){
        pinMode(e.a,INPUT);pinMode(e.b,INPUT);e.prev=pins(e);
        attachInterruptArg(e.a,edge,&e,CHANGE);attachInterruptArg(e.b,edge,&e,CHANGE);
    }
    Serial.println("Enter one command per line: z=zero DD at centre; q/w/e/r=UU/UD/DU/DD; p=start selected hold (q/w/e).");
    Serial.println("re=DD->DU; rq=DD->UU; x=OFF. Each start keeps the cart origin from z.");
    Serial.println("z saves DD angles and cart ramp origin; h reads encoder counters; lc clears RE learning. Motor OFF commands.");
#ifdef UPRIGHT_TRACE
    Serial.printf("TRACE: hold for up to %lu ms after p or catch, then OFF and CSV. Ramp x/v are NOT measured travel.\n",(unsigned long)(TRACE_US/1000));
#endif
}
static void idleCommand(char c,char dest){
        if(c=='l' && dest=='c'){reLearn=ReLearning{};Serial.println("RE learning cleared; angle zero unchanged.");return;}
        if(c=='h' && !dest){reportEdges("motor_off");return;}
        bool dd=false;
        const auto *from=poseFor(c);
        if(dest){
            const auto *to=poseFor(dest);
            if(!from||!to){Serial.println("Invalid route; use q/w/e/r.");return;}
            if(c!='r'||(dest!='e' && dest!='q')){
                Serial.printf("Reserved %c%c: %s->%s; motor OFF.\n",c,dest,from->name,to->name);return;
            }
            dd=true;
        }else if(from){
            mode=from;Serial.printf("Selected %c: %s. Motor OFF.\n",c,from->name);return;
        }
        if(c=='z'){
            referenced=false;
            const auto id=readRegister(REG_IOIN);
            if(!safeChip(id,readRegister(REG_DRV_STATUS)) || !(id&IOIN_DRV_ENN) || (readRegister(REG_VACTUAL)&0xffffff)){
                Serial.println("BLOCK z: need disabled driver and stationary ramp.");return;
            }
            writeRegister(REG_GSTAT,1); // acknowledge startup reset before recording this origin
            cartZeroSteps=readRegister(REG_XACTUAL);
            const auto frame=readEncoderFrame();
            for(int i=0;i<2;i++){offsets[i]=frame.pos[i];errors[i]=frame.bad[i];}
            referenced=true;Serial.println("Saved DD angles and cart ramp origin (0 mm). RE learning kept.");return;
        }
        if(c=='p'||dd){
            if(!dd && !mode->gain){Serial.println("BLOCK: DD hold not implemented; motor OFF.");return;}
            if(!referenced || !BASE_CPR || !REL_CPR){Serial.println("BLOCK: need signed CPR and z at hanging DD.");return;}
            if(enc[0].bad!=errors[0] || enc[1].bad!=errors[1]){stopDouble("BLOCK: encoder edges");return;}
            if(!startPoseOK(dd))return;
            long before[]={enc[0].pos,enc[1].pos};
            if(!prepareRevl(false)){stopDouble("BLOCK: driver preflight failed.");return;}
            const float x=rampPositionFromZero();
            if(fabsf(x)>CART_TRAVEL_LIMIT_M){
                stopDouble("BLOCK: ramp position", -1.f, "x_from_z_mm=%+.2f limit_mm=300\n", x*1000.f);return;
            }
            while(Serial.available()){if(Serial.read()=='x'){stopDouble("OFF during preflight");return;}}
            if(enc[0].bad!=errors[0] || enc[1].bad!=errors[1]){stopDouble("OFF: encoder error; reference cleared.");return;}
            if(!startPoseOK(dd))return;
            if(dd && (labs(enc[0].pos-before[0])>3 || labs(enc[1].pos-before[1])>3)){
                Serial.println("BLOCK: DD moved during preflight; let links settle.");return;}
            swinging=dd;
            holdTiltLimitRad = MANUAL_HOLD_TILT_RAD;
            frameEdges=DuEdges{errors[0],errors[1]};
            if(dd){
                mode=poseFor(dest);
                plan=dest=='q' ? &ddToUU : &ddToDU;
                swingStep=0;errRate[0]=errRate[1]=0;
                if(plan==&ddToDU)reLearn.begin();
            }
            else subtractTargetAngles(prev1,prev2);
            rampLimit(dd?16.f:MAX_ACCEL_MPS2);
            writeRegister(REG_RAMPMODE,1);writeRegister(REG_VMAX,0);
            tick=micros();w1=w2=vCmd=0;active=true;digitalWrite(PIN_EN,LOW);
            swingStart=tick;
            if(dd) Serial.printf("ACTIVE DD->%s (one attempt)\n", mode->name);
            else Serial.println("ACTIVE hold");
#ifdef UPRIGHT_TRACE
            used=0;traceStart=tick;
#endif
        }else Serial.println("Unknown command; motor OFF.");
}
static void doubleLoop(){
    static char cmd[2];static unsigned len=0;static bool discard=false;
    if(Serial.available()){
        char ch=Serial.read();
        if(ch=='x'){len=0;discard=true;stopDouble("OFF");return;}
        if(ch=='\r'||ch=='\n'){
            if(!active && !discard && len)idleCommand(cmd[0],len==2?cmd[1]:0);
            len=0;discard=false;
        }else if(active || ch==0){discard=true;}
        else if(!discard){
            if(len<2)cmd[len++]=ch;
            else {discard=true;Serial.println("Invalid: at most two letters per line.");}
        }
    }
    uint32_t now=micros(),us=now-tick;
    if(!active)return;
    if(us>20000){
        stopDouble("OFF: loop deadline", -1.f, "Loop interval: %lu us\n", (unsigned long)us);
        return;
    }
#ifdef UPRIGHT_TRACE
    if(!swinging && now-traceStart>=TRACE_US){stopDouble("OFF: trace duration reached");return;}
#endif
    if(us<5000)return;
    tick=now;float dt=us*1e-6f;
    uint32_t id=readRegister(REG_IOIN),drv=readRegister(REG_DRV_STATUS);
    float x=rampPositionFromZero();
    int32_t vr=readRegister(REG_VACTUAL)&0xffffff;if(vr&0x800000)vr-=0x1000000;
    float v=vr*(12000000.f/16777216.f)/SCALE;
    if(!safeChip(id,drv)||(id&IOIN_DRV_ENN)||(readRegister(REG_GSTAT)&7)){stopDouble("OFF: driver");return;}
    const auto frame=readEncoderFrame();
    const unsigned deltaBad[]={frame.bad[0]-frameEdges.seen1,frame.bad[1]-frameEdges.seen2};
    const auto edgeResult=frameEdges.check(now,frame.bad[0],frame.bad[1]);
    if(edgeResult==DuEdges::stop){
        stopDouble("OFF: repeated encoder edges", -1.f, "Frame errors HEDS1=+%u HEDS2=+%u\n",deltaBad[0],deltaBad[1]);return;
    }
    if(fabsf(x)>CART_TRAVEL_LIMIT_M){
        stopDouble("OFF: ramp position", -1.f, "x_from_z_mm=%+.2f limit_mm=300\n", x*1000.f);return;
    }
    float t1,t2;
    uint8_t estimatedHeds=0;
    absoluteAngles(frame,t1,t2);
    if(!swinging)subtractTargetAngles(t1,t2);
    if(edgeResult==DuEdges::warn){
        // tip = base + relative joint. Extrapolate only the affected encoder.
        float relative=remainderf(t2-t1,2*PI);
        if(deltaBad[BASE_CHANNEL])t1=remainderf(prev1+w1*dt,2*PI);
        else relative=remainderf(prev2-prev1+(w2-w1)*dt,2*PI);
        t2=remainderf(t1+relative,2*PI);
        estimatedHeds=deltaBad[0]?1:2;
        Serial.printf("WARN: HEDS%u angle estimated for one frame t_us=%lu\n",unsigned(estimatedHeds),(unsigned long)now);
    }
    w1=.6f*w1+.4f*remainderf(t1-prev1,2*PI)/dt;w2=.6f*w2+.4f*remainderf(t2-prev2,2*PI)/dt;prev1=t1;prev2=t2;
    if(swinging && swingStep<=plan->count){
        swingLog[swingStep]={now-swingStart,t1,t2,w1,w2,x,v,vCmd,NAN,NAN,estimatedHeds};
        swingUsed=swingStep+1;
    }
    if(swinging && now-swingStart>(swingStep+1)*5000u+15000u){stopDouble("OFF: swing timing");return;}
    if(swinging && swingStep==plan->count){
        // Keep terminal velocity for one tick; confirm catch on a measured frame.
        if(edgeResult==DuEdges::warn)return;
        t1=remainderf(t1-mode->base,2*PI);
        t2=remainderf(t2-mode->tip,2*PI);
        if(plan == &ddToUU){
            if(!uuCatchReady(t1, t2, w1, w2, v)){
                stopDouble("OFF: missed UU; no retry", -1.f,
                           "UU err_deg=%+.3f,%+.3f rate_rad_s=%+.3f,%+.3f ramp_v=%+.4f ramp_x=%+.4f\n",
                           degrees(t1), degrees(t2), w1, w2, v, x);
                return;
            }
        }else{
            const bool anglesOK = fabsf(t1) <= DU_CATCH_ANGLE_RAD && fabsf(t2) <= DU_CATCH_ANGLE_RAD;
            if(!anglesOK||fabsf(w1)>=.5f||fabsf(w2)>=.5f||fabsf(v)>DU_CATCH_SPEED_MPS){
                float score=fmaxf(fmaxf(fabsf(t1),fabsf(t2))/DU_CATCH_ANGLE_RAD,
                    fmaxf(fmaxf(fabsf(w1),fabsf(w2))/.5f,fabsf(v)/DU_CATCH_SPEED_MPS));
                stopDouble("OFF: missed DU; no retry", score,
                           "DU err_deg=%.2f,%.2f rate_rad_s=%.2f,%.2f ramp_v=%.3f\n",
                           degrees(t1), degrees(t2), w1, w2, v);
                return;
            }
            reLearn.catchDU(); w1=w2=0;
            holdTiltLimitRad = DU_POST_CATCH_TILT_RAD;
        }
        swinging=false;prev1=t1;prev2=t2;vCmd=v;rampLimit(MAX_ACCEL_MPS2);
        Serial.printf("CATCH %s -> %c\n", mode->name, mode->key);
#ifdef UPRIGHT_TRACE
        traceStart=now;used=0;
#endif
    }
    float u;
    if(swinging){
        const auto &r=plan->frames[swingStep];
        float e[]={0.f,remainderf(t1-r.base,2*PI),remainderf(t2-r.tip,2*PI),v-r.v,0,0};
        if(fabsf(e[1])>.45f || fabsf(e[2])>.45f){
            stopDouble("OFF: swing tracking error", -1.f,
                       "step=%u x_from_z_mm=%.1f base_err_deg=%.2f tip_err_deg=%.2f\n",
                       swingStep, 1000*x, degrees(e[1]), degrees(e[2]));
            return;
        }
        for(int j=0;j<2;j++){
            if(swingStep)errRate[j]=.6f*errRate[j]+.4f*remainderf(e[1+j]-lastErr[j],2*PI)/dt;
            lastErr[j]=e[1+j];e[4+j]=errRate[j];
        }
        float trim = plan==&ddToDU ? reLearn.at(swingStep) : 0.f;
        u=r.u+trim;for(int j=1;j<6;j++)u-=r.k[j]*e[j];
        if(plan==&ddToDU){
            float angleFeedback=-(r.k[1]*e[1]+r.k[2]*e[2]+r.k[4]*e[4]+r.k[5]*e[5]);
            reLearn.observe(swingStep,angleFeedback,r.v,v,edgeResult==DuEdges::ok && fabsf(u)<16.f && fabsf(vCmd+u*dt)<MAX_SPEED_MPS);
        }
        swingLog[swingStep].trim=trim;
        ++swingStep;
    }else{
        const bool baseOver = fabsf(t1) > holdTiltLimitRad;
        const bool tipOver = fabsf(t2) > holdTiltLimitRad;
        if(baseOver || tipOver){
            stopAndDump();
            const char *trigger = baseOver && tipOver ? "base+tip" : (baseOver ? "base" : "tip");
            Serial.printf("OFF: tilt trigger=%s limit_deg=%.2f base_err_deg=%+.2f(HEDS%d) "
                          "tip_err_deg=%+.2f(HEDS%d+HEDS%d)\n",
                          trigger, degrees(holdTiltLimitRad), degrees(t1), BASE_CHANNEL + 1,
                          degrees(t2), BASE_CHANNEL + 1, (1 - BASE_CHANNEL) + 1);
            return;
        }
        const float *K6=mode->gain;
        u=-(K6[1]*t1+K6[2]*t2+K6[3]*v+K6[4]*w1+K6[5]*w2);
    }
    float limit=swinging?16.f:MAX_ACCEL_MPS2;
    float aLimited=constrain(u,-limit,limit),requestedV=vCmd+aLimited*dt;
    vCmd=constrain(requestedV,-MAX_SPEED_MPS,MAX_SPEED_MPS);
    if(swinging)swingLog[swingStep-1].nextCmd=vCmd;
#ifdef UPRIGHT_TRACE
    if(!swinging && used<sizeof(rows)/sizeof(rows[0]))rows[used++]={now-traceStart,us,t1,t2,x,v,u,aLimited,vCmd,requestedV!=vCmd,estimatedHeds};
#endif
    writeRegister(REG_VMAX,velocityToInternal(lroundf(fabsf(vCmd)*SCALE)));
    writeRegister(REG_RAMPMODE,vCmd>=0?1:2);
    // Best effort, 20 Hz. Never wait for serial bandwidth in the control loop.
    static uint32_t lastPose;
    if(now-lastPose>=50000u && Serial.availableForWrite()>=72){
        lastPose=now;
        float base=t1,tip=t2;
        if(!swinging){base=remainderf(base+mode->base,2*PI);tip=remainderf(tip+mode->tip,2*PI);}
        Serial.printf("POSE,%lu,%c,%.5f,%.5f,%.5f\n",(unsigned long)now,swinging?plan->phase:mode->key,x,base,tip);
    }
}
