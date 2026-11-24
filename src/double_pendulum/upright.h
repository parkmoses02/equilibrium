#pragma once
#include "encoder_config.h"
#include "revl_drive.h"
#include "du_frames.h"
#include "ud_frames.h"
#include "uu_frames.h"
#include "ud_du_frames.h"
#include "du_ud_frames.h"
#include "uu_dd_frames.h"
#include "ud_dd_frames.h"
#include "du_dd_frames.h"
#include "uu_ud_frames.h"
#include "uu_du_frames.h"
#include "du_uu_frames.h"
#include "ud_uu_frames.h"
#include "re_learning.h"
#include "rail_guard.h"
#include "state_estimator.h"
constexpr float ARM_TILT=0.05f;
constexpr float DU_CATCH_ANGLE_RAD=0.10f;
constexpr float DU_CATCH_SPEED_MPS=0.30f; // inclusive; terminal ramp speed, not the target
// Hold-to-hold transitions: 'e' typed during UD hold -> DU, 'w' typed during DU hold -> UD.
// A transition starts only after the hold meets all of these for TRANSITION_READY_US in a
// row; it gives up after TRANSITION_ARM_TIMEOUT_US. Rates for the readiness and catch tests
// are low-passed (tau 0.1 s): single encoder count steps make 0.4-0.5 rad/s spikes in the
// EKF rate. Verified in simulation/sysid/sim-transition.cpp.
constexpr float TRANSITION_READY_ANGLE_RAD=radians(1.0f), TRANSITION_READY_RATE_RAD_S=0.20f;
constexpr float TRANSITION_READY_SPEED_MPS=0.05f, TRANSITION_READY_X_M=0.05f;
constexpr uint32_t TRANSITION_READY_US=1000000u, TRANSITION_ARM_TIMEOUT_US=10000000u;
constexpr float READY_RATE_TAU_S=0.10f;
// EKF process noise: 30 rad/s^2 during swings and transitions, 4 in holds. With 30 in a hold,
// single count steps put 0.4-0.5 rad/s spikes in the rates; times the DU gains (tip rate -27.6)
// that made the cart command jump about 12 m/s^2 per frame and hit the 12 m/s^2 limit in
// 6-11% of frames (100 Hz chatter). With 4: about 4 m/s^2, no saturation, same tilt; all
// transitions equal or better (simulation/sysid/sim-hold.cpp dugain, sim-transition.cpp).
constexpr float SWING_EKF_ACCEL_STD=30.0f, HOLD_EKF_ACCEL_STD=4.0f;
// UU never sits still: pivot Coulomb friction keeps a limit cycle of about +-2.5 deg and
// +-0.13 m/s. 'w' during UU hold (UU->UD) therefore waits for 1 s inside a wider window
// (simulation/sysid/sim-transition.cpp uuready: the 1 deg / 0.05 m/s window never opens).
constexpr float UU_READY_ANGLE_RAD=radians(4.0f), UU_READY_RATE_RAD_S=0.50f, UU_READY_SPEED_MPS=0.20f;
constexpr float UD_CATCH_ANGLE_RAD = 0.10f;
constexpr float UD_CATCH_RATE_RAD_S = 0.50f;
constexpr float UD_CATCH_SPEED_MPS = 0.30f;
constexpr float UU_CORE_ANGLE_RAD = 0.05f, UU_CORE_SPEED_MPS = 0.20f;
constexpr float UU_CATCH_ANGLE_RAD = 0.10f, UU_CATCH_SPEED_MPS = 0.30f;
constexpr float UU_CATCH_RATE_RAD_S = 0.50f;
constexpr float UU_EXTENDED_LINK_DIFF_RAD = 0.01f, UU_EXTENDED_RATE_RAD_S = 0.05f;
constexpr float MANUAL_HOLD_TILT_RAD = 0.17f;
constexpr float DU_POST_CATCH_TILT_RAD = radians(20.0f);
constexpr float MAX_ACCEL_MPS2=12.0f, MAX_SPEED_MPS=0.80f;
constexpr float CART_TRAVEL_LIMIT_M=0.30f;
// UU hold only: LQI on the sysid-fitted model, Q=(12,60,60,2,2,2), R=0.3, integral weight 4
// (check_model.py uu_hold_gain; simulation/sysid/sim-uu-swing.cpp hold). The hold adds
// -K_q[0]*x - KI_Q*int(x) so the cart returns to z: the integral absorbs the
// upright-angle bias that drove the cart off the rail after the catch.
// Previous nominal LQR, no x term: {3.464102,-116.851324,140.474145,5.061349,-4.628502,15.246215}
constexpr float K_q[]={11.105604f,-147.661261f,198.038011f,11.411039f,-4.511831f,21.653094f};
constexpr float KI_Q=3.651484f;
constexpr float UU_INTEGRAL_LIMIT_MPS2=1.5f; // anti-windup, about 1.5 deg of angle bias
constexpr float K_w[]={-3.464102f,-52.422759f,17.225801f,-4.465170f,-4.803921f,-0.558010f};
constexpr float K_e[]={-14.832397f,153.628010f,-244.278154f,-17.451330f,11.412767f,-27.573131f};
struct Equilibrium {char key;const char *name;float base,tip;const float *gain;};
// DD damping LQR, used only by the soft stop (check_model.py dd_gain: fitted model, Q=(12,60,60,2,2,2), R=1).
// DD is stable but its two modes are almost undamped; this damps them in a few tenths of a second.
constexpr float K_r[]={3.464102f,21.548065f,-9.099465f,4.209856f,2.398130f,0.873836f};
// Soft stop (x during a UU/UD/DU hold): descend to DD along a designed table, damp at DD,
// then motor off once still. simulation/sysid/sim-stop.cpp: 150/150 settled within 3.9 s,
// peak link rate 13-14 rad/s instead of 18-38 rad/s (and still swinging after 12 s) for motor-off.
constexpr float DD_SETTLE_ANGLE_RAD=radians(2.0f), DD_SETTLE_RATE_RAD_S=0.20f;
constexpr uint32_t DD_SETTLE_US=500000u, SOFT_STOP_DAMP_TIMEOUT_US=8000000u;
constexpr float DD_DAMP_TILT_RAD=radians(60.0f);
constexpr Equilibrium modes[]={{'q',"UU",0,0,K_q},{'w',"UD",0,PI,K_w},{'e',"DU",PI,0,K_e},{'r',"DD",PI,PI,K_r}};
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
// Angles and rates come from the EKF. An invalid encoder edge no longer stops
// the run: that channel runs on prediction until the slip is resolved.
static est::AngleEKF kf;
static est::ChannelGuard guard[2]; // [0]=base HEDS, [1]=elbow HEDS
static constexpr int GUARD_ENCODER[2]={BASE_CHANNEL,1-BASE_CHANNEL};
static constexpr int GUARD_CPR[2]={BASE_CPR,REL_CPR};
static float rampVPrev;
static bool kfPrimed=false;
static uint32_t tick;
static float prev1,prev2,w1,w2,vCmd;
static bool swinging=false;
static float holdTiltLimitRad = MANUAL_HOLD_TILT_RAD;
static float uuXIntegral=0; // m*s, UU hold only; cleared at p and at catch
static unsigned swingStep=0;
static uint32_t swingStart=0;
static constexpr unsigned DU_COUNT=sizeof(duFrames)/sizeof(duFrames[0]);
static_assert(DU_COUNT==336,"5 ms DD->DU trajectory expected");
static constexpr unsigned UU_COUNT=sizeof(uuFrames)/sizeof(uuFrames[0]);
static constexpr unsigned UD_COUNT=sizeof(udFrames)/sizeof(udFrames[0]);
struct SwingPlan { const DuFrame *frames; unsigned count; char phase; const char *csv; };
static constexpr SwingPlan ddToDU{duFrames, DU_COUNT, 's', "RE"};
static constexpr SwingPlan ddToUU{uuFrames, UU_COUNT, 'u', "RQ"};
static constexpr SwingPlan ddToUD{udFrames, UD_COUNT, 'd', "RW"};
static constexpr unsigned UD_DU_COUNT=sizeof(udDuFrames)/sizeof(udDuFrames[0]);
static constexpr SwingPlan udToDU{udDuFrames, UD_DU_COUNT, 't', "WE"};
static constexpr unsigned DU_UD_COUNT=sizeof(duUdFrames)/sizeof(duUdFrames[0]);
static constexpr SwingPlan duToUD{duUdFrames, DU_UD_COUNT, 'v', "EW"};
static constexpr unsigned UU_DD_COUNT=sizeof(uuDdFrames)/sizeof(uuDdFrames[0]);
static constexpr unsigned UD_DD_COUNT=sizeof(udDdFrames)/sizeof(udDdFrames[0]);
static constexpr unsigned DU_DD_COUNT=sizeof(duDdFrames)/sizeof(duDdFrames[0]);
static constexpr SwingPlan uuToDD{uuDdFrames, UU_DD_COUNT, 'n', "QR"};
static constexpr SwingPlan udToDD{udDdFrames, UD_DD_COUNT, 'n', "WR"};
static constexpr SwingPlan duToDD{duDdFrames, DU_DD_COUNT, 'n', "ER"};
static constexpr unsigned UU_UD_COUNT=sizeof(uuUdFrames)/sizeof(uuUdFrames[0]);
static constexpr SwingPlan uuToUD{uuUdFrames, UU_UD_COUNT, 'y', "QW"};
static constexpr unsigned UU_DU_COUNT=sizeof(uuDuFrames)/sizeof(uuDuFrames[0]);
static constexpr SwingPlan uuToDU{uuDuFrames, UU_DU_COUNT, 'z', "QE"};
static constexpr unsigned DU_UU_COUNT=sizeof(duUuFrames)/sizeof(duUuFrames[0]);
static constexpr SwingPlan duToUU{duUuFrames, DU_UU_COUNT, 'k', "EQ"};
static constexpr unsigned UD_UU_COUNT=sizeof(udUuFrames)/sizeof(udUuFrames[0]);
static constexpr SwingPlan udToUU{udUuFrames, UD_UU_COUNT, 'h', "WQ"};
static_assert(UU_DD_COUNT<=DU_UD_COUNT && UD_DD_COUNT<=DU_UD_COUNT && DU_DD_COUNT<=DU_UD_COUNT
              && UU_UD_COUNT<=DU_UD_COUNT && UU_DU_COUNT<=DU_UD_COUNT && DU_UU_COUNT<=DU_UD_COUNT && UD_UU_COUNT<=DU_UD_COUNT,"swingLog size");
static const SwingPlan *plan = &ddToDU;
static ReLearning reLearn;
static_assert(ReLearning::bins*ReLearning::width==DU_COUNT,"RE learning frame count");
// One sample per swing frame plus the catch decision. No serial I/O here.
struct SwingSample { uint32_t us; float base,tip,wb,wt,x,v,priorCmd,nextCmd,trim; uint8_t estimatedHeds; };
static constexpr unsigned maxFrames(unsigned a, unsigned b){return a > b ? a : b;}
static SwingSample swingLog[maxFrames(DU_UD_COUNT, maxFrames(UD_DU_COUNT, maxFrames(UD_COUNT, maxFrames(UU_COUNT, DU_COUNT))))+1];
// Hold-to-hold transition arming (UD->DU or DU->UD) and low-passed rates
// (readiness and transition catch tests only, never control).
static const SwingPlan *armedPlan=nullptr;
static char armedDest=0;
static uint32_t armStart=0, armStableUs=0;
static float rateLp1=0, rateLp2=0;
static bool holdTransition(const SwingPlan *p){return p==&udToDU || p==&duToUD || p==&uuToUD || p==&uuToDU || p==&duToUU || p==&udToUU;}
static bool descent(const SwingPlan *p){return p==&uuToDD || p==&udToDD || p==&duToDD;}
// Soft stop state: requested by x during a hold, then DD damping until still.
static bool softStopRequested=false, softStopping=false;
// DD wait ('r' during a UU/UD/DU hold): the same descent, then DD damping with the motor on
// until q/w/e (or x). A requested swing starts once DD is still and centred for DD_SETTLE_US.
static bool softStopWait=false, ddWait=false;
static char ddSwingDest=0;
static uint32_t ddSwingAsked=0, ddReadyUs=0;
static uint32_t softStopStart=0, ddStillUs=0;
static unsigned swingUsed;
// The terminal swing sample stays in RAM throughout hold, including its pre-handoff rates.
static const SwingSample *caughtFrame=nullptr;
static float swingOrigin,brakeX,brakeV;
static uint32_t brakeStart;
static bool railBraking=false;
// Apply a bounded fraction of the position term already designed with the angle/velocity gains.
static float positionFeedback(float gain,float error,float accelLimit){
    return constrain(-.5f*gain*error,-accelLimit,accelLimit);
}
static void dumpSwing(){
    if(!swingUsed)return;
    Serial.printf("%s_CSV_BEGIN\n",plan->csv);
    Serial.println("step,t_us,base_rad,tip_rad,base_rad_s,tip_rad_s,ramp_x_m,ramp_v_m_s,ref_v_m_s,prev_cmd_v_m_s,next_cmd_v_m_s,learned_trim_m_s2,estimated_heds");
    for(unsigned i=0;i<swingUsed;i++){
        const auto &s=swingLog[i];
        Serial.printf("%u,%lu,%.6f,%.6f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.5f,%u\n",i,
            (unsigned long)s.us,s.base,s.tip,s.wb,s.wt,s.x,s.v,i<plan->count?plan->frames[i].v:0.f,s.priorCmd,s.nextCmd,s.trim,unsigned(s.estimatedHeds));
    }
    Serial.printf("%s_CSV_END\n",plan->csv);swingUsed=0;
}
// Reference link rates from the 5 ms swing table (central differences).
static float refRate(const SwingPlan &p,unsigned i,bool tip){
    const unsigned lo=i?i-1:0, hi=i+1<p.count?i+1:p.count-1;
    const float a=tip?p.frames[lo].tip:p.frames[lo].base, b=tip?p.frames[hi].tip:p.frames[hi].base;
    return remainderf(b-a,2*PI)/((hi-lo)*.005f);
}
static void reportRecovery(){
    for(int ch=0;ch<2;ch++){
        const auto &g=guard[ch];
        if(g.fixed||g.unresolved||g.rejected||g.lost)
            Serial.printf("ENC_RECOVERY HEDS%d resolved=%u unresolved=%u rejected=%u worst_step_miss=%.2f lost=%d\n",
                          GUARD_ENCODER[ch]+1,g.fixed,g.unresolved,g.rejected,g.worstMiss,int(g.lost));
    }
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
    active=swinging=railBraking=false;
    softStopRequested=softStopping=false;armedPlan=nullptr;
    softStopWait=ddWait=false;ddSwingDest=0;
    if(wasActive)reportRecovery();
    const bool unresolvedSlip=guard[0].unresolved||guard[1].unresolved||guard[0].lost||guard[1].lost;
    if(referenced && (enc[0].bad!=errors[0] || enc[1].bad!=errors[1] || unresolvedSlip)){
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
static void stopDouble(const char *code, const char *why, float missScore=-1.f,
                       const char *detailFormat=nullptr, Args... detailArgs){
    stopAndDump(missScore);
    if(detailFormat) Serial.printf(detailFormat, detailArgs...);
    if(why) Serial.println(why);
    if(caughtFrame){
        const auto &s=*caughtFrame;
        Serial.printf("CATCH_ENTRY pose=%s t_us=%lu base_deg=%+.3f tip_deg=%+.3f "
                      "base_err_deg=%+.3f tip_err_deg=%+.3f base_rad_s=%+.5f tip_rad_s=%+.5f ramp_v_m_s=%+.6f\n",
                      mode->name,(unsigned long)s.us,degrees(s.base),degrees(s.tip),
                      degrees(remainderf(s.base-mode->base,2*PI)),degrees(remainderf(s.tip-mode->tip,2*PI)),
                      s.wb,s.wt,s.v);
        caughtFrame=nullptr;
    }
    Serial.printf("STOP_CODE=%s\n",code);
}
static void absoluteAngles(const EncoderFrame &frame,float &base,float &tip){
    absoluteAnglesFrom(frame,offsets,base,tip);
}
static void subtractTargetAngles(float &base,float &tip){
    base=remainderf(base-mode->base,2*PI);
    tip=remainderf(tip-mode->tip,2*PI);
}
static bool poseOK(float t){return fabsf(t)<=ARM_TILT;}
// Inputs are wrapped errors from UD (base up, tip down) and filtered absolute rates.
static bool udCatchReady(float base, float tip, float baseRate, float tipRate, float cartSpeed){
    return fabsf(base) <= UD_CATCH_ANGLE_RAD && fabsf(tip) <= UD_CATCH_ANGLE_RAD
        && fabsf(baseRate) < UD_CATCH_RATE_RAD_S && fabsf(tipRate) < UD_CATCH_RATE_RAD_S
        && fabsf(cartSpeed) <= UD_CATCH_SPEED_MPS;
}
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
    Serial.println("STOP_CODE=START_POSE_OUT_OF_RANGE");
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
    setupEncoders();
    Serial.println("Enter one command per line: z=zero DD at centre; q/w/e/r=UU/UD/DU/DD; p=start selected hold (q/w/e).");
    Serial.println("rw=DD->UD; re=DD->DU; rq=DD->UU; x=OFF. Each start keeps the cart origin from z.");
    Serial.println("During a hold: e = UD->DU or UU->DU; w = DU->UD or UU->UD; q = DU->UU or UD->UU. Each starts once the hold has been");
    Serial.println("still and centred for 1 s (gives up after 10 s).");
    Serial.println("x during a UU/UD/DU hold = soft stop: descend to DD, damp, then OFF. x again (or x in a swing) = OFF now.");
    Serial.println("r during a UU/UD/DU hold = descend to DD and wait with the motor on; then q/w/e swings up again.");
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
            if(c!='r'||(dest!='e' && dest!='q' && dest!='w')){
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
            // K_r is for the soft stop only; manual p at DD stays blocked.
            if(!dd && (mode->key=='r' || !mode->gain)){Serial.println("BLOCK: DD hold not implemented; motor OFF.\nSTOP_CODE=NO_HOLD_CONTROLLER");return;}
            if(!referenced || !BASE_CPR || !REL_CPR){Serial.println("BLOCK: need signed CPR and z at hanging DD.\nSTOP_CODE=REFERENCE_REQUIRED");return;}
            if(enc[0].bad!=errors[0] || enc[1].bad!=errors[1]){stopDouble("ENCODER_EDGES_BEFORE_START","BLOCK: encoder edges");return;}
            if(!startPoseOK(dd))return;
            long before[]={enc[0].pos,enc[1].pos};
            if(!prepareRevl(false)){stopDouble("DRIVER_PREFLIGHT","BLOCK: driver preflight failed.");return;}
            const float x=rampPositionFromZero();
            if(fabsf(x)>CART_TRAVEL_LIMIT_M){
                stopDouble("RAMP_POSITION_LIMIT","BLOCK: ramp position", -1.f, "x_from_z_mm=%+.2f limit_mm=300\n", x*1000.f);return;
            }
            while(Serial.available()){if(Serial.read()=='x'){stopDouble("MANUAL_STOP","OFF during preflight");return;}}
            if(enc[0].bad!=errors[0] || enc[1].bad!=errors[1]){stopDouble("ENCODER_EDGES_PREFLIGHT","OFF: encoder error; reference cleared.");return;}
            if(!startPoseOK(dd))return;
            if(dd && (labs(enc[0].pos-before[0])>3 || labs(enc[1].pos-before[1])>3)){
                Serial.println("BLOCK: DD moved during preflight; let links settle.\nSTOP_CODE=DD_MOVED_PREFLIGHT");return;}
            if(dd){
                const SwingPlan *route=dest=='q' ? &ddToUU : (dest=='w' ? &ddToUD : &ddToDU);
                if(dest!='q'){
                    for(unsigned i=0;i<route->count;++i){
                        const auto &ref=route->frames[i];
                        if(RailGuard::needsBrake(x+ref.x,ref.v)){
                            stopDouble("RAIL_START_MARGIN","BLOCK: insufficient rail for this swing", -1.f,
                                       "start_x_mm=%+.2f ref_step=%u\n",x*1000.f,i);return;
                        }
                    }
                }
                swingOrigin=x; // UD and UU swings track the cart table from here
                mode=poseFor(dest);plan=route;
                swingStep=0;
                if(plan==&ddToDU)reLearn.begin();
            }
            swinging=dd;
            holdTiltLimitRad = MANUAL_HOLD_TILT_RAD;
            kf.reset(prev1,prev2);kfPrimed=false; // absolute angles from startPoseOK
            for(int ch=0;ch<2;ch++){guard[ch]=est::ChannelGuard{};guard[ch].seenBad=errors[GUARD_ENCODER[ch]];}
            rampLimit(dd?16.f:MAX_ACCEL_MPS2);
            writeRegister(REG_RAMPMODE,1);writeRegister(REG_VMAX,0);
            tick=micros();w1=w2=vCmd=0;uuXIntegral=0;active=true;digitalWrite(PIN_EN,LOW);
            armedPlan=nullptr;rateLp1=rateLp2=0;softStopRequested=softStopping=false;
            softStopWait=ddWait=false;ddSwingDest=0;
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
        if(ch=='x'){
            len=0;discard=true;
            // First x during a UU/UD/DU hold: soft stop (the control frame starts the descent).
            // Anything else, including x again: motor off at once.
            if(active && !swinging && !softStopRequested && !softStopping && mode->key!='r'){
                softStopRequested=true;armedPlan=nullptr;
                Serial.printf("SOFT_STOP %s->DD requested (x again = OFF now).\n",mode->name);
            }else stopDouble("MANUAL_STOP","OFF");
            return;
        }
        if(ch=='r' && active && !swinging && !softStopRequested && !softStopping && !ddWait && mode->key!='r'){
            // Same descent as the soft stop, but stay on at DD afterwards.
            softStopRequested=softStopWait=true;armedPlan=nullptr;
            Serial.printf("DD_WAIT %s->DD requested (x = OFF now).\n",mode->name);
            len=0;discard=true;return;
        }
        if(active && ddWait && !swinging){
            // DD wait: single keys like the other hold transitions. q/w/e = DD->UU/UD/DU,
            // started once DD is still and centred (as from rest).
            if(ch=='q'||ch=='w'||ch=='e'){
                if(!ddSwingDest){
                    ddSwingDest=ch;ddSwingAsked=micros();ddReadyUs=0;
                    Serial.printf("DD->%s armed: waiting for still, centred DD (x stops).\n",poseFor(ch)->name);
                }
            }else if(ch!='\r' && ch!='\n')Serial.println("DD wait: q / w / e to swing up to UU / UD / DU, x = OFF.");
            len=0;discard=false;
            return;
        }
        const bool toDU = ch=='e' && (mode->key=='w' || mode->key=='q'), toUD = ch=='w' && (mode->key=='e' || mode->key=='q');
        const bool toUU = ch=='q' && (mode->key=='e' || mode->key=='w');
        if(active && !swinging && (toDU || toUD || toUU)){
            // Arm the transition; doubleLoop starts it once the hold is still and centred.
            if(!armedPlan){
                const bool fromUU = mode->key=='q';
                armedPlan=toUU ? (mode->key=='w' ? &udToUU : &duToUU) : toDU ? (fromUU ? &uuToDU : &udToDU) : (fromUU ? &uuToUD : &duToUD);
                armedDest=ch;armStart=micros();armStableUs=0;
                Serial.printf("%s->%s armed: waiting for 1 s of still, centred %s (x stops).\n",
                              mode->name,poseFor(ch)->name,mode->name);
            }
            discard=true;return;
        }
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
        stopDouble("LOOP_DEADLINE","OFF: loop deadline", -1.f, "Loop interval: %lu us\n", (unsigned long)us);
        return;
    }
#ifdef UPRIGHT_TRACE
    if(!railBraking && !swinging && now-traceStart>=TRACE_US){stopDouble("TRACE_COMPLETE","OFF: trace duration reached");return;}
#endif
    if(us<5000)return;
    tick=now;float dt=us*1e-6f;
    uint32_t id=readRegister(REG_IOIN),drv=readRegister(REG_DRV_STATUS);
    float x=rampPositionFromZero();
    int32_t vr;
    float v=rampVelocity(vr);
    if(!safeChip(id,drv)||(id&IOIN_DRV_ENN)||(readRegister(REG_GSTAT)&7)){stopDouble("DRIVER_FAULT","OFF: driver");return;}
    if(fabsf(x)>CART_TRAVEL_LIMIT_M){
        stopDouble("RAMP_POSITION_LIMIT","OFF: ramp position", -1.f, "x_from_z_mm=%+.2f limit_mm=300\n", x*1000.f);return;
    }
    const bool guardedRoute=(swinging || caughtFrame || softStopping) && (plan==&ddToUD || plan==&ddToDU || holdTransition(plan) || descent(plan));
    if(guardedRoute && (railBraking || RailGuard::needsBrake(x,v) || RailGuard::needsBrake(x,vCmd))){
        if(!railBraking){
            railBraking=true;brakeX=x;brakeV=v;brakeStart=now;
            rampLimit(8.f,true);
            writeRegister(REG_VMAX,0); // stay enabled until the ramp reaches standstill
        }
        if(vr==0 || now-brakeStart>=500000u){
            stopDouble(vr==0?"RAIL_BRAKE":"RAIL_BRAKE_TIMEOUT","OFF: rail braking", -1.f,
                       "brake_x_mm=%+.2f brake_v_m_s=%+.4f final_x_mm=%+.2f final_v_m_s=%+.4f\n",
                       brakeX*1000.f,brakeV,x*1000.f,v);
        }
        return;
    }
    // EKF: the ramp's velocity change over the last frame is the cart
    // acceleration that actually drove the links during it.
    kf.accelStd = swinging ? SWING_EKF_ACCEL_STD : HOLD_EKF_ACCEL_STD;
    kf.predict(kfPrimed?(v-rampVPrev)/dt:0.f,dt);
    rampVPrev=v;kfPrimed=true;
    const auto frame=readEncoderFrame();
    float raw1,raw2;
    absoluteAngles(frame,raw1,raw2);
    const float channel[2]={raw1,remainderf(raw2-raw1,2*PI)}; // base, elbow
    uint8_t estimatedHeds=0; // bit0=HEDS1, bit1=HEDS2 ran on prediction this frame
    for(int ch=0;ch<2;ch++){
        const int e=GUARD_ENCODER[ch];
        const auto step=est::processChannel(kf,ch,channel[ch],frame.bad[e],GUARD_CPR[ch],guard[ch]);
        offsets[e]-=step.countShift;
        errors[e]=frame.bad[e]; // accounted for; unresolved slips clear the reference at stop
        if(step.reading!=est::Reading::measured)estimatedHeds|=1u<<e;
        if(est::channelLost(guard[ch])){
            stopDouble("ENCODER_LOST","OFF: encoder lost beyond the prediction window", -1.f,
                       "HEDS%d pending_frames=%u slip_frames=%u reject_streak=%u\n",e+1,
                       guard[ch].pendingFrames,guard[ch].slipFrames,guard[ch].rejectStreak);
            return;
        }
    }
    float t1=kf.s[0],t2=kf.s[1];
    w1=kf.s[2];w2=kf.s[3];
    {   // Low-passed rates: transition readiness and catch tests only, never control.
        const float a=fminf(1.f,dt/READY_RATE_TAU_S);
        rateLp1+=a*(w1-rateLp1);rateLp2+=a*(w2-rateLp2);
    }
    if(!swinging)subtractTargetAngles(t1,t2);
    if(swinging && swingStep<=plan->count){
        swingLog[swingStep]={now-swingStart,t1,t2,w1,w2,x,v,vCmd,NAN,NAN,estimatedHeds};
        swingUsed=swingStep+1;
    }
    if(swinging && now-swingStart>(swingStep+1)*5000u+15000u){stopDouble("SWING_TIMING","OFF: swing timing");return;}
    if(swinging && swingStep==plan->count){
        // Keep terminal velocity for one tick; confirm catch on a measured frame.
        if(estimatedHeds)return;
        t1=remainderf(t1-mode->base,2*PI);
        t2=remainderf(t2-mode->tip,2*PI);
        if(plan == &ddToUD){
            if(!udCatchReady(t1, t2, w1, w2, v)){
                stopDouble("MISSED_UD","OFF: missed UD; no retry", -1.f,
                           "UD err_deg=%+.3f,%+.3f rate_rad_s=%+.3f,%+.3f ramp_v=%+.4f ramp_x=%+.4f\n",
                           degrees(t1), degrees(t2), w1, w2, v, x);
                return;
            }
        }else if(plan == &ddToUU){
            if(!uuCatchReady(t1, t2, w1, w2, v)){
                stopDouble("MISSED_UU","OFF: missed UU; no retry", -1.f,
                           "UU err_deg=%+.3f,%+.3f rate_rad_s=%+.3f,%+.3f ramp_v=%+.4f ramp_x=%+.4f\n",
                           degrees(t1), degrees(t2), w1, w2, v, x);
                return;
            }
        }else if(descent(plan)){
            // Soft stop reached the end of its table: no catch test, damp at DD until still.
            softStopping=true;softStopStart=now;ddStillUs=0;
            holdTiltLimitRad=DD_DAMP_TILT_RAD;
            if(softStopWait){softStopping=softStopWait=false;ddWait=true;
                Serial.println("DD_WAIT: motor on, damping at DD. q/w/e to swing up to UU/UD/DU, x = OFF.");}
        }else if(plan == &duToUD || plan == &uuToUD){
            // Same UD limits as DD->UD, but on low-passed rates (count-step spikes).
            if(!udCatchReady(t1, t2, rateLp1, rateLp2, v)){
                const bool fromUU = plan == &uuToUD;
                stopDouble(fromUU ? "MISSED_UD_FROM_UU" : "MISSED_UD_FROM_DU",
                           fromUU ? "OFF: missed UD after UU->UD; no retry" : "OFF: missed UD after DU->UD; no retry", -1.f,
                           "UD err_deg=%+.2f,%+.2f rate_lp_rad_s=%+.2f,%+.2f ramp_v=%+.3f ramp_x=%+.4f\n",
                           degrees(t1), degrees(t2), rateLp1, rateLp2, v, x);
                return;
            }
            holdTiltLimitRad = MANUAL_HOLD_TILT_RAD; // the DU hold may have widened it to 20 deg
        }else if(plan == &duToUU || plan == &udToUU){
            // Arriving at UU from DU or UD: the transition limits (0.10 rad, low-passed rate < 0.5 rad/s,
            // ramp <= 0.30 m/s), not uuCatchReady. In simulation the stricter DD->UU rule rejected
            // every catch near 3-4 deg that the UU LQI hold then kept (sim-transition.cpp).
            const bool anglesOK = fabsf(t1) <= UU_CATCH_ANGLE_RAD && fabsf(t2) <= UU_CATCH_ANGLE_RAD;
            if(!anglesOK||fabsf(rateLp1)>=UU_CATCH_RATE_RAD_S||fabsf(rateLp2)>=UU_CATCH_RATE_RAD_S||fabsf(v)>UU_CATCH_SPEED_MPS){
                const bool fromUD = plan == &udToUU;
                stopDouble(fromUD ? "MISSED_UU_FROM_UD" : "MISSED_UU_FROM_DU",
                           fromUD ? "OFF: missed UU after UD->UU; no retry" : "OFF: missed UU after DU->UU; no retry", -1.f,
                           "UU err_deg=%+.2f,%+.2f rate_lp_rad_s=%+.2f,%+.2f ramp_v=%+.3f ramp_x=%+.4f\n",
                           degrees(t1), degrees(t2), rateLp1, rateLp2, v, x);
                return;
            }
            holdTiltLimitRad = MANUAL_HOLD_TILT_RAD; // a DU hold may have widened it to 20 deg
        }else if(plan == &udToDU || plan == &uuToDU){
            // Same DU limits as DD->DU, but on low-passed rates (count-step spikes).
            const bool anglesOK = fabsf(t1) <= DU_CATCH_ANGLE_RAD && fabsf(t2) <= DU_CATCH_ANGLE_RAD;
            if(!anglesOK||fabsf(rateLp1)>=.5f||fabsf(rateLp2)>=.5f||fabsf(v)>DU_CATCH_SPEED_MPS){
                const bool fromUU = plan == &uuToDU;
                stopDouble(fromUU ? "MISSED_DU_FROM_UU" : "MISSED_DU_FROM_UD",
                           fromUU ? "OFF: missed DU after UU->DU; no retry" : "OFF: missed DU after UD->DU; no retry", -1.f,
                           "DU err_deg=%+.2f,%+.2f rate_lp_rad_s=%+.2f,%+.2f ramp_v=%+.3f ramp_x=%+.4f\n",
                           degrees(t1), degrees(t2), rateLp1, rateLp2, v, x);
                return;
            }
            holdTiltLimitRad = DU_POST_CATCH_TILT_RAD;
        }else{
            const bool anglesOK = fabsf(t1) <= DU_CATCH_ANGLE_RAD && fabsf(t2) <= DU_CATCH_ANGLE_RAD;
            if(!anglesOK||fabsf(w1)>=.5f||fabsf(w2)>=.5f||fabsf(v)>DU_CATCH_SPEED_MPS){
                float score=fmaxf(fmaxf(fabsf(t1),fabsf(t2))/DU_CATCH_ANGLE_RAD,
                    fmaxf(fmaxf(fabsf(w1),fabsf(w2))/.5f,fabsf(v)/DU_CATCH_SPEED_MPS));
                stopDouble("MISSED_DU","OFF: missed DU; no retry", score,
                           "DU err_deg=%.2f,%.2f rate_rad_s=%.2f,%.2f ramp_v=%.3f\n",
                           degrees(t1), degrees(t2), w1, w2, v);
                return;
            }
            reLearn.catchDU();
            holdTiltLimitRad = DU_POST_CATCH_TILT_RAD;
        }
        caughtFrame=&swingLog[swingStep];
        swinging=false;vCmd=v;uuXIntegral=0;rampLimit(MAX_ACCEL_MPS2);
        Serial.printf("CATCH %s -> %c\n", mode->name, mode->key);
#ifdef UPRIGHT_TRACE
        traceStart=now;used=0;
#endif
    }
    float u;
    bool startTransition=false; // set by an armed transition or a soft stop below; takes effect after this frame
    if(swinging){
        const auto &r=plan->frames[swingStep];
        float e[]={0.f,remainderf(t1-r.base,2*PI),remainderf(t2-r.tip,2*PI),v-r.v,
                   w1-refRate(*plan,swingStep,false),w2-refRate(*plan,swingStep,true)};
        if((fabsf(e[1])>.45f || fabsf(e[2])>.45f) && descent(plan)){
            // Soft stop lost the table: keep the motor on and damp at DD instead of dropping the links.
            swinging=false;softStopping=true;softStopStart=now;ddStillUs=0;
            holdTiltLimitRad=DD_DAMP_TILT_RAD;rampLimit(MAX_ACCEL_MPS2);
            Serial.printf("SOFT_STOP table lost at step %u; damping at DD.\n",swingStep);
            if(softStopWait){softStopping=softStopWait=false;ddWait=true;
                Serial.println("DD_WAIT: motor on, damping at DD. q/w/e to swing up to UU/UD/DU, x = OFF.");}
            return; // the previous velocity command stays for this one frame
        }
        if(fabsf(e[1])>.45f || fabsf(e[2])>.45f){
            stopDouble("SWING_TRACKING_ERROR","OFF: swing tracking error", -1.f,
                       "step=%u x_from_z_mm=%.1f base_err_deg=%.2f tip_err_deg=%.2f\n",
                       swingStep, 1000*x, degrees(e[1]), degrees(e[2]));
            return;
        }
        float trim = plan==&ddToDU ? reLearn.at(swingStep) : 0.f;
        u=r.u+trim;for(int j=1;j<6;j++)u-=r.k[j]*e[j];
        // UU: position term added 2026-11-24 with the friction-aware table
        // (simulation/sysid/sim-uu-swing.cpp robust: 10/10 for J,d1 +-2%, friction 0.7-1.3x).
        if(plan==&ddToUD || plan==&ddToUU || holdTransition(plan) || descent(plan))u+=positionFeedback(r.k[0],x-(swingOrigin+r.x),1.f);
        if(plan==&ddToDU){
            float angleFeedback=-(r.k[1]*e[1]+r.k[2]*e[2]+r.k[4]*e[4]+r.k[5]*e[5]);
            reLearn.observe(swingStep,angleFeedback,r.v,v,!estimatedHeds && fabsf(u)<16.f && fabsf(vCmd+u*dt)<MAX_SPEED_MPS);
        }
        swingLog[swingStep].trim=trim;
        ++swingStep;
    }else{
        const bool baseOver = fabsf(t1) > holdTiltLimitRad;
        const bool tipOver = fabsf(t2) > holdTiltLimitRad;
        if(baseOver || tipOver){
            const char *trigger = baseOver && tipOver ? "base+tip" : (baseOver ? "base" : "tip");
            stopDouble("TILT_LIMIT",nullptr,-1.f,"OFF: tilt trigger=%s limit_deg=%.2f base_err_deg=%+.2f(HEDS%d) "
                          "tip_err_deg=%+.2f(HEDS%d+HEDS%d)\n",
                          trigger, degrees(holdTiltLimitRad), degrees(t1), BASE_CHANNEL + 1,
                          degrees(t2), BASE_CHANNEL + 1, (1 - BASE_CHANNEL) + 1);
            return;
        }
        const float *K6=mode->gain;
        u=-(K6[1]*t1+K6[2]*t2+K6[3]*v+K6[4]*w1+K6[5]*w2);
        if(mode->key=='q'){
            // UU (manual p and after DD->UU): full LQI position + integral term toward z.
            const float iMax=UU_INTEGRAL_LIMIT_MPS2/KI_Q;
            uuXIntegral=constrain(uuXIntegral+x*dt,-iMax,iMax);
            u+=-K6[0]*x-KI_Q*uuXIntegral;
        }else u+=-K6[0]*x;
        // UD/DU, manual p and after any swing catch: the designed LQR u=-Kx with the cart term.
        // simulation/sysid/sim-hold.cpp: manual 220/220 held (DU 0/110 without x);
        // from hardware catch states 70/70 for both this and the former clamp(-0.5*K0*x,+-1.5),
        // cart ends 2-23 mm from z instead of 4-46 mm.
        if(armedPlan){
            const auto *dest=poseFor(armedDest);
            const bool uu = mode->key=='q'; // UU limit-cycles: wider window
            const float aMax = uu ? UU_READY_ANGLE_RAD : TRANSITION_READY_ANGLE_RAD;
            const float wMax = uu ? UU_READY_RATE_RAD_S : TRANSITION_READY_RATE_RAD_S;
            const float vMax = uu ? UU_READY_SPEED_MPS : TRANSITION_READY_SPEED_MPS;
            const bool still = fabsf(t1)<=aMax && fabsf(t2)<=aMax
                && fabsf(rateLp1)<=wMax && fabsf(rateLp2)<=wMax
                && fabsf(v)<=vMax && fabsf(x)<=TRANSITION_READY_X_M && !estimatedHeds;
            armStableUs = still ? armStableUs+us : 0;
            if(armStableUs>=TRANSITION_READY_US){
                startTransition=true;
                for(unsigned i=0;i<armedPlan->count;++i){
                    const auto &ref=armedPlan->frames[i];
                    if(RailGuard::needsBrake(x+ref.x,ref.v)){
                        startTransition=false;armedPlan=nullptr;
                        Serial.printf("%s->%s refused: insufficient rail (x_mm=%+.1f ref_step=%u); %s hold continues.\n",
                                      mode->name,dest->name,x*1000.f,i,mode->name);
                        break;
                    }
                }
            }else if(now-armStart>TRANSITION_ARM_TIMEOUT_US){
                armedPlan=nullptr;
                Serial.printf("%s->%s not started: %s not still/centred for 1 s within 10 s "
                              "(err_deg=%+.2f,%+.2f rate_lp=%+.2f,%+.2f v=%+.3f x_mm=%+.1f heds=%u). %s hold continues; %c to retry.\n",
                              mode->name,dest->name,mode->name,degrees(t1),degrees(t2),rateLp1,rateLp2,v,x*1000.f,
                              unsigned(estimatedHeds),mode->name,armedDest);
            }
        }
        if(softStopRequested){
            // x during a UU/UD/DU hold: start the descent to DD now (no readiness wait).
            softStopRequested=false;
            const SwingPlan *down = mode->key=='q' ? &uuToDD : (mode->key=='w' ? &udToDD : &duToDD);
            for(unsigned i=0;i<down->count;++i){
                if(RailGuard::needsBrake(x+down->frames[i].x,down->frames[i].v)){
                    stopDouble("MANUAL_STOP","OFF: no rail room for a soft stop", -1.f,
                               "x_from_z_mm=%+.1f ref_step=%u\n",x*1000.f,i);
                    return;
                }
            }
            armedPlan=down;armedDest='r';startTransition=true;
        }
        if(softStopping){
            // Damping at DD after the descent: motor off once still for DD_SETTLE_US.
            const bool calm = fabsf(t1)<=DD_SETTLE_ANGLE_RAD && fabsf(t2)<=DD_SETTLE_ANGLE_RAD
                && fabsf(rateLp1)<=DD_SETTLE_RATE_RAD_S && fabsf(rateLp2)<=DD_SETTLE_RATE_RAD_S;
            ddStillUs = calm ? ddStillUs+us : 0;
            if(ddStillUs>=DD_SETTLE_US){stopDouble("SOFT_STOP","OFF: settled at DD");return;}
            if(now-softStopStart>SOFT_STOP_DAMP_TIMEOUT_US){stopDouble("SOFT_STOP_TIMEOUT","OFF: DD damping did not settle in 8 s");return;}
        }
        if(ddWait && ddSwingDest){
            // Requested swing from the DD wait: start once DD is still and centred, as from rest.
            const bool ready = fabsf(t1)<=DD_SETTLE_ANGLE_RAD && fabsf(t2)<=DD_SETTLE_ANGLE_RAD
                && fabsf(rateLp1)<=DD_SETTLE_RATE_RAD_S && fabsf(rateLp2)<=DD_SETTLE_RATE_RAD_S
                && fabsf(v)<=TRANSITION_READY_SPEED_MPS && fabsf(x)<=TRANSITION_READY_X_M && !estimatedHeds;
            ddReadyUs = ready ? ddReadyUs+us : 0;
            if(ddReadyUs>=DD_SETTLE_US){
                const SwingPlan *route = ddSwingDest=='q' ? &ddToUU : (ddSwingDest=='w' ? &ddToUD : &ddToDU);
                bool room=true;
                if(ddSwingDest!='q'){ // same rail check as an idle rw/re start
                    for(unsigned i=0;i<route->count;++i){
                        if(RailGuard::needsBrake(x+route->frames[i].x,route->frames[i].v)){
                            room=false;
                            Serial.printf("DD->%s refused: insufficient rail (x_mm=%+.1f ref_step=%u); DD wait continues.\n",
                                          poseFor(ddSwingDest)->name,x*1000.f,i);
                            break;
                        }
                    }
                }
                if(room){armedPlan=route;armedDest=ddSwingDest;startTransition=true;}
                ddSwingDest=0;
            }else if(now-ddSwingAsked>TRANSITION_ARM_TIMEOUT_US){
                Serial.printf("DD->%s not started: DD not still/centred for 0.5 s within 10 s "
                              "(err_deg=%+.2f,%+.2f rate_lp=%+.2f,%+.2f v=%+.3f x_mm=%+.1f). DD wait continues.\n",
                              poseFor(ddSwingDest)->name,degrees(t1),degrees(t2),rateLp1,rateLp2,v,x*1000.f);
                ddSwingDest=0;
            }
        }
    }
    float limit=swinging?16.f:MAX_ACCEL_MPS2;
    float aLimited=constrain(u,-limit,limit),requestedV=vCmd+aLimited*dt;
    vCmd=constrain(requestedV,-MAX_SPEED_MPS,MAX_SPEED_MPS);
    if(swinging)swingLog[swingStep-1].nextCmd=vCmd;
#ifdef UPRIGHT_TRACE
    if(!swinging && used<sizeof(rows)/sizeof(rows[0]))rows[used++]={now-traceStart,us,t1,t2,x,v,u,aLimited,vCmd,requestedV!=vCmd,estimatedHeds};
#endif
    commandVelocity(vCmd);
    // Best effort, 20 Hz. Never wait for serial bandwidth in the control loop.
    static uint32_t lastPose;
    if(now-lastPose>=50000u && Serial.availableForWrite()>=72){
        lastPose=now;
        float base=t1,tip=t2;
        if(!swinging){base=remainderf(base+mode->base,2*PI);tip=remainderf(tip+mode->tip,2*PI);}
        Serial.printf("POSE,%lu,%c,%.5f,%.5f,%.5f\n",(unsigned long)now,swinging?plan->phase:mode->key,x,base,tip);
    }
    if(startTransition){
        // The transition begins next frame at step 0; switching only after this frame's
        // command and POSE keeps swingLog[swingStep-1] and the POSE angles consistent.
        const auto *from=mode;
        plan=armedPlan;mode=poseFor(armedDest);armedPlan=nullptr;
        swinging=true;swingStep=0;swingUsed=0;swingStart=now;swingOrigin=x;
        caughtFrame=nullptr; // an earlier swing log is overwritten by this transition
        holdTiltLimitRad=MANUAL_HOLD_TILT_RAD; // each catch sets its own limit
        if(plan==&ddToDU)reLearn.begin();     // as an idle re start
        if(!descent(plan))ddWait=false;
        rampLimit(16.f);
        Serial.printf("%s->%s start x_mm=%+.1f\n",from->name,mode->name,x*1000.f);
    }
}
