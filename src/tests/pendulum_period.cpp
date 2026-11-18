#include <Arduino.h>
#include "../double_pendulum/encoder_pair.h"
static int ch=0,nperiod=0;
// Display only. Replace with measured x4 counts per full turn for each joint.
static constexpr int32_t countsPerTurn[2]={4000,4000};
static int32_t zero[2]={},prev=0;
static unsigned badStart;
static bool ready=false,running=false,armed=false,pending=false,haveFirst=false;
static uint32_t lastSample,priorTime,candidate,first,lastCross;
static void help(){ Serial.println("a=T_combined (J5 HEDS1); b=T2 (HEDS2). Select at rest DOWN to zero; hold at 5-10 deg, press s, release. x=stop."); }
void setup(){
    digitalWrite(13,HIGH);pinMode(13,OUTPUT);digitalWrite(13,HIGH);
    Serial.begin(115200);
    enc[0].a=36;enc[0].b=39;enc[1].a=32;enc[1].b=33;
    for(auto &e:enc){
        pinMode(e.a,INPUT);pinMode(e.b,INPUT);e.prev=pins(e);
        attachInterruptArg(e.a,edge,&e,CHANGE);attachInterruptArg(e.b,edge,&e,CHANGE);
    }
    help();
}
void loop(){
    if(Serial.available()){
        int c=Serial.read();
        if(c=='a'||c=='b'){
            ch=c=='b';for(int i=0;i<2;++i)zero[i]=enc[i].pos;
            badStart=enc[ch].bad;running=false;ready=true;
            Serial.printf("%s zero set. Displace, hold, press s, release.\n",ch?"T2":"T_combined");
        } else if(c=='s' && !running){
            for(int i=0;i<2;++i){
                int32_t raw=enc[i].pos,offset=raw-zero[i];
                Serial.printf("HEDS%d%s raw=%ld",i+1,ready && i==ch?" [selected]":"",(long)raw);
                if(ready)Serial.printf(" zero=%ld delta=%ld angle_est_deg=%.2f (assumed %ld counts/rev)",
                    (long)zero[i],(long)offset,360.0*offset/countsPerTurn[i],(long)countsPerTurn[i]);
                Serial.println();
                if(i==ch)prev=offset;
            }
            if(!ready){Serial.println("No zero: press a or b at rest first.");return;}
            if(abs(prev)<3){Serial.println("Too close to zero. Hold displaced and retry s.");return;}
            running=true;haveFirst=pending=false;armed=prev<=-2;nperiod=0;
            Serial.println("Started. Release now.");
            priorTime=lastSample=micros();
        } else if(c=='x'){running=ready=false;help();}
    }
    uint32_t now=micros();
    if(!running || now-lastSample<1000)return;
    uint32_t dt=now-lastSample;
    lastSample=now;int32_t p=enc[ch].pos-zero[ch];
    if(enc[ch].bad!=badStart || (haveFirst && now-lastCross>5000000u) ||
       (!haveFirst && now-priorTime>5000000u)){
        running=ready=false;Serial.println("Incomplete: missing crossings or invalid encoder transition. Re-zero and repeat.");return;
    }
    // Interpolate the positive-going zero crossing, confirm at +2 counts.
    // A fresh excursion to -2 counts is required before each crossing.
    if(p<=-2){armed=true;pending=false;}
    if(p<0)pending=false;
    if(armed && prev<0 && p>=0){
        candidate=now-uint32_t(double(dt)*p/(p-prev));pending=true;
    }
    if(armed && pending && p>=2){
        armed=pending=false;
        if(!haveFirst){first=candidate;haveFirst=true;}
        else{
            ++nperiod;
            Serial.printf("cycle=%d, period_s=%.6f\n",nperiod,(candidate-lastCross)*1e-6);
            if(nperiod==10){
                Serial.printf("RESULT %s_mean_s=%.6f (10 cycles)\n",ch?"T2":"T_combined",(candidate-first)*1e-7);
                running=ready=false;
            }
        }
        lastCross=candidate;
    }
    prev=p;
}
