#pragma once
#include <stdint.h>

struct DuEdges {
    enum Result {ok,warn,stop};
    uint32_t seen1,seen2,last=0;
    bool pending=false;
    DuEdges(uint32_t a,uint32_t b):seen1(a),seen2(b){}
    Result check(uint32_t now,uint32_t a,uint32_t b){
        if(pending && now-last>50000u)pending=false;
        uint32_t da=a-seen1,db=b-seen2;
        seen1=a;seen2=b;
        if(!da && !db)return ok;
        if(pending || da>1 || db>1 || (da && db))return stop;
        last=now;pending=true;return warn;
    }
};
