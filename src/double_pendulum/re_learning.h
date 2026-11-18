#pragma once
#include <cmath>
#include <algorithm>

// Session-only feedforward trim. Current bins are immutable during each re.
struct ReLearning {
    static constexpr unsigned width=12, bins=28;
    float current[bins]{},accepted[bins]{},sum[bins]{};
    unsigned count[bins]{};
    float gain=.08f, baseline=0;
    bool candidate=false;
    static float clip(float x,float bound){return std::max(-bound,std::min(bound,x));}
    float at(unsigned i) const {return current[i/width];}
    void begin(){std::fill_n(sum,bins,0.f);std::fill_n(count,bins,0u);}
    void observe(unsigned i,float encoderFeedback,float refV,float rampV,bool usable){
        // Last two 60 ms bins: convert speed error to an acceleration correction
        // over the nominal braking window. This only affects the next trial.
        const float signal=i>=(bins-2)*width?(refV-rampV)/.120f:encoderFeedback;
        if(usable && std::isfinite(signal)){sum[i/width]+=clip(signal,2.f);++count[i/width];}
    }
    void abort(){
        if(candidate){std::copy_n(accepted,bins,current);gain=std::max(.01f,gain*.5f);candidate=false;}
    }
    void catchDU(){candidate=false;} // keep the successful command profile
    bool miss(float score){
        if(!std::isfinite(score)||(candidate && score>baseline)){abort();return false;}
        baseline=score;std::copy_n(current,bins,accepted);
        for(unsigned j=0;j<bins;j++)if(count[j]>=width/2){
            current[j]=clip(current[j]+clip(gain*sum[j]/count[j],.10f),.80f);
        }
        candidate=true;return true;
    }
};
