#include "../../src/double_pendulum/re_learning.h"
#include <cassert>
#include <cmath>
int main(){
    ReLearning l{};
    auto trial=[&](float correction){
        l.begin();
        for(unsigned i=0;i<336;++i)l.observe(i,correction,0.f,-.24f,i>=12);
    };
    trial(20);assert(l.at(0)==0 && l.at(12)==0);
    l.miss(5);assert(l.at(0)==0 && fabs(l.at(12)-.1)<1e-6);
    trial(-20);l.abort();assert(l.at(12)==0);
    trial(20);l.miss(5);float candidate=l.at(12);
    trial(20);l.miss(6);assert(l.at(12)==0);
    trial(20);l.miss(5);assert(l.at(12)>0 && l.at(12)<=candidate);
    l.catchDU();float caught=l.at(12);l.abort();assert(l.at(12)==caught);
    for(int j=0;j<100;++j){trial(20);l.miss(1.f/(j+1));}
    assert(l.at(335)<=.8f && l.at(335)>0);
    l=ReLearning{};assert(l.at(335)==0);
    auto checkTail=[](float ramp){
        ReLearning t{};t.begin();
        for(unsigned i=0;i<336;++i)t.observe(i,1.f,0.f,ramp,true);
        assert(t.at(335)==0);t.miss(5);
        assert(t.at(311)>0);
        for(unsigned i=312;i<336;++i){
            if(ramp>0)assert(t.at(i)<0);
            else if(ramp<0)assert(t.at(i)>0);
            else assert(t.at(i)==0);
            assert(fabs(t.at(i))<=.100001f);
        }
    };
    checkTail(.261f);checkTail(-.261f);checkTail(0);
}
