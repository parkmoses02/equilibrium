#include "../../src/double_pendulum/du_edges.h"
#include <cassert>
int main(){
    DuEdges e{7,9};
    assert(e.check(100,7,9)==DuEdges::ok);
    assert(e.check(5000,7,10)==DuEdges::warn);
    assert(e.check(10000,7,10)==DuEdges::ok); // unchanged cumulative count
    assert(e.check(55000,8,10)==DuEdges::stop); // other encoder, inclusive
    e=DuEdges{0,0};
    assert(e.check(0xfffffff0u,1,0)==DuEdges::warn);
    assert(e.check(20,1,1)==DuEdges::stop); // timer rollover
    e=DuEdges{0,0};
    assert(e.check(0,1,0)==DuEdges::warn);
    assert(e.check(50001,1,1)==DuEdges::warn);
    assert(e.check(60000,1,3)==DuEdges::stop);
    e=DuEdges{4,5};
    assert(e.check(0,5,6)==DuEdges::stop);
}
