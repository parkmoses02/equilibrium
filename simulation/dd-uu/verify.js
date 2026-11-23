// node simulation/dd-uu/verify.js
// Run the exported ESP32 table against the nonlinear plant with 4000-count encoders.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const {rk, dt} = require('../dd-du/model');
const firmware = path.join(__dirname, '../../src/double_pendulum');
const read = name => fs.readFileSync(path.join(firmware, name), 'utf8');
const numbers = row => row.split(',').map(s => Number(s.replace('f', '')));
const frames = [...read('uu_frames.h').matchAll(/^\s*\{([^{}]+)\},/gm)].map(m => numbers(m[1]));
const holdGain = numbers(read('upright.h').match(/K_q\[\]=\{([^}]+)\}/)[1]);
const pi = Math.PI, wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (x, limit) => Math.max(-limit, Math.min(limit, x));
const quant = a => Math.round(a * 4000 / (2*pi)) * 2*pi/4000;
const dot = (gain, state) => gain.reduce((sum, k, j) => sum + k*state[j], 0);
assert.equal(frames.length, 744);
assert(frames.every(row => row.length === 11 && row.every(Number.isFinite)));

function verify(baseOffsetDeg, tipOffsetDeg, delay, startX=0) {
    let state = [startX, pi+baseOffsetDeg*pi/180, pi+tipOffsetDeg*pi/180, 0, 0, 0];
    function angles() {
        const a = pi + quant(state[1]-pi);
        return [wrap(a), wrap(a+quant(state[2]-state[1]))];
    }
    let previous = angles(), previousError = [0,0], rate = [0,0], errorRate = [0,0];
    let cmd = 0, pending = 0, catchState, peakX = 0, peakV = 0, peakTilt = 0;
    for (let i=0; i<frames.length + 1000; i++) {
        const [a,b] = angles(), x = state[0], v = state[3];
        const measured = [a,b];
        rate = measured.map((s,j) => .6*rate[j]+.4*wrap(s-previous[j])/dt);
        previous = measured;
        let u;
        if (i < frames.length) {
            const [,ra,rb,rv,feed,...gain] = frames[i];
            const errors = [wrap(a-ra),wrap(b-rb)];
            assert(Math.abs(x)<=.30 && errors.every(e=>Math.abs(e)<=.45),
                   'swing travel or tracking at frame '+i);
            if (i) errorRate = errors.map((e,j)=>.6*errorRate[j]+.4*wrap(e-previousError[j])/dt);
            previousError = errors;
            u = feed - dot(gain,[0,...errors,v-rv,...errorRate]);
        } else {
            if (i === frames.length) {
                catchState = [a,b,...rate,v,x];
                const angle = Math.max(Math.abs(a), Math.abs(b));
                const spin = Math.max(...rate.map(Math.abs));
                const speed = Math.abs(v);
                const core = angle<=.05 && spin<.5 && speed<=.20;
                const extended = Math.abs(a-b)<=.01 && spin<=.05 && angle/.10+speed/.30<=1;
                assert(core || extended, 'missed catch '+JSON.stringify(catchState));
                cmd = v;
            }
            peakTilt = Math.max(peakTilt,Math.abs(a),Math.abs(b));
            assert(peakTilt<=.17 && Math.abs(x)<=.30,'hold guard');
            u = -dot(holdGain,[0,a,b,v,...rate]);
        }
        const limit = i<frames.length ? 16 : 12;
        const next = clamp(cmd+dt*clamp(u,limit),.8);
        const acceleration = (next-cmd)/dt;
        cmd = next;
        const applied = delay ? pending : acceleration;
        pending = acceleration;
        for(let sub=0; sub<4; sub++) state = rk(state,applied,dt/4);
        peakX=Math.max(peakX,Math.abs(state[0]));
        peakV=Math.max(peakV,Math.abs(state[3]));
    }
    assert(peakV<=.800001 && peakX<=.30);
    assert(angles().every(a=>Math.abs(a)<.02) && Math.abs(state[3])<.03,
           'terminal settling '+JSON.stringify(state));
    console.log({baseOffsetDeg,tipOffsetDeg,delay,startX,catchState,peakX,peakV,peakHoldDeg:peakTilt*180/pi});
}
for(const scenario of [[0,0,false],[1,0,false],[0,-1,false],[0,0,true],[0,0,false,-.10],[0,0,false,.10]]) verify(...scenario);
