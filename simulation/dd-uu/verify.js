// node simulation/dd-uu/verify.js
// node simulation/dd-uu/verify.js ud
// node simulation/dd-uu/verify.js du
// Run the exported ESP32 table against the nonlinear plant with 4000-count encoders.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const {rk, dt} = require('../dd-du/model');
const firmware = path.join(__dirname, '../../src/double_pendulum');
const read = name => fs.readFileSync(path.join(firmware, name), 'utf8');
const numbers = row => row.split(',').map(s => Number(s.replace('f', '')));
const pose = process.argv[2] ?? 'uu';
assert(['uu','ud','du'].includes(pose), 'select uu, ud or du');
const re = pose === 'du', rw = pose === 'ud', guarded = re || rw;
const frames = [...read(pose+'_frames.h').matchAll(/^\s*\{([^{}]+)\},/gm)].map(m => numbers(m[1]));
const gainPattern = {du:/K_e\[\]=\{([^}]+)\}/, ud:/K_w\[\]=\{([^}]+)\}/, uu:/K_q\[\]=\{([^}]+)\}/}[pose];
const holdGain = numbers(read('upright.h').match(gainPattern)[1]);
const catchAngle = guarded ? .10 : .05;
const catchSpeed = guarded ? .30 : .20;
const pi = Math.PI, wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const tipTarget = pose === 'ud' ? pi : 0;
const baseTarget = re ? pi : 0;
const clamp = (x, limit) => Math.max(-limit, Math.min(limit, x));
const quant = a => Math.round(a * 4000 / (2*pi)) * 2*pi/4000;
const dot = (gain, state) => gain.reduce((sum, k, j) => sum + k*state[j], 0);
assert.equal(frames.length, re ? 336 : 744);
assert(frames.every(row => row.length === 11 && row.every(Number.isFinite)));

const needsBrake = (x,v) => Math.abs(x)>=.28 || Math.abs(x + .020*v + v*Math.abs(v)/8)>=.28;
const positionFeedback = (k,error,limit) => clamp(-.5*k*error,limit);
const seed = re ? numbers(read('upright.h').match(/caughtTrim\[\]=\{([^}]+)\}/)[1].trim().replace(/,$/,'')) : [];
function verify(baseOffsetDeg, tipOffsetDeg, delay, startX=0, handoff=null) {
    let state = handoff ? handoff.slice() : [startX, pi+baseOffsetDeg*pi/180, pi+tipOffsetDeg*pi/180, 0, 0, 0];
    function angles() {
        const a = pi + quant(state[1]-pi);
        return [wrap(a), wrap(a+quant(state[2]-state[1]))];
    }
    let previous = angles(), previousError = [0,0], rate = [0,0], errorRate = [0,0];
    let cmd = 0, pending = 0, catchState, peakX = 0, peakV = 0, peakTilt = 0;
    for (let i=handoff ? frames.length : 0; i<frames.length + 1000; i++) {
        const [a,b] = angles(), x = state[0], v = state[3];
        const baseError = wrap(a-baseTarget), tipError = wrap(b-tipTarget);
        if (guarded) assert(!needsBrake(x,v) && !needsBrake(x,cmd),'rail braking at frame '+i);
        const measured = [a,b];
        rate = measured.map((s,j) => .6*rate[j]+.4*wrap(s-previous[j])/dt);
        if (handoff && i===frames.length) rate = handoff.slice(4);
        previous = measured;
        let u;
        if (i < frames.length) {
            const [rx,ra,rb,rv,feed,...gain] = frames[i];
            const errors = [wrap(a-ra),wrap(b-rb)];
            assert(Math.abs(x)<=.30 && errors.every(e=>Math.abs(e)<=.45),
                   'swing travel or tracking at frame '+i);
            if (i) errorRate = errors.map((e,j)=>.6*errorRate[j]+.4*wrap(e-previousError[j])/dt);
            previousError = errors;
            u = feed - dot(gain,[0,...errors,v-rv,...errorRate]);
            if (rw) u += positionFeedback(gain[0],x-(startX+rx),1);
            if (re) u += seed[Math.floor(i/12)];
        } else {
            if (i === frames.length) {
                catchState = [baseError,tipError,...rate,v,x];
                const angle = Math.max(Math.abs(baseError), Math.abs(tipError));
                const spin = Math.max(...rate.map(Math.abs));
                const speed = Math.abs(v);
                const core = angle<=catchAngle && spin<.5 && speed<=catchSpeed;
                const extended = Math.abs(a-b)<=.01 && spin<=.05 && angle/.10+speed/.30<=1;
                assert(core || (pose==='uu' && extended), 'missed catch '+JSON.stringify(catchState));
                cmd = v;
                if (re) rate = [0,0];
            }
            peakTilt = Math.max(peakTilt,Math.abs(baseError),Math.abs(tipError));
            assert(peakTilt<=(re ? 20*pi/180 : .17) && Math.abs(x)<=.30,'hold guard');
            u = -dot(holdGain,[0,baseError,tipError,v,...rate]);
            if (guarded) u += positionFeedback(holdGain[0],x,1.5);
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
    const [base,tip] = angles();
    assert(Math.abs(wrap(base-baseTarget))<.02 && Math.abs(wrap(tip-tipTarget))<.02 && Math.abs(state[3])<.03,
           'terminal settling '+JSON.stringify(state));
    console.log({baseOffsetDeg,tipOffsetDeg,delay,startX,catchState,peakX,peakV,peakHoldDeg:peakTilt*180/pi});
}
for(const scenario of [[0,0,false],[1,0,false],[0,-1,false],[0,0,true],[0,0,false,re ? -.005 : -.10],[0,0,false,.10]]) verify(...scenario);
// Captured DU handoffs that ended in RAMP_POSITION_LIMIT (23:16, 23:28).
if (re) {
    verify(0,0,false,0,[.040925,3.140022,.062832,.251770,-.04745,-.03152]);
    verify(0,0,false,0,[.038175,-3.136880,.067544,.263661,-.09259,-.03301]);
}
