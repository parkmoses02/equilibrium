const assert=require('node:assert/strict'),{search,sim,N,makeTracker}=require('./model');
assert(!sim(Array(N).fill(0),true).caught);
const rest=Array(N).fill(0),free=drag=>sim(rest,false,2,'offset',null,drag).end;
assert.deepEqual(free({b1:0,b2:0}),free(null));
assert.notDeepEqual(free({b1:.0001,b2:.00002}),free(null)); // synthetic, not calibrated
assert.throws(()=>free({b1:-1,b2:0}));
let best,count=0;
for(const c of search())if(c){best=c;count++;}
assert(best.result.held);
for(const variant of ['nominal','offset','delay']){
 const z=sim(best.v,true,4,variant);
 console.log(variant,{held:z.held,x:z.maxX,v:z.maxV,a:z.maxA,baseError:z.ea,tipError:z.eb});
 if(variant==='nominal'){
   assert(z.held && z.frames.at(-1)[0]>=3.68 && z.maxX<.30 && z.maxV<=.800001 && z.maxA<=16.00001);
 }
}
console.log(count,'improving candidates');
const ab=[null,makeTracker(best.v)].map(k=>sim(best.v,true,4,'offset',k));
assert.deepEqual(ab.map(z=>z.caught),[false,true]);
assert.deepEqual(ab.map(z=>z.held),[false,true]);
for(const z of ab){
 assert(z.maxX<.30 && z.maxV<=.800001 && z.maxA<=16.00001);
 console.log('offset A/B',z.held,z.ea*180/Math.PI,z.eb*180/Math.PI,z.maxX,z.maxV,z.maxA);
}
assert.equal(ab[1].frames.at(-1)[0],3.68);
if(process.argv.includes('--render')){
 const fs=require('node:fs'),path=require('node:path');
 const data=ab.map(z=>z.frames.map(f=>f.slice(1,4).map(x=>+x.toFixed(8))));
 const fragment=fs.readFileSync(path.join(__dirname,'compare.fragment.html'),'utf8').replace('/*RUNS*/',JSON.stringify(data));
 fs.writeFileSync(path.join(__dirname,'feedback-comparison.html'),'<!doctype html><meta charset="utf-8"><title>스윙업 피드백 비교</title>'+fragment);
 if(process.argv[3])fs.writeFileSync(process.argv[3],fragment);
}
