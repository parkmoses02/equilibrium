// SI units. State: cart x, absolute base/tip angles from up, cart v, angular rates.
const pi=Math.PI, dt=.005, stride=24, N=13;
const B=.002475,S=.0128,C=.16*B,g=9.80665;
const J2=B*g*(.792172/(2*pi))**2,J1=S*g*(.930958/(2*pi))**2-J2-2*C;
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a)),clip=(x,l)=>Math.max(-l,Math.min(l,x));
function deriv(y,u,drag){
  const [,a,b,v,w,z]=y,h=C*Math.cos(a-b),s=C*Math.sin(a-b),det=J1*J2-h*h;
  // Absolute link coordinates: elbow torque acts equally and oppositely.
  const elbow=drag?drag.b2*(z-w):0;
  const f=(S-B)*(g*Math.sin(a)-u*Math.cos(a))-s*z*z+elbow-(drag?drag.b1*w:0);
  const k=B*(g*Math.sin(b)-u*Math.cos(b))+s*w*w-elbow;
  return [v,w,z,u,(J2*f-h*k)/det,(J1*k-h*f)/det];
}
function rk(y,u,h,drag){
  const add=(a,b,t)=>a.map((v,i)=>v+t*b[i]);
  const a=deriv(y,u,drag),b=deriv(add(y,a,h/2),u,drag),c=deriv(add(y,b,h/2),u,drag),d=deriv(add(y,c,h),u,drag);
  return y.map((v,i)=>v+h*(a[i]+2*b[i]+2*c[i]+d[i])/6);
}
const K=[-3.464102,78.817707,-116.908943,-4.688550,6.365765,-13.358312];
// Linearize the 5 ms plant step along the nominal path, then solve finite-horizon
// discrete LQR backwards. No perturbed-state data is used to design the gains.
function makeTracker(knots){
  const frames=sim(knots,true,2).frames, n=(N+1)*stride, eps=1e-5;
  const Q=[12,60,60,2,2,2],dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0);
  let P=Q.map((q,i)=>Q.map((_,j)=>i===j?q*2:0)),gains=Array(n);
  for(let t=n-1;t>=0;t--){
    const y=frames[t].slice(1,7),u=frames[t+1][7];
    const cols=Array.from({length:7},(_,j)=>{
      const lo=y.slice(),hi=y.slice();if(j<6){lo[j]-=eps;hi[j]+=eps;}
      const l=rk(lo,u-(j===6?eps:0),dt),h=rk(hi,u+(j===6?eps:0),dt);
      return h.map((x,i)=>(x-l[i])/(2*eps));
    });
    const b=cols.pop(),pb=P.map(row=>dot(row,b)),den=dt+dot(b,pb);
    const pa=cols.map(a=>P.map(row=>dot(row,a))),s=cols.map(a=>dot(a,pb));
    gains[t]=s.map(v=>v/den);
    P=cols.map((a,i)=>cols.map((_,j)=>dot(a,pa[j])-s[i]*s[j]/den+(i===j?dt*Q[i]:0)));
  }
  return {frames,gains};
}
function sim(knots,record=false,sub=1,variant='nominal',tracker=null,drag=null){
  if(drag && ![drag.b1,drag.b2].every(x=>Number.isFinite(x)&&x>=0))throw Error('b1,b2 must be nonnegative N m s/rad');
  let y=[0,pi+(variant==='offset'?pi/180:0),pi,0,0,0],maxX=0,maxV=0,maxA=0,t=0,frames=[],pending=0;
  function advance(u){
    if(variant==='delay'){const prev=pending;pending=u;u=prev;}
    u=(clip(y[3]+dt*u,.8)-y[3])/dt;
    for(let j=0;j<sub;j++)y=rk(y,u,dt/sub,drag);
    t++;maxX=Math.max(maxX,Math.abs(y[0]));maxV=Math.max(maxV,Math.abs(y[3]));maxA=Math.max(maxA,Math.abs(u));
    if(record)frames.push([t*dt,...y,u]);
  }
  if(record)frames.push([0,...y,0]);
  let prev=0,prior=null,rate=[0,0];
  for(const next of [...knots,0]){
    const u=(next-prev)/(stride*dt); prev=next;
    for(let i=0;i<stride;i++){
      let command=u;
      if(tracker){
        const ref=tracker.frames[t],q=2*pi/4000,quant=x=>Math.round(x/q)*q;
        const base=pi+quant(y[1]-pi),tip=base+quant(y[2]-y[1]);
        const angles=[wrap(base-ref[2]),wrap(tip-ref[3])];
        if(prior)rate=rate.map((w,j)=>.6*w+.4*wrap(angles[j]-prior[j])/dt);
        prior=angles;
        const e=[y[0]-ref[1],...angles,y[3]-ref[4],...rate];
        command=clip(u-tracker.gains[t].reduce((s,k,j)=>s+k*e[j],0),16);
      }
      advance(command);
    }
  }
  const end=y.slice(),ea=wrap(y[1]-pi),eb=wrap(y[2]);
  const residual=[6*y[0],ea,eb,.16*y[4],.16*y[5],20*Math.max(0,maxX-.26)];
  const score=residual.reduce((v,x)=>v+x*x,0);
  let caught=false,held=false;
  if(record && Math.max(Math.abs(ea),Math.abs(eb))<.05 && Math.max(Math.abs(y[4]),Math.abs(y[5]))<.5 && Math.abs(y[3])<=.25 && maxX<.28){
    caught=true;
    const quantum=2*pi/4000, quant=a=>Math.round(a/quantum)*quantum;
    function enc(){let base=quant(y[1]-pi);return [wrap(base),wrap(pi+base+quant(y[2]-y[1]))];}
    let [pa,pb]=enc(),wa=0,wb=0;
    for(let i=0;i<400;i++){
      const [a,b]=enc();
      if(Math.max(Math.abs(a),Math.abs(b))>.17 || Math.abs(y[0])>=.30)break;
      wa=.6*wa+.4*wrap(a-pa)/dt;wb=.6*wb+.4*wrap(b-pb)/dt;pa=a;pb=b;
      const state=[y[0],a,b,y[3],wa,wb];
      const req=-state.reduce((sum,x,j)=>sum+x*K[j],0);
      advance((clip(y[3]+dt*clip(req,12),.8)-y[3])/dt);
      held=i===399 && Math.max(Math.abs(wrap(y[1]-pi)),Math.abs(wrap(y[2])))<=.17 && maxX<.30;
    }
  }
  return {frames,score,residual,end,ea,eb,caught,held,maxX,maxV,maxA};
}
// Damped least-squares shooting, constrained velocity knots. Multiple deterministic starts.
function solve(M,b){
  let a=M.map((r,i)=>[...r,b[i]]),n=b.length;
  for(let j=0;j<n;j++){
    let p=j;for(let i=j+1;i<n;i++)if(Math.abs(a[i][j])>Math.abs(a[p][j]))p=i;
    [a[j],a[p]]=[a[p],a[j]];let s=a[j][j];for(let k=j;k<=n;k++)a[j][k]/=s;
    for(let i=0;i<n;i++)if(i!==j){let v=a[i][j];for(let k=j;k<=n;k++)a[i][k]-=v*a[j][k];}
  }
  return a.map(r=>r[n]);
}
function* search(seed=1){
 let random=()=>((seed=(1664525*seed+1013904223)>>>0)/4294967296),best=Infinity;
 for(let trial=0;trial<60;trial++){
  let v=Array.from({length:N},()=>1.5*(random()-.5)),lam=.03;
  for(let it=0;it<55;it++){
    let z=sim(v),r=z.residual;
    if(z.score<best){
      best=z.score;let test=sim(v,true,2);
      yield {trial,it,v:v.slice(),result:test};
      if(test.held && best<.001)return;
    }else yield null;
    let J=v.map((x,i)=>{let w=v.slice();w[i]+=.00002;return sim(w).residual.map((y,j)=>(y-r[j])/.00002);});
    const M=r.map((_,i)=>r.map((_,j)=>J.reduce((s,row)=>s+row[i]*row[j],i===j?lam:0)));
    const d=solve(M,r),next=v.map((x,i)=>clip(x-J[i].reduce((s,y,j)=>s+y*d[j],0),.8));
    if(sim(next).score<z.score){v=next;lam=Math.max(.0001,lam*.7);}else lam*=3;
    if(lam>1e6)break;
  }
 }
}
if(typeof module!=="undefined")module.exports={search,sim,dt,stride,N,makeTracker};
