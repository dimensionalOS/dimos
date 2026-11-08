// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
// Operator calibration survives grip releases and never samples robot joints.
const add=(a,b)=>a.map((x,i)=>x+b[i]);
const sub=(a,b)=>a.map((x,i)=>x-b[i]);
const mul=([x,y,z,w],[a,b,c,d])=>[w*a+x*d+y*c-z*b,w*b-x*c+y*d+z*a,w*c+x*b-y*a+z*d,w*d-x*a-y*b-z*c];
const inverse=([x,y,z,w])=>[-x,-y,-z,w];
const rotate=(v,q)=>mul(mul(q,[...v,0]),inverse(q)).slice(0,3);
const yawQ=yaw=>[0,Math.sin(yaw/2),0,Math.cos(yaw/2)];
const flu=[0.5,-0.5,-0.5,0.5];
export class BodyFollow {
  constructor() { this.reset(); }
  reset() { this.calibration=null; this.error='Calibrate body with both arms extended forward and grips released.'; }
  frame(viewer, body) {
    const left=body?.['left-arm-upper'], right=body?.['right-arm-upper'];
    if(left && right) {
      const across=sub(left.position,right.position);
      if(Math.hypot(...across)<0.15) return null;
      const yaw=Math.atan2(across[2],-across[0]);
      return {rotation:mul(flu,yawQ(-yaw)),
        shoulders:{left:left.position,right:right.position},source:'tracked shoulders'};
    }
    if(!viewer) return null;
    const forward=rotate([0,0,-1],viewer.orientation);
    const yaw=this.calibration?.yaw ?? Math.atan2(-forward[0],-forward[2]);
    return {rotation:mul(flu,yawQ(-yaw)),yaw,
      shoulders:Object.fromEntries(['left','right'].map(side=>[side,
        add(viewer.position,rotate([side==='left'?-0.18:0.18,-0.22,0],yawQ(yaw)))])),
      source:'estimated shoulders (headset + controllers)'};
  }
  calibrate(viewer, hands, body) {
    // A new explicit calibration may change heading; grip engagement may not.
    this.calibration=null;
    const frame=this.frame(viewer,body);
    if(!frame || !hands.left || !hands.right) {this.error='Both hands and the headset must be tracked.';return false;}
    const arms={};
    for(const side of ['left','right']) {
      const hand=hands[side];
      if(!Number.isFinite(hand.clutchDistance) || hand.clutchDistance<0.04) {this.error='Release both grips before calibrating.';return false;}
      const offset=rotate(sub(hand.position,frame.shoulders[side]),frame.rotation);
      const reach=Math.hypot(...offset);
      if(reach<0.35 || reach>1.1 || offset[0]/reach<0.8) {
        this.error='Extend both arms forward, shoulder height, pointing controllers forward.';return false;
      }
      arms[side]={reach,orientation:inverse(mul(frame.rotation,hand.orientation))};
    }
    this.calibration={arms,yaw:frame.yaw,source:frame.source}; this.error=''; return true;
  }
  pose(side, hand, viewer, body) {
    if(!this.calibration) return null;
    const frame=this.frame(viewer,body);
    if(!frame || frame.source!==this.calibration.source) {this.calibration=null;this.error='Body tracking changed. Release grips and recalibrate.';return null;}
    this.error='';
    const arm=this.calibration.arms[side];
    return {position:rotate(sub(hand.position,frame.shoulders[side]),frame.rotation).map(x=>x/arm.reach),
      orientation:mul(mul(frame.rotation,hand.orientation),arm.orientation)};
  }
}
