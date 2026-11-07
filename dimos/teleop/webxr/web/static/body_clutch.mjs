// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
// Calibrated XR heading -> robot forward/left/up. No scene transforms or scale.
const multiply = ([x,y,z,w],[a,b,c,d]) => [
  w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a, w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c,
];
const rotate = (v,q) => multiply(multiply(q,[...v,0]),[-q[0],-q[1],-q[2],q[3]]).slice(0,3);

export class BodyClutchFrame {
  constructor() { this.reset(); }
  reset() { this.rotation=null; this.origin=null; }
  calibrate(position, orientation) {
    if(this.rotation) return true;
    const forward=rotate([0,0,-1],orientation);
    if(Math.hypot(forward[0],forward[2])<0.1) return false;
    const yaw=Math.atan2(-forward[0],-forward[2]);
    this.rotation=multiply([.5,-.5,-.5,.5],[0,-Math.sin(yaw/2),0,Math.cos(yaw/2)]);
    this.origin=[...position];
    return true;
  }
  pose(position, orientation) {
    if(!this.rotation) return null;
    return {position:rotate(position.map((v,i)=>v-this.origin[i]),this.rotation),
      orientation:multiply(this.rotation,orientation)};
  }
}
