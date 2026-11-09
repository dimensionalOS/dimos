// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
const fingers=['index','middle','ring','pinky'];
const vector=(a,b)=>a.map((x,i)=>x-b[i]);
const angle=(a,b)=>Math.acos(Math.max(-1,Math.min(1,a.reduce((s,x,i)=>s+x*b[i],0)/(Math.hypot(...a)*Math.hypot(...b)))));
const point=pose=>pose && [pose.transform.position.x,pose.transform.position.y,pose.transform.position.z];

// Joint bends, not pinch distance: finger size and wrist movement cancel out.
export function handClosure(poses) {
  const bends=[];
  for(const finger of fingers) {
    const joints=['phalanx-proximal','phalanx-intermediate','phalanx-distal','tip']
      .map(part=>point(poses[`${finger}-finger-${part}`]));
    if(joints.some(p=>!p || !p.every(Number.isFinite))) return null;
    const segments=joints.slice(1).map((p,i)=>vector(p,joints[i]));
    if(segments.some(v=>Math.hypot(...v)<.001)) return null;
    bends.push((angle(segments[0],segments[1])+angle(segments[1],segments[2]))/Math.PI);
  }
  const curl=bends.reduce((a,b)=>a+b,0)/bends.length;
  return Math.max(0,Math.min(1,(curl-.1)/.7));
}

export class HandFollowSession {
  constructor() { this.hold(); }
  hold(reason='Hands held') { this.enabled=false; this.waiting=false; this.reason=reason; this.sources=null; }
  enable() { this.enabled=true; this.waiting=true; this.reason='Release the menu pinch to begin following'; this.sources=null; }
  update(hands,{ready,calibrated,blocked=false}) {
    if(!this.enabled) return false;
    const pair=['left','right'].map(side=>hands[side]);
    if(!ready || !calibrated || blocked || pair.some(hand=>hand?.kind!=='hand' || !Number.isFinite(hand.trigger))) {
      this.hold('Tracking or control unavailable. Choose Follow hands to resume.'); return false;
    }
    if(this.sources && pair.some((hand,i)=>hand.source!==this.sources[i])) {
      this.hold('Hand tracking changed. Choose Follow hands to resume.');return false;
    }
    this.sources=pair.map(hand=>hand.source);
    // Only the initial menu pinch gates entry. A subsequent fist belongs to the jaws.
    if(this.waiting && pair.some(hand=>hand.distance<.04)) return false;
    this.waiting=false; this.reason='Following hands · open palm opens jaws · fist closes jaws';return true;
  }
}
