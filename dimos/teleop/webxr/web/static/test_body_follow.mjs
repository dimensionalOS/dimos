// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import assert from 'node:assert/strict';
import {test} from 'node:test';
import {BodyFollow} from './body_follow.mjs';
const viewer={position:[0,1.7,0],orientation:[0,0,0,1]};
const hands={left:{position:[-.18,1.48,-.65],orientation:[0,0,0,1],clutchDistance:.05},
  right:{position:[.18,1.48,-.65],orientation:[0,0,0,1],clutchDistance:.05}};
test('full extension stays absolute through arbitrary grip cycles',()=>{
  const mapper=new BodyFollow();assert(mapper.calibrate(viewer,hands,null));
  assert.deepEqual(mapper.pose('left',hands.left,viewer,null).position,[1,0,0]);
  const bent={...hands.left,position:[-.18,1.48,-.325]};
  assert.deepEqual(mapper.pose('left',bent,viewer,null).position,[.5,0,0]);
  for(let i=0;i<3;i++) assert.deepEqual(mapper.pose('left',hands.left,viewer,null).position,[1,0,0]);
});
test('looking sideways does not rotate estimated body targets',()=>{
  const mapper=new BodyFollow();mapper.calibrate(viewer,hands,null);
  const looking={...viewer,orientation:[0,Math.SQRT1_2,0,Math.SQRT1_2]};
  assert.deepEqual(mapper.pose('right',hands.right,looking,null),mapper.pose('right',hands.right,viewer,null));
});
test('body translation cancels and tracking source changes require calibration',()=>{
  const mapper=new BodyFollow();mapper.calibrate(viewer,hands,null);
  const shifted={...viewer,position:[1,1.7,0]};
  const hand={...hands.left,position:[.82,1.48,-.65]};
  assert(Math.abs(mapper.pose('left',hand,shifted,null).position[1])<1e-12);
  assert.equal(mapper.pose('left',hand,shifted,{'left-arm-upper':{position:[-.18,1.48,0]},'right-arm-upper':{position:[.18,1.48,0]}}),null);
});
test('a bent-arm calibration is rejected and reference reset disarms targets',()=>{
  const mapper=new BodyFollow();
  assert.equal(mapper.calibrate(viewer,{...hands,left:{...hands.left,position:[-.18,1.48,-.1]}},null),false);
  assert(mapper.calibrate(viewer,hands,null));mapper.reset();
  assert.equal(mapper.pose('left',hands.left,viewer,null),null);
});
test('tracked shoulders follow body heading independently of the headset',()=>{
  const mapper=new BodyFollow();
  const body={'left-arm-upper':{position:[-.18,1.48,0]},'right-arm-upper':{position:[.18,1.48,0]}};
  assert(mapper.calibrate(viewer,hands,body));
  const turnedBody={'left-arm-upper':{position:[0,1.48,.18]},'right-arm-upper':{position:[0,1.48,-.18]}};
  const turnedHand={position:[-.65,1.48,.18],orientation:[0,Math.SQRT1_2,0,Math.SQRT1_2]};
  const pose=mapper.pose('left',turnedHand,viewer,turnedBody);
  assert(Math.abs(pose.position[0]-1)<1e-12);
  assert(Math.hypot(...pose.position.slice(1))<1e-12);
  assert(Math.hypot(...pose.orientation.slice(0,3))<1e-12);
});
