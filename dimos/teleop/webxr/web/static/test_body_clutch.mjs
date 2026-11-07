// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {BodyClutchFrame} from './body_clutch.mjs';
const near=(a,b)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<1e-9,`${a} != ${b}`));

test('physical forward/right/up map to robot forward/right/up at any calibrated heading',()=>{
  for(const yaw of [0,Math.PI/2,-Math.PI/2,Math.PI]) {
    const frame=new BodyClutchFrame(), q=[0,Math.sin(yaw/2),0,Math.cos(yaw/2)];
    const origin=[2,1.6,3];
    assert.equal(frame.calibrate(origin,q),true);
    for(const [delta,expected] of [
      [[-.1*Math.sin(yaw),0,-.1*Math.cos(yaw)],[.1,0,0]],
      [[.1*Math.cos(yaw),0,-.1*Math.sin(yaw)],[0,-.1,0]],
      [[0,.1,0],[0,0,.1]],
    ]) near(frame.pose(origin.map((v,i)=>v+delta[i]),q).position,expected);
  }
});

test('looking around does not rotate calibration, explicit reset does',()=>{
  const frame=new BodyClutchFrame();
  frame.calibrate([0,0,0],[0,0,0,1]);
  const initial=frame.pose([0,0,-.1],[0,0,0,1]);
  frame.calibrate([1,1,1],[0,Math.SQRT1_2,0,Math.SQRT1_2]);
  near(frame.pose([0,0,-.1],[0,0,0,1]).position,initial.position);
  near(frame.pose([0,0,-.1],[0,0,0,1]).orientation,initial.orientation);
  frame.reset();
  frame.calibrate([0,0,0],[0,Math.SQRT1_2,0,Math.SQRT1_2]);
  near(frame.pose([0,0,-.1],[0,0,0,1]).position,[0,-.1,0]);
});

test('looking straight up cannot silently choose an arbitrary forward direction',()=>{
  const frame=new BodyClutchFrame();
  assert.equal(frame.calibrate([0,0,0],[Math.SQRT1_2,0,0,Math.SQRT1_2]),false);
  assert.equal(frame.pose([0,0,0],[0,0,0,1]),null);
});
