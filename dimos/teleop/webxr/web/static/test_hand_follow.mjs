// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import assert from 'node:assert/strict';
import {test} from 'node:test';
import {handClosure,HandFollowSession} from './hand_follow.mjs';
import {sampleXRInputs} from './xr_input.mjs';
const pose=p=>({transform:{position:{x:p[0],y:p[1],z:p[2]},orientation:{x:0,y:0,z:0,w:1}}});
function skeleton(bend,scale=1,shift=[0,0,0]) {
  const poses={wrist:pose(shift)};
  for(const finger of ['index','middle','ring','pinky']) {
    let p=[.03,0,0];
    for(const [i,part] of ['phalanx-proximal','phalanx-intermediate','phalanx-distal','tip'].entries()) {
      if(i) p=[p[0]+.025*Math.cos((i-1)*bend),p[1]+.025*Math.sin((i-1)*bend),0];
      poses[`${finger}-finger-${part}`]=pose(p.map((x,j)=>x*scale+shift[j]));
    }
  }
  poses['thumb-tip']=pose([shift[0],shift[1]+.06,shift[2]]);
  return poses;
}
test('finger bends map open palm to open jaws and fist to closed jaws at any hand size',()=>{
  assert.equal(handClosure(skeleton(0)),0);
  assert.equal(handClosure(skeleton(Math.PI/2)),1);
  const partial=handClosure(skeleton(Math.PI/4));assert(partial>0&&partial<1);
  assert(Math.abs(partial-handClosure(skeleton(Math.PI/4,1.4,[1,2,3])))<1e-12);
  const lost=skeleton(0);delete lost['ring-finger-tip'];assert.equal(handClosure(lost),null);
});
test('body-follow input uses the wrist, so closing fingers cannot displace the arm target',()=>{
  const source={handedness:'left',targetRayMode:'tracked-pointer'};
  let poses=skeleton(0,1,[.3,1.4,-.6]);
  source.hand=new Map(Object.keys(poses).map(name=>[name,name]));
  const frame={session:{inputSources:[source]},getPose:()=>null,getJointPose:name=>poses[name]};
  const open=sampleXRInputs(frame,{}, {bodyFollow:true}).left;
  poses=skeleton(Math.PI/2,1,[.3,1.4,-.6]);
  const fist=sampleXRInputs(frame,{}, {bodyFollow:true}).left;
  assert.deepEqual(open.position,fist.position);assert.equal(open.trigger,0);assert.equal(fist.trigger,1);
});
const options={ready:true,calibrated:true};
const hands={left:{kind:'hand',source:{},trigger:0,distance:.05},right:{kind:'hand',source:{},trigger:0,distance:.05}};
test('explicit follow waits for menu pinch release, then a fist only closes the jaws',()=>{
  const session=new HandFollowSession();assert.equal(session.update(hands,options),false);
  session.enable();assert.equal(session.update({...hands,right:{...hands.right,distance:.01}},options),false);
  assert.equal(session.update(hands,options),true);
  assert.equal(session.update({...hands,right:{...hands.right,distance:.01,trigger:1}},options),true);
  session.hold();assert.equal(session.update(hands,options),false);
});
test('tracking loss, menu use, missing calibration and source replacement require explicit re-enable',()=>{
  for(const [sample,opts] of [[{left:hands.left},options],[hands,{...options,blocked:true}],[hands,{...options,calibrated:false}],[{...hands,left:{...hands.left,source:{}}},options]]) {
    const session=new HandFollowSession();session.enable();assert(session.update(hands,options));
    assert.equal(session.update(sample,opts),false);assert.equal(session.update(hands,options),false);
  }
});
test('gaze/pinch transient pointers never masquerade as tracked hand poses',()=>{
  assert.deepEqual(sampleXRInputs({session:{inputSources:[{handedness:'left',targetRayMode:'transient-pointer'}]}},{}),{});
});
