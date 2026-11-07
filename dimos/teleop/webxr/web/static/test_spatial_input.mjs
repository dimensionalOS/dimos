// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {PinchGrab, ToolPinch} from './spatial_input.mjs';
test('release, grab, hold with hysteresis, release', () => {
  const hand = new PinchGrab();
  assert.equal(hand.update(.01, true).grab, false);
  hand.update(.05, true);
  assert.deepEqual(hand.update(.01, true), {grab:true, pressed:true});
  assert.equal(hand.update(.028, false).grab, true);
  assert.equal(hand.update(.05, false).grab, false);
});
test('opening drag starts at measured position and owns only its selected action', () => {
  const hand=new ToolPinch();
  const update=(d,target,x,opening=.6)=>hand.update(d,target,[x,0,0],[1,0,0],opening,.12);
  update(.05,'opening',0);
  assert.equal(update(.01,'opening',0).action,'opening');
  assert.equal(hand.opening,.6);
  assert.equal(update(.01,'move',-.03,.55).action,'opening');
  assert.ok(Math.abs(hand.opening-.35)<1e-12);
  update(.01,null,-1); assert.equal(hand.opening,0);
  update(.05,null,0); assert.equal(hand.action,null);
  update(.01,'opening',0,.2); assert.equal(hand.opening,.2);
});
test('a pinch begun away from tools cannot acquire one until release', () => {
  const hand=new ToolPinch();
  const update=(d,target)=>hand.update(d,target,[0,0,0],[1,0,0],.5,.12);
  update(.05,null); update(.01,null);
  assert.equal(update(.01,'move').action,null);
  update(.05,null); assert.equal(update(.01,'move').action,'move');
  hand.reset(); assert.equal(update(.01,'opening').action,null);
});
test('lost tracking and pinch away from a handle cannot activate an arm', () => {
  const hand = new PinchGrab();
  hand.update(.05, true); hand.update(.01, true);
  hand.update(NaN, true);
  assert.equal(hand.update(.01, true).grab, false);
  hand.update(.05, false);
  assert.deepEqual(hand.update(.01, false), {grab:false, pressed:true});
  assert.equal(hand.update(.01, true).grab, false);
});
test('an episode button pinch cannot acquire an arm until the hand releases', () => {
  const hand=new ToolPinch();
  const update=(distance,target)=>hand.update(distance,target,[0,0,0],[1,0,0],1,.12);
  update(.05,null);
  assert.equal(update(.01,'ui:record').action,'ui:record');
  assert.equal(update(.01,'move').action,'ui:record');
  assert.equal(update(.05,'move').action,null);
  assert.equal(update(.01,'move').action,'move');
  hand.reset();
  assert.equal(update(.01,'move').action,null);
});
test('free-space selection happens on release, excluding tool releases and cancelled pinches', () => {
  const hand = new ToolPinch();
  const update = (d, target = null) => hand.update(d, target, [0,0,0], [1,0,0], 1, .12);
  update(.05);
  assert.equal(update(.01).releasedFree, false);
  assert.equal(update(.028).releasedFree, false);
  assert.equal(update(.05).releasedFree, true);
  update(.01, 'move');
  assert.equal(update(.05).releasedFree, false);
  update(.01); hand.reset();
  assert.equal(update(.05).releasedFree, false);
});
test('a tool grab keeps its robot identity until release, independently of human handedness', () => {
  const hand = new ToolPinch();
  const update = (distance, arm, action = 'opening') => hand.update(distance, action, [0,0,0], [1,0,0], .6, .12, arm);
  update(.05, 'right'); update(.01, 'right');
  assert.equal(hand.arm, 'right'); assert.equal(hand.opening, .6);
  update(.01, 'left', 'move');
  assert.equal(hand.arm, 'right'); assert.equal(hand.action, 'opening');
  update(.05, 'left'); assert.equal(hand.arm, null);
  update(.01, 'left', 'move'); assert.equal(hand.arm, 'left');
  hand.reset(); assert.equal(hand.arm, null);
});
