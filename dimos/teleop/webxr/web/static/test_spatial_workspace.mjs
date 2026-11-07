// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {WorkspaceTransform} from './spatial_workspace.mjs';

const hand = (position, closed = true, available = true) => ({position, distance: closed ? .01 : .05, available});
const pair = (left = [-.5, 1, -1], right = [.5, 1, -1], closed = true) => ({left: hand(left, closed), right: hand(right, closed)});
const feed = (w, hands, now = 100, pivot = [0, 0, -3], blocked = false) => w.update(hands, pivot, now, blocked);
const close = (a, b) => a.forEach((v, i) => assert.ok(Math.abs(v-b[i]) < 1e-9, `${a} != ${b}`));
const grab = (w, pivot = [0, 0, -3]) => {
  feed(w, pair(undefined, undefined, false), 0, pivot);
  assert.equal(feed(w, pair(), 100, pivot), true);
};

test('two fresh free-air pinches start anywhere; one alone leaves the workspace parked', () => {
  const w = new WorkspaceTransform({position: [0, 0, -3]});
  feed(w, pair([3, 1, 0], [4, 1, 0], false), 0);
  assert.equal(feed(w, {left: hand([3, 1, 0]), right: hand([4, 1, 0], false)}, 100), false);
  close(w.position, [0, 0, -3]);
  assert.equal(feed(w, pair([3, 1, 0], [4, 1, 0]), 300), true);
  feed(w, pair([3.1, 1.2, .3], [4.1, 1.2, .3]), 320);
  close(w.position, [.1, .2, -2.7]);
});

test('spreading distant hands scales around the workspace, without pushing it away', () => {
  const w = new WorkspaceTransform({position: [0, 0, -3]});
  grab(w);
  feed(w, pair([-1, 1, -1], [1, 1, -1]));
  assert.equal(w.scale, 2);
  close(w.position, [0, 0, -3]);
});

test('two hands translate, scale and yaw about the workstation pivot while staying upright', () => {
  const w = new WorkspaceTransform({position: [0, .3, -2]});
  grab(w);
  feed(w, pair([.5, 1.2, .2], [.5, 1.2, -1.8]));
  close([...w.position, w.yaw, w.scale], [2.5, .8, -2.8, Math.PI/2, 2]);
});

test('release either hand to continue dragging and rejoin without a jump', () => {
  for (const kept of ['left', 'right']) {
    const w = new WorkspaceTransform(); grab(w);
    const hands = pair(), released = kept === 'left' ? 'right' : 'left';
    hands[released].distance = .05;
    const before = [...w.position];
    feed(w, hands); close(w.position, before);
    hands[kept].position = hands[kept].position.map((v, i) => v + [.1, .2, -.3][i]);
    feed(w, hands); close(w.position, before.map((v, i) => v + [.1, .2, -.3][i]));
    const moved = [...w.position];
    hands[released].distance = .01; feed(w, hands); close(w.position, moved);
    for (const h of Object.values(hands)) h.distance = .05;
    assert.equal(feed(w, hands), true); assert.equal(w.active, false);
    assert.equal(feed(w, hands), false); close(w.position, moved);
  }
});

test('tool/UI targets and existing ownership cannot become workspace grabs without release', () => {
  const w = new WorkspaceTransform();
  feed(w, pair(undefined, undefined, false), 0);
  const occupied = pair(); occupied.left.available = false;
  assert.equal(feed(w, occupied), false);
  assert.equal(feed(w, pair(), 150), false);
  feed(w, pair(undefined, undefined, false), 200);
  assert.equal(feed(w, pair(), 250, undefined, true), false);
  assert.equal(feed(w, pair(), 300), false);
  feed(w, pair(undefined, undefined, false), 400);
  assert.equal(feed(w, pair(), 500), true);
});

test('an old single pinch cannot combine with a later pinch to start arrangement', () => {
  const w = new WorkspaceTransform();
  feed(w, pair(undefined, undefined, false), 0);
  feed(w, {left: hand([-.5, 1, -1]), right: hand([.5, 1, -1], false)}, 100);
  assert.equal(feed(w, pair(), 600), false);
  feed(w, pair(undefined, undefined, false), 700);
  assert.equal(feed(w, pair(), 800), true);
});

test('losing either owned hand freezes the whole view and requires fresh pinches', () => {
  const w = new WorkspaceTransform(); grab(w);
  const before = [...w.position];
  assert.equal(feed(w, {left: hand([2, 2, 2])}), true);
  assert.equal(w.active, false); close(w.position, before);
  assert.equal(feed(w, pair()), false); close(w.position, before);
  feed(w, pair(undefined, undefined, false), 200);
  assert.equal(feed(w, pair(), 300), true);
});

test('scale limits and degenerate or invalid hand samples cannot corrupt placement', () => {
  const w = new WorkspaceTransform(); grab(w);
  feed(w, pair([-2, 1, -1], [2, 1, -1])); assert.equal(w.scale, 2);
  feed(w, pair([-.05, 1, -1], [.05, 1, -1])); assert.equal(w.scale, .25);
  const yaw = w.yaw;
  feed(w, pair([0, 1, -1], [0, 1, -1]));
  feed(w, pair([0, .5, -1], [0, 1.5, -1]));
  feed(w, pair([0, 1, -.5], [0, 1, -1.5]));
  assert.equal(w.yaw, yaw);
  assert.ok([...w.position, w.scale].every(Number.isFinite));
  const before = [...w.position];
  feed(w, pair([NaN, 1, -1], [.5, 1, -1]));
  assert.equal(w.active, false); close(w.position, before);
});

test('actual size keeps its pivot fixed and bringing the workspace here preserves scale', () => {
  const w = new WorkspaceTransform({position: [1, 2, 3], scale: 2});
  w.resize(1, [0, 1, 0]); close(w.position, [.5, 1.5, 1.5]);
  w.resize(.5, [0, 1, 0]); w.place([2, 3, 4], .7);
  close([...w.position, w.yaw, w.scale], [2, 3, 4, .7, .5]);
  assert.equal(w.active, false);
});
