import assert from "node:assert/strict";
import test from "node:test";
import { KeyboardClutch } from "./spatial_keyboard.mjs";
const measured = {
  grasp: [.3, .2, .4],
  orientation: [0, 0, 0, 1],
  opening: .7,
};
test("keyboard clutch moves relative to measured pose and releases without drift", () => {
  const input = new KeyboardClutch();
  input.update([], "world", 0);
  input.sample(measured, true, 0, .02);
  input.update(["KeyW"], "world", 10);
  input.sample(measured, true, 10, .02);
  const moving = input.sample(measured, true, 30, .02);
  assert.ok(moving.position[0] > .3);
  assert.equal(moving.position[1], .2);
  assert.equal(moving.trigger, 1 - .7);
  input.update([], "world", 40);
  assert.equal(input.sample(measured, true, 40, .02).pinched, false);
});
test("loss of key heartbeat requires release before fresh control", () => {
  const input = new KeyboardClutch();
  input.update(["KeyW"], "world", 0);
  input.sample(measured, true, 0, .02);
  assert.equal(input.sample(measured, true, 300, .02).pinched, false);
  input.update(["KeyW"], "world", 310);
  assert.equal(input.sample(measured, true, 310, .02).pinched, false);
  input.update([], "world", 320);
  input.sample(measured, true, 320, .02);
  input.update(["KeyW"], "world", 330);
  assert.equal(input.sample(measured, true, 330, .02).pinched, true);
});
test("tool-relative X follows tool orientation", () => {
  const input = new KeyboardClutch(),
    turned = { ...measured, orientation: [0, 0, Math.SQRT1_2, Math.SQRT1_2] };
  input.update(["KeyW"], "tool", 0);
  input.sample(turned, true, 0, .02);
  const moving = input.sample(turned, true, 20, .02);
  assert.ok(Math.abs(moving.position[0] - .3) < 1e-10);
  assert.ok(moving.position[1] > .2);
});
