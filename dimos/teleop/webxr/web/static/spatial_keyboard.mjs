// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
// Keyboard intent uses the same world-space clutch protocol as tracked inputs.
const KEY_TIMEOUT_MS = 250, SPEED_M_S = 0.06, MAX_LEAD_M = 0.08;
const AXES = {
  KeyW: [1, 0, 0],
  KeyS: [-1, 0, 0],
  KeyA: [0, 1, 0],
  KeyD: [0, -1, 0],
  KeyQ: [0, 0, 1],
  KeyE: [0, 0, -1],
};
export class KeyboardClutch {
  constructor() {
    this.keys = [];
    this.receivedAt = -Infinity;
    this.pose = null;
    this.blocked = false;
    this.frame = "world";
  }
  update(keys, frame, now) {
    this.keys = keys;
    this.frame = frame;
    this.receivedAt = now;
  }
  reset() {
    this.pose = null;
    this.blocked = this.keys.some((key) => AXES[key]);
  }
  sample(measured, ready, now, dt) {
    const active = this.keys.some((key) => AXES[key]);
    const release = {
      position: measured.grasp,
      orientation: measured.orientation,
      pinched: false,
      trigger: 1 - measured.opening,
    };
    if (!ready || now - this.receivedAt > KEY_TIMEOUT_MS) {
      this.reset();
      return release;
    }
    if (!active) {
      this.pose = null;
      this.blocked = false;
      return release;
    }
    if (this.blocked) return release;
    if (!this.pose) {
      this.pose = [...measured.grasp];
      return { ...release, position: this.pose, pinched: true };
    }
    let delta = [0, 0, 0];
    for (const key of this.keys) {
      if (AXES[key]) delta = delta.map((v, i) => v + AXES[key][i]);
    }
    const slow = this.keys.includes("ShiftLeft") ||
      this.keys.includes("ShiftRight");
    const length = Math.hypot(...delta),
      step = SPEED_M_S * Math.min(.05, Math.max(0, dt)) * (slow ? 0.25 : 1);
    delta = delta.map((v) => length ? v * step / length : 0);
    if (this.frame === "tool") {
      const [x, y, z, w] = measured.orientation, [a, b, c] = delta;
      const u = [y * c - z * b, z * a - x * c, x * b - y * a],
        v = [y * u[2] - z * u[1], z * u[0] - x * u[2], x * u[1] - y * u[0]];
      delta = delta.map((d, i) => d + 2 * (w * u[i] + v[i]));
    }
    this.pose = this.pose.map((v, i) => v + delta[i]);
    const lead = this.pose.map((v, i) => v - measured.grasp[i]),
      distance = Math.hypot(...lead);
    if (distance > MAX_LEAD_M) {
      this.pose = measured.grasp.map((v, i) =>
        v + lead[i] * MAX_LEAD_M / distance
      );
    }
    return { ...release, position: this.pose, pinched: true };
  }
}
