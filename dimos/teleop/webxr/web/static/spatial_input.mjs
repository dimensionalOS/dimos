// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
// Hysteresis and release-before-grab survive tracking loss without a surprise resume.
export class PinchGrab {
  constructor() { this.reset(); }
  reset() { this.armed = false; this.pinched = false; this.grabbed = false; }
  update(distance, nearHandle) {
    const before = this.pinched;
    if (!Number.isFinite(distance)) { this.reset(); return {grab: false, pressed: false}; }
    if (distance > 0.035) { this.pinched = false; this.armed = true; this.grabbed = false; }
    if (distance < 0.022 && this.armed && !this.pinched) {
      this.pinched = true; this.armed = false; this.grabbed = nearHandle;
    }
    return {grab: this.grabbed, pressed: this.pinched && !before};
  }
}

// One tool action belongs to a pinch until release, even if the hand crosses
// another control. Opening is a relative drag, seeded from measured feedback.
export class ToolPinch {
  constructor() { this.reset(); }
  reset() {
    this.pinch = new PinchGrab(); this.action = null; this.arm = null; this.opening = null;
    this.origin = null; this.axis = null; this.initial = 0; this.travel = 1;
  }
  get grabbed() { return this.action !== null; }
  update(distance, target, position, axis, measuredOpening, travel, arm = null) {
    const releasedFree=this.pinch.pinched && this.action===null && distance>0.035;
    const edge = this.pinch.update(distance, target !== null);
    if (edge.pressed && edge.grab) {
      this.action = target; this.arm = arm;
      this.origin = [...position]; this.axis = [...axis];
      this.initial = measuredOpening; this.travel = travel;
    }
    if (!edge.grab) { this.action = null; this.arm = null; this.opening = null; }
    if (this.action === 'opening') {
      const delta = position.reduce((sum, v, i) => sum + (v-this.origin[i])*this.axis[i], 0);
      this.opening = Math.max(0, Math.min(1, this.initial + delta/this.travel));
    }
    return {pinched: this.pinch.pinched, action: this.action, pressed: edge.pressed, releasedFree};
  }
}
