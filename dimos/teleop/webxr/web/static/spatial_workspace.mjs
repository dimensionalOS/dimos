// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import {PinchGrab} from './spatial_input.mjs';

const SIDES = ['left', 'right'];
const MIN_SPAN = 0.08; // XR metres; avoid unstable rotation when hands meet or stack vertically.
const AIR_GRAB_WINDOW_MS = 450;
const add = (a, b) => a.map((v, i) => v + b[i]);
const subtract = (a, b) => a.map((v, i) => v - b[i]);
const multiply = (a, s) => a.map(v => v * s);
const rotate = (p, yaw) => [
  Math.cos(yaw) * p[0] + Math.sin(yaw) * p[2], p[1],
  -Math.sin(yaw) * p[0] + Math.cos(yaw) * p[2],
];

// Viewer placement in XR coordinates. No robot state, scene truth or command transport.
export class WorkspaceTransform {
  constructor({position = [0, 0.8, -0.15], yaw = 0, scale = 1, minScale = 0.25, maxScale = 2} = {}) {
    this.position = [...position]; this.yaw = yaw; this.scale = scale;
    this.minScale = minScale; this.maxScale = maxScale;
    this.pinches = Object.fromEntries(SIDES.map(side => [side, new PinchGrab()]));
    this.started = {};
    this.reference = null;
  }

  get active() { return this.reference !== null; }

  cancel() {
    this.reference = null;
    this.started = {};
    for (const pinch of Object.values(this.pinches)) pinch.reset();
  }

  place(position, yaw) {
    this.cancel(); this.position = [...position]; this.yaw = yaw;
  }

  resize(scale, anchor) {
    this.cancel();
    const next = Math.max(this.minScale, Math.min(this.maxScale, scale));
    this.position = add(anchor, multiply(subtract(this.position, anchor), next / this.scale));
    this.scale = next;
  }

  // True means this frame belongs to arrangement, including the final release frame.
  update(hands, pivot, now, blocked = false) {
    const wasActive = this.active;
    const valid = side => hands[side]?.position.length === 3 && hands[side].position.every(Number.isFinite)
      && Number.isFinite(hands[side].distance);
    if ((!wasActive && blocked) || (wasActive && this.reference.key.split(',').some(side => !valid(side)))) {
      this.cancel(); return wasActive;
    }
    const held = [];
    for (const side of SIDES) {
      const hand = hands[side], pinch = this.pinches[side];
      if (!valid(side)) {
        pinch.reset(); delete this.started[side]; continue;
      }
      const edge = pinch.update(hand.distance, wasActive || hand.available);
      if (edge.pressed && edge.grab) this.started[side] = now;
      if (edge.grab) held.push([side, hand.position]);
      else delete this.started[side];
    }
    if (!held.length) { this.reference = null; return wasActive; }
    if (!wasActive && (held.length !== 2 || !held.every(([side]) => hands[side].available)
      || now - Math.min(...Object.values(this.started)) > AIR_GRAB_WINDOW_MS)) return false;
    const key = held.map(([side]) => side).join(',');
    const center = multiply(held.reduce((sum, [, p]) => add(sum, p), [0, 0, 0]), 1 / held.length);
    const vector = held.length === 2 ? subtract(held[1][1], held[0][1]) : null;
    const stable = vector !== null && Math.hypot(vector[0], vector[2]) >= MIN_SPAN;
    const span = vector ? Math.hypot(...vector) : 0;
    const angle = vector ? Math.atan2(-vector[2], vector[0]) : 0;
    const capture = () => ({key, center, span, angle, stable, pivot: [...pivot],
      position: [...this.position], yaw: this.yaw, scale: this.scale});
    if (this.reference?.key !== key) {
      this.reference = capture();
      return true;
    }
    const start = this.reference;
    if (held.length === 1 || !stable || !start.stable) {
      this.position = add(start.position, subtract(center, start.center));
      // Degenerate two-hand poses retain rotation/scale and establish a fresh baseline.
      if (held.length === 2) this.reference = capture();
    } else {
      const yaw = Math.atan2(Math.sin(angle - start.angle), Math.cos(angle - start.angle));
      this.scale = Math.max(this.minScale, Math.min(this.maxScale, start.scale * span / start.span));
      this.yaw = start.yaw + yaw;
      const movedPivot = add(start.pivot, subtract(center, start.center));
      this.position = add(movedPivot, multiply(rotate(subtract(start.position, start.pivot), yaw), this.scale / start.scale));
    }
    return true;
  }
}
