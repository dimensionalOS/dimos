export interface DriveVector {
  /** forward (+) / backward (−), in [-1, 1] */
  vx: number;
  /** turn right (+) / left (−), in [-1, 1] */
  turn: number;
  /** clamped knob offset from center, in px (for rendering) */
  knobX: number;
  knobY: number;
}

/**
 * Map a pointer offset (dx, dy from the pad center, in px) to a drive vector.
 * Up = forward, right = turn right. Clamps to the pad rim and applies a dead zone.
 */
export function computeDrive(
  dx: number,
  dy: number,
  radius: number,
  deadZone = 0.12,
): DriveVector {
  const dist = Math.hypot(dx, dy);

  // clamp the knob to the rim
  const scale = dist > radius ? radius / dist : 1;
  const knobX = dx * scale;
  const knobY = dy * scale;

  const magnitude = Math.min(dist / radius, 1);
  if (magnitude < deadZone) {
    return { vx: 0, turn: 0, knobX, knobY };
  }

  return {
    vx: -knobY / radius, // screen y grows downward, so up is forward
    turn: knobX / radius,
    knobX,
    knobY,
  };
}
