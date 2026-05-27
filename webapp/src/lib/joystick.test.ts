import { describe, it, expect } from "vitest";
import { computeDrive } from "./joystick";

const R = 100;

describe("computeDrive", () => {
  it("returns zero at center", () => {
    expect(computeDrive(0, 0, R)).toEqual({ vx: 0, turn: 0, knobX: 0, knobY: 0 });
  });

  it("full up = full forward", () => {
    const d = computeDrive(0, -R, R);
    expect(d.vx).toBeCloseTo(1, 5);
    expect(d.turn).toBeCloseTo(0, 5);
  });

  it("full down = full backward", () => {
    expect(computeDrive(0, R, R).vx).toBeCloseTo(-1, 5);
  });

  it("right = turn right, no forward", () => {
    const d = computeDrive(R, 0, R);
    expect(d.turn).toBeCloseTo(1, 5);
    expect(d.vx).toBeCloseTo(0, 5);
  });

  it("left = turn left", () => {
    expect(computeDrive(-R, 0, R).turn).toBeCloseTo(-1, 5);
  });

  it("clamps magnitude beyond the radius", () => {
    const d = computeDrive(0, -2 * R, R);
    expect(d.vx).toBeCloseTo(1, 5); // not 2
    expect(d.knobY).toBeCloseTo(-R, 5); // knob pinned to the rim
  });

  it("applies a dead zone near center", () => {
    const d = computeDrive(0, -0.05 * R, R); // 5% < 12% dead zone
    expect(d.vx).toBe(0);
    expect(d.turn).toBe(0);
  });

  it("normalizes a diagonal to the rim", () => {
    const d = computeDrive(R, -R, R); // distance = R*sqrt(2)
    expect(d.vx).toBeCloseTo(Math.SQRT1_2, 3);
    expect(d.turn).toBeCloseTo(Math.SQRT1_2, 3);
  });
});
