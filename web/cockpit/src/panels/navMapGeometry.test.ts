import { describe, expect, it } from "vitest";
import { canvasToWorld, fitTransform, worldToCanvas } from "./mapRenderer.ts";
import {
  clickToWorld,
  FALLBACK_HALF_M,
  fallbackTransform,
  GOAL_LIMIT_M,
  goalPayload,
  hitLabel,
  type LabelBox,
  readPath,
  readPlaces,
} from "./navMapGeometry.ts";

describe("fallbackTransform", () => {
  it("fits world (-2.2..2.2)^2 into the canvas, centred, y up", () => {
    const t = fallbackTransform(400, 300);
    expect(t.scale).toBeCloseTo(300 / (2 * FALLBACK_HALF_M));
    // World origin lands at the canvas centre.
    expect(worldToCanvas(t, 0, 0)).toEqual([200, 150]);
    // +y world is up on screen; +x is right.
    const [tx, ty] = worldToCanvas(t, FALLBACK_HALF_M, FALLBACK_HALF_M);
    expect(tx).toBeCloseTo(200 + 150);
    expect(ty).toBeCloseTo(0);
    const [bx, by] = worldToCanvas(t, -FALLBACK_HALF_M, -FALLBACK_HALF_M);
    expect(bx).toBeCloseTo(50);
    expect(by).toBeCloseTo(300);
  });
});

describe("clickToWorld", () => {
  it("maps a CSS click through the DPR-scaled backing store to world metres", () => {
    // 2x DPR: backing store 800x600 behind a 400x300 CSS box.
    const canvas = { width: 800, height: 600, clientWidth: 400, clientHeight: 300 };
    const t = fallbackTransform(canvas.width, canvas.height);
    expect(clickToWorld(t, 200, 150, canvas)[0]).toBeCloseTo(0);
    expect(clickToWorld(t, 200, 150, canvas)[1]).toBeCloseTo(0);
    // Top-right corner of the fitted square: (+2.2, +2.2).
    const [x, y] = clickToWorld(t, 350, 0, canvas);
    expect(x).toBeCloseTo(FALLBACK_HALF_M);
    expect(y).toBeCloseTo(FALLBACK_HALF_M);
    // Quarter of the way right of centre, a third down: 1.1 m, -0.733 m.
    const [qx, qy] = clickToWorld(t, 275, 200, canvas);
    expect(qx).toBeCloseTo(1.1);
    expect(qy).toBeCloseTo(-0.7333, 3);
  });

  it("inverts worldToCanvas for a real costmap placement (rotated origin too)", () => {
    const place = {
      w: 120,
      h: 80,
      res: 0.05,
      origin: [-1.5, -2.0, 0.3] as [number, number, number],
    };
    const canvas = { width: 300, height: 200, clientWidth: 300, clientHeight: 200 };
    const t = fitTransform(place, canvas.width, canvas.height);
    const [cx, cy] = worldToCanvas(t, 0.7, -0.4);
    const [wx, wy] = clickToWorld(t, cx, cy, canvas);
    expect(wx).toBeCloseTo(0.7);
    expect(wy).toBeCloseTo(-0.4);
    expect(canvasToWorld(t, cx, cy)).toEqual([wx, wy]);
  });

  it("falls back to a 1:1 scale when the CSS box has no size", () => {
    const t = fallbackTransform(100, 100);
    const canvas = { width: 100, height: 100, clientWidth: 0, clientHeight: 0 };
    expect(clickToWorld(t, 50, 50, canvas)[0]).toBeCloseTo(0);
  });
});

describe("goalPayload", () => {
  it("rounds to millimetres, tags the frame, and refuses far-off points", () => {
    expect(goalPayload(1.23456, -0.00049)).toEqual({ x: 1.235, y: -0, frame: "world" });
    expect(goalPayload(1, 2, 3.14159)).toEqual({ x: 1, y: 2, frame: "world", yaw: 3.142 });
    expect(goalPayload(GOAL_LIMIT_M + 0.01, 0)).toBeNull();
    expect(goalPayload(0, -GOAL_LIMIT_M - 1)).toBeNull();
    expect(goalPayload(Number.NaN, 0)).toBeNull();
  });
});

describe("hitLabel", () => {
  const labels: LabelBox[] = [
    { name: "kitchen", target: [1.2, 1, 0], x: 100, y: 50, w: 60, h: 20 },
    { name: "living", target: [-1.2, 1, 3.14], x: 140, y: 60, w: 60, h: 20 },
  ];

  it("returns the box under the point, the last drawn winning on overlap", () => {
    expect(hitLabel(labels, 110, 55)?.name).toBe("kitchen");
    expect(hitLabel(labels, 150, 65)?.name).toBe("living"); // overlap: topmost
    expect(hitLabel(labels, 190, 70)?.name).toBe("living");
    expect(hitLabel(labels, 10, 10)).toBeNull();
    expect(hitLabel([], 110, 55)).toBeNull();
  });
});

describe("readPlaces / readPath", () => {
  it("reads places.json.v1 rooms, objects and tagged places, skipping junk", () => {
    const places = readPlaces({
      frame: "world",
      rooms: [
        {
          name: "kitchen",
          aliases: ["space A", 3],
          bounds: [0, 2, 0, 2],
          target: [1.2, 1, 0],
        },
        { name: "broken", bounds: [0, 1] },
      ],
      objects: [{ name: "red_box", x: 1.5, y: 1.5 }, { name: "nope" }],
      tagged: [{ name: "charger", x: 0.1, y: -0.2 }],
    });
    expect(places).toEqual({
      frame: "world",
      rooms: [{ name: "kitchen", aliases: ["space A"], bounds: [0, 2, 0, 2], target: [1.2, 1, 0] }],
      objects: [{ name: "red_box", x: 1.5, y: 1.5 }],
      tagged: [{ name: "charger", x: 0.1, y: -0.2, yaw: 0 }],
    });
    expect(readPlaces(null)).toBeNull();
    expect(readPlaces({})?.rooms).toEqual([]);
  });

  it("reads path.json.v1 point lists", () => {
    expect(readPath({ frame: "world", points: [[0, 0], [0.5, 0.1], ["x", 1]] })).toEqual({
      frame: "world",
      points: [[0, 0], [0.5, 0.1]],
    });
    expect(readPath({ frame: "world" })).toBeNull();
  });
});
