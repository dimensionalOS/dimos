import { describe, expect, it } from "vitest";
import {
  canvasToWorld,
  fitTransform,
  gridBlit,
  gridToImageData,
  OCCUPANCY_PALETTE,
  posePath,
  worldToCanvas,
} from "./mapRenderer.ts";

function rgba(imageData: ImageData, x: number, y: number): number[] {
  const i = (y * imageData.width + x) * 4;
  return [...imageData.data.slice(i, i + 4)];
}

describe("occupancy palette", () => {
  it("maps the wire value classes to the operator scheme", () => {
    const at = (i: number) => [...OCCUPANCY_PALETTE.slice(i * 4, i * 4 + 4)];
    expect(at(0)).toEqual([0x1e, 0x3a, 0x44, 255]); // free
    expect(at(1)).toEqual([0x8f, 0xdc, 0xef, 255]); // cost
    expect(at(99)).toEqual([0x8f, 0xdc, 0xef, 255]);
    expect(at(100)).toEqual([255, 255, 255, 255]); // lethal
    expect(at(254)).toEqual([255, 255, 255, 255]); // out of contract: lethal
    expect(at(255)).toEqual([0, 0, 0, 0]); // unknown: transparent
  });
});

describe("gridToImageData", () => {
  it("blits through the palette with grid row 0 at the bottom", () => {
    // Row-major cells: grid row 0 = [free, cost, unknown] sits at world
    // min-y, so it must land on the BOTTOM canvas row.
    const cells = Uint8Array.from([0, 50, 255, 100, 0, 255]);
    const img = gridToImageData(cells, 3, 2);
    expect([img.width, img.height]).toEqual([3, 2]);
    expect(rgba(img, 0, 0)).toEqual([255, 255, 255, 255]); // top-left: grid row 1 lethal
    expect(rgba(img, 1, 0)).toEqual([0x1e, 0x3a, 0x44, 255]);
    expect(rgba(img, 0, 1)).toEqual([0x1e, 0x3a, 0x44, 255]); // bottom-left: grid row 0 free
    expect(rgba(img, 1, 1)).toEqual([0x8f, 0xdc, 0xef, 255]);
    expect(rgba(img, 2, 1)).toEqual([0, 0, 0, 0]);
  });

  it("rejects a cell count that does not match the dimensions", () => {
    expect(() => gridToImageData(new Uint8Array(5), 3, 2)).toThrow(/expected 3x2/);
  });

  it("reuses the caller's ImageData when dimensions match", () => {
    const first = gridToImageData(Uint8Array.from([0, 100, 50, 255]), 2, 2);
    expect(rgba(first, 0, 0)).toEqual([0x8f, 0xdc, 0xef, 255]); // grid row 1: cost
    const second = gridToImageData(Uint8Array.from([255, 0, 255, 0]), 2, 2, first);
    expect(second).toBe(first);
    // Every pixel is overwritten, alpha included: the previously opaque
    // top-left is now the transparent unknown.
    expect(rgba(second, 0, 0)).toEqual([0, 0, 0, 0]);
    expect(rgba(second, 1, 0)).toEqual([0x1e, 0x3a, 0x44, 255]);
  });

  it("allocates fresh when dimensions differ", () => {
    const first = gridToImageData(new Uint8Array(4), 2, 2);
    const second = gridToImageData(new Uint8Array(6), 3, 2, first);
    expect(second).not.toBe(first);
    expect([second.width, second.height]).toEqual([3, 2]);
  });
});

describe("map transform", () => {
  // 4x2 cells at 0.5 m: a 2x1 m world rect with the lower-left at (-1, 2).
  const place = { w: 4, h: 2, res: 0.5, origin: [-1.0, 2.0, 0.0] as [number, number, number] };

  it("fits, centers, and letterboxes the grid", () => {
    const t = fitTransform(place, 200, 200);
    expect(t.scale).toBe(100); // width-bound: 200 px / 2 m
    expect(worldToCanvas(t, -1.0, 2.0)).toEqual([0, 150]); // lower-left corner
    expect(worldToCanvas(t, 1.0, 3.0)).toEqual([200, 50]); // upper-right corner
    const blit = gridBlit(t, place);
    expect([blit.ax, blit.ay, blit.dw, blit.dh]).toEqual([0, 150, 200, 100]);
    expect(blit.rot).toBeCloseTo(0, 12); // -0 at yaw 0, so no toEqual
  });

  it("keeps the y-flip consistent: larger world y is smaller canvas y", () => {
    const t = fitTransform(place, 200, 200);
    const [, low] = worldToCanvas(t, 0, 2.0);
    const [, high] = worldToCanvas(t, 0, 3.0);
    expect(high).toBeLessThan(low);
  });

  it("round-trips world -> canvas -> world exactly", () => {
    const t = fitTransform(place, 317, 203); // deliberately awkward canvas
    for (const [wx, wy] of [[-1.0, 2.0], [0.25, 2.75], [1.0, 3.0], [-0.125, 2.5]]) {
      const [cx, cy] = worldToCanvas(t, wx, wy);
      const [rx, ry] = canvasToWorld(t, cx, cy);
      expect(rx).toBeCloseTo(wx, 9);
      expect(ry).toBeCloseTo(wy, 9);
    }
  });
});

describe("map transform with yaw", () => {
  // The with_yaw golden vector's placement (costmap_frames.json): a 0.3 m
  // square rotated 0.25 rad CCW about its origin corner (1.5, -0.75). All
  // expectations below are hand-computed, not derived from the transform.
  const yaw = 0.25;
  const place = { w: 3, h: 3, res: 0.1, origin: [1.5, -0.75, yaw] as [number, number, number] };

  it("fits the rotated grid's bounding box", () => {
    // AABB side: 0.3*(cos+sin)(0.25) = 0.3648949; height-bound in 200x100.
    const t = fitTransform(place, 200, 100);
    expect(t.scale).toBeCloseTo(274.05150, 4);
    expect(t.originX).toBeCloseTo(1.4257788, 6); // ox - 0.3*sin(yaw)
    expect(t.originY).toBeCloseTo(-0.75, 9); // all corner y-offsets are >= 0
    expect(t.cx0).toBeCloseTo(50, 6); // square AABB letterboxed in x
    expect(t.cy0).toBeCloseTo(100, 6);
  });

  it("anchors the blit at the origin corner, rotated by -yaw", () => {
    const t = fitTransform(place, 200, 100);
    const blit = gridBlit(t, place);
    expect(blit.ax).toBeCloseTo(70.34043, 4);
    expect(blit.ay).toBeCloseTo(100, 6);
    expect(blit.rot).toBeCloseTo(-yaw, 12);
    expect(blit.dw).toBeCloseTo(82.21545, 4);
    expect(blit.dh).toBeCloseTo(82.21545, 4);
  });

  it("lands every bitmap corner on its world corner", () => {
    // Bitmap corners pushed through translate(ax, ay) + rotate(rot) must
    // coincide with worldToCanvas of the true world corners; any sign error
    // in the yaw handling breaks this closure.
    const t = fitTransform(place, 200, 100);
    const { ax, ay, rot, dw, dh } = gridBlit(t, place);
    const u = [0.3 * Math.cos(yaw), 0.3 * Math.sin(yaw)]; // grid +x in world
    const v = [-0.3 * Math.sin(yaw), 0.3 * Math.cos(yaw)]; // grid +y in world
    const world: [number, number][] = [
      [1.5, -0.75],
      [1.5 + u[0], -0.75 + u[1]],
      [1.5 + u[0] + v[0], -0.75 + u[1] + v[1]],
      [1.5 + v[0], -0.75 + v[1]],
    ];
    const local: [number, number][] = [[0, 0], [dw, 0], [dw, -dh], [0, -dh]];
    for (let i = 0; i < 4; i++) {
      const [lx, ly] = local[i];
      const cx = ax + lx * Math.cos(rot) - ly * Math.sin(rot);
      const cy = ay + lx * Math.sin(rot) + ly * Math.cos(rot);
      const [ex, ey] = worldToCanvas(t, world[i][0], world[i][1]);
      expect(cx).toBeCloseTo(ex, 6);
      expect(cy).toBeCloseTo(ey, 6);
    }
  });

  it("round-trips world -> canvas -> world under yaw", () => {
    const t = fitTransform(place, 317, 203);
    for (const [wx, wy] of [[1.5, -0.75], [1.6, -0.6], [1.79, -0.45]]) {
      const [cx, cy] = worldToCanvas(t, wx, wy);
      const [rx, ry] = canvasToWorld(t, cx, cy);
      expect(rx).toBeCloseTo(wx, 9);
      expect(ry).toBeCloseTo(wy, 9);
    }
  });
});

describe("posePath", () => {
  // 100x100 cells at 1 m in a 100x100 canvas: scale 1, no letterbox.
  const t = fitTransform(
    { w: 100, h: 100, res: 1.0, origin: [0.0, 0.0, 0.0] },
    100,
    100,
  );

  it("points the nose along +x world for yaw 0", () => {
    const [nose, left, right] = posePath(t, { x: 50, y: 50, yaw: 0 });
    const [cx, cy] = worldToCanvas(t, 50, 50);
    expect(nose[0]).toBeGreaterThan(cx);
    expect(nose[1]).toBeCloseTo(cy, 9);
    expect(left[0]).toBeLessThan(cx);
    expect(right[0]).toBeLessThan(cx);
  });

  it("points the nose up the canvas for yaw +pi/2 (world +y)", () => {
    const [nose] = posePath(t, { x: 50, y: 50, yaw: Math.PI / 2 });
    const [cx, cy] = worldToCanvas(t, 50, 50);
    expect(nose[1]).toBeLessThan(cy); // canvas y grows down
    expect(nose[0]).toBeCloseTo(cx, 9);
  });

  it("scales the marker by dpr so its on-screen size is constant", () => {
    const [nose, left, right] = posePath(t, { x: 50, y: 50, yaw: 0 }, 2);
    const [cx, cy] = worldToCanvas(t, 50, 50);
    expect(nose[0] - cx).toBeCloseTo(14.4, 9); // 12 css px * 0.6 * dpr 2
    expect(nose[1]).toBeCloseTo(cy, 9);
    expect(left[0] - cx).toBeCloseTo(-9.6, 9);
    expect(left[1] - cy).toBeCloseTo(8.4, 9);
    expect(right[1] - cy).toBeCloseTo(-8.4, 9);
  });
});
