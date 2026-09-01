// Pure rendering pieces for the map2d panel: occupancy palette, grid blit,
// and the world<->canvas transform. Nothing here touches the DOM beyond
// ImageData and calls on a caller-supplied 2D context, so it all unit-tests
// without a canvas.

import type { CostmapValue } from "@dimos/sdk";

export type GridPlacement = Pick<CostmapValue, "w" | "h" | "res" | "origin">;

function buildPalette(): Uint8ClampedArray {
  const lut = new Uint8ClampedArray(256 * 4);
  const set = (i: number, r: number, g: number, b: number, a: number): void => {
    lut[i * 4] = r;
    lut[i * 4 + 1] = g;
    lut[i * 4 + 2] = b;
    lut[i * 4 + 3] = a;
  };
  set(0, 0x1e, 0x3a, 0x44, 255); // free: dark cyan
  for (let v = 1; v <= 99; v++) set(v, 0x8f, 0xdc, 0xef, 255); // cost: bright cyan
  // 101..254 are outside the wire contract; rendering them lethal is the
  // conservative reading.
  for (let v = 100; v <= 254; v++) set(v, 255, 255, 255, 255);
  set(255, 0, 0, 0, 0); // unknown: transparent, the panel background shows
  return lut;
}

// Operator scheme from the hosted-teleop map (_occupancy_to_bgra in
// dimos/teleop/hosted/map_compress.py), indexed by the wire's uint8 cells.
export const OCCUPANCY_PALETTE: Uint8ClampedArray = buildPalette();

// The same palette as native-endian u32 pixels: one read + one write per
// cell keeps budget-sized grids (2048^2) off the frame budget.
const PALETTE32 = new Uint32Array(OCCUPANCY_PALETTE.buffer);

/**
 * Cells to a screen-oriented RGBA bitmap. Grid row 0 sits at world min-y
 * (ROS row-major with the origin at the lower-left corner) while canvas row 0
 * is the top, so rows flip here and the transform below stays purely metric.
 * Pass the previous frame's ImageData back in to reuse its buffer while the
 * dimensions are unchanged; every pixel (alpha included) is overwritten, so
 * stale content cannot leak through.
 */
export function gridToImageData(
  cells: Uint8Array,
  w: number,
  h: number,
  reuse?: ImageData,
): ImageData {
  if (cells.length !== w * h) {
    throw new Error(`grid is ${cells.length} cells, expected ${w}x${h}`);
  }
  const out = reuse !== undefined && reuse.width === w && reuse.height === h
    ? reuse
    : new ImageData(w, h);
  const px = new Uint32Array(out.data.buffer, out.data.byteOffset, w * h);
  for (let row = 0; row < h; row++) {
    const src = (h - 1 - row) * w;
    const dst = row * w;
    for (let col = 0; col < w; col++) {
      px[dst + col] = PALETTE32[cells[src + col]];
    }
  }
  return out;
}

/**
 * Aspect-preserving whole-grid fit, letterboxed and centered: the world-space
 * bounding box of the (possibly yaw-rotated) grid rectangle fills the canvas.
 * The canvas stays world-axis-aligned - only the grid blit rotates (gridBlit)
 * - so worldToCanvas needs no rotation term. T10 extends this transform
 * (zoom, minimap crop) rather than the callers.
 */
export interface MapTransform {
  /** Device pixels per world meter. */
  scale: number;
  /** World coordinates of the fitted bounding box's min corner (the grid's
   * lower-left origin corner when yaw is 0). */
  originX: number;
  originY: number;
  /** Canvas position of that corner (canvas y grows downward). */
  cx0: number;
  cy0: number;
}

export function fitTransform(place: GridPlacement, canvasW: number, canvasH: number): MapTransform {
  const worldW = place.w * place.res;
  const worldH = place.h * place.res;
  const c = Math.cos(place.origin[2]);
  const s = Math.sin(place.origin[2]);
  // Corner offsets from the origin corner are u = worldW*(c, s) and
  // v = worldH*(-s, c); the AABB spans their per-axis extremes.
  const bw = Math.abs(worldW * c) + Math.abs(worldH * s);
  const bh = Math.abs(worldW * s) + Math.abs(worldH * c);
  const scale = Math.min(canvasW / bw, canvasH / bh);
  return {
    scale,
    originX: place.origin[0] + Math.min(0, worldW * c) + Math.min(0, -worldH * s),
    originY: place.origin[1] + Math.min(0, worldW * s) + Math.min(0, worldH * c),
    cx0: (canvasW - bw * scale) / 2,
    cy0: canvasH - (canvasH - bh * scale) / 2,
  };
}

/** World meters to canvas pixels; the y-flip lives here. */
export function worldToCanvas(t: MapTransform, wx: number, wy: number): [number, number] {
  return [t.cx0 + (wx - t.originX) * t.scale, t.cy0 - (wy - t.originY) * t.scale];
}

/** Exact inverse of worldToCanvas (T10 click-to-goal reads this). */
export function canvasToWorld(t: MapTransform, cx: number, cy: number): [number, number] {
  return [t.originX + (cx - t.cx0) / t.scale, t.originY + (t.cy0 - cy) / t.scale];
}

/**
 * Placement for the (screen-oriented) grid bitmap: translate to (ax, ay) -
 * the canvas position of the grid's origin corner - rotate by `rot`, then
 * draw the bitmap into [0, -dh, dw, dh]. World yaw is CCW with y up while
 * canvas y grows down, so the canvas angle is -yaw; at yaw 0 this reduces to
 * the axis-aligned blit [ax, ay - dh, dw, dh].
 */
export function gridBlit(
  t: MapTransform,
  place: GridPlacement,
): { ax: number; ay: number; rot: number; dw: number; dh: number } {
  const [ax, ay] = worldToCanvas(t, place.origin[0], place.origin[1]);
  return {
    ax,
    ay,
    rot: -place.origin[2],
    dw: place.w * place.res * t.scale,
    dh: place.h * place.res * t.scale,
  };
}

export const POSE_COLOR = "#ff5c5c";
// Triangle length in CSS pixels: screen-constant so the marker stays legible
// however far the fit zooms out. The canvas backing store is DPR-scaled, so
// callers pass their dpr to keep the on-screen size constant.
const POSE_PX = 12;

export interface Pose2d {
  x: number;
  y: number;
  yaw: number;
}

/**
 * Triangle vertices ([nose, left, right]) for the pose marker. World yaw is
 * CCW-positive with y up; canvas y grows down, so the canvas angle is -yaw.
 */
export function posePath(t: MapTransform, pose: Pose2d, dpr = 1): [number, number][] {
  const [cx, cy] = worldToCanvas(t, pose.x, pose.y);
  const cos = Math.cos(-pose.yaw);
  const sin = Math.sin(-pose.yaw);
  const size = POSE_PX * dpr;
  const local: [number, number][] = [
    [size * 0.6, 0],
    [-size * 0.4, size * 0.35],
    [-size * 0.4, -size * 0.35],
  ];
  return local.map(([px, py]) => [cx + px * cos - py * sin, cy + px * sin + py * cos]);
}

export function drawPose(
  ctx: CanvasRenderingContext2D,
  t: MapTransform,
  pose: Pose2d,
  dpr = 1,
): void {
  const [nose, left, right] = posePath(t, pose, dpr);
  ctx.beginPath();
  ctx.moveTo(nose[0], nose[1]);
  ctx.lineTo(left[0], left[1]);
  ctx.lineTo(right[0], right[1]);
  ctx.closePath();
  ctx.fillStyle = POSE_COLOR;
  ctx.fill();
}
