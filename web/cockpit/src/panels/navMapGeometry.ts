// Pure geometry and payload readers for the nav map overlay: fallback
// transform before the first costmap, click -> world, room-label hit test,
// and the places/path frame readers. No DOM, so the click math is unit
// tested without a canvas.

import {
  canvasToWorld,
  fitTransform,
  type GridPlacement,
  type MapTransform,
} from "./mapRenderer.ts";
import { readNumber, readString } from "./panelParams.ts";

/** Before any costmap: fit world (-2.2..2.2)^2 (the four-room scene plus a
 * margin) so rooms and places are visible immediately. */
export const FALLBACK_HALF_M = 2.2;
export const FALLBACK_PLACE: GridPlacement = {
  w: 1,
  h: 1,
  res: 2 * FALLBACK_HALF_M,
  origin: [-FALLBACK_HALF_M, -FALLBACK_HALF_M, 0],
};

export function fallbackTransform(canvasW: number, canvasH: number): MapTransform {
  return fitTransform(FALLBACK_PLACE, canvasW, canvasH);
}

/** The subset of a canvas the click math needs (DPR-scaled backing store vs
 * CSS box). */
export interface CanvasBox {
  width: number;
  height: number;
  clientWidth: number;
  clientHeight: number;
}

/** A click at CSS-pixel offset (cssX, cssY) inside the canvas -> world meters.
 * The backing store is DPR-scaled (MapPanel sizes it), so the offset is
 * scaled by width/clientWidth before the inverse transform. */
export function clickToWorld(
  t: MapTransform,
  cssX: number,
  cssY: number,
  canvas: CanvasBox,
): [number, number] {
  const sx = canvas.clientWidth > 0 ? canvas.width / canvas.clientWidth : 1;
  const sy = canvas.clientHeight > 0 ? canvas.height / canvas.clientHeight : 1;
  return canvasToWorld(t, cssX * sx, cssY * sy);
}

/** Bridge-side goal bound (pose_goal.json.v1: |x|, |y| <= 50). */
export const GOAL_LIMIT_M = 50;

export function roundCoord(v: number): number {
  return Math.round(v * 1000) / 1000;
}

/** The pose_goal.json.v1 payload for a world point, or null when the point
 * is outside what the bridge accepts (a click far off the map). */
export function goalPayload(x: number, y: number, yaw?: number): Record<string, unknown> | null {
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  if (Math.abs(x) > GOAL_LIMIT_M || Math.abs(y) > GOAL_LIMIT_M) return null;
  const goal: Record<string, unknown> = { x: roundCoord(x), y: roundCoord(y), frame: "world" };
  if (yaw !== undefined && Number.isFinite(yaw)) goal.yaw = roundCoord(yaw);
  return goal;
}

export interface Room {
  name: string;
  aliases: string[];
  /** [xmin, xmax, ymin, ymax] in world meters. */
  bounds: [number, number, number, number];
  /** [x, y, yaw]: where "go to the room" navigates. */
  target: [number, number, number];
}

export interface PlaceObject {
  name: string;
  x: number;
  y: number;
}

export interface TaggedPlace {
  name: string;
  x: number;
  y: number;
  yaw: number;
}

export interface Places {
  frame: string;
  rooms: Room[];
  objects: PlaceObject[];
  tagged: TaggedPlace[];
}

function numbers(v: unknown, count: number): number[] | null {
  if (!Array.isArray(v) || v.length < count) return null;
  const out: number[] = [];
  for (let i = 0; i < count; i++) {
    const n = readNumber(v[i]);
    if (n === null) return null;
    out.push(n);
  }
  return out;
}

export function readPlaces(v: unknown): Places | null {
  if (typeof v !== "object" || v === null) return null;
  const o = v as Record<string, unknown>;
  const rooms: Room[] = [];
  if (Array.isArray(o.rooms)) {
    for (const r of o.rooms) {
      if (typeof r !== "object" || r === null) continue;
      const e = r as Record<string, unknown>;
      const name = readString(e.name);
      const bounds = numbers(e.bounds, 4);
      const target = numbers(e.target, 2);
      if (name === null || bounds === null || target === null) continue;
      const yaw = Array.isArray(e.target) ? readNumber(e.target[2]) ?? 0 : 0;
      rooms.push({
        name,
        aliases: Array.isArray(e.aliases) ? e.aliases.filter((a) => typeof a === "string") : [],
        bounds: [bounds[0], bounds[1], bounds[2], bounds[3]],
        target: [target[0], target[1], yaw],
      });
    }
  }
  const objects: PlaceObject[] = [];
  if (Array.isArray(o.objects)) {
    for (const p of o.objects) {
      if (typeof p !== "object" || p === null) continue;
      const e = p as Record<string, unknown>;
      const name = readString(e.name);
      const x = readNumber(e.x);
      const y = readNumber(e.y);
      if (name !== null && x !== null && y !== null) objects.push({ name, x, y });
    }
  }
  const tagged: TaggedPlace[] = [];
  if (Array.isArray(o.tagged)) {
    for (const p of o.tagged) {
      if (typeof p !== "object" || p === null) continue;
      const e = p as Record<string, unknown>;
      const name = readString(e.name);
      const x = readNumber(e.x);
      const y = readNumber(e.y);
      if (name !== null && x !== null && y !== null) {
        tagged.push({ name, x, y, yaw: readNumber(e.yaw) ?? 0 });
      }
    }
  }
  return { frame: readString(o.frame) ?? "world", rooms, objects, tagged };
}

export interface PathValue {
  frame: string;
  points: [number, number][];
}

export function readPath(v: unknown): PathValue | null {
  if (typeof v !== "object" || v === null) return null;
  const o = v as Record<string, unknown>;
  if (!Array.isArray(o.points)) return null;
  const points: [number, number][] = [];
  for (const p of o.points) {
    const xy = numbers(p, 2);
    if (xy !== null) points.push([xy[0], xy[1]]);
  }
  return { frame: readString(o.frame) ?? "world", points };
}

/** A drawn room label's hit box in canvas (device) pixels. */
export interface LabelBox {
  name: string;
  target: [number, number, number];
  x: number;
  y: number;
  w: number;
  h: number;
}

/** The label under a canvas-pixel point; the last drawn wins when boxes
 * overlap (it is on top). */
export function hitLabel(labels: readonly LabelBox[], px: number, py: number): LabelBox | null {
  for (let i = labels.length - 1; i >= 0; i--) {
    const b = labels[i];
    if (px >= b.x && px <= b.x + b.w && py >= b.y && py <= b.y + b.h) return b;
  }
  return null;
}
