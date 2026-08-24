import type { FrameHeader } from "@dimos/shared";
import type { Decoded } from "./index.ts";

// Mirrors the bridge's costmap.zlib.v1 ingress cap (_MAX_PAYLOAD_BYTES in
// dimos/web/relay_bridge/_wt_session.py).
export const MAX_COSTMAP_PAYLOAD_BYTES = 8 * 1024 * 1024;

// Per-axis bound doubling as the render budget: 2048^2 is 4 Mi cells, 16 MiB
// as RGBA, which the main-thread palette convert + canvas upload can afford
// at 5 Hz. Must stay >= the bridge's _COSTMAP_MAX_SIDE (relay_bridge_module),
// which block-max downsamples larger grids before sending.
export const MAX_COSTMAP_DIM = 2048;

/** Validated `costmap.zlib.v1` slot value: deflated cells plus placement. */
export interface CostmapValue {
  /** zlib-deflated uint8 cells, row-major, row 0 at world min-y. */
  bytes: Uint8Array;
  w: number;
  h: number;
  /** Cell edge in meters. */
  res: number;
  /** Grid lower-left corner in world coordinates: [x, y, yaw]. */
  origin: [number, number, number];
}

function metaNumber(meta: Record<string, unknown>, key: string): number {
  const v = meta[key];
  if (typeof v !== "number" || !Number.isFinite(v)) {
    throw new Error(`costmap meta ${key} is not a finite number`);
  }
  return v;
}

/**
 * Decoder for `costmap.zlib.v1`: the value is the still-deflated payload plus
 * the validated placement meta folded in (slots carry no header, and the
 * panel-paced inflate needs w/h/res/origin). The meta is robot-controlled and
 * cannot be verified against compressed bytes cheaply, so this bounds it
 * instead: a lying meta allocates at most MAX_COSTMAP_DIM^2 cells and then
 * fails the inflate length check. Inflate itself is async (DecompressionStream) and
 * lives with the panel (jpeg.ts precedent): the store must always hold the
 * newest bytes even when inflate falls behind, and inflate must never run for
 * frames nobody draws.
 */
export function costmapDecoder(payload: Uint8Array, header: FrameHeader): Decoded {
  if (payload.byteLength > MAX_COSTMAP_PAYLOAD_BYTES) {
    throw new Error(
      `oversized costmap payload: ${payload.byteLength} B, cap ${MAX_COSTMAP_PAYLOAD_BYTES} B`,
    );
  }
  const meta = header.meta;
  if (meta === undefined) throw new Error("costmap frame has no meta");
  const w = metaNumber(meta, "w");
  const h = metaNumber(meta, "h");
  const res = metaNumber(meta, "res");
  if (!Number.isInteger(w) || !Number.isInteger(h) || w < 1 || h < 1) {
    throw new Error(`costmap dimensions ${w}x${h} are not positive integers`);
  }
  if (w > MAX_COSTMAP_DIM || h > MAX_COSTMAP_DIM) {
    throw new Error(`costmap dimensions ${w}x${h} out of bounds`);
  }
  if (res <= 0) throw new Error(`costmap resolution ${res} must be positive`);
  const origin = meta.origin;
  if (
    !Array.isArray(origin) || origin.length !== 3 ||
    !origin.every((n) => typeof n === "number" && Number.isFinite(n))
  ) {
    throw new Error("costmap origin must be [x, y, yaw]");
  }
  const value: CostmapValue = {
    bytes: payload,
    w,
    h,
    res,
    origin: [origin[0], origin[1], origin[2]],
  };
  return { value, preview: `(costmap ${w}x${h}, ${payload.byteLength} B)` };
}

/**
 * Inflate a validated slot value to exactly w*h cells. Python zlib.compress
 * emits RFC 1950 zlib framing, which is DecompressionStream("deflate");
 * "deflate-raw" is RFC 1951 and would reject every frame (the golden vectors
 * in shared/fixtures/costmap_frames.json pin this pairing). Output beyond
 * w*h throws mid-stream (decompression-bomb guard), a short stream throws at
 * the end, and a corrupt stream rejects from read().
 */
export async function inflateCostmap(value: CostmapValue): Promise<Uint8Array> {
  const expected = value.w * value.h;
  const cells = new Uint8Array(expected);
  let written = 0;
  const inflated = new Blob([value.bytes as BlobPart]).stream()
    .pipeThrough(new DecompressionStream("deflate"));
  const reader = inflated.getReader();
  try {
    while (true) {
      const { done, value: chunk } = await reader.read();
      if (done) break;
      if (written + chunk.length > expected) {
        throw new Error(`costmap inflates beyond ${expected} cells`);
      }
      cells.set(chunk, written);
      written += chunk.length;
    }
  } finally {
    void reader.cancel().catch(() => {});
  }
  if (written !== expected) {
    throw new Error(`costmap inflated to ${written} cells, expected ${expected}`);
  }
  return cells;
}
