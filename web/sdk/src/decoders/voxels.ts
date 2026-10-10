import type { FrameHeader } from "@dimos/shared";
import type { Decoded } from "./index.ts";
import { inflateExact } from "./inflate.ts";

// Mirrors the bridge's voxels.zlib.v1 ingress cap (_MAX_PAYLOAD_BYTES in
// dimos/web/relay_bridge/_wt_session.py).
export const MAX_VOXEL_PAYLOAD_BYTES = 8 * 1024 * 1024;

// Budgets shared with the encoder (_VOXEL_MAX_VOXELS / _VOXEL_MAX_CHUNKS in
// builtin_codecs.py), which coarsens a bigger cloud before sending: 1M voxels
// is 12 MiB of float32 positions, which a panel can rebuild at ~1 Hz.
export const MAX_VOXELS = 1_000_000;
export const MAX_VOXEL_CHUNKS = 32_768;

// One chunk record on the wire: int32 LE cx, cy, cz, then 16^3 occupancy
// bits at (lz * 16 + ly) * 16 + lx in little-endian bit order.
const CHUNK = 16;
const CHUNK_HEADER_BYTES = 12;
const CHUNK_MASK_BYTES = (CHUNK * CHUNK * CHUNK) / 8;
const CHUNK_RECORD_BYTES = CHUNK_HEADER_BYTES + CHUNK_MASK_BYTES;

/** Validated `voxels.zlib.v1` slot value: deflated chunk records plus counts. */
export interface VoxelsValue {
  bytes: Uint8Array;
  /** Voxel edge in meters. */
  res: number;
  /** Occupied voxels in the frame. */
  n: number;
  /** Chunk records in the payload. */
  chunks: number;
}

function metaNumber(meta: Record<string, unknown>, key: string): number {
  const v = meta[key];
  if (typeof v !== "number" || !Number.isFinite(v)) {
    throw new Error(`voxels meta ${key} is not a finite number`);
  }
  return v;
}

function metaCount(meta: Record<string, unknown>, key: string, max: number): number {
  const v = metaNumber(meta, key);
  if (!Number.isInteger(v) || v < 0) {
    throw new Error(`voxels meta ${key} must be a whole number`);
  }
  if (v > max) throw new Error(`voxels meta ${key} ${v} exceeds ${max}`);
  return v;
}

/**
 * Decoder for `voxels.zlib.v1`: the value is the still-deflated payload plus
 * the validated counts (costmap.ts precedent). The meta is robot-controlled
 * and bounds the inflate allocation; the content check (set bits against
 * `n`) happens in the panel-paced inflateVoxels. `n` and `chunks` are 0 for
 * an empty cloud, a frame that clears the map.
 */
export function voxelsDecoder(payload: Uint8Array, header: FrameHeader): Decoded {
  if (payload.byteLength > MAX_VOXEL_PAYLOAD_BYTES) {
    throw new Error(
      `oversized voxels payload: ${payload.byteLength} B, cap ${MAX_VOXEL_PAYLOAD_BYTES} B`,
    );
  }
  const meta = header.meta;
  if (meta === undefined) throw new Error("voxels frame has no meta");
  const res = metaNumber(meta, "res");
  if (res <= 0) throw new Error(`voxel resolution ${res} must be positive`);
  const n = metaCount(meta, "n", MAX_VOXELS);
  const chunks = metaCount(meta, "chunks", MAX_VOXEL_CHUNKS);
  if (chunks > n || (n > 0 && chunks === 0)) {
    throw new Error(`voxels meta declares ${chunks} chunks for ${n} voxels`);
  }
  const value: VoxelsValue = { bytes: payload, res, n, chunks };
  return { value, preview: `(voxels ${n}, ${chunks} chunks, ${payload.byteLength} B)` };
}

/**
 * Inflate a validated slot value and unpack it into `n` voxel centres: x, y,
 * z triplets in record order. A payload whose set bits disagree with `n`
 * throws.
 */
export async function inflateVoxels(value: VoxelsValue): Promise<Float32Array> {
  const records = await inflateExact(value.bytes, value.chunks * CHUNK_RECORD_BYTES);
  const view = new DataView(records.buffer, records.byteOffset, records.byteLength);
  const out = new Float32Array(value.n * 3);
  let k = 0;
  for (let c = 0; c < value.chunks; c++) {
    const base = c * CHUNK_RECORD_BYTES;
    const cx = view.getInt32(base, true) * CHUNK;
    const cy = view.getInt32(base + 4, true) * CHUNK;
    const cz = view.getInt32(base + 8, true) * CHUNK;
    for (let b = 0; b < CHUNK_MASK_BYTES; b++) {
      const byte = records[base + CHUNK_HEADER_BYTES + b];
      if (byte === 0) continue;
      for (let i = 0; i < 8; i++) {
        if ((byte & (1 << i)) === 0) continue;
        if (k === out.length) {
          throw new Error(`voxels payload holds more than ${value.n} voxels`);
        }
        const bit = b * 8 + i;
        out[k++] = (cx + (bit & 15) + 0.5) * value.res;
        out[k++] = (cy + ((bit >> 4) & 15) + 0.5) * value.res;
        out[k++] = (cz + (bit >> 8) + 0.5) * value.res;
      }
    }
  }
  if (k !== out.length) {
    throw new Error(`voxels payload holds ${k / 3} voxels, expected ${value.n}`);
  }
  return out;
}
