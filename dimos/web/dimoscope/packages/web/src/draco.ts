// Draco point-cloud variant (see gateway/cloud.py). The gateway publishes a full-resolution,
// Draco-quantized cloud on `<topic>_draco` with the custom wire type `draco.PointCloud2` and payload
// `[u32be seq][u32be nPoints][f64be ts][draco bytes]`. This module stays dependency-free: it only
// parses the envelope header (no wasm), which is all the client + benchmark need to measure the
// compressed wire size and keep seq/latency. The actual geometry decode (draco3d wasm, for RENDERING)
// lives in the app (app/src/panels/clouds/dracoDecode.ts) so the SDK carries no wasm/emscripten dep.

/** Custom wire type the gateway tags the Draco variant with; the client branches on it in onSample. */
export const DRACO_TYPE = "draco.PointCloud2";

/** The cheap, dependency-free decode: read the envelope header, keep the Draco blob for the renderer.
 *  Shaped so `seqFrom` reads `frame_id` (offered/delivered) and the renderer can find `_draco`. */
export interface DracoCloud {
  frame_id: string;
  n_points: number;
  ts: number; // source publish time (s) → the bench's end-to-end latency, like a header.stamp
  _draco: Uint8Array;
}

export function decodeDracoEnvelope(payload: Uint8Array): DracoCloud {
  const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const seq = dv.getUint32(0);
  const n = dv.getUint32(4);
  const ts = dv.getFloat64(8);
  return { frame_id: String(seq), n_points: n, ts, _draco: payload.subarray(16) };
}
