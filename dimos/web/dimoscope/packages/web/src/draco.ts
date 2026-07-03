// Draco point-cloud variant (see gateway/cloud.py). The gateway publishes a full-resolution,
// Draco-quantized cloud on `<topic>_draco` with the custom wire type `draco.PointCloud2` and payload
// `[u32be seq][u32be nPoints][draco bytes]`. Two decode depths:
//   • decodeDracoEnvelope() — pure DataView header parse, no wasm. Gives the bench a frame_id (→ seq/
//     loss) and the point count while measuring the compressed wire size. This is all the benchmark
//     needs, so the codec's bandwidth win is measurable with ZERO new dependencies.
//   • decodeDracoGeometry() — lazily loads the `draco3d` wasm decoder to reconstruct xyz for RENDERING
//     only; best-effort (returns null if draco3d isn't installed), so a missing decoder never breaks
//     the data path.

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

// --- geometry decode (render-only, lazy wasm) -------------------------------------------------
let _decoderModule: Promise<any> | null = null;
async function dracoModule(): Promise<any | null> {
  if (_decoderModule === null) {
    _decoderModule = (async () => {
      try {
        // Runtime-constructed specifier: the bundler can't statically resolve it, so an absent
        // draco3d never breaks the build/bench — it's a best-effort, render-only load that simply
        // fails (→ null) when draco3d isn't installed. Install draco3d to enable in-browser render.
        const spec = ["draco", "3d"].join("");
        const mod: any = await import(/* @vite-ignore */ spec);
        const draco3d = mod.default ?? mod;
        return await draco3d.createDecoderModule({});
      } catch {
        return null; // draco3d not installed → renderer falls back to the _ds variant
      }
    })();
  }
  return _decoderModule;
}

/** Reconstruct an (N×3) Float32Array of xyz from a Draco blob, or null if the decoder is unavailable
 *  or the blob is malformed. Render-only; never on the measurement path. */
export async function decodeDracoGeometry(blob: Uint8Array): Promise<Float32Array | null> {
  const m = await dracoModule();
  if (!m) return null;
  const decoder = new m.Decoder();
  const buf = new m.DecoderBuffer();
  try {
    buf.Init(blob, blob.length);
    const geomType = decoder.GetEncodedGeometryType(buf);
    const cloud = new m.PointCloud();
    const status = decoder.DecodeBufferToPointCloud(buf, cloud);
    if (!status.ok() || cloud.ptr === 0) return null;
    const posAttr = decoder.GetAttributeByUniqueId(cloud, 0) ??
      decoder.GetAttribute(cloud, decoder.GetAttributeId(cloud, m.POSITION));
    const n = cloud.num_points();
    const out = new m.DracoFloat32Array();
    decoder.GetAttributeFloatForAllPoints(cloud, posAttr, out);
    const xyz = new Float32Array(n * 3);
    for (let i = 0; i < n * 3; i++) xyz[i] = out.GetValue(i);
    m.destroy(out);
    m.destroy(cloud);
    void geomType;
    return xyz;
  } catch {
    return null;
  } finally {
    m.destroy(buf);
    m.destroy(decoder);
  }
}
