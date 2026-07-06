// Browser Draco geometry decode (render-only) for CloudCompare. Kept in the app — not the SDK — so
// the draco3d wasm/emscripten dependency stays out of @dimos/web. We hand the emscripten module the
// wasm bytes directly (wasmBinary) so its nodejs glue never touches require("fs"): Vite serves the
// wasm as a static asset via ?url, we fetch it once, and reuse the decoder module as a lazy singleton.
import { createDecoderModule } from "draco3d";
import decoderWasmUrl from "draco3d/draco_decoder.wasm?url";

let _mod: Promise<any | null> | null = null;

function decoderModule(): Promise<any | null> {
  if (_mod === null) {
    _mod = (async () => {
      try {
        const wasmBinary = await (await fetch(decoderWasmUrl)).arrayBuffer();
        return await createDecoderModule({ wasmBinary });
      } catch (e) {
        console.warn("[clouds] draco3d decoder unavailable", e);
        return null;
      }
    })();
  }
  return _mod;
}

/** Reconstruct an interleaved (N×3) xyz Float32Array from a Draco blob, or null if the decoder is
 *  unavailable / the blob is malformed. Never on the measurement path — CloudCompare rendering only. */
export async function decodeDracoGeometry(blob: Uint8Array): Promise<Float32Array | null> {
  const m = await decoderModule();
  if (!m) return null;
  const decoder = new m.Decoder();
  const buf = new m.DecoderBuffer();
  try {
    buf.Init(blob, blob.length);
    const cloud = new m.PointCloud();
    const status = decoder.DecodeBufferToPointCloud(buf, cloud);
    if (!status.ok() || cloud.ptr === 0) return null;
    const posAttr = decoder.GetAttribute(cloud, decoder.GetAttributeId(cloud, m.POSITION));
    const n = cloud.num_points();
    const out = new m.DracoFloat32Array();
    decoder.GetAttributeFloatForAllPoints(cloud, posAttr, out);
    const xyz = new Float32Array(n * 3);
    for (let i = 0; i < n * 3; i++) xyz[i] = out.GetValue(i);
    m.destroy(out);
    m.destroy(cloud);
    return xyz;
  } catch {
    return null;
  } finally {
    m.destroy(buf);
    m.destroy(decoder);
  }
}
