import type { FrameHeader } from "@dimos/shared";
import type { Decoded } from "./index.ts";

// Mirrors the bridge's jpeg.v1 ingress cap (_MAX_PAYLOAD_BYTES in
// dimos/web/relay_bridge/_wt_session.py).
export const MAX_JPEG_PAYLOAD_BYTES = 8 * 1024 * 1024;

// Per-axis decoded-dimension bound: go2 streams 1280x720 and 4K UHD fits;
// 4096x4096 RGBA caps the decoded bitmap at 64 MiB, and bounding each axis
// also rejects degenerate 65536x64 shapes.
export const MAX_JPEG_DIM = 4096;

/**
 * Walk the marker segments from the SOI to the first SOF and return the
 * declared frame dimensions. Headers precede scan data, so a tail-truncated
 * jpeg still passes; a non-jpeg or header-truncated payload throws.
 */
function scanDimensions(payload: Uint8Array): { w: number; h: number } {
  if (payload.length < 4 || payload[0] !== 0xff || payload[1] !== 0xd8) {
    throw new Error("jpeg payload has no SOI marker");
  }
  let i = 2;
  while (true) {
    if (i + 1 >= payload.length) throw new Error("jpeg truncated before SOF");
    if (payload[i] !== 0xff) throw new Error("jpeg marker walk desynced");
    while (payload[i + 1] === 0xff) {
      i += 1; // fill bytes before a marker
      if (i + 1 >= payload.length) throw new Error("jpeg truncated before SOF");
    }
    const marker = payload[i + 1];
    i += 2;
    if (marker === 0xd9 || marker === 0xda) {
      throw new Error("jpeg SOS/EOI before SOF");
    }
    if (marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) {
      continue; // standalone marker, no length
    }
    if (i + 2 > payload.length) throw new Error("jpeg truncated before SOF");
    const len = (payload[i] << 8) | payload[i + 1];
    if (len < 2 || i + len > payload.length) throw new Error("jpeg segment overruns payload");
    // SOF = C0-CF except DHT (C4), JPG (C8), DAC (CC); its payload is
    // precision(1) height(2 BE) width(2 BE).
    if (marker >= 0xc0 && marker <= 0xcf && marker !== 0xc4 && marker !== 0xc8 && marker !== 0xcc) {
      if (len < 7) throw new Error("jpeg SOF segment too short");
      const h = (payload[i + 3] << 8) | payload[i + 4];
      const w = (payload[i + 5] << 8) | payload[i + 6];
      return { w, h };
    }
    i += len;
  }
}

/**
 * Decoder for `jpeg.v1`: the payload IS the value (raw JPEG bytes). The
 * marker scan guarantees the store only ever holds plausibly-decodable,
 * bounded frames; a throw is counted as a decode error by the session. Pixel
 * decode is async (createImageBitmap) and panel-paced, so it lives in
 * VideoPanel, not here: the store must always hold the newest bytes even when
 * decode falls behind, and decode must never run for frames nobody draws.
 */
export function jpegDecoder(payload: Uint8Array, _header: FrameHeader): Decoded {
  if (payload.byteLength > MAX_JPEG_PAYLOAD_BYTES) {
    throw new Error(
      `oversized jpeg payload: ${payload.byteLength} B, cap ${MAX_JPEG_PAYLOAD_BYTES} B`,
    );
  }
  const { w, h } = scanDimensions(payload);
  if (w < 1 || h < 1 || w > MAX_JPEG_DIM || h > MAX_JPEG_DIM) {
    throw new Error(`jpeg dimensions ${w}x${h} out of bounds`);
  }
  return { value: payload, preview: `(jpeg ${w}x${h}, ${payload.byteLength} B)` };
}
