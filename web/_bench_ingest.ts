// Throwaway benchmark: cost of the browser-side per-received-frame ingest path
// (DataFrameStreamReader.push -> jpegDecoder -> ChannelStore.ingest).
import { DataFrameStreamReader, encodeDataFrame } from "./shared/protocol.ts";
import { jpegDecoder } from "./sdk/src/decoders/jpeg.ts";
import { ChannelStore } from "./sdk/src/store.ts";

function fakeJpeg(bytes: number): Uint8Array {
  const out = new Uint8Array(bytes);
  let i = 0;
  out[i++] = 0xff;
  out[i++] = 0xd8; // SOI
  // APP0 (JFIF-ish), length 16
  out[i++] = 0xff;
  out[i++] = 0xe0;
  out[i++] = 0x00;
  out[i++] = 0x10;
  i += 14;
  // SOF0: len 17, precision 8, h=360, w=640, 3 components
  out[i++] = 0xff;
  out[i++] = 0xc0;
  out[i++] = 0x00;
  out[i++] = 0x11;
  out[i++] = 0x08;
  out[i++] = (360 >> 8) & 0xff;
  out[i++] = 360 & 0xff;
  out[i++] = (640 >> 8) & 0xff;
  out[i++] = 640 & 0xff;
  out[i++] = 0x03;
  for (let k = i; k < bytes; k++) out[k] = (k * 31) & 0xff;
  return out;
}

const PAYLOAD = Number(Deno.args[0] ?? 20000);
const N = Number(Deno.args[1] ?? 20000);
const payload = fakeJpeg(PAYLOAD);
const frames: Uint8Array[] = [];
for (let i = 0; i < N; i++) {
  frames.push(encodeDataFrame({ ch: "chase_image", seq: i, ts: 1e9 + i / 40, delivery: "latest" } as any, payload));
}

const store = new ChannelStore();
const reader = new DataFrameStreamReader();
// warm up
for (let i = 0; i < 200; i++) {
  for (const f of reader.push(frames[i])) {
    const { value, preview } = jpegDecoder(f.payload, f.header);
    store.ingest(f.header.ch, f.header, value, true, preview);
  }
}

const t0 = performance.now();
for (let i = 0; i < N; i++) {
  for (const f of reader.push(frames[i])) {
    const { value, preview } = jpegDecoder(f.payload, f.header);
    store.ingest(f.header.ch, f.header, value, true, preview);
  }
}
const dt = performance.now() - t0;
console.log(
  `payload=${PAYLOAD}B frames=${N} total=${dt.toFixed(1)}ms ` +
    `per-frame=${((dt / N) * 1000).toFixed(1)}us ` +
    `=> max ingest rate ~${Math.round(N / (dt / 1000))} fps`,
);
