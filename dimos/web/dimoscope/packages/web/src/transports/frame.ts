// Shared gateway-frame decoder. Every gateway-fed delivery mechanism (ws, sse,
// httpPoll) carries the same wire frame — [f64 BE gateway-send-ms][LC02 packet] —
// so they all decode through here and emit identical RawSamples (same gatewaySendMs
// ⇒ identical latency accounting, the whole point of comparing mechanisms fairly).
import { decodeChannel } from "@dimos/msgs";

import { splitChannel } from "../decode.ts";
import type { RawSample } from "../transport.ts";

/** Decode one gateway frame ([f64 gateway-send-ms][LC02]) into a RawSample. */
export function frameToSample(frame: Uint8Array, recvTs: number): RawSample | undefined {
  if (frame.byteLength < 8) return undefined;
  const dv = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  const gatewaySendMs = dv.getFloat64(0, false);
  const packet = frame.subarray(8);
  let channel: string;
  let payload: Uint8Array;
  try {
    ({ channel, payload } = decodeChannel(packet));
  } catch {
    return undefined; // not an LC02 packet we can split
  }
  const { topic, type } = splitChannel(channel);
  return { topic, type, payload, recvTs, gatewaySendMs };
}

/** Portable base64 → bytes (SSE data lines). Works in the browser, Deno and Bun. */
export function b64ToBytes(s: string): Uint8Array {
  const bin = atob(s);
  const u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8;
}
