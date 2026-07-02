// Shared decoder for the common gateway wire frame [f64 BE gateway-send-ms][LC02 packet] used by
// ws/sse/httpPoll, so all mechanisms emit identical RawSamples + gatewaySendMs for fair latency comparison.
import { decodeChannel } from "@dimos/msgs";

import type { RawSample } from "../types.ts";

/** Split a DimOS channel "<topic>#<pkg>.<Type>" into its parts. */
export function splitChannel(channel: string): { topic: string; type: string } {
  const h = channel.indexOf("#");
  return h >= 0
    ? { topic: channel.slice(0, h), type: channel.slice(h + 1) }
    : { topic: channel, type: "?" };
}

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

/** Portable base64 → bytes (SSE data lines). Works in any JS runtime with `atob`. */
export function b64ToBytes(s: string): Uint8Array {
  const bin = atob(s);
  const u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8;
}
