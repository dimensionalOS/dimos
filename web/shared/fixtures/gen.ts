// Regenerates the golden fixture vectors from protocol.ts.
//
// Run from web/:  deno run --allow-write=shared/fixtures shared/fixtures/gen.ts
//
// The vectors pin the byte-level protocol across the TS and Python codecs, so
// changes here are protocol changes and need review on both sides. Vector
// rules: no integral-valued floats (JS renders 1.0 as "1", Python as "1.0")
// and at least one non-ASCII string (pins ensure_ascii=False on the Python
// side). Message literals below are written in canonical field order - the
// same order the Python dataclasses declare.

import {
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  type Msg,
} from "../protocol.ts";

function b64(bytes: Uint8Array): string {
  let s = "";
  for (const b of bytes) s += String.fromCharCode(b);
  return btoa(s);
}

const controlMsgs: Record<string, Msg> = {
  hello_robot: { t: "hello", v: 1, role: "robot" },
  hello_viewer: { t: "hello", v: 1, role: "viewer" },
  welcome: { t: "welcome", v: 1 },
  ping: { t: "ping", n: 7, ts: 1752576000.5 },
  pong: { t: "pong", n: 7, ts: 1752576000.5 },
  error_version: {
    t: "error",
    code: "version_mismatch",
    message: "protocol v1 required, versiune neacceptată",
  },
};

const teleopMsgs: Record<string, Msg> = {
  twist: { t: "twist", vx: 0.5, wz: -0.25, seq: 12, ts: 1752576000.5 },
  stop: { t: "stop", seq: 13, ts: 1752576000.5 },
};

const dataFrames: Record<string, { header: FrameHeader; payload: Uint8Array }> = {
  odom_reliable: {
    header: { ch: "odom", seq: 42, ts: 1752576000.5, delivery: "reliable" },
    payload: new TextEncoder().encode('{"x":1.5,"y":-2.5,"yaw":0.75}'),
  },
  image_latest_meta: {
    header: {
      ch: "color_image",
      seq: 100,
      ts: 1752576001.25,
      delivery: "latest",
      meta: { w: 320, h: 240 },
    },
    payload: Uint8Array.from({ length: 256 }, (_, i) => i),
  },
  empty_payload: {
    header: { ch: "odom", seq: 0, ts: 0.5, delivery: "reliable" },
    payload: new Uint8Array(0),
  },
};

const control = {
  vectors: Object.entries(controlMsgs).map(([name, message]) => ({
    name,
    message,
    b64: b64(encodeControlFrame(message)),
  })),
};

const datagrams = {
  vectors: Object.entries({ ...controlMsgs, ...teleopMsgs }).map(([name, message]) => ({
    name,
    message,
    b64: b64(encodeDatagram(message)),
  })),
};

const data = {
  vectors: Object.entries(dataFrames).map(([name, { header, payload }]) => ({
    name,
    header,
    payload_b64: b64(payload),
    frame_b64: b64(encodeDataFrame(header, payload)),
  })),
};

const dir = new URL(".", import.meta.url);
for (
  const [file, obj] of [
    ["control_frames.json", control],
    ["datagrams.json", datagrams],
    ["data_frames.json", data],
  ] as const
) {
  await Deno.writeTextFile(new URL(file, dir), JSON.stringify(obj, null, 2) + "\n");
  console.log(`wrote ${file}`);
}
