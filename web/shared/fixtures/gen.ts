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
  PROTOCOL_VERSION,
} from "../protocol.ts";
import { ManifestError, parseManifest } from "../manifest.ts";

function b64(bytes: Uint8Array): string {
  let s = "";
  for (const b of bytes) s += String.fromCharCode(b);
  return btoa(s);
}

const controlMsgs: Record<string, Msg> = {
  hello_robot: {
    t: "hello",
    v: PROTOCOL_VERSION,
    role: "robot",
    robot: { id: "go2-lab", name: "Go2 Laborator β", model: "unitree-go2" },
    manifest: {
      channels: [
        { ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 15.5 },
        { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 },
      ],
    },
  },
  hello_viewer: { t: "hello", v: PROTOCOL_VERSION, role: "viewer" },
  welcome: { t: "welcome", v: PROTOCOL_VERSION },
  ping: { t: "ping", n: 7, ts: 1752576000.5 },
  pong: { t: "pong", n: 7, ts: 1752576000.5 },
  error_version: {
    t: "error",
    code: "version_mismatch",
    message: `protocol v${PROTOCOL_VERSION} required, versiune neacceptată`,
  },
  robots_two: {
    t: "robots",
    robots: [
      { id: "go2-lab", name: "Go2 Laborator β", model: "unitree-go2" },
      { id: "arm-01", name: "Braț 01", model: "xarm7" },
    ],
  },
  robots_empty: { t: "robots", robots: [] },
  watch: { t: "watch", robotId: "go2-lab" },
  manifest_reply: {
    t: "manifest",
    robotId: "go2-lab",
    channels: [{ ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 }],
  },
  sub: { t: "sub", ch: "color_image" },
  unsub: { t: "unsub", ch: "color_image" },
  subs_snapshot: { t: "subs", chs: ["color_image", "odom"], n: 3 },
  subs_empty: { t: "subs", chs: [], n: 4 },
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

// Manifest vectors: valid cases pin normalization, invalid cases pin the
// rejection code. Inputs are declared here (invalid ones cannot come from an
// encoder); outputs come from parseManifest, and test_manifest.py pins the
// Python mirror against them.
const chOdom = { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 };
const chImage = { ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 15.5 };
const longId = "x".repeat(65);
const manifestCases: Record<string, unknown> = {
  channels_only: { channels: [chImage, chOdom] },
  empty: { channels: [] },
  full: {
    channels: [chImage, chOdom],
    panels: [
      { id: "camera", kind: "video", channels: ["color_image"] },
      { id: "pose", kind: "readout", channels: ["odom"] },
      { id: "drive", kind: "teleop", channels: [] },
    ],
    layout: ["camera", "pose", "drive"],
  },
  extra_keys_ignored: {
    channels: [{ ...chOdom, later: 1.5 }],
    panels: [{ id: "pose", kind: "readout", channels: ["odom"], future: true }],
    unknown_top_level: { x: 1 },
  },
  not_an_object: [1, 2],
  missing_channels: {},
  channel_bad_shape: { channels: [{ ch: "odom" }] },
  bad_delivery: { channels: [{ ...chOdom, delivery: "sometimes" }] },
  max_hz_string: { channels: [{ ...chOdom, maxHz: "20" }] },
  max_hz_bool: { channels: [{ ...chOdom, maxHz: true }] },
  empty_channel_id: { channels: [{ ...chOdom, ch: "" }] },
  long_channel_id: { channels: [{ ...chOdom, ch: longId }] },
  duplicate_channel_id: { channels: [chOdom, { ...chOdom, encoding: "jpeg.v1" }] },
  empty_encoding: { channels: [{ ...chOdom, encoding: "" }] },
  long_encoding: { channels: [{ ...chOdom, encoding: longId }] },
  zero_max_hz: { channels: [{ ...chOdom, maxHz: 0 }] },
  negative_max_hz: { channels: [{ ...chOdom, maxHz: -2.5 }] },
  panels_not_list: { channels: [chOdom], panels: {} },
  null_panels: { channels: [chOdom], panels: null },
  panel_bad_shape: { channels: [chOdom], panels: [{ id: "a" }] },
  panel_channels_not_strings: {
    channels: [chOdom],
    panels: [{ id: "a", kind: "readout", channels: [1.5] }],
  },
  empty_panel_id: { channels: [chOdom], panels: [{ id: "", kind: "readout", channels: [] }] },
  empty_panel_kind: { channels: [chOdom], panels: [{ id: "a", kind: "", channels: [] }] },
  duplicate_panel_id: {
    channels: [chOdom],
    panels: [
      { id: "a", kind: "readout", channels: [] },
      { id: "a", kind: "video", channels: [] },
    ],
  },
  panel_unknown_channel: {
    channels: [chOdom],
    panels: [{ id: "a", kind: "video", channels: ["lidar"] }],
  },
  layout_not_list: { channels: [chOdom], layout: "row" },
  layout_not_strings: { channels: [chOdom], layout: [1.5] },
  layout_unknown_panel: { channels: [chOdom], layout: ["ghost"] },
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

const manifests = {
  vectors: Object.entries(manifestCases).map(([name, data]) => {
    try {
      return { name, data, manifest: parseManifest(data) };
    } catch (e) {
      if (e instanceof ManifestError) return { name, data, error: e.code };
      throw e;
    }
  }),
};

const dir = new URL(".", import.meta.url);
for (
  const [file, obj] of [
    ["control_frames.json", control],
    ["datagrams.json", datagrams],
    ["data_frames.json", data],
    ["manifests.json", manifests],
  ] as const
) {
  await Deno.writeTextFile(new URL(file, dir), JSON.stringify(obj, null, 2) + "\n");
  console.log(`wrote ${file}`);
}
