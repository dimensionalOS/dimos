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
//
// costmap_frames.json and voxel_frames.json are NOT written here: their
// payloads are Python zlib bytes (CompressionStream output differs
// byte-wise), so gen_costmap_fixtures and gen_voxel_fixtures in
// dimos/web/relay_bridge generate them (`uv run python -m ...`).

import {
  CONTROL_CHANNEL,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  type Msg,
  PROTOCOL_VERSION,
} from "../protocol.ts";
import { ManifestError, parseManifest, TRACK_ENCODING } from "../manifest.ts";

function b64(bytes: Uint8Array): string {
  let s = "";
  for (const b of bytes) s += String.fromCharCode(b);
  return btoa(s);
}

// Shared by the pub vector and pub_tx_frame: the frame payload must be
// exactly the JSON.stringify of the pub's data (the relay's forwarding rule).
const pubData = { text: "Salut, robotule! β", urgency: 0.75 };

// Abridged SDPs for the rtc vectors (CRLF line ends like the real thing).
const sdpOffer = "v=0\r\no=- 4611731400430051336 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n" +
  "a=group:BUNDLE 0 1\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\nc=IN IP4 0.0.0.0\r\na=mid:0\r\n" +
  "a=sendonly\r\na=rtpmap:96 H264/90000\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n" +
  "c=IN IP4 0.0.0.0\r\na=mid:1\r\na=sendonly\r\na=rtpmap:96 H264/90000\r\n";
const sdpAnswer = "v=0\r\no=- 7 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\na=group:BUNDLE 0 1\r\n" +
  "m=video 9 UDP/TLS/RTP/SAVPF 96\r\nc=IN IP4 0.0.0.0\r\na=mid:0\r\na=recvonly\r\n" +
  "a=rtpmap:96 H264/90000\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\nc=IN IP4 0.0.0.0\r\n" +
  "a=mid:1\r\na=recvonly\r\na=rtpmap:96 H264/90000\r\n";

// The settings a media provider advertises per profile (PLAN_2a), as
// dimos/web/relay_bridge/topic_profiles.py derives them from its table.
const topicProfileOptions = {
  raw: { maxHz: { kind: "number", max: 1000, default: 1000 } },
  "image.jpeg.low.v1": {
    maxHz: { kind: "number", max: 1000, default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [640, 360] },
    quality: { kind: "int", min: 1, max: 100, default: 60 },
  },
  "video.h264.v1": {
    maxHz: { kind: "number", max: 1000, default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [1920, 1080] },
    maxBitrate: { kind: "int", min: 100000, max: 4500000, default: 4500000 },
  },
  "cloud.q16.low.v1": {
    maxHz: { kind: "number", max: 1000, default: 5 },
    keepEvery: { kind: "int", min: 1, max: 64, default: 4 },
    fields: { kind: "enum", values: ["intensity", "color"], default: ["intensity", "color"] },
  },
} as const;

const controlMsgs: Record<string, Msg> = {
  hello_robot: {
    t: "hello",
    v: PROTOCOL_VERSION,
    role: "robot",
    robot: { id: "go2-lab", name: "Go2 Laborator β", model: "unitree-go2" },
    // A fully explicit (normalized) manifest, as the bridge emits it.
    manifest: {
      version: 1,
      channels: [
        {
          ch: "color_image",
          dir: "rx",
          encoding: "jpeg.v1",
          delivery: "latest",
          maxHz: 15.5,
          params: { quality: 75.5 },
          publish: "none",
          requiredScope: null,
        },
        {
          ch: "odom",
          dir: "rx",
          encoding: "pose.json.v1",
          delivery: "reliable",
          maxHz: 20.5,
          params: {},
          publish: "none",
          requiredScope: null,
        },
      ],
      panels: [
        { id: "p0", kind: "video", title: "Camera β", channels: ["color_image"], params: {} },
      ],
      layout: { row: ["p0"] },
      pages: [],
    },
  },
  hello_viewer: { t: "hello", v: PROTOCOL_VERSION, role: "viewer" },
  // The auth token (T12d) rides hello as one optional field; absent when off.
  hello_viewer_token: {
    t: "hello",
    v: PROTOCOL_VERSION,
    role: "viewer",
    token: "viewer-token-fixture-0123456789abcdef",
  },
  welcome: { t: "welcome", v: PROTOCOL_VERSION },
  hello_topic_source: {
    t: "hello",
    v: PROTOCOL_VERSION,
    role: "robot",
    token: "provider-token-fixture-0123456789",
    topicSource: "workspace",
    topicsVersion: 1,
    topicCapabilities: {
      read: true,
      write: false,
      profiles: ["raw", "depth.lossless.v1"],
      guardedPublishers: false,
      videoReconfigure: false,
    },
  },
  // A media provider advertises the settings each profile accepts; the
  // profile value is the default and the ceiling (floor for keepEvery).
  hello_topic_source_options: {
    t: "hello",
    v: PROTOCOL_VERSION,
    role: "robot",
    topicSource: "workspace",
    topicsVersion: 1,
    topicCapabilities: {
      read: true,
      write: false,
      profiles: ["raw", "image.jpeg.low.v1", "cloud.q16.low.v1"],
      guardedPublishers: false,
      profileOptions: topicProfileOptions,
      videoReconfigure: true,
    },
  },
  topic_sources: {
    t: "topic_sources",
    v: 1,
    sources: [{
      source: "workspace",
      gen: 3,
      read: true,
      write: false,
      profiles: ["raw"],
      guardedPublishers: false,
      videoReconfigure: false,
    }],
  },
  topic_sources_options: {
    t: "topic_sources",
    v: 1,
    sources: [{
      source: "workspace",
      gen: 3,
      read: true,
      write: false,
      profiles: ["raw", "image.jpeg.low.v1", "cloud.q16.low.v1"],
      guardedPublishers: false,
      profileOptions: topicProfileOptions,
      videoReconfigure: true,
    }],
  },
  topic_sub: {
    t: "topic_sub",
    id: "s1",
    source: "workspace",
    keyExpr: "demo/β/**",
    delivery: "reliable",
    profile: "raw",
    maxHz: 1000,
  },
  // Initial settings beyond the rate: a sparse request, normalized by the relay.
  topic_sub_settings: {
    t: "topic_sub",
    id: "c1",
    source: "workspace",
    keyExpr: "sensor/β/camera",
    delivery: "latest",
    profile: "image.jpeg.low.v1",
    maxHz: 15.5,
    settings: { maxResolution: [320, 240], quality: 50 },
  },
  topic_open: {
    t: "topic_open",
    route: "@t:p1",
    gen: 3,
    keyExpr: "demo/β/**",
    delivery: "reliable",
    profile: "raw",
    maxHz: 1000,
  },
  // The route's first variant (v1) rides the open with its full settings.
  topic_open_settings: {
    t: "topic_open",
    route: "@t:p2",
    gen: 3,
    keyExpr: "sensor/β/camera",
    delivery: "latest",
    profile: "image.jpeg.low.v1",
    maxHz: 15.5,
    settings: { maxHz: 15.5, maxResolution: [320, 240], quality: 50 },
  },
  topic_opened: { t: "topic_opened", route: "@t:p1", gen: 3 },
  topic_ready: { t: "topic_ready", id: "s1", source: "workspace", route: "@t:v2", gen: 3 },
  topic_ready_settings: {
    t: "topic_ready",
    id: "c1",
    source: "workspace",
    route: "@t:v3",
    gen: 3,
    settings: { maxHz: 15.5, maxResolution: [320, 240], quality: 50 },
  },
  topic_activate: { t: "topic_activate", route: "@t:v2", gen: 3 },
  topic_rate_update: { t: "topic_activate", route: "@t:p1", gen: 3, maxHz: 30 },
  topic_close: { t: "topic_close", route: "@t:p1", gen: 3 },
  topic_credit: { t: "topic_credit", route: "@t:v2", gen: 3, frames: 40, bytes: 13000 },
  topic_unsub: { t: "topic_unsub", id: "s1" },
  // Direct topic video (PLAN_2b): the viewer leg names the subscription and
  // viewer route; the relay forwards the offer with a private peer id, the
  // provider route and the variant, and maps the replies back.
  topic_rtc_offer: { t: "topic_rtc_offer", id: "s1", route: "@t:v2", gen: 3, sdp: sdpOffer },
  topic_rtc_offer_provider: {
    t: "topic_rtc_offer",
    id: "r1",
    route: "@t:p1",
    gen: 3,
    sdp: sdpOffer,
    variant: "v1",
  },
  topic_rtc_answer: { t: "topic_rtc_answer", id: "s1", route: "@t:v2", gen: 3, sdp: sdpAnswer },
  topic_rtc_candidate: {
    t: "topic_rtc_candidate",
    id: "s1",
    route: "@t:v2",
    gen: 3,
    candidate: "candidate:1 1 udp 2130706431 192.168.1.20 51234 typ host generation 0",
    mid: "0",
  },
  topic_rtc_candidate_end: {
    t: "topic_rtc_candidate",
    id: "r1",
    route: "@t:p1",
    gen: 3,
    candidate: "",
    mid: "0",
  },
  topic_rtc_close: { t: "topic_rtc_close", id: "s1", route: "@t:v2", gen: 3, reason: "rtc_limit" },
  // Settings updates (PLAN_2a): null resets one key to the profile default,
  // the acknowledgement carries the normalized settings in force.
  topic_update: {
    t: "topic_update",
    id: "c1",
    revision: 2,
    options: { maxHz: 7.5, maxResolution: null, quality: 40 },
  },
  topic_update_cloud: {
    t: "topic_update",
    id: "c2",
    revision: 1,
    options: { keepEvery: 2, fields: ["color"] },
  },
  topic_applied: {
    t: "topic_applied",
    id: "c1",
    revision: 2,
    settings: { maxHz: 7.5, maxResolution: [640, 360], quality: 40 },
  },
  topic_variant: {
    t: "topic_variant",
    route: "@t:p2",
    gen: 3,
    variant: "v12",
    settings: { maxHz: 7.5, maxResolution: [640, 360], quality: 40 },
  },
  topic_variant_ready: {
    t: "topic_variant_ready",
    route: "@t:p2",
    gen: 3,
    variant: "v2",
    settings: { maxHz: 7.5, maxResolution: [640, 360], quality: 40 },
  },
  topic_variant_cloud: {
    t: "topic_variant",
    route: "@t:p3",
    gen: 3,
    variant: "v1",
    settings: { maxHz: 2.5, keepEvery: 4, fields: ["color"] },
  },
  topic_variant_close: { t: "topic_variant_close", route: "@t:p2", gen: 3, variant: "v2" },
  topic_update_rejected: {
    t: "topic_error",
    id: "c1",
    revision: 3,
    code: "option_out_of_range",
    message: "calitate în afara intervalului β",
    outcome: "rejected",
  },
  topic_guard_ready: {
    t: "topic_guard_ready",
    id: "drive",
    handle: "@p:1",
    gen: 3,
    ticket: 1,
    timeoutMs: 400,
  },
  topic_guard_heartbeat: { t: "topic_guard_heartbeat", handle: "@p:1", gen: 3, ticket: 1 },
  topic_guard_ticket: { t: "topic_guard_ticket", handle: "@p:1", gen: 3, ticket: 2 },
  topic_guard_stop: { t: "topic_guard_stop", handle: "@p:1", gen: 3 },
  topic_guard_closed: { t: "topic_guard_closed", handle: "@p:1", gen: 3, code: "guard_expired" },
  topic_guard_result: {
    t: "topic_guard_result",
    handle: "@p:1",
    gen: 3,
    seq: 10,
    outcome: "accepted",
    code: "",
    providerTs: 100.5,
  },
  topic_error: {
    t: "topic_error",
    id: "s1",
    code: "source_offline",
    message: "Source interrupted",
    outcome: "interrupted",
  },
  topic_list: {
    t: "topic_list",
    id: "l1",
    source: "workspace",
    prefix: "demo/",
    cursor: "",
    limit: 32,
  },
  topic_page: {
    t: "topic_page",
    id: "l1",
    source: "workspace",
    gen: 3,
    entries: [{
      key: "demo/β/clock",
      encoding: "application/json",
      type: "",
      availability: "observed",
    }],
    cursor: "",
  },
  topic_pub_result: {
    t: "topic_pub_result",
    id: "w1",
    outcome: "accepted",
    code: "",
    message: "",
    providerTs: 1752576000.5,
  },
  ping: { t: "ping", n: 7, ts: 1752576000.5 },
  pong: { t: "pong", n: 7, ts: 1752576000.5 },
  topic_bitrate_update: {
    t: "topic_update",
    id: "cam",
    revision: 1,
    options: { maxBitrate: 10000000 },
  },
  topic_bitrate_applied: {
    t: "topic_applied",
    id: "cam",
    revision: 1,
    settings: { maxHz: 30, maxResolution: [1280, 720], maxBitrate: 2500000 },
  },
  pong_aborted: { t: "pong", n: 7, ts: 1752576000.5, aborted: 3 },
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
  // Deliberately partial (un-normalized) manifest: the relay forwards the
  // robot's manifest verbatim, never normalizing it.
  manifest_reply: {
    t: "manifest",
    robotId: "go2-lab",
    manifest: {
      version: 1,
      channels: [{ ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 }],
      panels: [{ id: "pose", kind: "readout", title: "Poziție", channels: ["odom"] }],
      layout: "pose",
    },
  },
  // Watch reply for a robot that registered without a manifest.
  manifest_reply_bare: { t: "manifest", robotId: "arm-01" },
  sub: { t: "sub", ch: "color_image" },
  unsub: { t: "unsub", ch: "color_image" },
  subs_snapshot: { t: "subs", chs: ["color_image", "odom"], n: 3 },
  subs_empty: { t: "subs", chs: [], n: 4 },
  // Bare lease messages are the viewer-leg shapes (control stream); the
  // gen-stamped robot-bound forms are pinned in teleopMsgs below.
  teleop_start: { t: "teleop_start" },
  teleop_started: { t: "teleop_started" },
  teleop_stop: { t: "teleop_stop" },
  // Generic publish (W7): pub and pub_ack ride the viewer control stream
  // (pub_ack.id is the viewer's request id there). pub_nack is robot->relay
  // on a one-shot @control stream (id = the relay token); the relay maps it
  // to a correlated error like error_with_request_id.
  pub: {
    t: "pub",
    id: "s7k2β-1",
    ch: "human_input",
    data: pubData,
    clientTs: 1752576000.25,
  },
  pub_no_client_ts: { t: "pub", id: "s7k2β-2", ch: "human_input", data: [1.5, "β", true] },
  // data spans all of JSON: a top-level null must survive both mirrors'
  // encode paths (pins Python against exclude_none eating it).
  pub_null_data: { t: "pub", id: "s7k2β-3", ch: "human_input", data: null },
  pub_ack: {
    t: "pub_ack",
    id: "s7k2β-1",
    ch: "human_input",
    relayTs: 1752576000.5,
    bridgeTs: 1752576000.75,
  },
  pub_nack: { t: "pub_nack", id: "p3", code: "decode_failed", message: "text prea lung β" },
  error_with_request_id: {
    t: "error",
    code: "rate_limited",
    message: "publish peste limită β",
    requestId: "s7k2β-1",
  },
  rtc_ice: {
    t: "rtc_ice",
    iceServers: [
      { urls: ["stun:stun.cloudflare.com:3478"] },
      {
        urls: [
          "turn:turn.cloudflare.com:3478?transport=udp",
          "turns:turn.cloudflare.com:443?transport=tcp",
        ],
        username: "utilizator-β",
        credential: "parolă-fixture",
      },
    ],
  },
  rtc_offer_robot: {
    t: "rtc_offer",
    sdp: sdpOffer,
    tracks: [{ ch: "color_image", mid: "0" }, { ch: "rear_camera", mid: "1" }],
  },
  rtc_offer_viewer: { t: "rtc_offer", sdp: sdpOffer },
  rtc_offer_pull: {
    t: "rtc_offer",
    sdp: sdpOffer,
    tracks: [{ ch: "color_image", mid: "3" }],
    robotId: "go2-lab",
  },
  rtc_answer: { t: "rtc_answer", sdp: sdpAnswer },
  rtc_stalled: { t: "rtc_stalled", ch: "color_image" },
};

const teleopMsgs: Record<string, Msg> = {
  twist: { t: "twist", vx: 0.5, vy: 0.25, wz: -0.25, seq: 12, ts: 1752576000.5 },
  stop: { t: "stop", seq: 13, ts: 1752576000.5 },
  // Robot-bound forms: the relay stamps the lease generation.
  twist_gen: { t: "twist", vx: 0.5, vy: 0.25, wz: -0.25, seq: 12, ts: 1752576000.5, gen: 3 },
  stop_gen: { t: "stop", seq: 13, ts: 1752576000.5, gen: 3 },
  teleop_start_gen: { t: "teleop_start", gen: 3 },
  teleop_stop_gen: { t: "teleop_stop", gen: 3 },
};

const dataFrames: Record<string, { header: FrameHeader; payload: Uint8Array }> = {
  topic_sample: {
    header: {
      ch: "@t:v2",
      seq: 9,
      ts: 1752576000.75,
      delivery: "reliable",
      meta: {
        key: "demo/β/clock",
        encoding: "application/octet-stream",
        gen: 3,
        ingressTs: 1752576000.5,
        sourceTs: 1752576000.25,
      },
    },
    payload: new Uint8Array([0, 255, 128, 1]),
  },
  // A media sample stamped with its encode variant; the relay dispatches on
  // meta.variant and strips it before forwarding to the viewer.
  topic_variant_sample: {
    header: {
      ch: "@t:p2",
      seq: 10,
      ts: 1752576000.75,
      delivery: "latest",
      meta: {
        key: "sensor/β/camera",
        encoding: "image/jpeg",
        gen: 3,
        ingressTs: 1752576000.5,
        variant: "v2",
        w: 320,
        h: 240,
        quality: 40,
      },
    },
    payload: new Uint8Array([255, 216, 255, 217]),
  },
  topic_publish: {
    header: {
      ch: "@put",
      seq: 1,
      ts: 1752576000.75,
      delivery: "reliable",
      meta: {
        id: "w1",
        source: "workspace",
        gen: 3,
        key: "demo/β/command",
        encoding: "application/octet-stream",
      },
    },
    payload: new Uint8Array([255, 0, 1, 128]),
  },
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
  // v5 robot hello: an @control frame whose payload is the datagram encoding
  // of the hello message (pins the payload-reuses-datagram-encoding rule).
  control_hello: {
    header: { ch: CONTROL_CHANNEL, seq: 1, ts: 1752576000.5, delivery: "reliable" },
    payload: encodeDatagram(controlMsgs.hello_robot),
  },
  // Robot-bound subs snapshot as the relay's control carrier sends it: an
  // @control frame on the relay-opened reliable uni stream, same
  // payload-reuses-datagram-encoding rule as control_hello.
  control_subs: {
    header: { ch: CONTROL_CHANNEL, seq: 2, ts: 1752576000.75, delivery: "reliable" },
    payload: encodeDatagram(controlMsgs.subs_snapshot),
  },
  // Generic publish forwarded relay->robot (W7): the pub's JSON data as a
  // tx-channel data frame on the robot control carrier, provenance in meta
  // (meta.id is the relay-authored token, never the viewer's request id).
  pub_tx_frame: {
    header: {
      ch: "human_input",
      seq: 3,
      ts: 1752576000.5,
      delivery: "reliable",
      meta: { id: "p1", principal: "local", relayTs: 1752576000.5, clientTs: 1752576000.25 },
    },
    payload: new TextEncoder().encode(JSON.stringify(pubData)),
  },
  // The robot's rtc_offer and rtc_stalled and the relay's rtc_answer reuse
  // the datagram encoding like control_hello.
  control_rtc_offer: {
    header: { ch: CONTROL_CHANNEL, seq: 4, ts: 1752576001.5, delivery: "reliable" },
    payload: encodeDatagram(controlMsgs.rtc_offer_robot),
  },
  control_rtc_answer: {
    header: { ch: CONTROL_CHANNEL, seq: 5, ts: 1752576001.75, delivery: "reliable" },
    payload: encodeDatagram(controlMsgs.rtc_answer),
  },
  control_rtc_stalled: {
    header: { ch: CONTROL_CHANNEL, seq: 6, ts: 1752576002, delivery: "reliable" },
    payload: encodeDatagram(controlMsgs.rtc_stalled),
  },
};

// Manifest vectors: valid cases pin normalization, invalid cases pin the
// rejection code. Inputs are declared here (invalid ones cannot come from an
// encoder); outputs come from parseManifest, and test_manifest.py pins the
// Python mirror against them. Most inputs leave dir/params/title absent to
// pin the defaulting; chImageFull pins explicit-field round-trips.
const chOdom = { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 };
const chImage = { ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 15.5 };
const chImageFull = {
  ch: "color_image",
  dir: "rx",
  encoding: "jpeg.v1",
  delivery: "latest",
  maxHz: 15.5,
  params: { quality: 75.5 },
};
const chTrack = { ch: "color_image", encoding: TRACK_ENCODING, delivery: "latest", maxHz: 15.5 };
const chCostmap = {
  ch: "global_costmap",
  encoding: "costmap.zlib.v1",
  delivery: "latest",
  maxHz: 5.5,
};
const chVoxels = {
  ch: "global_map",
  encoding: "voxels.zlib.v1",
  delivery: "latest",
  maxHz: 1.5,
  params: { res: 0.05 },
};
const chTwist = {
  ch: "tele_cmd_vel",
  dir: "tx",
  encoding: "twist.json.v1",
  delivery: "latest",
  maxHz: 15.5,
  params: { maxLinear: 0.85, maxAngular: 1.5, boost: 2.5, watchdogMs: 300.5 },
};
const chChat = {
  ch: "human_input",
  dir: "tx",
  encoding: "text.json.v1",
  delivery: "reliable",
  maxHz: 2.5,
  publish: "shared",
  requiredScope: "chat:send",
};
const chAgent = { ch: "agent", encoding: "chat.json.v1", delivery: "reliable", maxHz: 20.5 };
const chIdle = { ch: "agent_idle", encoding: "json.v1", delivery: "latest", maxHz: 20.5 };
const chAudio = {
  ch: "audio_in",
  dir: "tx",
  encoding: "audio.json.v1",
  delivery: "reliable",
  maxHz: 20.5,
  publish: "shared",
};
const chStats = {
  ch: "resource_stats",
  encoding: "stats.json.v1",
  delivery: "latest",
  maxHz: 2.5,
};
// A *.lcm.v1 channel's params carry its LCM schema: nested lists with nulls
// must round-trip untouched on both sides.
const chLcmPose = {
  ch: "lcm_pose",
  encoding: "geometry_msgs.PoseStamped.lcm.v1",
  delivery: "reliable",
  maxHz: 20.5,
  params: {
    lcm: {
      type: "t.P",
      fp: "6a82696458c279a0",
      structs: {
        "t.P": [["x", "double", null], ["cov", "double", [9]], ["names", "string", ["n"]]],
      },
    },
  },
};
const pCamera = { id: "camera", kind: "video", channels: ["color_image"] };
const pStats = { id: "stats", kind: "stats", channels: ["resource_stats"] };
const pChat = {
  id: "chat",
  kind: "chat",
  channels: ["human_input", "agent", "agent_idle", "audio_in"],
};
const pPose = { id: "pose", kind: "readout", channels: ["odom"] };
const longId = "x".repeat(65);
const manifestCases: Record<string, unknown> = {
  channels_only: { version: 1, channels: [chImage, chOdom] },
  empty: { version: 1, channels: [] },
  full: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [
      { ...pCamera, title: "Cameră față" },
      pPose,
      { id: "drive", kind: "joystick", channels: [] },
      { id: "aux", kind: "readout", channels: ["odom"] },
    ],
    layout: { row: [{ col: ["camera", "pose"], shares: [2.5, 1.5] }, "drive"] },
    pages: ["aux"],
  },
  extra_keys_ignored: {
    version: 1,
    channels: [{ ...chOdom, later: 1.5 }],
    panels: [{ ...pPose, future: true }],
    layout: { row: ["pose"], future: true },
    unknown_top_level: { x: 1 },
  },
  not_an_object: [1, 2],
  version_missing: { channels: [] },
  version_string: { version: "1", channels: [] },
  version_bool: { version: true, channels: [] },
  version_two: { version: 2, channels: [] },
  version_float: { version: 1.5, channels: [] },
  // The version gate runs before the channel shape check, so a future
  // manifest whose channels no longer parse still reports its version.
  version_before_channel_shape: { version: 2, channels: { alien: true } },
  missing_channels: { version: 1 },
  channel_bad_shape: { version: 1, channels: [{ ch: "odom" }] },
  bad_delivery: { version: 1, channels: [{ ...chOdom, delivery: "sometimes" }] },
  max_hz_string: { version: 1, channels: [{ ...chOdom, maxHz: "20" }] },
  max_hz_bool: { version: 1, channels: [{ ...chOdom, maxHz: true }] },
  empty_channel_id: { version: 1, channels: [{ ...chOdom, ch: "" }] },
  long_channel_id: { version: 1, channels: [{ ...chOdom, ch: longId }] },
  reserved_channel_id: { version: 1, channels: [{ ...chOdom, ch: CONTROL_CHANNEL }] },
  duplicate_channel_id: { version: 1, channels: [chOdom, { ...chOdom, encoding: "jpeg.v1" }] },
  empty_encoding: { version: 1, channels: [{ ...chOdom, encoding: "" }] },
  long_encoding: { version: 1, channels: [{ ...chOdom, encoding: longId }] },
  zero_max_hz: { version: 1, channels: [{ ...chOdom, maxHz: 0 }] },
  negative_max_hz: { version: 1, channels: [{ ...chOdom, maxHz: -2.5 }] },
  dir_tx_channel: { version: 1, channels: [{ ...chOdom, dir: "tx" }] },
  dir_invalid: { version: 1, channels: [{ ...chOdom, dir: "both" }] },
  dir_null: { version: 1, channels: [{ ...chOdom, dir: null }] },
  // Generic-publish rules (W7). The manifest layer accepts "exclusive"
  // (authoring and the bridge reject it until W8).
  publish_shared_tx: { version: 1, channels: [chChat] },
  publish_exclusive_tx: { version: 1, channels: [{ ...chChat, publish: "exclusive" }] },
  publish_none_roundtrip: { version: 1, channels: [{ ...chOdom, publish: "none" }] },
  required_scope_null_roundtrip: { version: 1, channels: [{ ...chChat, requiredScope: null }] },
  publish_on_rx: { version: 1, channels: [{ ...chOdom, publish: "shared" }] },
  publish_bad_value: { version: 1, channels: [{ ...chOdom, publish: "always" }] },
  publish_null: { version: 1, channels: [{ ...chOdom, publish: null }] },
  publish_shared_latest: { version: 1, channels: [{ ...chChat, delivery: "latest" }] },
  publish_shared_binary_encoding: { version: 1, channels: [{ ...chChat, encoding: "jpeg.v1" }] },
  scope_without_publish: { version: 1, channels: [{ ...chOdom, requiredScope: "chat:send" }] },
  scope_empty: { version: 1, channels: [{ ...chChat, requiredScope: "" }] },
  scope_too_long: { version: 1, channels: [{ ...chChat, requiredScope: longId }] },
  scope_not_string: { version: 1, channels: [{ ...chChat, requiredScope: 1.5 }] },
  // The rx-publish rule fires before the scope rules (document order).
  publish_rx_before_scope: {
    version: 1,
    channels: [{ ...chOdom, publish: "shared", requiredScope: "" }],
  },
  channel_params_roundtrip: { version: 1, channels: [chImageFull] },
  lcm_channel_params_roundtrip: { version: 1, channels: [chLcmPose] },
  channel_params_not_object: { version: 1, channels: [{ ...chOdom, params: 1.5 }] },
  panels_not_list: { version: 1, channels: [chOdom], panels: {} },
  null_panels: { version: 1, channels: [chOdom], panels: null },
  panel_bad_shape: { version: 1, channels: [chOdom], panels: [{ id: "a" }] },
  panel_channels_not_strings: {
    version: 1,
    channels: [chOdom],
    panels: [{ id: "a", kind: "readout", channels: [1.5] }],
  },
  panel_title_roundtrip: {
    version: 1,
    channels: [chOdom],
    panels: [{ ...pPose, title: "Poziție" }],
  },
  panel_title_not_string: { version: 1, channels: [chOdom], panels: [{ ...pPose, title: 1.5 }] },
  panel_params_not_object: { version: 1, channels: [chOdom], panels: [{ ...pPose, params: [] }] },
  empty_panel_id: {
    version: 1,
    channels: [chOdom],
    panels: [{ id: "", kind: "readout", channels: [] }],
  },
  empty_panel_kind: {
    version: 1,
    channels: [chOdom],
    panels: [{ id: "a", kind: "", channels: [] }],
  },
  duplicate_panel_id: {
    version: 1,
    channels: [chOdom],
    panels: [
      { id: "a", kind: "readout", channels: [] },
      { id: "a", kind: "video", channels: [] },
    ],
  },
  panel_unknown_channel: {
    version: 1,
    channels: [chOdom],
    panels: [{ id: "a", kind: "video", channels: ["lidar"] }],
  },
  video_panel_no_channel: {
    version: 1,
    channels: [chImage],
    panels: [{ id: "cam", kind: "video", channels: [] }],
  },
  video_panel_two_channels: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [{ id: "cam", kind: "video", channels: ["color_image", "odom"] }],
  },
  video_panel_wrong_encoding: {
    version: 1,
    channels: [chOdom],
    panels: [{ id: "cam", kind: "video", channels: ["odom"] }],
  },
  video_panel_wrong_delivery: {
    version: 1,
    channels: [{ ...chImage, delivery: "reliable" }],
    panels: [{ id: "cam", kind: "video", channels: ["color_image"] }],
  },
  video_panel_tx_channel: {
    version: 1,
    channels: [{ ...chImage, dir: "tx" }],
    panels: [{ id: "cam", kind: "video", channels: ["color_image"] }],
  },
  video_panel_webrtc: { version: 1, channels: [chTrack], panels: [pCamera] },
  video_panel_webrtc_wrong_delivery: {
    version: 1,
    channels: [{ ...chTrack, delivery: "reliable" }],
    panels: [pCamera],
  },
  map2d_panel_full: {
    version: 1,
    channels: [chCostmap, chOdom],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] }],
  },
  map2d_panel_no_pose: {
    version: 1,
    channels: [chCostmap],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap"] }],
  },
  map2d_panel_no_channel: {
    version: 1,
    channels: [chCostmap],
    panels: [{ id: "map", kind: "map2d", channels: [] }],
  },
  map2d_panel_three_channels: {
    version: 1,
    channels: [chCostmap, chOdom, chImage],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap", "odom", "color_image"] }],
  },
  map2d_panel_wrong_encoding: {
    version: 1,
    channels: [chOdom],
    panels: [{ id: "map", kind: "map2d", channels: ["odom"] }],
  },
  map2d_panel_wrong_delivery: {
    version: 1,
    channels: [{ ...chCostmap, delivery: "reliable" }],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap"] }],
  },
  map2d_panel_bad_pose: {
    version: 1,
    channels: [chCostmap, chImage],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap", "color_image"] }],
  },
  map2d_panel_tx_costmap: {
    version: 1,
    channels: [{ ...chCostmap, dir: "tx" }],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap"] }],
  },
  map2d_panel_tx_pose: {
    version: 1,
    channels: [chCostmap, { ...chOdom, dir: "tx" }],
    panels: [{ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] }],
  },
  // Map3D: the voxel map first (voxels.zlib.v1 latest rx), the pose marker
  // optional, the same shape as map2d.
  map3d_panel_full: {
    version: 1,
    channels: [chVoxels, chOdom],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_map", "odom"] }],
  },
  map3d_panel_no_pose: {
    version: 1,
    channels: [chVoxels],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_map"] }],
  },
  map3d_panel_no_channel: {
    version: 1,
    channels: [chVoxels],
    panels: [{ id: "map3d", kind: "map3d", channels: [] }],
  },
  map3d_panel_three_channels: {
    version: 1,
    channels: [chVoxels, chOdom, chImage],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_map", "odom", "color_image"] }],
  },
  // A 2D costmap in the 3D panel: the likely authoring slip.
  map3d_panel_wrong_encoding: {
    version: 1,
    channels: [chCostmap],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_costmap"] }],
  },
  map3d_panel_wrong_delivery: {
    version: 1,
    channels: [{ ...chVoxels, delivery: "reliable" }],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_map"] }],
  },
  map3d_panel_bad_pose: {
    version: 1,
    channels: [chVoxels, chImage],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_map", "color_image"] }],
  },
  map3d_panel_tx_cloud: {
    version: 1,
    channels: [{ ...chVoxels, dir: "tx" }],
    panels: [{ id: "map3d", kind: "map3d", channels: ["global_map"] }],
  },
  teleop_panel_valid: {
    version: 1,
    channels: [chTwist],
    panels: [{ id: "drive", kind: "teleop", channels: ["tele_cmd_vel"] }],
  },
  teleop_panel_no_channel: {
    version: 1,
    channels: [chTwist],
    panels: [{ id: "drive", kind: "teleop", channels: [] }],
  },
  teleop_panel_two_channels: {
    version: 1,
    channels: [chTwist, chOdom],
    panels: [{ id: "drive", kind: "teleop", channels: ["tele_cmd_vel", "odom"] }],
  },
  teleop_panel_rx_channel: {
    version: 1,
    channels: [{ ...chTwist, dir: "rx" }],
    panels: [{ id: "drive", kind: "teleop", channels: ["tele_cmd_vel"] }],
  },
  teleop_panel_wrong_encoding: {
    version: 1,
    channels: [{ ...chOdom, dir: "tx" }],
    panels: [{ id: "drive", kind: "teleop", channels: ["odom"] }],
  },
  teleop_panel_wrong_delivery: {
    version: 1,
    channels: [{ ...chTwist, delivery: "reliable" }],
    panels: [{ id: "drive", kind: "teleop", channels: ["tele_cmd_vel"] }],
  },
  layout_single_panel: { version: 1, channels: [chOdom], panels: [pPose], layout: "pose" },
  layout_row_shares: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [pCamera, pPose],
    layout: { row: ["camera", "pose"], shares: [2.5, 1.5] },
  },
  // Outer col has no shares: pins that absent shares stay absent in the
  // normalized tree (the equal-split default belongs to the renderer).
  layout_nested_tree: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [pCamera, pPose, { id: "drive", kind: "joystick", channels: [] }],
    layout: { col: [{ row: ["camera", "pose"], shares: [1.5, 2.5] }, "drive"] },
  },
  layout_null: { version: 1, channels: [chOdom], panels: [pPose], layout: null },
  layout_bad_node_type: { version: 1, channels: [chOdom], panels: [pPose], layout: 1.5 },
  layout_flat_list_rejected: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: ["pose"],
  },
  layout_row_and_col: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [pCamera, pPose],
    layout: { row: ["camera"], col: ["pose"] },
  },
  layout_neither_row_nor_col: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { shares: [1.5] },
  },
  layout_children_not_array: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: "pose" },
  },
  layout_empty_children: { version: 1, channels: [chOdom], panels: [pPose], layout: { row: [] } },
  layout_nested_bad_node: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: [{ col: [2.5] }] },
  },
  layout_shares_not_array: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: ["pose"], shares: "big" },
  },
  layout_shares_not_numbers: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: ["pose"], shares: ["2"] },
  },
  layout_shares_bool: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: ["pose"], shares: [true] },
  },
  layout_shares_length_mismatch: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [pCamera, pPose],
    layout: { row: ["camera", "pose"], shares: [1.5] },
  },
  layout_share_zero: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: ["pose"], shares: [0] },
  },
  layout_share_negative: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: ["pose"], shares: [-1.5] },
  },
  layout_share_null: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: ["pose"], shares: [null] },
  },
  layout_unknown_panel: { version: 1, channels: [chOdom], panels: [pPose], layout: "ghost" },
  layout_deep_unknown_panel: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: { row: [{ col: ["ghost"] }] },
  },
  duplicate_layout_panel: {
    version: 1,
    channels: [chImage],
    panels: [pCamera],
    layout: { row: ["camera", "camera"] },
  },
  duplicate_panel_across_layout_and_pages: {
    version: 1,
    channels: [chImage],
    panels: [pCamera],
    layout: "camera",
    pages: ["camera"],
  },
  duplicate_panel_within_pages: {
    version: 1,
    channels: [chImage],
    panels: [pCamera],
    pages: ["camera", "camera"],
  },
  pages_only: { version: 1, channels: [chOdom], panels: [pPose], pages: ["pose"] },
  pages_and_layout: {
    version: 1,
    channels: [chImage, chOdom],
    panels: [pCamera, pPose],
    layout: "camera",
    pages: ["pose"],
  },
  unknown_page_panel: { version: 1, channels: [chOdom], panels: [pPose], pages: ["ghost"] },
  // Layout is validated before pages: the layout's unknown panel wins.
  layout_error_before_pages: {
    version: 1,
    channels: [chOdom],
    panels: [pPose],
    layout: "ghost",
    pages: ["phantom"],
  },
  pages_not_list: { version: 1, channels: [chOdom], panels: [pPose], pages: {} },
  pages_not_strings: { version: 1, channels: [chOdom], panels: [pPose], pages: [1.5] },
  pages_null: { version: 1, channels: [chOdom], panels: [pPose], pages: null },
  // Chat panel: text input (publish tx), messages, idle flag, push-to-talk
  // audio (publish tx), in that order.
  chat_panel: {
    version: 1,
    channels: [chChat, chAgent, chIdle, chAudio],
    panels: [pChat],
  },
  chat_panel_two_channels: {
    version: 1,
    channels: [chChat, chAgent],
    panels: [{ ...pChat, channels: ["human_input", "agent"] }],
  },
  // The pre-audio shape: three channels stopped being a chat panel.
  chat_panel_three_channels: {
    version: 1,
    channels: [chChat, chAgent, chIdle],
    panels: [{ ...pChat, channels: ["human_input", "agent", "agent_idle"] }],
  },
  chat_panel_input_not_publishable: {
    version: 1,
    channels: [
      { ch: "human_input", dir: "tx", encoding: "text.json.v1", delivery: "reliable", maxHz: 2.5 },
      chAgent,
      chIdle,
      chAudio,
    ],
    panels: [pChat],
  },
  chat_panel_messages_wrong_encoding: {
    version: 1,
    channels: [chChat, chOdom, chIdle, chAudio],
    panels: [{ ...pChat, channels: ["human_input", "odom", "agent_idle", "audio_in"] }],
  },
  chat_panel_idle_wrong_delivery: {
    version: 1,
    channels: [chChat, chAgent, { ...chIdle, delivery: "reliable" }, chAudio],
    panels: [pChat],
  },
  chat_panel_audio_not_publishable: {
    version: 1,
    channels: [
      chChat,
      chAgent,
      chIdle,
      { ch: "audio_in", dir: "tx", encoding: "audio.json.v1", delivery: "reliable", maxHz: 20.5 },
    ],
    panels: [pChat],
  },
  // Stats page: one stats.json.v1 latest rx channel, placed as a page tab.
  stats_panel: { version: 1, channels: [chStats], panels: [pStats], pages: ["stats"] },
  stats_panel_two_channels: {
    version: 1,
    channels: [chStats, chOdom],
    panels: [{ ...pStats, channels: ["resource_stats", "odom"] }],
  },
  stats_panel_wrong_encoding: {
    version: 1,
    channels: [chOdom],
    panels: [{ ...pStats, channels: ["odom"] }],
  },
  stats_panel_wrong_delivery: {
    version: 1,
    channels: [{ ...chStats, delivery: "reliable" }],
    panels: [pStats],
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
const invalidTopics = {
  vectors: [
    ...[-1, 0.5, null, true, 9007199254740992].map((aborted, i) => ({
      name: `invalid_pong_aborted_${i}`,
      message: { t: "pong", n: 7, ts: 1752576000.5, aborted },
    })),
    ...[null, true, 0, -1, 1001, "30"].map((maxHz, i) => ({
      name: `invalid_rate_update_${i}`,
      message: { ...controlMsgs.topic_activate, maxHz },
    })),
    ...[
      ["invalid_guard_handle", { handle: "@t:1" }],
      ["guard_handle_newline", { handle: "@p:1\n" }],
      ["boolean_guard_ticket", { ticket: true }],
      ["zero_guard_ticket", { ticket: 0 }],
      ["fractional_guard_timeout", { timeoutMs: 400.5 }],
      ["unbounded_guard_timeout", { timeoutMs: 1001 }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_guard_ready, ...fields as Record<string, unknown> },
    })),
    ...[
      null,
      { read: 1, write: false, profiles: ["raw"] },
      {
        read: true,
        write: false,
        profiles: new Array(17).fill("raw"),
      },
      { read: true, write: false, profiles: [""] },
      {
        read: true,
        write: false,
        profiles: ["raw"],
        videoReconfigure: 1,
      },
    ].map((topicCapabilities, i) => ({
      name: `invalid_capabilities_${i}`,
      message: { ...controlMsgs.hello_topic_source, topicCapabilities },
    })),
    { name: "boolean_version", message: { t: "topic_sources", v: true, sources: [] } },
    {
      name: "boolean_hello_version",
      message: { t: "hello", v: PROTOCOL_VERSION, role: "viewer", topicsVersion: true },
    },
    {
      name: "null_hello_source",
      message: { t: "hello", v: PROTOCOL_VERSION, role: "robot", topicSource: null },
    },
    ...[
      ["empty_key", { keyExpr: "" }],
      ["long_unicode_key", { keyExpr: "β".repeat(513) }],
      ["bad_source", { source: "../workspace" }],
      ["source_cr", { source: "workspace\r" }],
      ["source_lf", { source: "workspace\n" }],
      ["source_unicode_line_end", { source: "workspace\u2028" }],
      ["negative_rate", { maxHz: -1 }],
      ["boolean_rate", { maxHz: true }],
      ["unknown_delivery", { delivery: "datagram" }],
      ["empty_profile", { profile: "" }],
      ["long_id", { id: "x".repeat(65) }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_sub, ...fields as Record<string, unknown> },
    })),
    ...[
      ["literal_key_route", { route: "demo/a" }],
      ["route_cr", { route: "@t:v2\r" }],
      ["route_unicode_line_end", { route: "@t:v2\u2029" }],
      ["negative_gen", { gen: -1 }],
      ["fractional_gen", { gen: 1.5 }],
      ["boolean_gen", { gen: true }],
      ["unsafe_gen", { gen: 9007199254740992 }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_ready, ...fields as Record<string, unknown> },
    })),
    ...[
      ["unknown_option", { options: { bandwidth: 1 } }],
      ["negative_keep_every", { options: { keepEvery: -1 } }],
      ["fractional_bitrate", { options: { maxBitrate: 100000.5 } }],
      ["negative_bitrate", { options: { maxBitrate: -1 } }],
      ["unsafe_bitrate", { options: { maxBitrate: 9007199254740992 } }],
      ["boolean_bitrate", { options: { maxBitrate: true } }],
      ["fractional_quality", { options: { quality: 75.5 } }],
      ["boolean_quality", { options: { quality: true } }],
      ["short_resolution", { options: { maxResolution: [640] } }],
      ["string_resolution", { options: { maxResolution: ["640", 360] } }],
      ["zero_rate_option", { options: { maxHz: 0 } }],
      ["duplicate_fields", { options: { fields: ["color", "color"] } }],
      ["numeric_fields", { options: { fields: [1] } }],
      ["null_options", { options: null }],
      ["unsafe_revision", { revision: 9007199254740992 }],
      ["negative_revision", { revision: -1 }],
      ["fractional_revision", { revision: 1.5 }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_update, ...fields as Record<string, unknown> },
    })),
    ...[
      ["null_setting_in_force", { settings: { maxHz: 7.5, quality: null } }],
      ["settings_without_rate", { settings: { quality: 40 } }],
      ["null_settings", { settings: null }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_applied, ...fields as Record<string, unknown> },
    })),
    ...[
      ["variant_zero", { variant: "v0" }],
      ["variant_beyond_bound", { variant: "v12345678901234567" }],
      ["variant_leading_zero", { variant: "v01" }],
      ["variant_prefix", { variant: "w1" }],
      ["variant_newline", { variant: "v1\n" }],
      ["variant_null_settings", { settings: null }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_variant, ...fields as Record<string, unknown> },
    })),
    { name: "sub_settings_scalar", message: { ...controlMsgs.topic_sub, settings: 5 } },
    { name: "sub_settings_null", message: { ...controlMsgs.topic_sub, settings: null } },
    {
      name: "sub_settings_reset",
      message: { ...controlMsgs.topic_sub_settings, settings: { quality: null } },
    },
    { name: "ready_null_settings", message: { ...controlMsgs.topic_ready, settings: null } },
    { name: "error_null_revision", message: { ...controlMsgs.topic_error, revision: null } },
    ...[{ maxHz: { kind: "ratio", default: 30 } }, { maxHz: { kind: "number", min: "0" } }, {
      fields: { kind: "enum", values: new Array(17).fill("color") },
    }, {
      maxResolution: {
        kind: "size",
        default: [640, 360, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
      },
    }, {
      maxHz: null,
    }].map((ranges, i) => ({
      name: `invalid_profile_options_${i}`,
      message: {
        ...controlMsgs.hello_topic_source,
        topicCapabilities: {
          read: true,
          write: false,
          profiles: ["raw"],
          profileOptions: { raw: ranges },
        },
      },
    })),
    ...[
      ["oversized_sdp", { sdp: "v=0\r\n".padEnd(48 * 1024 + 1, "a") }],
      ["empty_sdp", { sdp: "" }],
      ["numeric_sdp", { sdp: 1 }],
      ["rtc_variant_beyond_bound", { variant: "v12345678901234567" }],
      ["rtc_null_variant", { variant: null }],
      ["rtc_literal_key_route", { route: "demo/a" }],
      ["rtc_negative_gen", { gen: -1 }],
      ["rtc_missing_id", { id: undefined }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_rtc_offer, ...fields as Record<string, unknown> },
    })),
    ...[
      ["long_candidate", { candidate: "a".repeat(513) }],
      ["null_candidate", { candidate: null }],
      ["long_mid", { mid: "0".repeat(17) }],
      ["numeric_mid", { mid: 0 }],
      ["candidate_fractional_gen", { gen: 1.5 }],
    ].map(([name, fields]) => ({
      name,
      message: { ...controlMsgs.topic_rtc_candidate, ...fields as Record<string, unknown> },
    })),
    { name: "rtc_close_empty_reason", message: { ...controlMsgs.topic_rtc_close, reason: "" } },
    {
      name: "rtc_close_long_reason",
      message: { ...controlMsgs.topic_rtc_close, reason: "x".repeat(65) },
    },
    { name: "oversized_page", message: { ...controlMsgs.topic_list, limit: 33 } },
    { name: "fractional_limit", message: { ...controlMsgs.topic_list, limit: 1.5 } },
    { name: "negative_credit", message: { ...controlMsgs.topic_credit, bytes: -1 } },
    { name: "boolean_credit", message: { ...controlMsgs.topic_credit, frames: true } },
    { name: "unknown_outcome", message: { ...controlMsgs.topic_error, outcome: "probably" } },
    {
      name: "bad_descriptors",
      message: {
        ...controlMsgs.topic_page,
        entries: [{ key: "a", type: "", encoding: "", availability: "live" }],
      },
    },
  ],
};
for (
  const [file, obj] of [
    ["control_frames.json", control],
    ["datagrams.json", datagrams],
    ["data_frames.json", data],
    ["manifests.json", manifests],
    ["topics_invalid.json", invalidTopics],
  ] as const
) {
  await Deno.writeTextFile(new URL(file, dir), JSON.stringify(obj, null, 2) + "\n");
  console.log(`wrote ${file}`);
}
