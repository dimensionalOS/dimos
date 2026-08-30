// Wire protocol shared by the relay (Deno) and the Cockpit (browser).
// Mirrored in Python at dimos/web/relay_bridge/protocol.py and pinned by the
// golden vectors in ./fixtures/ (tested from both deno test and pytest).
//
// Framing (see web/README.md for the upstream-bug rationale):
// - Control stream frame: u32-LE length | UTF-8 JSON.
// - Datagram: raw UTF-8 JSON, no length prefix.
// - Data frame: u32-LE headerLen | u32-LE payloadLen | header JSON | payload.
//   Latest channels send one frame per stream; a reliable channel packs its
//   frames back to back on one persistent stream. Receivers count bytes and
//   must never treat stream EOF as a message boundary (Deno 2.6.x delays FIN
//   by up to ~1 s, and a persistent stream has no EOF between frames).
//   Channel ids beginning with "@" are reserved for protocol control: the
//   robot's hello rides an @control frame (datagram-encoded payload) on a
//   one-shot bidi stream, and @-frames are never forwarded to viewers.
//
// Validation policy (mirrored in protocol.py): decoders validate shape
// strictly, and receivers drop invalid or unknown messages -- a peer's bytes
// must never kill a session. Framing-level corruption (absurd length
// prefixes) throws and kills only the affected stream.

import { type Delivery, MAX_MANIFEST_ID_LEN } from "./manifest.ts";

// Channel/manifest domain types live in manifest.ts; re-exported so protocol
// consumers keep a single import surface.
export type { ChannelSpec, Delivery, Dir, PanelSpec } from "./manifest.ts";
export { RESERVED_CHANNEL_PREFIX } from "./manifest.ts";

// v5: the robot hello leaves datagrams (and their ~1100 B budget) and rides
// an @control data frame on a robot-opened one-shot bidi stream; channel ids
// beginning with "@" are reserved for protocol control; a robot datagram
// hello is rejected. v4: the twist datagram gains vy (strafe) and the teleop
// lease messages (teleop_start/teleop_started/teleop_stop) enter the control
// plane; robot-bound twist/stop/teleop_start/teleop_stop carry the
// relay-stamped lease generation `gen` (amended into v4 pre-release: an
// older v4 peer without gen gets dead teleop, never unsafe motion). v3: the
// manifest travels as one opaque record nested in hello/manifest messages
// (v2 carried flat channels/panels fields, which a v2 peer would silently
// misread in both directions). v2: a reliable channel packs all its frames
// onto one persistent stream. Bump on any change an old peer would silently
// misparse.
export const PROTOCOL_VERSION = 5;

// The reserved data-frame channel carrying robot-leg control messages (v5+:
// the robot's hello; the relay never forwards @-prefixed frames to viewers).
// The payload reuses the datagram encoding (raw UTF-8 JSON).
export const CONTROL_CHANNEL = "@control";

// Cap for an @control frame's payload, far below MAX_DATA_FRAME_BYTES: the
// relay enforces it before buffering the payload (pre-authentication frames
// must not allocate unbounded state) and the robot client refuses to send
// beyond it.
export const MAX_CONTROL_PAYLOAD_BYTES = 64 * 1024;

// Reject absurd header lengths before allocating.
export const MAX_HEADER_LEN = 65536;

// Upper bound for a whole data frame; guards receivers against buffering a
// hostile/corrupt payloadLen (also the relay's ingress cap).
export const MAX_DATA_FRAME_BYTES = 64 * 1024 * 1024;

export type Role = "robot" | "viewer";

export interface RobotInfo {
  id: string;
  name: string;
  model: string;
}

// The manifest rides the wire as one opaque record: the transport checks
// only record-ness, and parseManifest (manifest.ts) is the single owner of
// its structure. Additive manifest changes therefore never touch the
// protocol or the relay, and a structurally-alien future manifest still
// reaches the domain parser (which reports unsupported_version) instead of
// being silently dropped here.
export type RobotManifest = Record<string, unknown>;

export interface HelloMsg {
  t: "hello";
  v: number;
  role: Role;
  // role=robot only: identity + channel manifest, registered by the relay.
  robot?: RobotInfo;
  manifest?: RobotManifest;
}

export interface WelcomeMsg {
  t: "welcome";
  v: number;
}

export interface PingMsg {
  t: "ping";
  n: number;
  ts: number;
}

export interface PongMsg {
  t: "pong";
  n: number;
  ts: number;
}

export interface ErrorMsg {
  t: "error";
  code: string;
  message: string;
}

// Session messages (T2): robot registration, viewer watch + per-channel
// subscriptions, and the relay->robot subscription snapshot.
export interface RobotsMsg {
  t: "robots";
  robots: RobotInfo[];
}

export interface WatchMsg {
  t: "watch";
  robotId: string;
}

export interface ManifestMsg {
  t: "manifest";
  robotId: string;
  // Absent = the robot registered without a manifest.
  manifest?: RobotManifest;
}

export interface SubMsg {
  t: "sub";
  ch: string;
}

export interface UnsubMsg {
  t: "unsub";
  ch: string;
}

// Relay->robot: the full set of channels with >= 1 subscribed viewer. A
// snapshot (not a delta) because it rides lossy datagrams: any single delivery
// heals the state. `n` is monotonic per robot; receivers ignore stale/reordered
// snapshots.
export interface SubsMsg {
  t: "subs";
  chs: string[];
  n: number;
}

// Teleop (T6). twist/stop ride datagrams viewer->relay->robot (loss-tolerant:
// commands repeat and the bridge deadman covers silence). The lease messages
// ride the viewer's control stream so they are ordered after watch:
// teleop_start requests the per-robot exclusive lease (relay acks with
// teleop_started, or replies error code "teleop_held"), teleop_stop releases
// it. Robot-bound teleop messages are datagrams stamped with `gen`, the
// relay-issued lease generation: teleop_start announces a granted lease,
// teleop_stop means "the lease ended" (holder gone), twist/stop are the
// forwarded holder commands. The bridge permanently rejects generations
// below its floor, so a released holder's delayed datagrams cannot move the
// robot after a stop. Viewer-authored messages never carry gen.
export interface TwistMsg {
  t: "twist";
  vx: number;
  vy: number;
  wz: number;
  seq: number;
  ts: number;
  gen?: number;
}

export interface StopMsg {
  t: "stop";
  seq: number;
  ts: number;
  gen?: number;
}

export interface TeleopStartMsg {
  t: "teleop_start";
  gen?: number;
}

export interface TeleopStartedMsg {
  t: "teleop_started";
}

export interface TeleopStopMsg {
  t: "teleop_stop";
  gen?: number;
}

export type ControlMsg = HelloMsg | WelcomeMsg | PingMsg | PongMsg | ErrorMsg;
export type SessionMsg = RobotsMsg | WatchMsg | ManifestMsg | SubMsg | UnsubMsg | SubsMsg;
export type TeleopMsg = TwistMsg | StopMsg | TeleopStartMsg | TeleopStartedMsg | TeleopStopMsg;
export type Msg = ControlMsg | SessionMsg | TeleopMsg;

// Data-plane frame header. `delivery` tells the relay how to forward frames
// on channels the robot's manifest does not declare (the manifest's delivery
// wins when present). `meta` carries encoding-specific extras (e.g. {w, h}
// for images).
export interface FrameHeader {
  ch: string;
  seq: number;
  ts: number;
  delivery: Delivery;
  meta?: Record<string, unknown>;
}

export interface DataFrame {
  header: FrameHeader;
  payload: Uint8Array;
}

const enc = new TextEncoder();
// fatal: reject invalid UTF-8 like Python does, instead of U+FFFD-substituting
// corrupted bytes into channel names or message fields.
const dec = new TextDecoder("utf-8", { fatal: true });

// Runtime field validation, mirrored by the pydantic models in protocol.py:
// "string" is a JSON string, "number" any JSON number (booleans excluded by
// typeof). Structured fields (nested objects/arrays) are checked by
// MSG_VALIDATORS below.
const MSG_FIELDS: Record<string, Record<string, "string" | "number">> = {
  hello: { v: "number", role: "string" },
  welcome: { v: "number" },
  ping: { n: "number", ts: "number" },
  pong: { n: "number", ts: "number" },
  error: { code: "string", message: "string" },
  robots: {},
  watch: { robotId: "string" },
  manifest: { robotId: "string" },
  sub: { ch: "string" },
  unsub: { ch: "string" },
  subs: { n: "number" },
  twist: { vx: "number", vy: "number", wz: "number", seq: "number", ts: "number" },
  stop: { seq: "number", ts: "number" },
  teleop_start: {},
  teleop_started: {},
  teleop_stop: {},
};

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function isRobotInfo(value: unknown): value is RobotInfo {
  return (
    isRecord(value) &&
    typeof value.id === "string" &&
    typeof value.name === "string" &&
    typeof value.model === "string"
  );
}

// Structural checks for nested fields, run after the flat MSG_FIELDS pass.
// Optional fields (hello.robot, hello/manifest.manifest, teleop gen) accept
// absent but reject null: JSON encoders on both sides omit absent fields and
// never emit null. The manifest is only checked for record-ness here -- its
// structure belongs to parseManifest (see RobotManifest above).
const genAbsentOrNumber = (v: Record<string, unknown>) =>
  v.gen === undefined || typeof v.gen === "number";
const MSG_VALIDATORS: Record<string, (value: Record<string, unknown>) => boolean> = {
  hello: (v) =>
    (v.robot === undefined || isRobotInfo(v.robot)) &&
    (v.manifest === undefined || isRecord(v.manifest)),
  robots: (v) => Array.isArray(v.robots) && v.robots.every(isRobotInfo),
  manifest: (v) => v.manifest === undefined || isRecord(v.manifest),
  subs: (v) => Array.isArray(v.chs) && v.chs.every((c) => typeof c === "string"),
  twist: genAbsentOrNumber,
  stop: genAbsentOrNumber,
  teleop_start: genAbsentOrNumber,
  teleop_stop: genAbsentOrNumber,
};

/** Validated message from parsed JSON; null for unknown or malformed ones. */
export function msgFromUnknown(value: unknown): Msg | null {
  if (!isRecord(value) || typeof value.t !== "string") return null;
  const fields = Object.hasOwn(MSG_FIELDS, value.t) ? MSG_FIELDS[value.t] : undefined;
  if (fields === undefined) return null;
  for (const [name, kind] of Object.entries(fields)) {
    const actual = typeof value[name];
    if (actual !== kind) return null;
  }
  const structural = Object.hasOwn(MSG_VALIDATORS, value.t) ? MSG_VALIDATORS[value.t] : undefined;
  if (structural !== undefined && !structural(value)) return null;
  return value as unknown as Msg;
}

/** Validated data-frame header from parsed JSON; null if malformed. */
export function frameHeaderFromUnknown(value: unknown): FrameHeader | null {
  if (
    isRecord(value) &&
    typeof value.ch === "string" &&
    // Declared channels are already bounded to this by manifest validation,
    // so only unroutable undeclared names are dropped (they count
    // framesDropped at the relay).
    value.ch.length <= MAX_MANIFEST_ID_LEN &&
    typeof value.seq === "number" &&
    typeof value.ts === "number" &&
    (value.delivery === "latest" || value.delivery === "reliable") &&
    (value.meta === undefined || isRecord(value.meta))
  ) {
    return value as unknown as FrameHeader;
  }
  return null;
}

export function encodeControlFrame(msg: Msg): Uint8Array {
  const body = enc.encode(JSON.stringify(msg));
  const out = new Uint8Array(4 + body.length);
  new DataView(out.buffer).setUint32(0, body.length, true);
  out.set(body, 4);
  return out;
}

/**
 * Incremental parser for a control stream (frames may split across chunks).
 * Malformed or unknown messages are dropped with a console line (the length
 * prefix keeps framing intact); framing errors still throw.
 */
export class ControlFrameReader {
  #buf = new Uint8Array(0);

  push(chunk: Uint8Array): Msg[] {
    const merged = new Uint8Array(this.#buf.length + chunk.length);
    merged.set(this.#buf, 0);
    merged.set(chunk, this.#buf.length);
    this.#buf = merged;
    const msgs: Msg[] = [];
    while (this.#buf.length >= 4) {
      const len = new DataView(this.#buf.buffer, this.#buf.byteOffset).getUint32(0, true);
      if (len === 0 || len > MAX_HEADER_LEN) {
        throw new Error(`invalid control frame length: ${len}`);
      }
      if (this.#buf.length < 4 + len) break;
      const body = this.#buf.subarray(4, 4 + len);
      this.#buf = this.#buf.subarray(4 + len);
      let msg: Msg | null = null;
      try {
        msg = msgFromUnknown(JSON.parse(dec.decode(body)));
      } catch {
        // bad UTF-8 or bad JSON: dropped below
      }
      if (msg === null) console.warn("[protocol] dropping bad control message");
      else msgs.push(msg);
    }
    return msgs;
  }
}

export function encodeDatagram(msg: Msg): Uint8Array {
  return enc.encode(JSON.stringify(msg));
}

/** Returns null for datagrams that are not our JSON messages. */
export function decodeDatagram(data: Uint8Array): Msg | null {
  try {
    return msgFromUnknown(JSON.parse(dec.decode(data)));
  } catch {
    return null;
  }
}

export function encodeDataFrame(header: FrameHeader, payload: Uint8Array): Uint8Array {
  const hdr = enc.encode(JSON.stringify(header));
  const out = new Uint8Array(8 + hdr.length + payload.length);
  const dv = new DataView(out.buffer);
  dv.setUint32(0, hdr.length, true);
  dv.setUint32(4, payload.length, true);
  out.set(hdr, 8);
  out.set(payload, 8 + hdr.length);
  return out;
}

/** Byte lengths of a data frame, or null if fewer than 8 bytes are available. */
export function peekDataFrameLengths(
  buf: Uint8Array,
): { headerLen: number; payloadLen: number; total: number } | null {
  if (buf.length < 8) return null;
  const dv = new DataView(buf.buffer, buf.byteOffset);
  const headerLen = dv.getUint32(0, true);
  const payloadLen = dv.getUint32(4, true);
  if (headerLen > MAX_HEADER_LEN) throw new Error(`data frame header too large: ${headerLen}`);
  const total = 8 + headerLen + payloadLen;
  if (total > MAX_DATA_FRAME_BYTES) throw new Error(`data frame too large: ${total} bytes`);
  return { headerLen, payloadLen, total };
}

export function decodeDataFrame(frame: Uint8Array): DataFrame {
  const lens = peekDataFrameLengths(frame);
  if (lens === null || frame.length < lens.total) {
    throw new Error(`truncated data frame: ${frame.length} bytes`);
  }
  const header = frameHeaderFromUnknown(
    JSON.parse(dec.decode(frame.subarray(8, 8 + lens.headerLen))),
  );
  if (header === null) throw new Error("invalid data frame header");
  return { header, payload: frame.subarray(8 + lens.headerLen, lens.total) };
}

/** First `limit` bytes of `chunks`, copied into one fresh buffer. */
export function concatBytes(chunks: Uint8Array[], limit: number): Uint8Array {
  const out = new Uint8Array(limit);
  let off = 0;
  for (const c of chunks) {
    if (off >= limit) break;
    const take = Math.min(c.byteLength, limit - off);
    out.set(take === c.byteLength ? c : c.subarray(0, take), off);
    off += take;
  }
  return out;
}

/**
 * Framing/header corruption on a data-frame stream. `frames` carries the
 * frames decoded before the corrupt one so the caller can still deliver them
 * before dropping the stream (framing is unrecoverable mid-stream).
 */
export class DataFrameStreamError extends Error {
  constructor(message: string, readonly frames: DataFrame[]) {
    super(message);
    this.name = "DataFrameStreamError";
  }
}

/**
 * Incremental reader for a stream carrying sequential data frames (a reliable
 * channel's persistent stream; a latest stream is the one-frame case). Frames
 * are returned as soon as their bytes have arrived; EOF is never a boundary.
 *
 * Chunks are queued behind a read cursor, never concatenated: a completed
 * frame is copied out exactly once, so a 64 MiB frame arriving in 64 KiB
 * chunks costs 64 MiB of copying, not gigabytes. On corruption push() throws
 * DataFrameStreamError (with the frames decoded before it) and the reader
 * rejects all further input.
 */
export class DataFrameStreamReader {
  #chunks: Uint8Array[] = [];
  #head = 0; // index of the chunk holding the read cursor
  #cursor = 0; // byte offset into #chunks[#head]
  #size = 0; // unread bytes across all queued chunks
  #failed: string | null = null;

  push(chunk: Uint8Array): DataFrame[] {
    if (this.#failed !== null) throw new DataFrameStreamError(this.#failed, []);
    if (chunk.byteLength > 0) {
      this.#chunks.push(chunk);
      this.#size += chunk.byteLength;
    }
    const frames: DataFrame[] = [];
    try {
      while (this.#size >= 8) {
        const lens = peekDataFrameLengths(this.#peek(8))!;
        if (this.#size < lens.total) break;
        frames.push(decodeDataFrame(this.#take(lens.total)));
      }
    } catch (e) {
      this.#failed = (e as Error).message;
      throw new DataFrameStreamError(this.#failed, frames);
    } finally {
      this.#chunks.splice(0, this.#head);
      this.#head = 0;
    }
    return frames;
  }

  /** First `n` unread bytes without consuming them (requires #size >= n). */
  #peek(n: number): Uint8Array {
    const first = this.#chunks[this.#head];
    if (first.byteLength - this.#cursor >= n) {
      return first.subarray(this.#cursor, this.#cursor + n);
    }
    const out = new Uint8Array(n);
    let off = 0;
    let cursor = this.#cursor;
    for (let i = this.#head; off < n; i++) {
      const chunk = this.#chunks[i];
      const step = Math.min(chunk.byteLength - cursor, n - off);
      out.set(chunk.subarray(cursor, cursor + step), off);
      off += step;
      cursor = 0;
    }
    return out;
  }

  /** Consume `n` unread bytes into one fresh buffer (requires #size >= n). */
  #take(n: number): Uint8Array {
    const out = new Uint8Array(n);
    let off = 0;
    while (off < n) {
      const chunk = this.#chunks[this.#head];
      const step = Math.min(chunk.byteLength - this.#cursor, n - off);
      out.set(chunk.subarray(this.#cursor, this.#cursor + step), off);
      off += step;
      this.#cursor += step;
      if (this.#cursor === chunk.byteLength) {
        this.#head++;
        this.#cursor = 0;
      }
    }
    this.#size -= n;
    return out;
  }
}
