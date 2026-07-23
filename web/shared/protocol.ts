// Wire protocol shared by the relay (Deno) and the Cockpit (browser).
// Mirrored in Python at dimos/web/relay_bridge/protocol.py and pinned by the
// golden vectors in ./fixtures/ (tested from both deno test and pytest).
//
// Framing (see web/README.md for the upstream-bug rationale):
// - Control stream frame: u32-LE length | UTF-8 JSON.
// - Datagram: raw UTF-8 JSON, no length prefix.
// - Data frame (one message per stream): u32-LE headerLen | u32-LE payloadLen
//   | header JSON | payload. Receivers count bytes and must never treat
//   stream EOF as a message boundary (Deno 2.6.x delays FIN by up to ~1 s).
//
// Validation policy (mirrored in protocol.py): decoders validate shape
// strictly, and receivers drop invalid or unknown messages -- a peer's bytes
// must never kill a session. Framing-level corruption (absurd length
// prefixes) throws and kills only the affected stream.

export const PROTOCOL_VERSION = 1;

// Reject absurd header lengths before allocating.
export const MAX_HEADER_LEN = 65536;

// Upper bound for a whole data frame; guards receivers against buffering a
// hostile/corrupt payloadLen (also the relay's ingress cap).
export const MAX_DATA_FRAME_BYTES = 64 * 1024 * 1024;

export type Role = "robot" | "viewer";
export type Delivery = "latest" | "reliable";

export interface HelloMsg {
  t: "hello";
  v: number;
  role: Role;
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

// Teleop datagrams (carried from T6 on; declared here so the wire format is
// pinned by fixtures from day one).
export interface TwistMsg {
  t: "twist";
  vx: number;
  wz: number;
  seq: number;
  ts: number;
}

export interface StopMsg {
  t: "stop";
  seq: number;
  ts: number;
}

export type ControlMsg = HelloMsg | WelcomeMsg | PingMsg | PongMsg | ErrorMsg;
export type TeleopMsg = TwistMsg | StopMsg;
export type Msg = ControlMsg | TeleopMsg;

// Data-plane frame header. `delivery` tells the relay how to forward the
// frame without a manifest (T1 only; the T2+ manifest replaces it). `meta`
// carries encoding-specific extras (e.g. {w, h} for images).
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

// Runtime field validation, mirror of _MSG_FIELD_KINDS in protocol.py:
// "string" is a JSON string, "number" any JSON number (booleans excluded by
// typeof).
const MSG_FIELDS: Record<string, Record<string, "string" | "number">> = {
  hello: { v: "number", role: "string" },
  welcome: { v: "number" },
  ping: { n: "number", ts: "number" },
  pong: { n: "number", ts: "number" },
  error: { code: "string", message: "string" },
  twist: { vx: "number", wz: "number", seq: "number", ts: "number" },
  stop: { seq: "number", ts: "number" },
};

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

/** Validated message from parsed JSON; null for unknown or malformed ones. */
export function msgFromUnknown(value: unknown): Msg | null {
  if (!isRecord(value) || typeof value.t !== "string") return null;
  const fields = MSG_FIELDS[value.t];
  if (fields === undefined) return null;
  for (const [name, kind] of Object.entries(fields)) {
    const actual = typeof value[name];
    if (actual !== kind) return null;
  }
  return value as unknown as Msg;
}

/** Validated data-frame header from parsed JSON; null if malformed. */
export function frameHeaderFromUnknown(value: unknown): FrameHeader | null {
  if (
    isRecord(value) &&
    typeof value.ch === "string" &&
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
 * Incremental reader for a single-message stream. Returns the frame as soon
 * as headerLen + payloadLen bytes have arrived; never waits for EOF. Bytes
 * past the frame are ignored. Chunks are held by reference and copied once
 * at completion (a per-push merge is quadratic at multi-MB frame sizes).
 */
export class DataFrameReader {
  #chunks: Uint8Array[] = [];
  #size = 0;
  #lens: { headerLen: number; payloadLen: number; total: number } | null = null;
  #done = false;

  push(chunk: Uint8Array): DataFrame | null {
    if (this.#done) return null;
    this.#chunks.push(chunk);
    this.#size += chunk.byteLength;
    if (this.#lens === null && this.#size >= 8) {
      this.#lens = peekDataFrameLengths(concatBytes(this.#chunks, 8));
    }
    if (this.#lens === null || this.#size < this.#lens.total) return null;
    this.#done = true;
    const frame = decodeDataFrame(concatBytes(this.#chunks, this.#lens.total));
    this.#chunks = [];
    return frame;
  }
}
