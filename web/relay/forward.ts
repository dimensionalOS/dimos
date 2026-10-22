// Robot->viewer forwarding primitives: per-(viewer, channel) delivery
// policies over a transport-blind ViewerSink (unit-testable without QUIC),
// plus the raw-QUIC robot stream reader. Routing lives in registry.ts; the
// relay never parses payloads, only frame headers.
import {
  type Delivery,
  type FrameHeader,
  frameHeaderFromUnknown,
  MAX_CONTROL_PAYLOAD_BYTES,
  peekDataFrameLengths,
  RESERVED_CHANNEL_PREFIX,
} from "@dimos/shared";

// Relay->viewer send priorities, all in one place. WebTransport sendOrder
// (backed by quinn stream priority): queued bytes of a higher-order stream
// are sent before those of a lower-order one, so control outranks reliable
// telemetry, which outranks latest video. Every stream of a band shares its
// order (quinn round-robins equals), one persistent stream per channel.
// Datagrams (the Python viewer's control leg) have no sendOrder API.
export const CONTROL_SEND_ORDER = 2;
export const RELIABLE_SEND_ORDER = 1;
export const LATEST_SEND_ORDER = 0;

// Reliable channels: a viewer this far behind is dead weight; kick it so it
// reconnects with a clean slate. A lone frame over the byte cap (up to
// MAX_DATA_FRAME_BYTES) still queues, as on the Python leg: kicking would
// repeat on every frame of that size and never deliver one.
const RELIABLE_MAX_QUEUE = 64;
const RELIABLE_MAX_BYTES = 16 * 1024 * 1024;

// A latest write the transport has not accepted after this long means the
// viewer's stream window is full: it stopped reading (a suspended tab) or
// its link is slower than the channel. Counted per channel as `stalled`,
// the stalled-viewer signal in /api/stats. Matches the Python robot leg's
// stale_after.
export const LATEST_STALL_MS = 500;

// Latest frames written to a viewer and not yet acknowledged by it (v8 ack
// messages, sent as the viewer reads a frame) before the channel holds the
// next one: the backlog a slow link can carry is this many frames, not the
// stream window's worth (20 s of video at 3 fps). Two keeps the link busy
// while the ack of the previous frame travels back. An ack that never comes
// (a lost datagram from the Python viewer, a viewer bug) releases its frame
// after ACK_TIMEOUT_MS, counted as `ackTimeouts`.
export const ACK_BUDGET = 2;
export const ACK_TIMEOUT_MS = 2000;

// One decoder for every robot frame header (fatal so corrupt UTF-8 drops the
// frame rather than routing a mangled channel name).
const headerDecoder = new TextDecoder("utf-8", { fatal: true });

/** Transport surface a policy writes to. */
export interface ViewerSink {
  /** One persistent uni stream at the given sendOrder; the policy packs its
   * frames onto it back to back (receivers dispatch on byte count). Never
   * FIN'd: on dispose it is aborted, which on Deno 2.6.10 delivers what was
   * buffered and ends the stream when the wrapper is finalized. */
  openStream(order: number): Promise<FrameWriter>;
  kick(reason: string): void;
}

export interface FrameWriter {
  write(bytes: Uint8Array): Promise<void>;
  abort(reason?: unknown): Promise<void>;
}

export interface PolicyOptions {
  stallMs?: number;
  ackTimeoutMs?: number;
  now?: () => number;
}

export interface ChannelPolicy {
  readonly delivery: Delivery;
  /** Frames accepted by the transport (acceptance, not delivery). */
  sent: number;
  /** Frames shed before reaching the wire (latest: pending-slot replacement;
   * reliable: never, it kicks instead). Outside dispose/kick:
   * offers == sent + dropped + queued(). */
  dropped: number;
  /** Latest writes the transport accepted only after LATEST_STALL_MS: the
   * viewer's stream window was full (a suspended tab, a slow link). The
   * stalled-viewer signal, 0 when healthy. Fixed 0 for reliable. */
  stalled: number;
  /** Payload bytes accepted by the transport. */
  bytesOut: number;
  /** Acceptance rate over the trailing window. */
  readonly rate: Rate;
  queued(): number;
  /** True while a write waits for the transport to accept it. */
  writing(): boolean;
  /** Latest frames written and not yet acknowledged by the viewer. */
  unacked(): number;
  /** Frames released by the ack timeout instead of an ack. */
  ackTimeouts: number;
  /** Route one frame; `seq` is its header seq (the ack currency). */
  offer(bytes: Uint8Array, seq: number): void;
  /** The viewer has read every frame up to `seq`. */
  ack(seq: number): void;
  /** Periodic housekeeping (the ack timeout); clock passed in. */
  tick(nowMs: number): void;
  /** Discard queued frames and release the persistent stream; later offers
   * and in-flight drain completions become no-ops. Idempotent. */
  dispose(): void;
}

/**
 * Frames/bytes rate over a trailing window: a ring of fixed time buckets,
 * advanced on touch. The current partial bucket is included, so a cold start
 * under-reads briefly. No clock inside: callers pass nowMs (one Date.now()
 * per offer/frame on the hot path; tests fabricate time).
 */
export class Rate {
  static readonly BUCKET_MS = 500;
  static readonly BUCKETS = 10; // 5 s window
  #frames = new Array<number>(Rate.BUCKETS).fill(0);
  #bytes = new Array<number>(Rate.BUCKETS).fill(0);
  #head = -1; // absolute index of the newest bucket; -1 = empty

  #advance(bucket: number): void {
    if (this.#head === -1 || bucket - this.#head >= Rate.BUCKETS) {
      this.#frames.fill(0);
      this.#bytes.fill(0);
    } else {
      for (let b = this.#head + 1; b <= bucket; b++) {
        this.#frames[b % Rate.BUCKETS] = 0;
        this.#bytes[b % Rate.BUCKETS] = 0;
      }
    }
    this.#head = bucket;
  }

  push(bytes: number, nowMs: number): void {
    let bucket = Math.floor(nowMs / Rate.BUCKET_MS);
    if (bucket > this.#head) this.#advance(bucket);
    else bucket = this.#head; // clock wobble: count into the newest bucket
    this.#frames[bucket % Rate.BUCKETS] += 1;
    this.#bytes[bucket % Rate.BUCKETS] += bytes;
  }

  snapshot(nowMs: number): { fps: number; bps: number } {
    const bucket = Math.floor(nowMs / Rate.BUCKET_MS);
    if (bucket > this.#head && this.#head !== -1) this.#advance(bucket);
    let frames = 0;
    let bytes = 0;
    for (const n of this.#frames) frames += n;
    for (const n of this.#bytes) bytes += n;
    const windowS = (Rate.BUCKETS * Rate.BUCKET_MS) / 1000;
    return { fps: Math.round((frames / windowS) * 10) / 10, bps: Math.round(bytes / windowS) };
  }
}

/**
 * Publish-rate limiter: `capacity` tokens refilled continuously at
 * `ratePerSec`, so a short burst up to the capacity passes and the sustained
 * rate converges on ratePerSec. take() consumes one token when available.
 * No clock inside: callers pass nowMs (tests fabricate time).
 */
export class TokenBucket {
  #tokens: number;
  #lastMs: number | null = null;

  constructor(
    readonly ratePerSec: number,
    readonly capacity: number = Math.max(1, Math.ceil(ratePerSec)),
  ) {
    this.#tokens = this.capacity;
  }

  take(nowMs: number): boolean {
    if (this.#lastMs !== null) {
      // max(0, ...): clock wobble must not drain tokens.
      const refill = (Math.max(0, nowMs - this.#lastMs) / 1000) * this.ratePerSec;
      this.#tokens = Math.min(this.capacity, this.#tokens + refill);
    }
    this.#lastMs = nowMs;
    if (this.#tokens < 1) return false;
    this.#tokens -= 1;
    return true;
  }
}

/**
 * Latest-wins: a 1-slot pending buffer feeds sequential writes on one
 * persistent stream; a frame arriving while a write is in flight replaces
 * the pending one (newest wins), and the final frame is always eventually
 * delivered. A slow viewer sheds its own frames and nothing else.
 *
 * Acceptance is not delivery: `write()` resolves when quinn took the bytes
 * into the stream's send window. The viewer's acks (v8) are the delivery
 * signal: a frame is written only while fewer than ACK_BUDGET written frames
 * await their ack, so the backlog on a slow link is bounded by frames, not
 * by the window. A viewer that stops reading (a suspended tab) therefore
 * holds at most that many frames; the write itself stalls (counted as
 * `stalled`) only once the window is full. One stream per channel instead of
 * one per frame: Deno 2.6.10 never resets or ends a stream before its
 * wrapper is finalized and leaks ~400 B per stream for the connection's
 * life, and every client allows only ~100 open streams.
 */
export class LatestChannel implements ChannelPolicy {
  readonly delivery: Delivery = "latest";
  sent = 0;
  dropped = 0;
  stalled = 0;
  ackTimeouts = 0;
  bytesOut = 0;
  readonly rate = new Rate();
  #pending: { bytes: Uint8Array; seq: number } | null = null;
  #unacked: { seq: number; at: number }[] = [];
  #writing = false;
  #writer: FrameWriter | null = null;
  #disposed = false;
  readonly #stallMs: number;
  readonly #ackTimeoutMs: number;
  readonly #now: () => number;

  constructor(readonly sink: ViewerSink, opts: PolicyOptions = {}) {
    this.#stallMs = opts.stallMs ?? LATEST_STALL_MS;
    this.#ackTimeoutMs = opts.ackTimeoutMs ?? ACK_TIMEOUT_MS;
    this.#now = opts.now ?? Date.now;
  }

  queued(): number {
    return this.#pending ? 1 : 0;
  }

  writing(): boolean {
    return this.#writing;
  }

  unacked(): number {
    return this.#unacked.length;
  }

  offer(bytes: Uint8Array, seq: number): void {
    if (this.#disposed) return;
    if (this.#pending) this.dropped++;
    this.#pending = { bytes, seq };
    this.#drain();
  }

  ack(seq: number): void {
    const before = this.#unacked.length;
    // Cumulative, and a restarted producer's low seqs must not pin old
    // entries: anything at or below the acked seq is delivered.
    this.#unacked = this.#unacked.filter((u) => u.seq > seq);
    if (this.#unacked.length < before) this.#drain();
  }

  tick(nowMs: number): void {
    const live = this.#unacked.filter((u) => nowMs - u.at < this.#ackTimeoutMs);
    if (live.length === this.#unacked.length) return;
    this.ackTimeouts += this.#unacked.length - live.length;
    this.#unacked = live;
    this.#drain();
  }

  dispose(): void {
    if (this.#disposed) return;
    // Set first: the in-flight rejection this triggers must classify as
    // disposal, not viewer failure.
    this.#disposed = true;
    this.#pending = null;
    this.#unacked.length = 0;
    this.#writer?.abort().catch(() => {});
    this.#writer = null;
  }

  #drain(): void {
    if (this.#writing || this.#disposed) return;
    if (this.#pending === null || this.#unacked.length >= ACK_BUDGET) return;
    this.#writing = true;
    (async () => {
      const writer = this.#writer ??= await this.sink.openStream(LATEST_SEND_ORDER);
      if (this.#disposed) {
        // dispose() ran while the stream was opening and saw no writer to
        // abort; release the stream here.
        writer.abort().catch(() => {});
        this.#writer = null;
        return;
      }
      while (this.#pending && this.#unacked.length < ACK_BUDGET) {
        const { bytes, seq } = this.#pending;
        this.#pending = null;
        const started = this.#now();
        this.#unacked.push({ seq, at: started });
        await writer.write(bytes);
        if (this.#disposed) return;
        const now = this.#now();
        if (now - started >= this.#stallMs) this.stalled++;
        this.sent++;
        this.bytesOut += bytes.byteLength;
        this.rate.push(bytes.byteLength, now);
      }
    })()
      .catch(() => {
        if (this.#disposed) return; // failure caused by disposal, not the viewer
        this.sink.kick("write failed");
        this.dispose();
      })
      .finally(() => {
        // Clearing #writing and rechecking must be one synchronous step: a
        // frame offered between the loop observing an empty slot and this
        // callback saw #writing still true and started no drain, so only
        // this recheck can pick it up.
        this.#writing = false;
        this.#drain();
      });
  }
}

/**
 * Reliable: bounded per-viewer FIFO, no drops, delivery order preserved. On
 * overflow the viewer is kicked once and the channel self-disposes (better a
 * visible reconnect than silent loss); frames offered before transport
 * teardown completes are ignored.
 *
 * All frames ride ONE persistent uni stream (opened on first use): QUIC
 * streams deliver in order (protocol.md workaround 11).
 */
export class ReliableChannel implements ChannelPolicy {
  readonly delivery: Delivery = "reliable";
  sent = 0;
  dropped = 0;
  stalled = 0; // reliable queues instead of shedding; fixed 0
  ackTimeouts = 0; // reliable frames are not acked; fixed 0
  bytesOut = 0;
  readonly rate = new Rate();
  #fifo: Uint8Array[] = [];
  #bytes = 0;
  #writing = false;
  #writer: FrameWriter | null = null;
  #disposed = false;
  readonly #now: () => number;

  constructor(readonly sink: ViewerSink, opts: PolicyOptions = {}) {
    this.#now = opts.now ?? Date.now;
  }

  queued(): number {
    return this.#fifo.length;
  }

  writing(): boolean {
    return this.#writing;
  }

  unacked(): number {
    return 0;
  }

  ack(_seq: number): void {
    // reliable frames are not acked
  }

  tick(_nowMs: number): void {
    // nothing periodic
  }

  offer(bytes: Uint8Array, _seq: number): void {
    if (this.#disposed) return;
    this.#fifo.push(bytes);
    this.#bytes += bytes.byteLength;
    if (
      this.#fifo.length > RELIABLE_MAX_QUEUE ||
      (this.#bytes > RELIABLE_MAX_BYTES && this.#fifo.length > 1)
    ) {
      this.sink.kick("reliable channel overflow");
      // wt.closed teardown is async; until it runs, later offers must be
      // no-ops, not re-queue + re-kick.
      this.dispose();
      return;
    }
    this.#drain();
  }

  dispose(): void {
    this.#disposed = true;
    this.#fifo.length = 0;
    this.#bytes = 0;
    // Abort, not close: a closed stream cannot be aborted, and both
    // receivers treat the end of the stream as end-of-stream either way.
    this.#writer?.abort().catch(() => {});
    this.#writer = null;
  }

  #drain(): void {
    if (this.#writing || this.#disposed) return;
    this.#writing = true;
    (async () => {
      const writer = this.#writer ??= await this.sink.openStream(RELIABLE_SEND_ORDER);
      if (this.#disposed) {
        // dispose() ran while the stream was opening and saw no writer to
        // abort; release the stream here.
        writer.abort().catch(() => {});
        this.#writer = null;
        return;
      }
      for (let bytes = this.#fifo.shift(); bytes; bytes = this.#fifo.shift()) {
        this.#bytes -= bytes.byteLength;
        await writer.write(bytes);
        this.sent++;
        this.bytesOut += bytes.byteLength;
        this.rate.push(bytes.byteLength, this.#now());
      }
    })()
      .catch(() => {
        if (this.#disposed) return; // failure caused by disposal, not the viewer
        this.sink.kick("write failed");
        this.dispose();
      })
      .finally(() => {
        // Same lost-wakeup guard as LatestChannel: recheck in the step that
        // clears #writing.
        this.#writing = false;
        if (this.#fifo.length > 0) this.#drain();
      });
  }
}

/**
 * Header of a length-complete robot data frame (raw bytes), or null if the
 * header is truncated, malformed, or not valid UTF-8.
 */
export function parseRobotFrameHeader(bytes: Uint8Array): FrameHeader | null {
  const lens = peekDataFrameLengths(bytes);
  if (lens === null) return null;
  try {
    return frameHeaderFromUnknown(
      JSON.parse(headerDecoder.decode(bytes.subarray(8, 8 + lens.headerLen))),
    );
  } catch {
    return null; // bad UTF-8 or bad JSON
  }
}

// First varint of a WebTransport bidi data stream (the preamble is stream
// type + session id, both QUIC varints).
const WT_BIDI_STREAM_TYPE = 0x41;
// Read buffer: a QUIC packet's worth of stream data is ~1.2 KB, so a 4 KiB
// buffer takes the preamble, the 8-byte length prefix and the header in one
// read. The payload buffer starts at the second size and doubles.
const READ_BYTES = 4096;
const FIRST_PAYLOAD_BYTES = 256 * 1024;

function varintAt(buf: Uint8Array, at: number): { value: number; end: number } | null {
  if (at >= buf.length) return null;
  const size = 1 << (buf[at] >> 6);
  if (at + size > buf.length) return null;
  let value = buf[at] & 0x3f;
  for (let i = 1; i < size; i++) value = value * 256 + buf[at + i];
  return { value, end: at + size };
}

/**
 * Byte length of the WebTransport preamble at the start of `buf`, null if
 * more bytes are needed. Throws on a non-WebTransport stream type; the
 * session id's value is not checked (a robot connection carries exactly one
 * WT session).
 */
export function parseWebTransportPreamble(buf: Uint8Array): number | null {
  const type = varintAt(buf, 0);
  if (type === null) return null;
  if (type.value !== WT_BIDI_STREAM_TYPE) {
    throw new Error(`not a WebTransport data stream (type ${type.value})`);
  }
  const session = varintAt(buf, type.end);
  return session === null ? null : session.end;
}

/** An @-channel frame claimed a payload beyond MAX_CONTROL_PAYLOAD_BYTES.
 * Typed so sessions can reject the peer instead of the usual silent
 * per-stream drop. */
export class ControlPayloadTooLargeError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "ControlPayloadTooLargeError";
  }
}

export interface RobotFrame {
  /** Parsed header; null = malformed. */
  header: FrameHeader | null;
  /** The whole data frame (length prefix, header, payload), preamble excluded. */
  bytes: Uint8Array;
  /** True when the raw header JSON named an @-prefixed channel, independent
   * of full header validity, so sessions fail closed on malformed control
   * instead of misfiling it as ordinary data. */
  reserved: boolean;
}

/**
 * Read one robot data stream, accepted at the QUIC level (protocol.md
 * workaround 10): the WebTransport preamble, then one length-prefixed data
 * frame, stopping at the frame's byte count, never at EOF (Deno's own client
 * FINs late, and the stream's slot returns only at finalization either way,
 * so there is nothing to gain from reading on). BYOB reads throughout (default readers never deliver
 * on Deno 2.6.10 incoming WT streams), each into a whole zero-offset buffer
 * (a read into a view at an offset never resolves there, workaround 8): the
 * preamble and header come out of one small read, and the payload is copied
 * chunk by chunk into its final buffer, which grows as bytes arrive.
 *
 * `onHeader` fires once the header bytes are in, before the payload is read,
 * so a session can order frames without waiting for a big one. Reserved
 * (@-channel) frames are capped at MAX_CONTROL_PAYLOAD_BYTES before their
 * payload is buffered: pre-authentication control must not allocate
 * unbounded state.
 */
export async function readRobotStream(
  rs: ReadableStream<Uint8Array>,
  onHeader?: (header: FrameHeader | null, reserved: boolean) => void,
): Promise<RobotFrame> {
  const reader = rs.getReader({ mode: "byob" });
  try {
    // The read buffer: transferred to the stream and handed back with each
    // read, always whole.
    let scratch = new Uint8Array(READ_BYTES);
    // Preamble, length prefix and header (and whatever payload came along).
    let head = new Uint8Array(READ_BYTES);
    let size = 0;
    let preambleLen: number | null = null;
    let lens: { headerLen: number; payloadLen: number; total: number } | null = null;
    while (true) {
      const { value, done } = await reader.read(scratch);
      if (value !== undefined) {
        scratch = new Uint8Array(value.buffer);
        if (head.byteLength < size + value.byteLength) {
          const grown = new Uint8Array(Math.max(head.byteLength * 2, size + value.byteLength));
          grown.set(head.subarray(0, size));
          head = grown;
        }
        head.set(value, size);
        size += value.byteLength;
      }
      if (preambleLen === null) {
        preambleLen = parseWebTransportPreamble(head.subarray(0, size));
        if (preambleLen === null) {
          if (done) throw new Error("stream ended mid-preamble");
          continue;
        }
      }
      if (lens === null && size >= preambleLen + 8) {
        // Throws on an oversize total (MAX_DATA_FRAME_BYTES) or header; null
        // only under 8 bytes, which the guard above excludes.
        lens = peekDataFrameLengths(head.subarray(preambleLen, preambleLen + 8))!;
      }
      if (lens !== null && size >= preambleLen + 8 + lens.headerLen) break;
      if (done) throw new Error(`robot stream ended mid-frame (${size - preambleLen} bytes)`);
    }
    const frameStart = preambleLen;
    let raw: unknown = null;
    try {
      raw = JSON.parse(
        headerDecoder.decode(head.subarray(frameStart + 8, frameStart + 8 + lens.headerLen)),
      );
    } catch {
      // bad UTF-8 or bad JSON: header stays null, dropped downstream
    }
    const ch = typeof raw === "object" && raw !== null && !Array.isArray(raw)
      ? (raw as Record<string, unknown>).ch
      : undefined;
    const reserved = typeof ch === "string" && ch.startsWith(RESERVED_CHANNEL_PREFIX);
    if (reserved && lens.payloadLen > MAX_CONTROL_PAYLOAD_BYTES) {
      throw new ControlPayloadTooLargeError(
        `control frame claims a ${lens.payloadLen} B payload (cap ${MAX_CONTROL_PAYLOAD_BYTES})`,
      );
    }
    const header = frameHeaderFromUnknown(raw);
    onHeader?.(header, reserved);

    // The payload, copied into the frame buffer as it arrives. The buffer
    // starts small and doubles, so a peer claiming a huge payload and
    // sending nothing holds little.
    let filled = Math.min(size - frameStart, lens.total);
    let frame = new Uint8Array(Math.max(Math.min(lens.total, FIRST_PAYLOAD_BYTES), filled));
    frame.set(head.subarray(frameStart, frameStart + filled));
    while (filled < lens.total) {
      const remaining = lens.total - filled;
      // The last read's buffer is exactly the remainder, so a stream that
      // (against the protocol) carries more never over-reads into it.
      if (scratch.byteLength > remaining) scratch = new Uint8Array(remaining);
      const { value, done } = await reader.read(scratch);
      if (value !== undefined) {
        scratch = new Uint8Array(value.buffer);
        if (frame.byteLength < filled + value.byteLength) {
          const grown = new Uint8Array(
            Math.min(lens.total, Math.max(frame.byteLength * 2, filled + value.byteLength)),
          );
          grown.set(frame.subarray(0, filled));
          frame = grown;
        }
        frame.set(value, filled);
        filled += value.byteLength;
      }
      if (done && filled < lens.total) {
        throw new Error(`robot stream ended mid-frame (${filled} bytes)`);
      }
    }
    return {
      header,
      bytes: frame.byteLength === lens.total ? frame : frame.subarray(0, lens.total),
      reserved,
    };
  } finally {
    reader.releaseLock();
  }
}
