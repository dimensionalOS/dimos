// Robot->viewer forwarding primitives: per-(viewer, channel) delivery
// policies over a transport-blind ViewerSink (unit-testable without QUIC),
// plus the raw-QUIC robot stream readers. Routing lives in registry.ts; the
// relay never parses payloads, only frame headers.
import {
  concatBytes,
  type Delivery,
  type FrameHeader,
  frameHeaderFromUnknown,
  MAX_CONTROL_PAYLOAD_BYTES,
  peekDataFrameLengths,
  RESERVED_CHANNEL_PREFIX,
} from "@dimos/shared";

// Reliable channels: a viewer this far behind is dead weight; kick it so it
// reconnects with a clean slate.
/** How long a LatestPersistentChannel write may stay outstanding before the
 * persistent stream is reset to discard its stale backlog. Well past
 * LATEST_STALE_MS so an ordinarily slow viewer keeps its stream and only a
 * genuinely stuck one (a suspended tab) is reset. */
const LATEST_PERSISTENT_RESET_MS = 2000;

const RELIABLE_MAX_QUEUE = 64;
const RELIABLE_MAX_BYTES = 16 * 1024 * 1024;

// A latest stream still un-reset this long after its frame was accepted is
// presumed stale: a newer offer resets it. Matches the Python robot leg's
// stale_after. Healthy viewers read frames long before this; the reset then
// only ends an already-consumed stream (and returns its Firefox uni-stream
// credit far faster than Deno's ~1 s lazy FIN would).
export const LATEST_STALE_MS = 500;

// One decoder for every robot frame header (fatal so corrupt UTF-8 drops the
// frame rather than routing a mangled channel name).
const headerDecoder = new TextDecoder("utf-8", { fatal: true });

/** One latest-channel frame handed to the transport. */
export interface FrameSend {
  /** Settles when the transport ACCEPTED the whole frame into its send
   * buffers (flow-control/credit acceptance, NOT delivery); rejects on
   * abort() or transport failure. */
  readonly done: Promise<void>;
  /** True once abort() was called; a rejection of `done` is then expected. */
  readonly aborted: boolean;
  /** Reset the frame's stream: quinn discards buffered unsent data and the
   * receiver discards buffered unread data. Rejects `done` immediately even
   * while the stream is still being created. Idempotent; never throws. */
  abort(): void;
  /** Replace the payload if the stream write has not started (the send is
   * still wedged in stream creation); false once the write began or after
   * abort(). */
  supersede(bytes: Uint8Array): boolean;
}

/** Transport surface a policy writes to. */
export interface ViewerSink {
  /** One uni stream per call, never FIN'd (a closed WritableStream cannot be
   * aborted, per spec abort() on a closed stream is a no-op): every latest
   * stream ends in a reset - reap, dispose, or session teardown. Receivers
   * dispatch frames on byte count and treat the reset as end-of-stream.
   * Returns synchronously; all async work rides `done`. */
  sendFrame(bytes: Uint8Array): FrameSend;
  /** One persistent uni stream (reliable channels pack frames onto it). */
  openStream(): Promise<FrameWriter>;
  kick(reason: string): void;
}

export interface FrameWriter {
  write(bytes: Uint8Array): Promise<void>;
  abort(reason?: unknown): Promise<void>;
}

export interface PolicyOptions {
  staleMs?: number;
  now?: () => number;
}

export interface ChannelPolicy {
  readonly delivery: Delivery;
  /** Frames accepted by the transport (acceptance, not delivery). */
  sent: number;
  /** Frames shed before reaching the wire (latest: pending-slot replacement,
   * carrier supersede replacement, and the payload of a stale write-wedged
   * carrier reset; reliable: never, it kicks instead). Outside dispose/kick:
   * offers == sent + dropped + queued(). */
  dropped: number;
  /** Latest streams reset while the channel was backpressured (the newest
   * send still unaccepted): stale accepted streams reaped, plus stale
   * write-wedged carriers themselves. The stalled/suspended-viewer signal,
   * 0 when healthy. */
  aborted: number;
  /** Latest streams reset as routine end-of-life (~= sent when healthy:
   * every latest stream ends in a reset). */
  expired: number;
  /** Payload bytes accepted by the transport (a superseded carrier counts
   * the payload it finally carried). */
  bytesOut: number;
  /** Acceptance rate over the trailing window. */
  readonly rate: Rate;
  queued(): number;
  /** Latest streams handed to the transport and not yet reset. */
  inflight(): number;
  offer(bytes: Uint8Array): void;
  /** Reset accepted latest streams older than staleMs; clock-free (callers
   * pass nowMs). Never touches the unaccepted carrier. No-op for reliable. */
  reap(nowMs: number): void;
  /** Discard queued frames, reset in-flight sends, and release any
   * persistent stream; later offers and in-flight drain completions become
   * no-ops. Idempotent. */
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

interface OutstandingSend {
  send: FrameSend;
  at: number;
  size: number;
  accepted: boolean;
}

/**
 * Latest-wins: a 1-slot pending buffer feeds sequential sends; a frame
 * arriving while a send is in flight replaces the pending one (newest wins),
 * and the final frame is always eventually delivered. A slow viewer sheds
 * its own frames and nothing else.
 *
 * Acceptance is not delivery (quinn buffers writes without bound and QUIC
 * exposes no delivery signal to JS), so accepted streams are tracked and
 * REAPED: newer offers (and the relay's periodic reap; see server.ts) reset
 * streams older than staleMs, discarding their buffered-but-undelivered
 * bytes on both ends. A suspended viewer therefore resumes to at most
 * ~staleMs of backlog instead of a connection window's worth of stale video
 * replayed oldest-first.
 */
export class LatestChannel implements ChannelPolicy {
  readonly delivery: Delivery = "latest";
  sent = 0;
  dropped = 0;
  aborted = 0;
  expired = 0;
  bytesOut = 0;
  readonly rate = new Rate();
  #pending: Uint8Array | null = null;
  #outstanding: OutstandingSend[] = []; // append order = age order
  #writing = false;
  #disposed = false;
  readonly #staleMs: number;
  readonly #now: () => number;

  constructor(readonly sink: ViewerSink, opts: PolicyOptions = {}) {
    this.#staleMs = opts.staleMs ?? LATEST_STALE_MS;
    this.#now = opts.now ?? Date.now;
  }

  queued(): number {
    return this.#pending ? 1 : 0;
  }

  inflight(): number {
    return this.#outstanding.length;
  }

  offer(bytes: Uint8Array): void {
    if (this.#disposed) return;
    const now = this.#now();
    this.reap(now);
    const last = this.#outstanding[this.#outstanding.length - 1];
    if (last !== undefined && !last.accepted) {
      // The carrier (sends are sequential: at most one unaccepted entry,
      // always the tail). While it is still wedged in stream creation the
      // newest payload replaces its old one in place: no reset, no
      // replacement stream (README bug 12).
      if (last.send.supersede(bytes)) {
        this.dropped++; // the displaced carrier payload
        last.at = now;
        last.size = bytes.byteLength;
        // Provably already null (parking only happens after supersede has
        // latched false for this carrier); kept as cheap insurance.
        this.#pending = null;
        return;
      }
      // The write has begun, so the payload can no longer change: reset a
      // stale carrier and let the newest go out on a fresh stream.
      if (now - last.at >= this.#staleMs) {
        // Pop first so reap and inflight() never see a dead unaccepted entry.
        this.#outstanding.pop();
        last.send.abort();
        this.aborted++;
        this.dropped++; // its payload never reached the wire
      }
    }
    if (this.#pending) this.dropped++;
    this.#pending = bytes;
    this.#drain();
  }

  /**
   * Reset accepted sends older than staleMs. The unaccepted tail (at most
   * one: sends are sequential) is never reaped: it is the drain's pacing
   * carrier, and resetting a create-wedged send would only spawn a
   * replacement wedged on the same exhausted stream credit - resets do not
   * replenish credit while the viewer's application is not reading its
   * incoming streams (verified against Deno's client; a frozen tab behaves
   * the same way). Newer offers supersede its payload in place instead
   * (details in README bug 12).
   */
  reap(nowMs: number): void {
    const backpressured = this.#outstanding.length > 0 &&
      !this.#outstanding[this.#outstanding.length - 1].accepted;
    while (this.#outstanding.length > 0) {
      const oldest = this.#outstanding[0];
      if (!oldest.accepted || nowMs - oldest.at < this.#staleMs) break;
      this.#outstanding.shift();
      oldest.send.abort();
      if (backpressured) this.aborted++;
      else this.expired++;
    }
  }

  dispose(): void {
    if (this.#disposed) return;
    // Set first: the in-flight rejection this triggers must classify as
    // disposal, not viewer failure.
    this.#disposed = true;
    this.#pending = null;
    for (const entry of this.#outstanding) entry.send.abort();
    this.#outstanding.length = 0;
  }

  #drain(): void {
    if (this.#writing || this.#disposed) return;
    this.#writing = true;
    (async () => {
      while (this.#pending) {
        const bytes = this.#pending;
        this.#pending = null;
        const send = this.sink.sendFrame(bytes);
        const entry: OutstandingSend = {
          send,
          at: this.#now(),
          size: bytes.byteLength,
          accepted: false,
        };
        this.#outstanding.push(entry);
        try {
          await send.done;
          entry.accepted = true;
          // Staleness is measured from acceptance (per the LATEST_STALE_MS
          // doc): a long-wedged carrier accepted on resume must not be
          // instantly reaped by the idle reap timer.
          entry.at = this.#now();
          this.sent++;
          this.bytesOut += entry.size;
          this.rate.push(entry.size, entry.at);
        } catch (e) {
          // dispose() or a stale-carrier reset in offer() aborted it (the
          // entry is already off the list), or the transport failed.
          if (!send.aborted && !this.#disposed) throw e;
        }
      }
    })()
      .catch(() => {
        if (this.#disposed) return; // failure caused by disposal, not the viewer
        this.sink.kick("write failed");
        this.dispose();
      })
      .finally(() => {
        // Clearing #writing and rechecking must be one synchronous step: a
        // frame offered between the loop observing an empty queue and this
        // callback saw #writing still true and started no drain, so only
        // this recheck can pick it up.
        this.#writing = false;
        if (this.#pending) this.#drain();
      });
  }
}

/**
 * Latest, carried on ONE persistent uni stream instead of a stream per frame.
 *
 * Same contract as LatestChannel - a depth-1 queue, newest frame wins, an
 * offer arriving mid-write displaces the one waiting rather than queueing
 * behind it - but without the stream-per-frame pattern that the
 * ReliableChannel docstring below already indicts: uni-stream credit is only
 * replenished when streams complete, latest streams are never FIN'd (README
 * bug 12), so a 20 Hz video channel spends most of its time wedged in
 * createUnidirectionalStream waiting for credit and sheds every frame offered
 * meanwhile. Measured on the microduck cockpit over a local relay, a headed
 * Chrome drew 10.6 fps with 263 drops on LatestChannel and 17.4 fps with zero
 * drops here, and multi-second freezes while the robot walked disappeared.
 *
 * Bounded latency is what separates this from ReliableChannel: that one keeps
 * a 64-deep FIFO and would sit a viewer three seconds behind a walking robot
 * before kicking it. Here a slow viewer sees dropped frames, never lag.
 *
 * `delivery` stays "latest": the wire format and the (frozen) manifest are
 * unchanged, and the viewer never reads the field - it is the relay's own
 * forwarding choice. Frames are self-delimiting, so packing many onto one
 * stream needs nothing from the receiver.
 *
 * A persistent stream loses two things the stream-per-frame design got for
 * free, so reap() rebuilds both. A viewer that stops reading (a suspended
 * tab) parks the write in `writer.write` indefinitely:
 *
 *   1. `aborted` stays the suspended-viewer signal - it counts reap ticks
 *      spent with a write outstanding past staleMs, rather than stale streams
 *      reset. Still 0 when healthy, still the "this viewer is stuck" gauge in
 *      /api/stats.
 *   2. Past RESET_MS the persistent stream is reset and reopened. That is
 *      README bug 12's reason for resetting stale latest streams: the reset
 *      discards buffered-but-undelivered bytes on both ends, so a resuming
 *      tab sees fresh video instead of replaying a connection window of stale
 *      frames oldest-first. The healthy path never reaches either branch (a
 *      local write settles in ~1 ms), so no stream churn.
 */
export class LatestPersistentChannel implements ChannelPolicy {
  readonly delivery: Delivery = "latest";
  sent = 0;
  dropped = 0;
  aborted = 0; // no per-frame streams to reset; fixed 0
  expired = 0; // ditto
  bytesOut = 0;
  readonly rate = new Rate();
  #pending: Uint8Array | null = null;
  #writing = false;
  #writer: FrameWriter | null = null;
  #disposed = false;
  /** When the outstanding write started, or null between writes. */
  #writeStartedAt: number | null = null;
  /** reap() reset the stream: the drain must reopen, not kick the viewer. */
  #resetting = false;
  readonly #staleMs: number;
  readonly #now: () => number;

  constructor(readonly sink: ViewerSink, opts: PolicyOptions = {}) {
    this.#staleMs = opts.staleMs ?? LATEST_STALE_MS;
    this.#now = opts.now ?? Date.now;
  }

  queued(): number {
    return this.#pending ? 1 : 0;
  }

  inflight(): number {
    return 0; // one persistent stream, no per-frame streams to reset
  }

  offer(bytes: Uint8Array): void {
    if (this.#disposed) return;
    if (this.#pending) this.dropped++; // superseded by this newer frame
    this.#pending = bytes;
    this.#drain();
  }

  /** Stall watchdog (see the class doc): count a stuck write, and past
   * RESET_MS reset the stream so the backlog cannot follow the viewer into
   * its resume. No per-frame streams to reap. */
  reap(nowMs: number): void {
    const startedAt = this.#writeStartedAt;
    if (startedAt === null || this.#disposed || this.#resetting) return;
    const stalledFor = nowMs - startedAt;
    if (stalledFor < this.#staleMs) return;
    this.aborted++;
    if (stalledFor < LATEST_PERSISTENT_RESET_MS) return;
    // Abort rather than close, for the reason ReliableChannel.dispose gives:
    // close would still flush the stale frames the reset exists to discard.
    // #resetting makes the drain's rejection a reopen instead of a kick; the
    // writer is dropped here so the next drain opens a fresh stream.
    this.#resetting = true;
    const writer = this.#writer;
    this.#writer = null;
    this.#writeStartedAt = null;
    writer?.abort().catch(() => {});
  }

  dispose(): void {
    this.#disposed = true;
    this.#pending = null;
    this.#writeStartedAt = null;
    // Abort, not close, for the reason ReliableChannel.dispose gives: close
    // would still flush the stale frame and hold the stream's credit.
    this.#writer?.abort().catch(() => {});
    this.#writer = null;
  }

  #drain(): void {
    if (this.#writing || this.#disposed) return;
    this.#writing = true;
    (async () => {
      const writer = this.#writer ??= await this.sink.openStream();
      if (this.#disposed) {
        // dispose() ran while the stream was opening and found no writer to
        // abort; release it here.
        writer.abort().catch(() => {});
        this.#writer = null;
        return;
      }
      while (this.#pending) {
        const bytes = this.#pending;
        this.#pending = null; // frames offered during the write displace *this*
        this.#writeStartedAt = this.#now();
        try {
          await writer.write(bytes);
        } finally {
          this.#writeStartedAt = null;
        }
        // A reset landed on this write. Whether it rejected or resolved, the
        // stream is gone: bail out and let the finally reopen. Checked here
        // as well as in the catch so a transport whose abort() does not
        // reject a parked write cannot wedge the channel forever.
        if (this.#resetting) {
          this.#resetting = false;
          return;
        }
        this.sent++;
        this.bytesOut += bytes.byteLength;
        this.rate.push(bytes.byteLength, this.#now());
      }
    })()
      .catch(() => {
        if (this.#disposed) return; // failure caused by disposal, not the viewer
        if (this.#resetting) {
          this.#resetting = false; // our own reset, not a viewer failure
          return;
        }
        this.sink.kick("write failed");
        this.dispose();
      })
      .finally(() => {
        // Same lost-wakeup guard as the other two policies: clear #writing and
        // recheck in one synchronous step.
        this.#writing = false;
        if (this.#pending) this.#drain();
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
 * streams deliver in order, and stream-per-frame exhausts Firefox's ~100
 * uni-stream credit, which is only replenished when streams complete - and
 * Deno's lazy FIN (README bug 2) keeps delivered streams incomplete for
 * seconds (README bug 11).
 */
export class ReliableChannel implements ChannelPolicy {
  readonly delivery: Delivery = "reliable";
  sent = 0;
  dropped = 0;
  aborted = 0; // reliable never resets frames; fixed 0
  expired = 0;
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

  inflight(): number {
    return 0; // one persistent stream, no per-frame streams to reset
  }

  offer(bytes: Uint8Array): void {
    if (this.#disposed) return;
    this.#fifo.push(bytes);
    this.#bytes += bytes.byteLength;
    if (this.#fifo.length > RELIABLE_MAX_QUEUE || this.#bytes > RELIABLE_MAX_BYTES) {
      this.sink.kick("reliable channel overflow");
      // wt.closed teardown is async; until it runs, later offers must be
      // no-ops, not re-queue + re-kick.
      this.dispose();
      return;
    }
    this.#drain();
  }

  reap(_nowMs: number): void {
    // no-op: one persistent stream, no per-frame streams to reset
  }

  dispose(): void {
    this.#disposed = true;
    this.#fifo.length = 0;
    this.#bytes = 0;
    // Abort, not close: close would still deliver the queued stale frames and
    // (with Deno's lazy FIN, README bug 2) hold the stream's credit for
    // seconds; both receivers treat a reset as end-of-stream, dropping a
    // partial frame.
    this.#writer?.abort().catch(() => {});
    this.#writer = null;
  }

  #drain(): void {
    if (this.#writing || this.#disposed) return;
    this.#writing = true;
    (async () => {
      const writer = this.#writer ??= await this.sink.openStream();
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

/**
 * Consume the WebTransport preamble of a raw incoming QUIC bidi stream, then
 * release the lock so readDataFrameBytes can take over. Robot streams are
 * accepted at the QUIC level because a reset racing the preamble read inside
 * wt.incomingBidirectionalStreams errors that stream permanently (rejected
 * pull) and kills the whole accept loop. Throws on a non-WebTransport type or
 * a stream reset/ended mid-preamble; the session id's value is not checked (a
 * robot connection carries exactly one WT session).
 */
export async function readWebTransportPreamble(rs: ReadableStream<Uint8Array>): Promise<number> {
  const reader = rs.getReader({ mode: "byob" });
  try {
    const type = await readVarint(reader);
    if (type !== WT_BIDI_STREAM_TYPE) {
      throw new Error(`not a WebTransport data stream (type ${type})`);
    }
    return await readVarint(reader);
  } finally {
    reader.releaseLock();
  }
}

async function readVarint(reader: ReadableStreamBYOBReader): Promise<number> {
  const first = await readByte(reader);
  const size = 1 << (first >> 6);
  let value = first & 0x3f;
  for (let i = 1; i < size; i++) {
    value = value * 256 + (await readByte(reader));
  }
  return value;
}

async function readByte(reader: ReadableStreamBYOBReader): Promise<number> {
  const { value, done } = await reader.read(new Uint8Array(1));
  if (done || value === undefined || value.byteLength !== 1) {
    throw new Error("stream ended mid-preamble");
  }
  return value[0];
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

/**
 * Read one length-prefixed data frame from a robot stream, stopping at the
 * frame's byte count - never at EOF (Deno 2.6.x delays FIN by up to ~1 s, and
 * a reset-stale writer may never send one). BYOB reader: default readers were
 * observed to never deliver on Deno 2.6.10 incoming WT streams.
 *
 * The header is parsed as soon as its bytes arrive (headerLen is bounded by
 * MAX_HEADER_LEN) and returned alongside the frame so callers dispatch
 * without re-parsing; null = malformed header. `reserved` reports whether
 * the raw header JSON named an @-prefixed channel, independent of full
 * header validity, so sessions fail closed on malformed control instead of
 * misfiling it as ordinary data. Reserved channels are capped at
 * MAX_CONTROL_PAYLOAD_BYTES before their payload is buffered -
 * pre-authentication control must not allocate unbounded state.
 */
export async function readRobotFrame(
  rs: ReadableStream<Uint8Array>,
): Promise<{ header: FrameHeader | null; bytes: Uint8Array; reserved: boolean }> {
  const reader = rs.getReader({ mode: "byob" });
  const chunks: Uint8Array[] = [];
  let size = 0;
  let lens: { headerLen: number; payloadLen: number; total: number } | null = null;
  let header: FrameHeader | null = null;
  let headerParsed = false;
  let reserved = false;
  try {
    while (lens === null || size < lens.total) {
      const { value, done } = await reader.read(new Uint8Array(64 * 1024));
      if (value && value.byteLength) {
        chunks.push(value);
        size += value.byteLength;
        if (lens === null && size >= 8) {
          // peekDataFrameLengths throws on an oversize total (MAX_DATA_FRAME_BYTES).
          lens = peekDataFrameLengths(concatBytes(chunks, 8));
        }
        if (lens !== null && !headerParsed && size >= 8 + lens.headerLen) {
          headerParsed = true;
          let raw: unknown = null;
          try {
            raw = JSON.parse(
              headerDecoder.decode(concatBytes(chunks, 8 + lens.headerLen).subarray(8)),
            );
          } catch {
            // bad UTF-8 or bad JSON: header stays null, dropped downstream
          }
          const ch = typeof raw === "object" && raw !== null && !Array.isArray(raw)
            ? (raw as Record<string, unknown>).ch
            : undefined;
          reserved = typeof ch === "string" && ch.startsWith(RESERVED_CHANNEL_PREFIX);
          if (reserved && lens.payloadLen > MAX_CONTROL_PAYLOAD_BYTES) {
            throw new ControlPayloadTooLargeError(
              `control frame claims a ${lens.payloadLen} B payload ` +
                `(cap ${MAX_CONTROL_PAYLOAD_BYTES})`,
            );
          }
          header = frameHeaderFromUnknown(raw);
        }
      }
      if (done) break;
    }
  } finally {
    reader.releaseLock();
  }
  if (lens === null || size < lens.total) {
    throw new Error(`robot stream ended mid-frame (${size} bytes)`);
  }
  return { header, bytes: concatBytes(chunks, lens.total), reserved };
}
