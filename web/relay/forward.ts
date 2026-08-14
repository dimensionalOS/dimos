// Robot->viewer forwarding primitives: per-(viewer, channel) delivery
// policies over a transport-blind ViewerSink (unit-testable without QUIC),
// plus the raw-QUIC robot stream readers. Routing lives in registry.ts; the
// relay never parses payloads, only frame headers.
import {
  concatBytes,
  type Delivery,
  type FrameHeader,
  frameHeaderFromUnknown,
  peekDataFrameLengths,
} from "@dimos/shared";

// Reliable channels: a viewer this far behind is dead weight; kick it so it
// reconnects with a clean slate (T5 hardens and tunes these).
const RELIABLE_MAX_QUEUE = 64;
const RELIABLE_MAX_BYTES = 16 * 1024 * 1024;

// One decoder for every robot frame header (fatal so corrupt UTF-8 drops the
// frame rather than routing a mangled channel name).
const headerDecoder = new TextDecoder("utf-8", { fatal: true });

/** Transport surface a policy writes to. */
export interface ViewerSink {
  /** One uni stream per call (latest channels; reset semantics need it). */
  sendFrame(bytes: Uint8Array): Promise<void>;
  /** One persistent uni stream (reliable channels pack frames onto it). */
  openStream(): Promise<FrameWriter>;
  kick(reason: string): void;
}

export interface FrameWriter {
  write(bytes: Uint8Array): Promise<void>;
  abort(reason?: unknown): Promise<void>;
}

export interface ChannelPolicy {
  readonly delivery: Delivery;
  sent: number;
  dropped: number;
  queued(): number;
  offer(bytes: Uint8Array): void;
  /** Discard queued frames and release any persistent stream; later offers
   * and in-flight drain completions become no-ops. Idempotent. */
  dispose(): void;
}

/**
 * Latest-wins: a 1-slot pending buffer. A frame arriving while a write is in
 * flight replaces the pending one (newest wins); the final frame is always
 * eventually delivered. A slow viewer sheds its own frames and nothing else.
 */
export class LatestChannel implements ChannelPolicy {
  readonly delivery: Delivery = "latest";
  sent = 0;
  dropped = 0;
  #pending: Uint8Array | null = null;
  #writing = false;
  #disposed = false;

  constructor(readonly sink: ViewerSink) {}

  queued(): number {
    return this.#pending ? 1 : 0;
  }

  offer(bytes: Uint8Array): void {
    if (this.#disposed) return;
    if (this.#pending) this.dropped++;
    this.#pending = bytes;
    this.#drain();
  }

  dispose(): void {
    this.#disposed = true;
    this.#pending = null;
  }

  #drain(): void {
    if (this.#writing || this.#disposed) return;
    this.#writing = true;
    (async () => {
      while (this.#pending) {
        const bytes = this.#pending;
        this.#pending = null;
        await this.sink.sendFrame(bytes);
        this.sent++;
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
 * Reliable: bounded per-viewer FIFO, no drops, delivery order preserved. On
 * overflow the viewer is kicked (better a visible reconnect than silent loss).
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
  #fifo: Uint8Array[] = [];
  #bytes = 0;
  #writing = false;
  #writer: FrameWriter | null = null;
  #disposed = false;

  constructor(readonly sink: ViewerSink) {}

  queued(): number {
    return this.#fifo.length;
  }

  offer(bytes: Uint8Array): void {
    if (this.#disposed) return;
    this.#fifo.push(bytes);
    this.#bytes += bytes.byteLength;
    if (this.#fifo.length > RELIABLE_MAX_QUEUE || this.#bytes > RELIABLE_MAX_BYTES) {
      this.sink.kick("reliable channel overflow");
      return;
    }
    this.#drain();
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

/**
 * Read one length-prefixed data frame from a robot stream, stopping at the
 * frame's byte count - never at EOF (Deno 2.6.x delays FIN by up to ~1 s, and
 * a reset-stale writer may never send one). BYOB reader: default readers were
 * observed to never deliver on Deno 2.6.10 incoming WT streams.
 */
export async function readDataFrameBytes(rs: ReadableStream<Uint8Array>): Promise<Uint8Array> {
  const reader = rs.getReader({ mode: "byob" });
  const chunks: Uint8Array[] = [];
  let size = 0;
  let total: number | null = null;
  try {
    while (total === null || size < total) {
      const { value, done } = await reader.read(new Uint8Array(64 * 1024));
      if (value && value.byteLength) {
        chunks.push(value);
        size += value.byteLength;
        if (total === null && size >= 8) {
          // peekDataFrameLengths throws on an oversize total (MAX_DATA_FRAME_BYTES).
          const lens = peekDataFrameLengths(concatBytes(chunks, 8));
          if (lens !== null) total = lens.total;
        }
      }
      if (done) break;
    }
  } finally {
    reader.releaseLock();
  }
  if (total === null || size < total) {
    throw new Error(`robot stream ended mid-frame (${size} bytes)`);
  }
  return concatBytes(chunks, total);
}
