// Robot->viewer forwarding: per-(viewer, channel) delivery policies over a
// transport-blind ViewerSink, so the policy logic is unit-testable without
// QUIC. The relay never parses payloads; it routes on the frame header only.
import {
  concatBytes,
  type Delivery,
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

/** Transport surface a policy writes to: one uni stream per sendFrame call. */
export interface ViewerSink {
  sendFrame(bytes: Uint8Array): Promise<void>;
  kick(reason: string): void;
}

interface ChannelPolicy {
  readonly delivery: Delivery;
  sent: number;
  dropped: number;
  queued(): number;
  offer(bytes: Uint8Array): void;
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

  constructor(readonly sink: ViewerSink) {}

  queued(): number {
    return this.#pending ? 1 : 0;
  }

  offer(bytes: Uint8Array): void {
    if (this.#pending) this.dropped++;
    this.#pending = bytes;
    this.#drain();
  }

  #drain(): void {
    if (this.#writing) return;
    this.#writing = true;
    (async () => {
      while (this.#pending) {
        const bytes = this.#pending;
        this.#pending = null;
        await this.sink.sendFrame(bytes);
        this.sent++;
      }
    })()
      .catch(() => this.sink.kick("write failed"))
      .finally(() => {
        this.#writing = false;
      });
  }
}

/**
 * Reliable: bounded per-viewer FIFO, no drops, delivery order preserved. On
 * overflow the viewer is kicked (better a visible reconnect than silent loss).
 */
export class ReliableChannel implements ChannelPolicy {
  readonly delivery: Delivery = "reliable";
  sent = 0;
  dropped = 0;
  #fifo: Uint8Array[] = [];
  #bytes = 0;
  #writing = false;

  constructor(readonly sink: ViewerSink) {}

  queued(): number {
    return this.#fifo.length;
  }

  offer(bytes: Uint8Array): void {
    this.#fifo.push(bytes);
    this.#bytes += bytes.byteLength;
    if (this.#fifo.length > RELIABLE_MAX_QUEUE || this.#bytes > RELIABLE_MAX_BYTES) {
      this.sink.kick("reliable channel overflow");
      return;
    }
    this.#drain();
  }

  #drain(): void {
    if (this.#writing) return;
    this.#writing = true;
    (async () => {
      for (let bytes = this.#fifo.shift(); bytes; bytes = this.#fifo.shift()) {
        this.#bytes -= bytes.byteLength;
        await this.sink.sendFrame(bytes);
        this.sent++;
      }
    })()
      .catch(() => this.sink.kick("write failed"))
      .finally(() => {
        this.#writing = false;
      });
  }
}

export interface ViewerHandle {
  id: number;
  sink: ViewerSink;
  channels: Map<string, ChannelPolicy>;
}

interface ChannelInStats {
  delivery: Delivery;
  framesIn: number;
  bytesIn: number;
}

/** Routes robot frames to every viewer through its per-channel policy. */
export class Forwarder {
  #viewers = new Set<ViewerHandle>();
  #channelsIn = new Map<string, ChannelInStats>();
  #nextViewerId = 1;
  #framesDropped = 0;

  addViewer(sink: ViewerSink): ViewerHandle {
    const handle: ViewerHandle = { id: this.#nextViewerId++, sink, channels: new Map() };
    this.#viewers.add(handle);
    return handle;
  }

  removeViewer(handle: ViewerHandle): void {
    this.#viewers.delete(handle);
  }

  get viewerCount(): number {
    return this.#viewers.size;
  }

  /** Route one robot data frame (raw bytes, already length-complete). */
  onRobotFrame(bytes: Uint8Array): void {
    const lens = peekDataFrameLengths(bytes);
    if (lens === null) return;
    let header: ReturnType<typeof frameHeaderFromUnknown> = null;
    try {
      header = frameHeaderFromUnknown(
        JSON.parse(headerDecoder.decode(bytes.subarray(8, 8 + lens.headerLen))),
      );
    } catch {
      // bad UTF-8 or bad JSON: dropped below
    }
    if (header === null) {
      this.#framesDropped++;
      console.log("[relay] dropping robot frame with invalid header");
      return;
    }
    const { ch, delivery } = header;

    const stats = this.#channelsIn.get(ch) ?? { delivery, framesIn: 0, bytesIn: 0 };
    stats.delivery = delivery;
    stats.framesIn++;
    stats.bytesIn += bytes.byteLength;
    this.#channelsIn.set(ch, stats);

    for (const viewer of this.#viewers) {
      let policy = viewer.channels.get(ch);
      if (policy === undefined || policy.delivery !== delivery) {
        policy = delivery === "reliable"
          ? new ReliableChannel(viewer.sink)
          : new LatestChannel(viewer.sink);
        viewer.channels.set(ch, policy);
      }
      policy.offer(bytes);
    }
  }

  stats(): unknown {
    return {
      viewers: this.#viewers.size,
      framesDropped: this.#framesDropped,
      channels: Object.fromEntries(this.#channelsIn),
      perViewer: [...this.#viewers].map((v) => ({
        id: v.id,
        channels: Object.fromEntries(
          [...v.channels].map(([ch, p]) => [
            ch,
            { sent: p.sent, dropped: p.dropped, queued: p.queued() },
          ]),
        ),
      })),
    };
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
