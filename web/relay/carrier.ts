// The robot control carrier: one relay-opened reliable uni stream per robot
// session, created lazily on the first control send after registration. It
// carries robot-bound @control data frames (subs snapshots today; tx data and
// acked control later), freeing control from the datagram size ceiling and
// giving it ordered delivery. Transport-blind (a CarrierSink seam like
// forward.ts's ViewerSink) so lifecycle is unit-testable without QUIC.
//
// The carrier is a control dependency: overflow and write failure fail the
// WHOLE robot session via sink.fail() (the bridge reconnects and gets a fresh
// baseline) - control state must never be lost silently on a live session.
// One FIFO also means later frames can never bypass queued subscription state.
import {
  CONTROL_CHANNEL,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  MAX_CONTROL_PAYLOAD_BYTES,
  type Msg,
} from "@dimos/shared";
import type { FrameWriter } from "./forward.ts";

// Generous for control-sized frames: a robot session this far behind has a
// wedged connection, and a reconnect beats an unbounded queue.
const CARRIER_MAX_QUEUE = 256;
const CARRIER_MAX_BYTES = 4 * 1024 * 1024;

/** Transport surface the carrier writes to. */
export interface CarrierSink {
  /** The one persistent uni stream toward the robot. Called at most once. */
  openStream(): Promise<FrameWriter>;
  /** Fail the whole robot session (close it so the bridge reconnects).
   * Called at most once per carrier. */
  fail(reason: string): void;
}

export interface CarrierStats {
  queued: number;
  queuedBytes: number;
  sent: number;
  bytesOut: number;
}

export class RobotCarrier {
  #fifo: Uint8Array[] = [];
  #bytes = 0;
  #writing = false;
  #writer: FrameWriter | null = null;
  #disposed = false;
  #seq = 0;
  #sent = 0;
  #bytesOut = 0;

  constructor(readonly sink: CarrierSink) {}

  /** Queue one robot-bound control message as an @control frame. Delivery
   * order is queue order; a message that cannot be delivered fails the
   * session - control is never dropped or truncated. */
  sendControl(msg: Msg): void {
    if (this.#disposed) return;
    const payload = encodeDatagram(msg);
    if (payload.byteLength > MAX_CONTROL_PAYLOAD_BYTES) {
      // Unreachable for subs snapshots (channel ids are bounded by the
      // hello-capped manifest and the sub-id length check), but never
      // truncate control.
      this.sink.fail(`@control payload is ${payload.byteLength} B (over the control cap)`);
      this.dispose();
      return;
    }
    this.#push(CONTROL_CHANNEL, payload, undefined);
  }

  /** Queue one robot-bound tx data frame (a forwarded viewer publish, its
   * relay provenance in `meta`). Same FIFO as control: a publish can never
   * bypass queued subscription state, and a frame that cannot be delivered
   * fails the session. The registry bounds payload size and pending volume
   * before queueing. */
  sendFrame(ch: string, payload: Uint8Array, meta: Record<string, unknown>): void {
    if (this.#disposed) return;
    this.#push(ch, payload, meta);
  }

  #push(ch: string, payload: Uint8Array, meta: Record<string, unknown> | undefined): void {
    const header: FrameHeader = {
      ch,
      seq: ++this.#seq,
      ts: Date.now() / 1000,
      delivery: "reliable",
      ...(meta !== undefined ? { meta } : {}),
    };
    const frame = encodeDataFrame(header, payload);
    this.#fifo.push(frame);
    this.#bytes += frame.byteLength;
    if (this.#fifo.length > CARRIER_MAX_QUEUE || this.#bytes > CARRIER_MAX_BYTES) {
      this.sink.fail("carrier overflow");
      // Session teardown is async; until it runs, later sends must be
      // no-ops, not re-queue + re-fail.
      this.dispose();
      return;
    }
    this.#drain();
  }

  stats(): CarrierStats {
    return {
      queued: this.#fifo.length,
      queuedBytes: this.#bytes,
      sent: this.#sent,
      bytesOut: this.#bytesOut,
    };
  }

  dispose(): void {
    this.#disposed = true;
    this.#fifo.length = 0;
    this.#bytes = 0;
    // Abort, not close: a closed stream cannot be aborted, Deno's lazy FIN
    // (README bug 2) would hold the stream for seconds, and the bridge reads
    // frames by byte count anyway.
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
      for (let frame = this.#fifo.shift(); frame; frame = this.#fifo.shift()) {
        this.#bytes -= frame.byteLength;
        await writer.write(frame);
        this.#sent++;
        this.#bytesOut += frame.byteLength;
      }
    })()
      .catch(() => {
        if (this.#disposed) return; // failure caused by disposal, not the robot
        this.sink.fail("carrier write failed");
        this.dispose();
      })
      .finally(() => {
        // Same lost-wakeup guard as ReliableChannel: recheck in the step that
        // clears #writing.
        this.#writing = false;
        if (this.#fifo.length > 0) this.#drain();
      });
  }
}
