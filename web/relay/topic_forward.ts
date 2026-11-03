import { concatBytes, type Delivery } from "@dimos/shared";
import type { FrameSend, FrameWriter, ViewerSink } from "./forward.ts";

export const TOPIC_QUEUE_BYTES = 16 * 1024 * 1024;
export const TOPIC_QUEUE_FRAMES = 512;
const LATEST_LIFETIME_MS = 500;

/** Shared by all routes of a viewer, and by the whole topic registry. */
export class TopicBudget {
  bytes = 0;
  constructor(readonly limit: number) {}
}

interface Frame {
  bytes: Uint8Array;
  key: string;
}
interface LatestSend {
  send: FrameSend;
  size: number;
  key: string;
  at: number;
  accepted: boolean;
}

/** Topic-only delivery policy. Reliable credit counts all offered bytes,
 * including transport-accepted bytes, until cumulative browser receipts arrive.
 * Latest keeps a bounded pending slot per concrete key and resets old streams. */
export class TopicEgress {
  #queue: Frame[] = [];
  #latest = new Set<LatestSend>();
  #receipts: { frames: number; bytes: number; size: number }[] = [];
  #bytes = 0;
  #frames = 0;
  #sentFrames = 0;
  #sentBytes = 0;
  #ackedFrames = 0;
  #ackedBytes = 0;
  #active = false;
  #closed = false;
  #draining = false;
  #writer: FrameWriter | null = null;
  #nextDue = new Map<string, number>();
  #timer: ReturnType<typeof setTimeout> | null = null;
  #timerAt = Infinity;

  constructor(
    readonly delivery: Delivery,
    readonly sink: ViewerSink,
    readonly budgets: readonly TopicBudget[],
    readonly fail: (code: string) => void,
    readonly now: () => number = Date.now,
    public maxHz = Infinity,
  ) {}

  /** Change this viewer's sampling rate in place, keeping each key's last
   * selection time: no burst and no gap at the switch. */
  setRate(maxHz: number): void {
    const previous = Number.isFinite(this.maxHz) ? 1000 / this.maxHz : 0;
    const period = Number.isFinite(maxHz) ? 1000 / maxHz : 0;
    this.maxHz = maxHz;
    for (const [key, due] of this.#nextDue) this.#nextDue.set(key, due + period - previous);
    if (this.delivery === "latest") this.#drain();
  }

  #release(size: number): void {
    this.#bytes -= size;
    this.#frames--;
    for (const budget of this.budgets) budget.bytes -= size;
  }

  #abortLatest(frame: LatestSend): void {
    if (!this.#latest.delete(frame)) return;
    frame.send.abort();
    this.#release(frame.size);
  }

  #fits(size: number): boolean {
    return this.#frames < TOPIC_QUEUE_FRAMES && this.#bytes + size <= TOPIC_QUEUE_BYTES &&
      this.budgets.every((b) => b.bytes + size <= b.limit);
  }

  offer(key: string, bytes: Uint8Array): void {
    if (this.#closed) return;
    if (this.delivery === "latest") {
      const prior = this.#queue.findIndex((f) => f.key === key);
      if (prior >= 0) this.#release(this.#queue.splice(prior, 1)[0].bytes.byteLength);
      this.reap(this.now());
      // Reset accepted streams before rejecting new work at the byte bound.
      // An unaccepted stream is left alone so a slow link can finish a frame.
      for (const frame of this.#latest) {
        if (this.#fits(bytes.byteLength)) break;
        if (frame.accepted) this.#abortLatest(frame);
      }
    }
    if (!this.#fits(bytes.byteLength)) {
      this.#failure("topic_overflow");
      return;
    }
    this.#bytes += bytes.byteLength;
    this.#frames++;
    for (const budget of this.budgets) budget.bytes += bytes.byteLength;
    this.#queue.push({ key, bytes });
    this.#drain();
  }

  activate(): void {
    this.#active = true;
    this.#drain();
  }

  /** Cumulative receipts must identify an actual frame boundary. Duplicates
   * are harmless; invented, fractional or mismatched credits close the route. */
  credit(frames: number, bytes: number): void {
    if (this.#closed || this.delivery !== "reliable") return;
    if (frames === this.#ackedFrames && bytes === this.#ackedBytes) return;
    const end = this.#receipts.findIndex((r) => r.frames === frames && r.bytes === bytes);
    if (end < 0) {
      this.#failure("invalid_credit");
      return;
    }
    for (const receipt of this.#receipts.splice(0, end + 1)) this.#release(receipt.size);
    this.#ackedFrames = frames;
    this.#ackedBytes = bytes;
  }

  #failure(code: string): void {
    if (this.#closed) return;
    this.dispose();
    this.fail(code);
  }

  #drain(): void {
    if (!this.#active || this.#closed) return;
    if (this.delivery === "latest") {
      let next = Infinity;
      for (let index = 0; index < this.#queue.length;) {
        const frame = this.#queue[index];
        if ([...this.#latest].some((f) => f.key === frame.key && !f.accepted)) {
          index++;
          continue;
        }
        const due = this.#nextDue.get(frame.key) ?? 0;
        if (due > this.now()) {
          next = Math.min(next, due);
          index++;
          continue;
        }
        this.#queue.splice(index, 1);
        if (Number.isFinite(this.maxHz)) {
          this.#nextDue.set(frame.key, this.now() + 1000 / this.maxHz);
        }
        const send = this.sink.sendFrame(frame.bytes);
        const pending: LatestSend = {
          send,
          size: frame.bytes.byteLength,
          key: frame.key,
          at: this.now(),
          accepted: false,
        };
        this.#latest.add(pending);
        send.done.then(() => {
          if (!this.#latest.has(pending)) return;
          pending.accepted = true;
          this.#drain();
        }).catch(() => {
          if (!send.aborted && !this.#closed) this.#failure("topic_send_failed");
        });
      }
      if (next < this.#timerAt) {
        if (this.#timer !== null) clearTimeout(this.#timer);
        this.#timerAt = next;
        this.#timer = setTimeout(() => {
          this.#timer = null;
          this.#timerAt = Infinity;
          this.#drain();
        }, Math.max(1, Math.ceil(next - this.now())));
      }
      return;
    }
    if (this.#draining) return;
    this.#draining = true;
    (async () => {
      // Frames parsed from one provider read arrive in the same JS turn.
      // Coalesce that burst before crossing the WebTransport writer boundary.
      await Promise.resolve();
      if (this.#closed) return;
      if (this.#writer === null) {
        const writer = await this.sink.openStream();
        if (this.#closed) {
          await writer.abort();
          return;
        }
        this.#writer = writer;
      }
      while (!this.#closed && this.#queue.length) {
        const batch: Uint8Array[] = [];
        let batchBytes = 0;
        do {
          const frame = this.#queue.shift()!;
          batch.push(frame.bytes);
          batchBytes += frame.bytes.byteLength;
          this.#sentFrames++;
          this.#sentBytes += frame.bytes.byteLength;
          this.#receipts.push({
            frames: this.#sentFrames,
            bytes: this.#sentBytes,
            size: frame.bytes.byteLength,
          });
        } while (this.#queue.length && batchBytes + this.#queue[0].bytes.byteLength <= 64 * 1024);
        await this.#writer.write(batch.length === 1 ? batch[0] : concatBytes(batch, batchBytes));
      }
    })().catch(() => this.#failure("topic_send_failed")).finally(() => {
      this.#draining = false;
      // An offer can land between the async loop's last condition and this
      // continuation. Do not strand the final reliable event in that gap.
      if (this.#queue.length) this.#drain();
    });
  }

  reap(nowMs: number): void {
    for (const frame of this.#latest) {
      if (nowMs - frame.at >= LATEST_LIFETIME_MS) this.#abortLatest(frame);
    }
    if (this.delivery === "latest") this.#drain();
  }

  dispose(): void {
    if (this.#closed) return;
    this.#closed = true;
    if (this.#timer !== null) clearTimeout(this.#timer);
    this.#timer = null;
    this.#nextDue.clear();
    for (const budget of this.budgets) budget.bytes -= this.#bytes;
    this.#bytes = 0;
    this.#frames = 0;
    this.#queue = [];
    this.#receipts = [];
    for (const frame of this.#latest) frame.send.abort();
    this.#latest.clear();
    this.#writer?.abort().catch(() => {});
    this.#writer = null;
  }
}
