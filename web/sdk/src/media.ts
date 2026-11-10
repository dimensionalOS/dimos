import { MAX_TOPIC_KEYS, MAX_TOPIC_PAYLOAD_BYTES } from "@dimos/shared";
import { decodeTopicMedia, type TopicMedia } from "./decoders/topicMedia.ts";
import type { TopicSample } from "./topics.ts";

type Decode = (sample: TopicSample) => Promise<TopicMedia>;
const MAX_PENDING_BYTES = 16 * 1024 * 1024;
const MAX_TOTAL_BYTES = 64 * 1024 * 1024;
const MAX_WAITING = 128;

// Shared by every media consumer in this SDK instance. One queued drain per
// handle, two active decodes total; a frame never creates an executor backlog.
const waiting = new Set<TopicMediaDecoder>();
let active = 0;
let retainedBytes = 0;
function pump(): void {
  while (active < 2 && waiting.size) {
    const decoder = waiting.values().next().value!;
    waiting.delete(decoder);
    active++;
    void decoder.drain().finally(() => {
      active--;
      pump();
    });
  }
}

/** Latest-per-key media decoding outside React/UI snapshots. Feed it raw topic
 * callbacks; reset it on subscription interruptions/updates, and close it with
 * the subscription. Replacement discards old work before decoding; in-flight
 * results from a previous generation or a closed consumer are never delivered.
 * onMedia owns any renderer resources it subsequently creates (e.g. bitmaps).
 */
export class TopicMediaDecoder {
  #pending = new Map<string, TopicSample>();
  #bytes = 0;
  #epoch = 0;
  #closed = false;
  #busy = false;
  #overBudget = false;
  dropped = 0;

  constructor(
    readonly onMedia: (media: TopicMedia, sample: TopicSample) => void,
    readonly onError: (error: Error) => void = () => {},
    readonly decode: Decode = decodeTopicMedia,
  ) {}

  get pendingBytes(): number {
    return this.#bytes;
  }

  #error(message: string): void {
    try {
      this.onError(new Error(message));
    } catch { /* Consumer error isolation. */ }
  }

  #release(sample: TopicSample): void {
    this.#bytes -= sample.payload.byteLength;
    retainedBytes -= sample.payload.byteLength;
  }

  push(sample: TopicSample): void {
    if (this.#closed) return;
    const previous = this.#pending.get(sample.key);
    const released = previous?.payload.byteLength ?? 0;
    const size = sample.payload.byteLength;
    if (
      size > MAX_TOPIC_PAYLOAD_BYTES || this.#bytes + size - released > MAX_PENDING_BYTES ||
      retainedBytes + size - released > MAX_TOTAL_BYTES ||
      (!previous && this.#pending.size >= MAX_TOPIC_KEYS) ||
      (!this.#busy && !waiting.has(this) && waiting.size >= MAX_WAITING)
    ) {
      this.dropped++;
      if (!this.#overBudget) this.#error("media decode budget exceeded");
      this.#overBudget = true;
      return;
    }
    this.#overBudget = false;
    if (previous) {
      this.#release(previous);
      this.dropped++;
    }
    this.#pending.set(sample.key, sample);
    this.#bytes += size;
    retainedBytes += size;
    if (!this.#busy) {
      waiting.add(this);
      pump();
    }
  }

  /** Invalidate pending/in-flight results, retaining an active worker's charge
   * until it really completes. A reconnect cannot free memory still in use. */
  reset(): void {
    this.#epoch++;
    this.#overBudget = false;
    waiting.delete(this);
    for (const sample of this.#pending.values()) this.#release(sample);
    this.#pending.clear();
  }

  close(): void {
    this.#closed = true;
    this.reset();
  }

  /** @internal Called only by the shared scheduler. */
  async drain(): Promise<void> {
    const sample = this.#pending.values().next().value;
    if (!sample) return;
    this.#pending.delete(sample.key);
    this.#busy = true;
    const epoch = this.#epoch;
    try {
      const result = await this.decode(sample);
      // Present completed work even when another sample arrived during decode;
      // otherwise a producer faster than the decoder can starve the display.
      if (!this.#closed && this.#epoch === epoch) {
        this.onMedia(result, sample);
      } else this.dropped++;
    } catch (error) {
      if (!this.#closed && this.#epoch === epoch) this.#error(String(error));
    } finally {
      this.#release(sample);
      this.#busy = false;
      if (this.#pending.size && !this.#closed) waiting.add(this);
    }
  }
}
