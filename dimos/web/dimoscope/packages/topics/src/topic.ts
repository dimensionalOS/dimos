// Topic<T> — a typed handle to one DimOS topic. Manages subscribers, the latest
// value, rate-limiting (backpressure), and live stats. On-demand: it tells the
// transport to subscribe only while it has ≥1 subscriber, and unsubscribe at 0.
import type { Handler, MessageMeta, Subscription, TopicStats } from "./types";

export interface TopicWiring {
  subscribe(topic: string, maxHz?: number): void;
  unsubscribe(topic: string): void;
}

export class Topic<T = unknown> {
  private handlers = new Set<Handler<T>>();
  private latest?: T;
  private paused = false;
  private maxHz = 0;
  private lastDeliver = 0;
  private dropped = 0;
  private count = 0;
  private lastLatencyMs?: number;
  private recent: number[] = [];
  private recentBytes: number[] = [];

  constructor(
    public readonly name: string,
    public type: string,
    private wiring: TopicWiring,
  ) {}

  /** Subscribe to every message (subject to rate-limit). Returns an unsubscribe handle. */
  subscribe(handler: Handler<T>): Subscription {
    this.handlers.add(handler);
    if (this.handlers.size === 1) this.wiring.subscribe(this.name, this.maxHz || undefined);
    return {
      unsubscribe: () => {
        this.handlers.delete(handler);
        if (this.handlers.size === 0) this.wiring.unsubscribe(this.name);
      },
    };
  }

  /** Same wire behavior — delivery is always newest-first (we never queue). */
  subscribeLatest(handler: Handler<T>): Subscription {
    return this.subscribe(handler);
  }

  getLatest(): T | undefined {
    return this.latest;
  }

  /** Cap delivery to `hz` (also asks the gateway to downsample → saves bandwidth). */
  setRateLimit(hz: number) {
    this.maxHz = hz;
    if (this.handlers.size > 0) this.wiring.subscribe(this.name, hz || undefined);
  }

  pause() {
    this.paused = true;
  }
  resume() {
    this.paused = false;
  }

  stats(): TopicStats {
    const now = Date.now();
    let i = 0;
    while (i < this.recent.length && now - this.recent[i] > 1000) i++;
    const hz = this.recent.length - i;
    let bytes = 0;
    for (let j = i; j < this.recentBytes.length; j++) bytes += this.recentBytes[j];
    return { hz, bytesPerSec: bytes, dropped: this.dropped, lastLatencyMs: this.lastLatencyMs, count: this.count };
  }

  /** @internal — the client calls this with each decoded message. */
  _deliver(data: T, meta: MessageMeta) {
    this.count++;
    this.latest = data;
    this.lastLatencyMs = meta.latencyMs;
    const now = meta.recvTs;
    this.recent.push(now);
    this.recentBytes.push(meta.sizeBytes);
    if (this.recent.length > 256) {
      this.recent.shift();
      this.recentBytes.shift();
    }
    if (this.paused) return;
    if (this.maxHz > 0 && now - this.lastDeliver < 1000 / this.maxHz) {
      this.dropped++;
      return;
    }
    this.lastDeliver = now;
    const m = { ...meta, dropped: this.dropped };
    this.handlers.forEach((h) => h(data, m));
  }
}
