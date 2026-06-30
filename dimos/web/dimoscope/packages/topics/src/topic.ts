// Topic<T> — a typed handle to one DimOS topic. Manages subscribers, the latest
// value, rate-limiting (backpressure), and live stats. On-demand: it tells the
// transport to subscribe only while it has ≥1 subscriber, and unsubscribe at 0.
import type { Handler, MessageMeta, Subscription, TopicStats } from "./types.ts";

export interface TopicWiring {
  subscribe(topic: string, maxHz?: number): void;
  unsubscribe(topic: string): void;
}

export interface Topic<T = unknown> {
  readonly name: string;
  type: string;
  /** Subscribe to every message (subject to rate-limit). Returns an unsubscribe handle. */
  subscribe(handler: Handler<T>): Subscription;
  /** Same wire behavior — delivery is always newest-first (we never queue). */
  subscribeLatest(handler: Handler<T>): Subscription;
  getLatest(): T | undefined;
  /** Cap delivery to `hz` (also asks the gateway to downsample → saves bandwidth). */
  setRateLimit(hz: number): void;
  pause(): void;
  resume(): void;
  stats(): TopicStats;
  /** @internal — the client calls this with each decoded message. */
  _deliver(data: T, meta: MessageMeta): void;
}

export interface TopicDeps {
  name: string;
  type: string;
  wiring: TopicWiring;
}

export const createTopic = <T = unknown>(deps: TopicDeps): Topic<T> => {
  const { name, wiring } = deps;
  const handlers = new Set<Handler<T>>();
  let latest: T | undefined;
  let paused = false;
  let maxHz = 0;
  let lastDeliver = 0;
  let dropped = 0;
  let count = 0;
  let lastLatencyMs: number | undefined;
  const recent: number[] = [];
  const recentBytes: number[] = [];

  function subscribe(handler: Handler<T>): Subscription {
    handlers.add(handler);
    if (handlers.size === 1) wiring.subscribe(name, maxHz || undefined);
    return {
      unsubscribe: () => {
        handlers.delete(handler);
        if (handlers.size === 0) wiring.unsubscribe(name);
      },
    };
  }

  function setRateLimit(hz: number) {
    maxHz = hz;
    if (handlers.size > 0) wiring.subscribe(name, hz || undefined);
  }

  function stats(): TopicStats {
    const now = Date.now();
    let i = 0;
    while (i < recent.length && now - recent[i] > 1000) i++;
    const hz = recent.length - i;
    let bytes = 0;
    for (let j = i; j < recentBytes.length; j++) bytes += recentBytes[j];
    return { hz, bytesPerSec: bytes, dropped, lastLatencyMs, count };
  }

  function _deliver(data: T, meta: MessageMeta) {
    count++;
    latest = data;
    lastLatencyMs = meta.latencyMs;
    const now = meta.recvTs;
    recent.push(now);
    recentBytes.push(meta.sizeBytes);
    if (recent.length > 256) {
      recent.shift();
      recentBytes.shift();
    }
    if (paused) return;
    if (maxHz > 0 && now - lastDeliver < 1000 / maxHz) {
      dropped++;
      return;
    }
    lastDeliver = now;
    const m = { ...meta, dropped };
    handlers.forEach((h) => h(data, m));
  }

  return {
    name,
    type: deps.type,
    subscribe,
    subscribeLatest: subscribe,
    getLatest: () => latest,
    setRateLimit,
    pause: () => {
      paused = true;
    },
    resume: () => {
      paused = false;
    },
    stats,
    _deliver,
  };
};
