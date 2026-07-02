// Topic<T> — a typed handle to one DimOS topic. Manages subscribers, the latest
// value, and live stats. On-demand: it tells the transport to subscribe only
// while it has ≥1 subscriber, and unsubscribe at 0. Delivery is verbatim —
// rate shaping is a server concern, requested via QoS (setQos → the wire).
import type { Handler, Message, MessageMeta, Qos, Subscription, TopicStats } from "./types.ts";

export interface TopicWiring {
  subscribe(topic: string, qos?: Qos): void;
  unsubscribe(topic: string): void;
}

export interface Topic<T = unknown> {
  readonly name: string;
  type: string;
  /** Subscribe to every message the transport delivers. Returns an unsubscribe handle. */
  subscribe(handler: Handler<T>): Subscription;
  /** Same wire behavior — delivery is always newest-first (we never queue). */
  subscribeLatest(handler: Handler<T>): Subscription;
  getLatest(): T | undefined;
  /** Declare per-subscription QoS — a request to the path's server. Re-setting re-issues the
   *  wire subscribe with the new QoS. */
  setQos(qos: Qos): void;
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
  let qos: Qos = {};
  let count = 0;
  let lastLatencyMs: number | undefined;
  const recent: number[] = [];
  const recentBytes: number[] = [];

  function subscribe(handler: Handler<T>): Subscription {
    handlers.add(handler);
    if (handlers.size === 1) wiring.subscribe(name, qos);
    return {
      unsubscribe: () => {
        handlers.delete(handler);
        if (handlers.size === 0) wiring.unsubscribe(name);
      },
    };
  }

  function setQos(next: Qos) {
    qos = next;
    if (handlers.size > 0) wiring.subscribe(name, qos); // re-subscribe = propagate the new QoS
  }

  function stats(): TopicStats {
    const now = Date.now();
    let i = 0;
    while (i < recent.length && now - recent[i] > 1000) i++;
    const hz = recent.length - i;
    let bytes = 0;
    for (let j = i; j < recentBytes.length; j++) bytes += recentBytes[j];
    return { hz, bytesPerSec: bytes, lastLatencyMs, count };
  }

  function _deliver(data: T, meta: MessageMeta) {
    count++;
    latest = data;
    lastLatencyMs = meta.latencyMs;
    recent.push(meta.recvTs);
    recentBytes.push(meta.sizeBytes);
    if (recent.length > 256) {
      recent.shift();
      recentBytes.shift();
    }
    if (paused) return;
    const message: Message<T> = { data, ts: meta.recvTs, meta };
    handlers.forEach((h) => {
      try {
        h(message);
      } catch (e) {
        console.error(`[@dimos/web] subscriber for ${name} threw`, e);
      }
    });
  }

  return {
    name,
    type: deps.type,
    subscribe,
    subscribeLatest: subscribe,
    getLatest: () => latest,
    setQos,
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
