// Public types for @dimos/topics.

/** Metadata delivered alongside every decoded message. */
export interface MessageMeta {
  topic: string;
  type: string;
  /** Client receive time (ms since epoch). */
  recvTs: number;
  /** Source timestamp (ms) parsed from a std_msgs/Header, if present. */
  srcTs?: number;
  /** recvTs - srcTs, when a source stamp is available (same-host meaningful). */
  latencyMs?: number;
  /** Encoded payload size in bytes. */
  sizeBytes: number;
  /** Cumulative messages coalesced/dropped for this topic by client rate-limit. */
  dropped: number;
  /** Per-topic source sequence number, parsed from a numeric `frame_id` (or
   *  `header.seq`) when present — lets a consumer detect wire drops/gaps. The
   *  bench source stamps `frame_id = str(seq)`; normal named-frame topics
   *  (e.g. "base_link") yield `undefined`. */
  seq?: number;
}

/** The one message envelope delivered to every subscriber (topic().subscribe, client.subscribe,
 *  subscribeAll, peek, and the react hooks). `ts` is the source publish time (ms), falling back to
 *  the client receive time; `meta` carries type/latency/seq/size. */
export interface Message<T = unknown> {
  data: T;
  ts: number;
  meta: MessageMeta;
}

export type Handler<T> = (message: Message<T>) => void;

export interface Subscription {
  unsubscribe(): void;
}

export interface TopicStats {
  /** Messages received in the last ~1s. */
  hz: number;
  /** Bytes received in the last ~1s. */
  bytesPerSec: number;
  /** Cumulative client-side coalesced messages. */
  dropped: number;
  /** Most recent latency sample (ms), if a source stamp was available. */
  lastLatencyMs?: number;
  /** Total messages seen. */
  count: number;
}

/** A discovery snapshot of one topic — the immutable `{topic, type}` metadata the SDK learns off
 *  the bus. Distinct from `Topic<T>` (topic.ts), which is the stateful subscription handle
 *  (handlers, rate-limit, stats) created on demand by `client.topic<T>(name)`. `client.topic()`
 *  itself stays string-keyed; strong per-message typing is opt-in — the blueprint codegen
 *  (`cli/genTypes.ts`) turns these pairs into a `DimosTopics` map a consumer annotates against
 *  manually (`const p: DimosTopics["/odom"] = client.topic("/odom").getLatest()!`). */
export interface TopicInfo {
  topic: string;
  type: string;
}

/**
 * Per-subscription Quality-of-Service. Two tiers, both actually honored end-to-end:
 *
 *  • CLIENT-SIDE, TRANSPORT-AGNOSTIC — `maxHz` + `rateLimit` location + `conflation`. These work
 *    identically on every transport (they live in topic.ts).
 *  • GATEWAY-SCHEDULER — `priority` / `reliability` / `depth`. The gateway-WS adapter forwards these
 *    in its subscribe op and the Python gateway's per-client priority outbox honors them
 *    (gateway/qos.py `declared_to_class`), so important topics survive a saturated link.
 *    Transports without that server outbox (zenoh-ts, webrtc, sse, http-poll) advertise no
 *    `caps.qos.transport`, so `applyCaps` (qos.ts) strips these before they'd be sent.
 *
 * Per-provider QoS knobs that no transport honors yet (zenoh congestion/express, WebRTC
 * ordered/maxRetransmits — the latter set per-channel via the adapter's deps, not per-subscription)
 * are intentionally NOT modeled here; add them when a transport actually reads them.
 */
export interface Qos {
  /** Cap delivery to at most this many Hz (0/undefined = unlimited). */
  maxHz?: number;
  /**
   * WHERE `maxHz` is enforced:
   *  • "server" (default) — ask the gateway to downsample → fewer bytes on the wire.
   *  • "client" — gateway keeps sending; the client drops locally → bytes unchanged,
   *    only delivery/CPU is capped. (zenoh-ts/webrtc are always client-side: no
   *    per-subscriber server downsample — see each adapter's `caps.qos`.)
   */
  rateLimit?: "server" | "client";
  /**
   * Delivery discipline. The SDK never queues (delivery is synchronous, newest-wins),
   * so this is a label over `maxHz`, not a buffer: "latest" = today's default (drop
   * intermediates under a rate cap), "all" = deliver every message (forces maxHz=0).
   */
  conflation?: "latest" | "all";
  // ── gateway-scheduler (honored where caps.qos.transport advertises them) ───────
  /** Scheduler priority band — the gateway drains higher first and sheds lower first under
   *  contention (the per-client priority outbox in gateway/data.py). */
  priority?: "low" | "normal" | "high" | "critical";
  /** Delivery guarantee: "reliable" keeps a bounded keep_last deque; "best-effort" conflates to
   *  the latest frame (the gateway sheds it first under load). */
  reliability?: "reliable" | "best-effort";
  /** Outbox depth for the gateway's keep_last deque (per topic). best-effort lanes use 1. */
  depth?: number;
}

/** Which QoS fields a transport actually honors (the rest are ignored / client-emulated). */
export interface QosCaps {
  /** Where a `maxHz` request takes effect: "server" downsamples on the wire,
   *  "client" only throttles delivery locally. */
  maxHz: "server" | "client";
  /** Gateway-scheduler QoS fields this transport forwards + the server honors (the gateway-WS
   *  adapter advertises ["priority","reliability","depth"]; client-only transports advertise none). */
  transport?: ReadonlyArray<keyof Qos>;
}
