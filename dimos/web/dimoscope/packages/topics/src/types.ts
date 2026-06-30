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

export type Handler<T> = (data: T, meta: MessageMeta) => void;

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

export interface TopicInfo {
  topic: string;
  type: string;
}

/**
 * Per-subscription Quality-of-Service. Two tiers:
 *
 *  • CLIENT-SIDE, TRANSPORT-AGNOSTIC (implemented) — `maxHz` + `rateLimit` location +
 *    `conflation`. These work identically on every transport (they live in topic.ts).
 *  • TRANSPORT-LEVEL (declared, not yet wired) — `reliability`/`priority`/`congestion`/
 *    `express` (zenoh) and `ordered`/`maxRetransmits`/`maxPacketLifetimeMs` (WebRTC
 *    DataChannel). Honored only where the active transport's `caps.qos` advertises it;
 *    elsewhere they're ignored (the UI greys them out). Wiring these through the Python
 *    gateways is the documented next phase — see docs/data-path.md.
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
  // ── transport-level (declared; honored per caps.qos) ──────────────────────────
  /** Delivery guarantee (DDS/ROS 2): "reliable" retries + keeps order; "best-effort" drops under
   *  pressure (the gateway sheds it first). zenoh subscriber / WebRTC channel reliability. */
  reliability?: "reliable" | "best-effort";
  /** Late-joiner behavior (DDS durability): "transient_local" = the gateway replays the last value to a
   *  new subscriber (latch); "volatile" = nothing persisted. */
  durability?: "volatile" | "transient_local";
  /** Buffering discipline (DDS history): "keep_last" bounds the outbox to `depth`; "keep_all" keeps all. */
  history?: "keep_last" | "keep_all";
  /** Outbox depth for "keep_last" (per topic). best-effort lanes use 1 (latest-wins / conflation). */
  depth?: number;
  /** Scheduler priority band — the gateway drains higher first and sheds lower first under contention
   *  (the per-client priority outbox in servers/data.py). Also a zenoh priority band. */
  priority?: "low" | "normal" | "high" | "critical";
  /** zenoh congestion behavior: drop messages vs block the pipe. */
  congestion?: "drop" | "block";
  /** zenoh express (batching off → lower latency, more overhead). */
  express?: boolean;
  /** WebRTC DataChannel ordered delivery (set at channel creation). */
  ordered?: boolean;
  /** WebRTC DataChannel max retransmits (set at channel creation). */
  maxRetransmits?: number;
  /** WebRTC DataChannel max packet lifetime, ms (set at channel creation). */
  maxPacketLifetimeMs?: number;
}

/** Which QoS fields a transport actually honors (the rest are ignored / client-emulated). */
export interface QosCaps {
  /** Where a `maxHz` request takes effect: "server" downsamples on the wire,
   *  "client" only throttles delivery locally. */
  maxHz: "server" | "client";
  /** Transport-level QoS fields this transport can honor (none yet wired). */
  transport?: ReadonlyArray<keyof Qos>;
}
