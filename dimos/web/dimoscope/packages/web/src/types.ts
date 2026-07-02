// Public types for @dimos/web.

/** Metadata delivered alongside every decoded message. */
export interface MessageMeta {
  topic: string;
  type: string;
  /** Client receive time (ms since epoch). */
  recvTs: number;
  /** Source timestamp (ms) parsed from a std_msgs/Header, if present (feeds the bench's
   *  end-to-end latency mode). */
  srcTs?: number;
  /** The gateway→browser transport hop: recvTs - gatewaySendMs (same-host meaningful). */
  latencyMs?: number;
  /** Encoded payload size in bytes. */
  sizeBytes: number;
  /** Per-topic source sequence number, parsed from a numeric `frame_id` (or
   *  `header.seq`) when present — lets a consumer detect wire drops/gaps. The
   *  bench source stamps `frame_id = str(seq)`; normal named-frame topics
   *  (e.g. "base_link") yield `undefined`. */
  seq?: number;
}

/** The one message envelope delivered to every subscriber (topic().subscribe, client.subscribe,
 *  subscribeAll, peek, and the react hooks). `ts` is the client receive time (ms); `meta` carries
 *  type/latency/seq/size (and the source stamp as `meta.srcTs`). */
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
  /** Most recent latency sample (ms), if a source stamp was available. */
  lastLatencyMs?: number;
  /** Total messages seen. */
  count: number;
}

/** Discovery snapshot of one topic — the immutable `{topic, type}` the SDK learns off the bus.
 *  Distinct from the stateful `Topic<T>` handle (topic.ts). The blueprint codegen
 *  (`packages/web/scripts/gen_types.py`) turns these pairs into the typed `DimosTopics` map. */
export interface TopicInfo {
  topic: string;
  type: string;
}

/** Per-subscription Quality-of-Service — a request the server on the transport's path enforces;
 *  the client's control is what it asks for. Sent verbatim in the subscribe op; servers ignore
 *  fields they don't honor (`caps.qos` declares which). */
export interface Qos {
  /** Downsample to at most this many Hz at the gateway — bytes leave the wire (0/undefined = full rate). */
  maxHz?: number;
  /** Scheduler priority band — the gateway drains higher first and sheds lower first under
   *  contention (the per-client priority outbox in gateway/data.py). */
  priority?: "low" | "normal" | "high" | "critical";
  /** Delivery guarantee: "reliable" keeps a bounded keep_last deque; "best-effort" conflates to
   *  the latest frame (the gateway sheds it first under load). */
  reliability?: "reliable" | "best-effort";
  /** Outbox depth for the gateway's keep_last deque (per topic). best-effort lanes use 1. */
  depth?: number;
}

/** A `Qos` field a server can honor — by design that's every `Qos` field ("maxHz" ⇒ downsampled
 *  on the wire; the rest ⇒ gateway-scheduler inputs). */
export type ServerQosField = keyof Qos;

// Transport implementations live in ./transports/ (default gatewayWs), with experimental ones under
// ./transports/experimental/. The client is transport-agnostic.

export type Status = "connecting" | "open" | "closed";

/** A raw, undecoded message off the bus. `payload` starts with the 8-byte type hash. */
export interface RawSample {
  topic: string;
  type: string;
  payload: Uint8Array;
  recvTs: number;
  /** Gateway send time (ms) from the frame prefix — for true transport latency. */
  gatewaySendMs?: number;
  /** Pre-decoded message: a test-injection seam (unit tests deliver a message without a real
   *  @dimos/msgs frame). No production transport sets this; the client decodes `payload` itself. */
  decoded?: unknown;
}

export interface TransportCaps {
  /** Does unsubscribe actually stop bytes flowing (vs client-side filtering)? */
  onDemand: boolean;
  discovery: "live" | "wildcard" | "passive";
  /** `Qos` fields the server on this path honors (declarative — drives UI affordances).
   *  undefined ⇒ no server on the path: qos is a no-op there. */
  qos?: ReadonlyArray<ServerQosField>;
}

/** One clock-sync round-trip to the gateway. `offsetMs` estimates (gateway clock − client clock)
 *  NTP-style — `serverTs − (tSend + rtt/2)` — so a cross-machine `recvTs − srcTs` latency can be
 *  corrected: true ≈ measured − offsetMs (gateway and source share a clock when co-located). */
export interface ClockSample {
  rttMs: number;
  offsetMs: number;
}

/** A dimos `@rpc` command the gateway advertises as browser-callable (from its hello). */
export interface CommandInfo {
  target: string; // module class, e.g. "GO2Connection"
  method: string; // @rpc method, e.g. "standup"
  label: string; // human label for a button
}

export interface Transport {
  connect(): Promise<void>;
  close(): void;
  /** Subscribe to `topic`, optionally declaring per-subscription QoS. The gateway honors the fields its
   *  `caps.qos` advertises (maxHz/priority/reliability/depth) and ignores the rest. Re-subscribing with a
   *  new `qos` updates it (this is how `setQos` propagates) — no separate op needed. */
  subscribe(topic: string, qos?: Qos): void;
  unsubscribe(topic: string): void;
  publishTeleop(linearX: number, angularZ: number, ttlMs?: number): void;
  /** Send a navigation goal (world metres) — gateway publishes a PointStamped to clicked_point. */
  publishGoal(x: number, y: number, z?: number): void;
  /** Invoke a whitelisted dimos `@rpc` command via the gateway; resolves with its return value.
   *  Stays string-keyed at the transport layer (any blueprint, any provider). Consumers get typed
   *  target+method from the generated `DimosCommands` / `RpcTarget` /
   *  `RpcMethod` (packages/web/scripts/gen_types.py), checked against live discovery; full arg/return typing follows
   *  when the static Python `@rpc`-annotation introspection pass lands. */
  rpc(target: string, method: string, args?: unknown[]): Promise<unknown>;
  /** Clock-sync probe: one `{op:"ping"}` round-trip echoing the gateway clock. Optional — only the
   *  duplex control paths (ws / WebTransport) implement it; read-only mechanisms leave it undefined. */
  ping?(): Promise<ClockSample>;
  requestList(): void;
  onSample(cb: (s: RawSample) => void): void;
  onTopics(cb: (topics: TopicInfo[]) => void): void;
  onStatus(cb: (s: Status) => void): void;
  /** Commands the gateway advertised as browser-callable (empty / undefined if none). */
  readonly commands?: CommandInfo[];
  onCommands?(cb: (commands: CommandInfo[]) => void): void;
  readonly caps: TransportCaps;
  /** Human label the gateway reports for itself (e.g. "Bun↔LCM", "Python↔Zenoh"). */
  label?: string;
}

// Media contract: pluggable opaque-video delivery (jpeg floor / webrtc / webcodecs), chosen by capability
// negotiation (selectMediaChannel, media.ts). Unlike Transport it carries no decodable RawSamples.

export type MediaKind = "webcodecs" | "webrtc" | "jpeg";

export interface VideoMeta {
  width: number;
  height: number;
  fps: number;
  codec: string;
}

export interface MediaCaps {
  /** How the app renders: a GPU-composited MediaStream (<video>) vs decoded frames (canvas). */
  output: "stream" | "frames";
  /** Wire codec; also identifies the jpeg floor for the runtime fallback. */
  codec: "h264" | "vp8" | "av1" | "jpeg";
}

/** streamId is the camera topic, e.g. "/dimos/color_image". */
export interface MediaChannel {
  connect(): Promise<void>;
  close(): void;
  subscribe(streamId: string): void;
  unsubscribe(streamId: string): void;
  // An impl fires exactly one of these, per caps.output.
  onStream(cb: (streamId: string, stream: MediaStream) => void): void; // "stream"
  onFrame(cb: (streamId: string, frame: VideoFrame | ImageBitmap, m: VideoMeta) => void): void; // "frames"
  onStatus(cb: (s: Status) => void): void;
  readonly caps: MediaCaps;
  label?: string;
}
