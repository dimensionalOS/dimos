// The transport abstraction — mirrors DimOS's Python Transport layer.
// Implementations: GatewayWsTransport (Bun/LCM or Python/Zenoh gateway, primary),
// and (future) a direct zenoh-ts adapter. The client is transport-agnostic.
import type { QosCaps, TopicInfo } from "./types.ts";

export type Status = "connecting" | "open" | "closed";

/** A raw, undecoded message off the bus. `payload` starts with the 8-byte type hash. */
export interface RawSample {
  topic: string;
  type: string;
  payload: Uint8Array;
  recvTs: number;
  /** Gateway send time (ms) from the frame prefix — for true transport latency. */
  gatewaySendMs?: number;
  /** Pre-decoded message (server-side decode mode): when set, the client uses it directly
   *  instead of decoding `payload` itself — the "where to decode" axis (rosbridge-style). */
  decoded?: unknown;
}

export interface TransportCaps {
  /** Does unsubscribe actually stop bytes flowing (vs client-side filtering)? */
  onDemand: boolean;
  discovery: "live" | "wildcard" | "passive";
  /** Which QoS knobs this transport honors (undefined → client-side maxHz only). */
  qos?: QosCaps;
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
  subscribe(topic: string, maxHz?: number): void;
  unsubscribe(topic: string): void;
  publishTeleop(linearX: number, angularZ: number, ttlMs?: number): void;
  /** Send a navigation goal (world metres) — gateway publishes a PointStamped to clicked_point. */
  publishGoal(x: number, y: number, z?: number): void;
  /** Invoke a whitelisted dimos `@rpc` command via the gateway; resolves with its return value. */
  rpc(target: string, method: string, args?: unknown[]): Promise<unknown>;
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
