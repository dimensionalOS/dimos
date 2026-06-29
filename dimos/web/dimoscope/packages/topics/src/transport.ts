// The transport abstraction — mirrors DimOS's Python Transport layer.
// Implementations: GatewayWsTransport (Bun/LCM or Python/Zenoh gateway, primary),
// and (future) a direct zenoh-ts adapter. The client is transport-agnostic.
import type { TopicInfo } from "./types";

export type Status = "connecting" | "open" | "closed";

/** A raw, undecoded message off the bus. `payload` starts with the 8-byte type hash. */
export interface RawSample {
  topic: string;
  type: string;
  payload: Uint8Array;
  recvTs: number;
  /** Gateway send time (ms) from the frame prefix — for true transport latency. */
  gatewaySendMs?: number;
}

export interface TransportCaps {
  /** Does unsubscribe actually stop bytes flowing (vs client-side filtering)? */
  onDemand: boolean;
  discovery: "live" | "wildcard" | "passive";
}

export interface Transport {
  connect(): Promise<void>;
  close(): void;
  subscribe(topic: string, maxHz?: number): void;
  unsubscribe(topic: string): void;
  publishTeleop(linearX: number, angularZ: number, ttlMs?: number): void;
  /** Send a navigation goal (world metres) — gateway publishes a PointStamped to clicked_point. */
  publishGoal(x: number, y: number, z?: number): void;
  requestList(): void;
  onSample(cb: (s: RawSample) => void): void;
  onTopics(cb: (topics: TopicInfo[]) => void): void;
  onStatus(cb: (s: Status) => void): void;
  readonly caps: TransportCaps;
  /** Human label the gateway reports for itself (e.g. "Bun↔LCM", "Python↔Zenoh"). */
  label?: string;
}
