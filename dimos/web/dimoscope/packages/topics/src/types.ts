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
