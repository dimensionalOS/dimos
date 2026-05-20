// LCM Type Definitions (vendored from @dimos/lcm@0.2.0)

/** LCM message as received */
export interface LCMMessage<T = Uint8Array> {
  channel: string;
  data: T;
  timestamp: number;
}

/** Interface for LCM message classes (generated types) */
export interface MessageClass<T> {
  readonly _HASH: bigint;
  readonly _NAME: string;
  decode(data: Uint8Array): T;
  new (init?: Partial<T>): T & { encode(): Uint8Array };
}

/** Subscription handler function */
export type SubscriptionHandler<T = Uint8Array> = (msg: LCMMessage<T>) => void;

/** Packet handler function (for raw UDP packets) */
export type PacketHandler = (packet: Uint8Array) => void;

/** LCM configuration options */
export interface LCMOptions {
  /** LCM URL (e.g., "udpm://239.255.76.67:7667?ttl=1") */
  url?: string;
  /** Multicast TTL (time-to-live) */
  ttl?: number;
  /** Network interface to bind to */
  iface?: string;
}

/** Parsed LCM URL */
export interface ParsedUrl {
  scheme: string;
  host: string;
  port: number;
  ttl: number;
  iface?: string;
}

/** Internal subscription record */
export interface Subscription {
  channel: string;
  pattern: RegExp;
  handler: SubscriptionHandler<unknown>;
  msgClass?: MessageClass<unknown>;
}

/** Internal packet subscription record */
export interface PacketSubscription {
  pattern: RegExp | null; // null = match all
  handler: PacketHandler;
}

// LCM Protocol constants
export const MAGIC_SHORT = 0x4c433032; // "LC02"
export const MAGIC_LONG = 0x4c433033; // "LC03"
export const MAX_SMALL_MESSAGE = 65535;
export const SHORT_HEADER_SIZE = 8;
export const FRAGMENT_HEADER_SIZE = 20;
