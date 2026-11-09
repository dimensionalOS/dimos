// Topic wire v1. These messages are separate from manifest channels and do
// not grant authority: the relay admits them only while it runs with open
// topics, and checks each operation against the provider's capabilities.
import type { Delivery } from "./manifest.ts";

export const TOPIC_VERSION = 1;
export const MAX_TOPIC_KEY_BYTES = 1024;
export const MAX_TOPIC_PAYLOAD_BYTES = 8 * 1024 * 1024;
export const MAX_TOPIC_HANDLES = 64;
/** Sources a relay serves at once: the bound of a topic_sources snapshot. */
export const MAX_TOPIC_SOURCES = 64;
export const MAX_TOPIC_ROUTES = 256;
export const MAX_TOPIC_KEYS = 128;
export const MAX_TOPIC_PAGE = 32;
export const MAX_TOPIC_PUBLISH_PENDING = 16;
export const MAX_TOPIC_PUBLISH_BYTES = 16 * 1024 * 1024;
export const TOPIC_ROUTE_PREFIX = "@t:";
export const TOPIC_PUBLISH_CHANNEL = "@put";
export const TOPIC_GUARD_CHANNEL = "@guard";
export const MAX_TOPIC_GUARD_BYTES = 8192;
export const MAX_TOPIC_GUARDS = 32;
/** Little-endian "DIMP": distinguishes a bulk bidi stream from length-prefixed
 * control JSON. Client-to-Deno uni stream payloads are broken in Deno 2.6.10. */
export const TOPIC_PUBLISH_STREAM_MAGIC = 0x504d4944;
/** "DIMG": independent command lane, so a bulk put cannot hold up stop/drive. */
export const TOPIC_GUARD_STREAM_MAGIC = 0x474d4944;

export interface TopicCapabilities {
  read: boolean;
  write: boolean;
  profiles: string[];
  /** Provider-owned stop/expiry. Older peers do not advertise this. */
  guardedPublishers?: boolean;
}
export interface TopicSourceInfo extends TopicCapabilities {
  source: string;
  gen: number;
}
/** The connected sources. A source that disconnects leaves the list and its
 * routes end with `source_offline`; the next list carries its new generation. */
export interface TopicSourcesMsg {
  t: "topic_sources";
  v: 1;
  sources: TopicSourceInfo[];
}
export interface TopicSubMsg {
  t: "topic_sub";
  id: string;
  source: string;
  keyExpr: string;
  delivery: Delivery;
  profile: string;
  maxHz: number;
}
export interface TopicOpenMsg {
  t: "topic_open";
  route: string;
  gen: number;
  keyExpr: string;
  delivery: Delivery;
  profile: string;
  maxHz: number;
}
export interface TopicOpenedMsg {
  t: "topic_opened";
  route: string;
  gen: number;
}
export interface TopicReadyMsg {
  t: "topic_ready";
  id: string;
  source: string;
  route: string;
  gen: number;
}
export interface TopicActivateMsg {
  t: "topic_activate";
  route: string;
  gen: number;
  /** Relay-to-provider only. */
  maxHz?: number;
}
export interface TopicCloseMsg {
  t: "topic_close";
  route: string;
  gen: number;
}
/** Cumulative browser receipts for reliable routes. A bounded pipeline uses
 * these to include QUIC-buffered bytes in its budget, without per-frame RTT waits. */
export interface TopicCreditMsg {
  t: "topic_credit";
  route: string;
  gen: number;
  frames: number;
  bytes: number;
}
export interface TopicUnsubMsg {
  t: "topic_unsub";
  id: string;
}
export interface TopicErrorMsg {
  t: "topic_error";
  id: string;
  code: string;
  message: string;
  outcome: "rejected" | "unknown" | "interrupted";
}
export interface TopicListMsg {
  t: "topic_list";
  id: string;
  source: string;
  prefix: string;
  cursor: string;
  limit: number;
}
export interface TopicDescriptor {
  key: string;
  encoding: string;
  type: string;
  availability: "known" | "observed" | "advertised";
}
export interface TopicPageMsg {
  t: "topic_page";
  id: string;
  source: string;
  gen: number;
  entries: TopicDescriptor[];
  cursor: string;
}
export interface TopicPubResultMsg {
  t: "topic_pub_result";
  id: string;
  outcome: "accepted" | "rejected" | "unknown";
  code: string;
  message: string;
  providerTs: number;
}
export interface TopicGuardReadyMsg {
  t: "topic_guard_ready";
  id: string;
  handle: string;
  gen: number;
  ticket: number;
  timeoutMs: number;
}
export interface TopicGuardHeartbeatMsg {
  t: "topic_guard_heartbeat";
  handle: string;
  gen: number;
  ticket: number;
}
export interface TopicGuardTicketMsg {
  t: "topic_guard_ticket";
  handle: string;
  gen: number;
  ticket: number;
}
export interface TopicGuardStopMsg {
  t: "topic_guard_stop";
  handle: string;
  gen: number;
}
export interface TopicGuardClosedMsg {
  t: "topic_guard_closed";
  handle: string;
  gen: number;
  code: string;
}
export interface TopicGuardResultMsg {
  t: "topic_guard_result";
  handle: string;
  gen: number;
  seq: number;
  outcome: "accepted" | "rejected" | "unknown";
  code: string;
  providerTs: number;
}
export type TopicGuardMsg =
  | TopicGuardReadyMsg
  | TopicGuardHeartbeatMsg
  | TopicGuardTicketMsg
  | TopicGuardStopMsg
  | TopicGuardClosedMsg
  | TopicGuardResultMsg;
export type TopicMsg =
  | TopicGuardMsg
  | TopicSourcesMsg
  | TopicSubMsg
  | TopicOpenMsg
  | TopicOpenedMsg
  | TopicReadyMsg
  | TopicActivateMsg
  | TopicCloseMsg
  | TopicCreditMsg
  | TopicUnsubMsg
  | TopicErrorMsg
  | TopicListMsg
  | TopicPageMsg
  | TopicPubResultMsg;

/** FrameHeader.meta for @t: routes. ts is send time; ingressTs and optional
 * sourceTs retain the actual acquisition/receive times in epoch seconds. */
export interface TopicSampleMeta extends Record<string, unknown> {
  key: string;
  encoding: string;
  gen: number;
  ingressTs: number;
  sourceTs?: number;
}
/** Binary @put frame metadata. The relay replaces id with a private receipt
 * token before forwarding; a source generation is required, never guessed. */
export interface TopicPublishMeta extends Record<string, unknown> {
  id: string;
  source: string;
  gen: number;
  key: string;
  encoding: string;
}
export interface TopicGuardOpenMeta extends TopicPublishMeta {
  action: "open";
  group: string;
  timeoutMs: number;
  takeover: boolean;
}
export interface TopicGuardCommandMeta extends Record<string, unknown> {
  action: "command";
  source: string;
  gen: number;
  handle: string;
  ticket: number;
}

const utf8 = new TextEncoder();
const record = (v: unknown): v is Record<string, unknown> =>
  typeof v === "object" && v !== null && !Array.isArray(v);
const string = (v: unknown, max: number, min = 1): v is string =>
  typeof v === "string" && v.length >= min && v.length <= max;
export const topicSourceOk = (v: unknown): v is string =>
  typeof v === "string" && /^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}(?![\s\S])/.test(v);
export const topicKeyOk = (v: unknown, empty = false): v is string =>
  string(v, MAX_TOPIC_KEY_BYTES, empty ? 0 : 1) && utf8.encode(v).length <= MAX_TOPIC_KEY_BYTES;
export const topicGenerationOk = (v: unknown): v is number =>
  typeof v === "number" && Number.isSafeInteger(v) && v >= 0;
export const topicRouteOk = (v: unknown): v is string =>
  typeof v === "string" && /^@t:[A-Za-z0-9_-]{1,48}(?![\s\S])/.test(v);
const id = (v: unknown) => string(v, 64);
const finite = (v: unknown) => typeof v === "number" && Number.isFinite(v);
const options = (v: Record<string, unknown>) =>
  topicKeyOk(v.keyExpr) && (v.delivery === "latest" || v.delivery === "reliable") &&
  string(v.profile, 64) && typeof v.maxHz === "number" && Number.isFinite(v.maxHz) &&
  v.maxHz > 0 && v.maxHz <= 1000;
const routed = (v: Record<string, unknown>) => topicRouteOk(v.route) && topicGenerationOk(v.gen);
export const topicCapabilitiesOk = (v: unknown): v is TopicCapabilities =>
  record(v) && typeof v.read === "boolean" && typeof v.write === "boolean" &&
  (v.guardedPublishers === undefined || typeof v.guardedPublishers === "boolean") &&
  Array.isArray(v.profiles) && v.profiles.length <= 16 && v.profiles.every((p) => string(p, 64));
const sourceInfo = (v: unknown) =>
  record(v) && topicSourceOk(v.source) && topicGenerationOk(v.gen) && topicCapabilitiesOk(v);
const descriptor = (v: unknown) =>
  record(v) && topicKeyOk(v.key) && string(v.encoding, 256, 0) && string(v.type, 256, 0) &&
  ["known", "observed", "advertised"].includes(v.availability as string);

export function topicSampleMeta(v: unknown): TopicSampleMeta | null {
  return record(v) && topicKeyOk(v.key) && string(v.encoding, 256, 0) &&
      topicGenerationOk(v.gen) && finite(v.ingressTs) &&
      (v.sourceTs === undefined || finite(v.sourceTs))
    ? v as TopicSampleMeta
    : null;
}
export function topicPublishMeta(v: unknown): TopicPublishMeta | null {
  return record(v) && id(v.id) && topicSourceOk(v.source) && topicGenerationOk(v.gen) &&
      topicKeyOk(v.key) && !v.key.includes("*") && string(v.encoding, 256, 0)
    ? v as TopicPublishMeta
    : null;
}
export const topicGuardHandleOk = (v: unknown): v is string =>
  typeof v === "string" && /^@p:[A-Za-z0-9_-]{1,48}(?![\s\S])/.test(v);
export const topicGuardTimeoutOk = (v: unknown): v is number =>
  typeof v === "number" && Number.isInteger(v) && v >= 100 && v <= 1000;
export function topicGuardMeta(v: unknown): TopicGuardOpenMeta | TopicGuardCommandMeta | null {
  if (!record(v)) return null;
  if (v.action === "open") {
    return topicPublishMeta(v) && string(v.group, 64) && topicGuardTimeoutOk(v.timeoutMs) &&
        typeof v.takeover === "boolean"
      ? v as TopicGuardOpenMeta
      : null;
  }
  return v.action === "command" && topicSourceOk(v.source) && topicGenerationOk(v.gen) &&
      topicGuardHandleOk(v.handle) && topicGenerationOk(v.ticket) && v.ticket > 0
    ? v as TopicGuardCommandMeta
    : null;
}

/** Shape/size validation only; Zenoh validates key-expression semantics at
 * the provider. No approximate glob or imports derived from type names. */
export function topicMsgFromUnknown(v: Record<string, unknown>): TopicMsg | null {
  let ok = false;
  switch (v.t) {
    case "topic_guard_ready":
      ok = id(v.id) && topicGuardHandleOk(v.handle) && topicGenerationOk(v.gen) &&
        topicGenerationOk(v.ticket) && v.ticket > 0 && topicGuardTimeoutOk(v.timeoutMs);
      break;
    case "topic_guard_heartbeat":
    case "topic_guard_ticket":
      ok = topicGuardHandleOk(v.handle) && topicGenerationOk(v.gen) &&
        topicGenerationOk(v.ticket) && v.ticket > 0;
      break;
    case "topic_guard_stop":
      ok = topicGuardHandleOk(v.handle) && topicGenerationOk(v.gen);
      break;
    case "topic_guard_closed":
      ok = topicGuardHandleOk(v.handle) && topicGenerationOk(v.gen) && string(v.code, 64);
      break;
    case "topic_guard_result":
      ok = topicGuardHandleOk(v.handle) && topicGenerationOk(v.gen) &&
        topicGenerationOk(v.seq) && v.seq > 0 &&
        ["accepted", "rejected", "unknown"].includes(v.outcome as string) &&
        string(v.code, 64, 0) && finite(v.providerTs);
      break;
    case "topic_sources":
      ok = v.v === TOPIC_VERSION && Array.isArray(v.sources) &&
        v.sources.length <= MAX_TOPIC_SOURCES &&
        v.sources.every(sourceInfo);
      break;
    case "topic_sub":
      ok = id(v.id) && topicSourceOk(v.source) && options(v);
      break;
    case "topic_open":
      ok = routed(v) && options(v);
      break;
    case "topic_ready":
      ok = id(v.id) && topicSourceOk(v.source) && routed(v);
      break;
    case "topic_opened":
    case "topic_close":
      ok = routed(v);
      break;
    case "topic_activate":
      ok = routed(v) && (v.maxHz === undefined ||
        typeof v.maxHz === "number" && Number.isFinite(v.maxHz) && v.maxHz > 0 && v.maxHz <= 1000);
      break;
    case "topic_credit":
      ok = routed(v) && topicGenerationOk(v.frames) && topicGenerationOk(v.bytes);
      break;
    case "topic_unsub":
      ok = id(v.id);
      break;
    case "topic_error":
      ok = id(v.id) && string(v.code, 64) && string(v.message, 256, 0) &&
        ["rejected", "unknown", "interrupted"].includes(v.outcome as string);
      break;
    case "topic_list":
      ok = id(v.id) && topicSourceOk(v.source) && topicKeyOk(v.prefix, true) &&
        string(v.cursor, 128, 0) && typeof v.limit === "number" &&
        Number.isInteger(v.limit) && v.limit > 0 && v.limit <= MAX_TOPIC_PAGE;
      break;
    case "topic_page":
      ok = id(v.id) && topicSourceOk(v.source) && topicGenerationOk(v.gen) &&
        string(v.cursor, 128, 0) && Array.isArray(v.entries) &&
        v.entries.length <= MAX_TOPIC_PAGE && v.entries.every(descriptor);
      break;
    case "topic_pub_result":
      ok = id(v.id) && ["accepted", "rejected", "unknown"].includes(v.outcome as string) &&
        string(v.code, 64, 0) && string(v.message, 256, 0) && finite(v.providerTs);
      break;
  }
  return ok ? v as unknown as TopicMsg : null;
}
