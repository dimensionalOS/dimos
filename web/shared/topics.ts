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
/** Encode variants one provider route serves at once. Ids count up from v1 and
 * are never reused on a route, so a late frame cannot pass for its successor. */
export const MAX_TOPIC_VARIANTS = 4;
export const MAX_TOPIC_PROFILE_OPTIONS = 8;
/** An SDP rides a control frame or an @control payload, so it stays well
 * under MAX_CONTROL_PAYLOAD_BYTES (aiortc emits ~2 KB per m-section, at most
 * MAX_RTC_TRACKS of them). SDP is ASCII, so this length and the Python
 * mirror's character count agree. */
export const MAX_SDP_LEN = 48 * 1024;
export const MAX_MID_LEN = 16;
/** Trickled ICE candidates one direct video peer may add, and the bound of
 * one candidate line. Host candidates only: a LAN has a handful. */
export const MAX_TOPIC_RTC_CANDIDATES = 64;
export const MAX_TOPIC_RTC_CANDIDATE_LEN = 512;
export const TOPIC_SETTING_NAMES = [
  "maxHz",
  "maxResolution",
  "quality",
  "keepEvery",
  "fields",
  "maxBitrate",
] as const;
export type TopicSettingName = typeof TOPIC_SETTING_NAMES[number];
/** Little-endian "DIMP": distinguishes a bulk bidi stream from length-prefixed
 * control JSON. Client-to-Deno uni stream payloads are broken in Deno 2.6.10. */
export const TOPIC_PUBLISH_STREAM_MAGIC = 0x504d4944;
/** "DIMG": independent command lane, so a bulk put cannot hold up stop/drive. */
export const TOPIC_GUARD_STREAM_MAGIC = 0x474d4944;

/** One updatable setting of a profile. `default` is the profile value, which
 * is also the ceiling (the floor for keepEvery): requests beyond it come back
 * clamped, requests outside min..max or values are rejected. */
export interface OptionRange {
  kind: "int" | "number" | "size" | "enum" | "boolean";
  min?: number;
  max?: number;
  default?: number | boolean | readonly number[] | readonly string[];
  values?: readonly string[];
}
export type OptionRanges = Record<string, OptionRange>;
export interface TopicCapabilities {
  read: boolean;
  write: boolean;
  profiles: string[];
  /** Provider-owned stop/expiry. Older peers do not advertise this. */
  guardedPublishers?: boolean;
  /** Updatable settings per profile. An older provider omits this and
   * accepts nothing beyond rate changes. */
  profileOptions?: Record<string, OptionRanges>;
}
/** The settings a subscription runs with. Keys beyond maxHz exist only where
 * the profile supports them; the acknowledgement reports the values in force. */
export interface TopicSettings {
  maxHz: number;
  maxResolution?: [number, number];
  quality?: number;
  maxBitrate?: number;
  keepEvery?: number;
  fields?: string[];
}
/** A change request: absent keys keep their value, null resets one to the
 * profile default. */
export type TopicSettingsUpdate = { [K in keyof TopicSettings]?: TopicSettings[K] | null };
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
  /** Initial settings beyond the rate, so a handle starts with them in one round trip. */
  settings?: Partial<TopicSettings>;
}
export interface TopicOpenMsg {
  t: "topic_open";
  route: string;
  gen: number;
  keyExpr: string;
  delivery: Delivery;
  profile: string;
  maxHz: number;
  /** The route's first variant, v1, with its full normalized settings. */
  settings?: TopicSettings;
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
  /** The settings in force; an older relay omits them. */
  settings?: TopicSettings;
}
/** Viewer -> relay: change the settings of one desired subscription. The
 * revision is echoed by the acknowledgement or rejection. */
export interface TopicUpdateMsg {
  t: "topic_update";
  id: string;
  revision: number;
  options: TopicSettingsUpdate;
}
/** Relay -> viewer: the viewer now receives frames with these settings. Also
 * sent without a request when the relay moves a viewer itself. */
export interface TopicAppliedMsg {
  t: "topic_applied";
  id: string;
  revision: number;
  settings: TopicSettings;
}
/** Relay -> provider: open variant `variant` of a route, or change its rate. */
export interface TopicVariantMsg {
  t: "topic_variant";
  route: string;
  gen: number;
  variant: string;
  settings: TopicSettings;
}
export interface TopicVariantReadyMsg {
  t: "topic_variant_ready";
  route: string;
  gen: number;
  variant: string;
  settings: TopicSettings;
}
export interface TopicVariantCloseMsg {
  t: "topic_variant_close";
  route: string;
  gen: number;
  variant: string;
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
/** Direct WebRTC video for a `video.*` route, signaled through the relay. On
 * the viewer leg `id` is the subscription and `route` the viewer route; the
 * relay forwards with a private peer id, the provider route and the variant
 * the viewer receives, and maps replies back. No data frames flow on a video
 * route: the provider's peer sends H.264 straight to the browser. */
export interface TopicRtcOfferMsg {
  t: "topic_rtc_offer";
  id: string;
  route: string;
  gen: number;
  sdp: string;
  /** Relay-to-provider only: the encode variant this peer is fed from. */
  variant?: string;
}
export interface TopicRtcAnswerMsg {
  t: "topic_rtc_answer";
  id: string;
  route: string;
  gen: number;
  sdp: string;
}
/** Either direction, trickled. An empty candidate is end-of-candidates. */
export interface TopicRtcCandidateMsg {
  t: "topic_rtc_candidate";
  id: string;
  route: string;
  gen: number;
  candidate: string;
  mid: string;
}
/** Either direction: the peer is gone, with the reason the other side
 * reports (`rtc_limit`, `ice_failed`, `encoder_failed`, `route_closed`, ...). */
export interface TopicRtcCloseMsg {
  t: "topic_rtc_close";
  id: string;
  route: string;
  gen: number;
  reason: string;
}
export interface TopicErrorMsg {
  t: "topic_error";
  id: string;
  /** Present when the error rejects one topic_update; the handle keeps its settings. */
  revision?: number;
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
  | TopicRtcOfferMsg
  | TopicRtcAnswerMsg
  | TopicRtcCandidateMsg
  | TopicRtcCloseMsg
  | TopicUpdateMsg
  | TopicAppliedMsg
  | TopicVariantMsg
  | TopicVariantReadyMsg
  | TopicVariantCloseMsg
  | TopicErrorMsg
  | TopicListMsg
  | TopicPageMsg
  | TopicPubResultMsg;

/** FrameHeader.meta for @t: routes. ts is send time; ingressTs and optional
 * sourceTs retain the actual acquisition/receive times in epoch seconds.
 * A media provider stamps the encode variant; the relay dispatches on it and
 * strips it before forwarding. */
export interface TopicSampleMeta extends Record<string, unknown> {
  key: string;
  encoding: string;
  gen: number;
  ingressTs: number;
  sourceTs?: number;
  variant?: string;
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
const count = (v: unknown): v is number =>
  typeof v === "number" && Number.isSafeInteger(v) && v >= 0;
const rate = (v: unknown): v is number =>
  typeof v === "number" && Number.isFinite(v) && v > 0 && v <= 1000;
const names = (v: unknown, max: number): v is string[] =>
  Array.isArray(v) && v.length <= max && v.every((s) => string(s, 32)) &&
  new Set(v).size === v.length;
export const topicVariantOk = (v: unknown): v is string =>
  typeof v === "string" && /^v[1-9][0-9]{0,15}(?![\s\S])/.test(v);
const settingOk = (name: string, v: unknown): boolean => {
  switch (name) {
    case "maxHz":
      return rate(v);
    case "maxResolution":
      return Array.isArray(v) && v.length === 2 && v.every(count);
    case "quality":
    case "keepEvery":
    case "maxBitrate":
      return count(v);
    case "fields":
      return names(v, 8);
  }
  return false;
};
/** Shape and bounds of a settings object. `full` requires the rate (the
 * normalized form the relay and provider exchange); `update` admits null
 * resets; `sparse` is a request on topic_sub. Range checks happen against the
 * advertised profile options, not here. */
export const topicSettingsOk = (
  v: unknown,
  mode: "full" | "sparse" | "update",
): v is TopicSettingsUpdate =>
  record(v) && (mode !== "full" || rate(v.maxHz)) &&
  Object.entries(v).every(([name, value]) =>
    (TOPIC_SETTING_NAMES as readonly string[]).includes(name) &&
    (mode === "update" && value === null || settingOk(name, value))
  );
const optionRangeOk = (v: unknown): v is OptionRange =>
  record(v) && ["int", "number", "size", "enum", "boolean"].includes(v.kind as string) &&
  (v.min === undefined || finite(v.min)) && (v.max === undefined || finite(v.max)) &&
  (v.values === undefined || names(v.values, 16)) &&
  (v.default === undefined || finite(v.default) || typeof v.default === "boolean" ||
    Array.isArray(v.default) && v.default.length <= 16 &&
      v.default.every((d) => finite(d) || string(d, 32)));
const profileOptionsOk = (v: unknown): boolean =>
  record(v) && Object.keys(v).length <= 16 &&
  Object.entries(v).every(([profile, ranges]) =>
    string(profile, 64) && record(ranges) &&
    Object.keys(ranges).length <= MAX_TOPIC_PROFILE_OPTIONS &&
    Object.entries(ranges).every(([name, range]) => string(name, 32) && optionRangeOk(range))
  );
export const topicCapabilitiesOk = (v: unknown): v is TopicCapabilities =>
  record(v) && typeof v.read === "boolean" && typeof v.write === "boolean" &&
  (v.guardedPublishers === undefined || typeof v.guardedPublishers === "boolean") &&
  (v.profileOptions === undefined || profileOptionsOk(v.profileOptions)) &&
  Array.isArray(v.profiles) && v.profiles.length <= 16 && v.profiles.every((p) => string(p, 64));

const RATE_RANGE: OptionRange = { kind: "number", max: 1000, default: 1000 };
function clamp(name: string, range: OptionRange, value: unknown): unknown {
  const d = range.default;
  switch (range.kind) {
    case "number":
    case "int": {
      const n = value as number;
      if (
        !(range.kind === "number" ? finite(n) : count(n)) || (name === "maxHz" && !rate(n)) ||
        (range.min !== undefined && n < range.min) ||
        (range.max !== undefined && n > range.max && name !== "maxBitrate")
      ) return null;
      if (typeof d !== "number") return n;
      // The profile bounds the data: a coarser stride is a smaller output.
      return name === "keepEvery" ? Math.max(n, d) : Math.min(n, d);
    }
    case "size": {
      if (
        !Array.isArray(value) || value.length !== 2 ||
        !value.every((n) =>
          count(n) && (range.min === undefined || n >= range.min) &&
          (range.max === undefined || n <= range.max)
        )
      ) return null;
      const [w, h] = value as [number, number];
      return Array.isArray(d) && d.length === 2 && typeof d[0] === "number" &&
          typeof d[1] === "number"
        ? [Math.min(w, d[0]), Math.min(h, d[1])]
        : [w, h];
    }
    case "enum": {
      const values = range.values ?? [];
      if (!names(value, values.length) || !value.every((f) => values.includes(f))) return null;
      return values.filter((f) => value.includes(f));
    }
    case "boolean":
      return typeof value === "boolean" ? value : null;
  }
}
export type TopicSettingsResult =
  | { settings: TopicSettings; error?: undefined }
  | { settings?: undefined; error: "unsupported_option" | "option_out_of_range" };
/** The settings in force for a request against one profile's advertised
 * options: unknown names and values outside a range are rejected; the profile
 * value is the ceiling and the default, so nothing is accepted as a no-op.
 * Mirrors normalize_settings in dimos/web/relay_bridge/topic_profiles.py. */
export function normalizeTopicSettings(
  ranges: OptionRanges,
  requested: Record<string, unknown>,
): TopicSettingsResult {
  const options: OptionRanges = { maxHz: RATE_RANGE, ...ranges };
  for (const name of Object.keys(requested)) {
    if (requested[name] !== undefined && requested[name] !== null && !(name in options)) {
      return { error: "unsupported_option" };
    }
  }
  const settings: Record<string, unknown> = {};
  for (const name of TOPIC_SETTING_NAMES) {
    const range = options[name];
    if (range === undefined) continue;
    const value = requested[name] ?? range.default;
    if (value === undefined) continue;
    const normalized = clamp(name, range, value);
    if (normalized === null) return { error: "option_out_of_range" };
    settings[name] = normalized;
  }
  if (typeof settings.maxBitrate === "number" && Array.isArray(settings.maxResolution)) {
    settings.maxBitrate = Math.min(
      settings.maxBitrate,
      videoBitrate(settings.maxResolution as [number, number]),
    );
  }
  return { settings: settings as unknown as TopicSettings };
}
/** H.264 class ceilings, mirrored by topic_profiles.video_bitrate. */
export function videoBitrate([width, height]: readonly [number, number]): number {
  const pixels = width * height;
  return pixels <= 640 * 360 ? 1_000_000 : pixels <= 1280 * 720 ? 2_500_000 : 4_500_000;
}
const sourceInfo = (v: unknown) =>
  record(v) && topicSourceOk(v.source) && topicGenerationOk(v.gen) && topicCapabilitiesOk(v);
const descriptor = (v: unknown) =>
  record(v) && topicKeyOk(v.key) && string(v.encoding, 256, 0) && string(v.type, 256, 0) &&
  ["known", "observed", "advertised"].includes(v.availability as string);

export function topicSampleMeta(v: unknown): TopicSampleMeta | null {
  return record(v) && topicKeyOk(v.key) && string(v.encoding, 256, 0) &&
      topicGenerationOk(v.gen) && finite(v.ingressTs) &&
      (v.sourceTs === undefined || finite(v.sourceTs)) &&
      (v.variant === undefined || topicVariantOk(v.variant))
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
      ok = id(v.id) && topicSourceOk(v.source) && options(v) &&
        (v.settings === undefined || topicSettingsOk(v.settings, "sparse"));
      break;
    case "topic_open":
      ok = routed(v) && options(v) &&
        (v.settings === undefined || topicSettingsOk(v.settings, "full"));
      break;
    case "topic_ready":
      ok = id(v.id) && topicSourceOk(v.source) && routed(v) &&
        (v.settings === undefined || topicSettingsOk(v.settings, "full"));
      break;
    case "topic_update":
      ok = id(v.id) && topicGenerationOk(v.revision) && topicSettingsOk(v.options, "update");
      break;
    case "topic_applied":
      ok = id(v.id) && topicGenerationOk(v.revision) && topicSettingsOk(v.settings, "full");
      break;
    case "topic_variant":
    case "topic_variant_ready":
      ok = routed(v) && topicVariantOk(v.variant) && topicSettingsOk(v.settings, "full");
      break;
    case "topic_variant_close":
      ok = routed(v) && topicVariantOk(v.variant);
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
    case "topic_rtc_offer":
      ok = id(v.id) && routed(v) && string(v.sdp, MAX_SDP_LEN) &&
        (v.variant === undefined || topicVariantOk(v.variant));
      break;
    case "topic_rtc_answer":
      ok = id(v.id) && routed(v) && string(v.sdp, MAX_SDP_LEN);
      break;
    case "topic_rtc_candidate":
      ok = id(v.id) && routed(v) && string(v.candidate, MAX_TOPIC_RTC_CANDIDATE_LEN, 0) &&
        string(v.mid, MAX_MID_LEN, 0);
      break;
    case "topic_rtc_close":
      ok = id(v.id) && routed(v) && string(v.reason, 64);
      break;
    case "topic_error":
      ok = id(v.id) && string(v.code, 64) && string(v.message, 256, 0) &&
        ["rejected", "unknown", "interrupted"].includes(v.outcome as string) &&
        (v.revision === undefined || topicGenerationOk(v.revision));
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
