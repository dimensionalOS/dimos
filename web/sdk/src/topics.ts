import {
  type DataFrame,
  type Delivery,
  encodeDataFrame,
  MAX_TOPIC_HANDLES,
  MAX_TOPIC_KEYS,
  MAX_TOPIC_PAYLOAD_BYTES,
  MAX_TOPIC_PUBLISH_BYTES,
  MAX_TOPIC_PUBLISH_PENDING,
  type Msg,
  normalizeTopicSettings,
  TOPIC_PUBLISH_CHANNEL,
  TOPIC_SETTING_NAMES,
  type TopicAppliedMsg,
  type TopicDescriptor,
  type TopicMsg,
  topicMsgFromUnknown,
  type TopicPageMsg,
  topicPublishMeta,
  type TopicPubResultMsg,
  topicSampleMeta,
  type TopicSettings,
  type TopicSettingsUpdate,
  type TopicSourceInfo,
  type TopicSubMsg,
} from "@dimos/shared";
import {
  adaptationPolicy,
  type AdaptOptions,
  type AdaptPolicy,
  type AdaptTarget,
  applyCap,
  TopicAdaptation,
  type TopicStats,
  validAdaptation,
} from "./topicAdapt.ts";
import { createPeerConnection, type PeerConnectionFactory } from "./rtc.ts";
import {
  type TopicPublisher,
  type TopicPublisherOptions,
  TopicPublishers,
} from "./topicPublishers.ts";
import {
  type TopicRtcSignal,
  type TopicVideo,
  TopicVideoHandle,
  type TopicVideoOptions,
  type VideoHost,
} from "./topicVideo.ts";

export type { TopicSettings } from "@dimos/shared";

export type TopicPhase = "connecting" | "ready" | "interrupted" | "rejected" | "closed";
export class TopicError extends Error {
  constructor(
    readonly code: string,
    readonly outcome: "rejected" | "unknown" | "interrupted",
    message = code.replaceAll("_", " "),
  ) {
    super(message);
    this.name = "TopicError";
  }
}
export interface TopicStatus {
  phase: TopicPhase;
  error?: TopicError;
}
/** Identity (source, delivery, profile) and settings. On update(), a changed
 * identity reopens the subscription; settings change in place, and null
 * resets one to the profile value. The profile is the ceiling: larger values
 * come back clamped in the acknowledgement. */
export interface TopicOptions extends AdaptOptions {
  source?: string;
  delivery?: Delivery;
  profile?: string;
  /** Latest sampling rate. Reliable event streams preserve every sample. */
  maxHz?: number | null;
  /** The box a JPEG picture is fitted into, never above the profile size. */
  maxResolution?: [number, number] | null;
  /** JPEG quality, 1..100. */
  quality?: number | null;
  /** H.264 target in bits per second, clamped to the resolution class. */
  maxBitrate?: number | null;
  /** One cloud point in N, after the finite filter. */
  keepEvery?: number | null;
  /** Optional cloud buffers to encode: "intensity", "color". */
  fields?: string[] | null;
}
export interface TopicSample {
  /** Borrowed bytes: copy before mutating or transferring to a worker. */
  payload: Uint8Array;
  key: string;
  encoding: string;
  source: string;
  generation: number;
  sequence: number;
  ingressTs: number;
  sourceTs?: number;
  sentTs: number;
  metadata: Readonly<Record<string, unknown>>;
}
export interface TopicSubscription {
  ready(): Promise<void>;
  status(): TopicStatus;
  onStatus(callback: (status: TopicStatus) => void): () => void;
  /** The settings in force: the profile values until the relay acknowledges. */
  settings(): TopicSettings;
  onSettings(callback: (settings: TopicSettings) => void): () => void;
  stats(): TopicStats | null;
  /** Resolves with the settings in force. A rejected change (unsupported
   * option, value out of range, variant limit) leaves settings and status
   * untouched; a changed source, delivery or profile reopens the handle. */
  update(options: TopicOptions): Promise<TopicSettings>;
  close(): void;
}
export interface TopicListOptions {
  source?: string;
  prefix?: string;
  cursor?: string;
  limit?: number;
}
export interface TopicPage {
  source: string;
  generation: number;
  entries: readonly TopicDescriptor[];
  cursor: string;
}
export interface TopicPublishOptions {
  source?: string;
  encoding?: string;
}
export interface TopicPublishReceipt {
  id: string;
  /** Provider local put completion, epoch seconds; not robot execution. */
  providerTs: number;
}
export interface Topics {
  setAdaptation(enabled: boolean): void;
  /** Guarded commands require an explicit safe stop and a robot-side watchdog. */
  publisher(key: string, safeStop: Uint8Array, options: TopicPublisherOptions): TopicPublisher;
  subscribe(
    keyExpr: string,
    options: TopicOptions,
    onMessage: (sample: TopicSample) => void,
  ): TopicSubscription;
  list(options?: TopicListOptions): Promise<TopicPage>;
  /** Checked local Zenoh put. Never automatically retried, including reconnect. */
  publish(
    key: string,
    bytes: Uint8Array,
    options?: TopicPublishOptions,
  ): Promise<TopicPublishReceipt>;
  sources(): readonly TopicSourceInfo[];
  onSources(callback: (sources: readonly TopicSourceInfo[]) => void): () => void;
  /** Fires after available subscriptions recover, for HTTP resync. Offline sources do not block it. */
  onReconnect(callback: () => void): () => void;
  /** A camera as a MediaStreamTrack over a direct WebRTC connection to the
   * provider (profile video.h264.v1). `onTrack` fires with the track on
   * arrival and after a reconnect, and with null when it ends. */
  video(
    keyExpr: string,
    options: TopicVideoOptions,
    onTrack: (track: MediaStreamTrack | null) => void,
  ): TopicVideo;
}

function deferred() {
  let resolve!: () => void;
  let reject!: (error: TopicError) => void;
  const promise = new Promise<void>((yes, no) => {
    resolve = yes;
    reject = no;
  });
  // A handle need not call ready(); its status callback remains sufficient.
  promise.catch(() => {});
  return { promise, resolve, reject, settled: false };
}
function notify<T>(callbacks: Iterable<(value: T) => void>, value: T) {
  for (const callback of [...callbacks]) {
    try {
      callback(value);
    } catch (error) {
      console.error("topic callback failed", error);
    }
  }
}
const IDENTITY_KEYS = ["source", "delivery", "profile"] as const;
const OPTION_KEYS: readonly string[] = [
  ...IDENTITY_KEYS,
  ...TOPIC_SETTING_NAMES,
  "adapt",
  "priority",
  "keep",
];
interface Identity extends AdaptPolicy {
  source?: string;
  delivery: Delivery;
  profile: string;
}
/** A settings change of a handle alone on its group. Changes go to the
 * relay one at a time, each merged into the acknowledged request. */
interface PendingUpdate {
  handle: Handle;
  cap?: Partial<TopicSettings>;
  /** Runs with the acknowledgement, before the next queued change. */
  applied?: () => void;
  options: TopicSettingsUpdate;
  resolve: (settings: TopicSettings) => void;
  reject: (error: TopicError) => void;
}
interface InFlight {
  revision: number;
  cap: Partial<TopicSettings>;
  applied?: () => void;
  /** The request in force once the relay acknowledges. */
  requested: Record<string, unknown>;
  resolve: (settings: TopicSettings) => void;
  reject: (error: TopicError) => void;
}
interface Group {
  id: string;
  signature: string;
  /** Never shared or re-indexed: a video handle owns its relay route. */
  exclusive: boolean;
  keyExpr: string;
  identity: Identity;
  /** The acknowledged settings request, rate included; what topic_sub carries. */
  requested: Record<string, unknown>;
  settings: TopicSettings;
  cap: Partial<TopicSettings>;
  adaptation: AdaptTarget;
  handles: Set<Handle>;
  sent: boolean;
  source: string | null;
  route: string | null;
  gen: number;
  status: TopicStatus;
  seq: number | null;
  keys: Map<string, number>;
  frames: number;
  bytes: number;
  retry: ReturnType<typeof setTimeout> | null;
  failures: number;
  revision: number;
  /** The change the relay has not answered yet. */
  pending: InFlight | null;
  queued: PendingUpdate[];
}
/** A handle moving to another group: it keeps its old one until the new one
 * is ready, so a settings change on a shared subscription never gaps. */
interface Move {
  group: Group;
  queued: PendingUpdate[];
  resolve: (settings: TopicSettings) => void;
  reject: (error: TopicError) => void;
}

function settingsOf(options: TopicOptions): Record<string, unknown> {
  const requested: Record<string, unknown> = {};
  for (const name of TOPIC_SETTING_NAMES) {
    const value = options[name];
    if (value !== undefined && value !== null) requested[name] = value;
  }
  return requested;
}
function signatureOf(keyExpr: string, identity: Identity, requested: Record<string, unknown>) {
  return JSON.stringify([
    identity.source,
    keyExpr,
    identity.delivery,
    identity.profile,
    identity.adapt,
    identity.priority,
    identity.keep,
    TOPIC_SETTING_NAMES.map((name) => requested[name] ?? null),
  ]);
}

class Handle implements TopicSubscription {
  group: Group | null = null;
  move: Move | null = null;
  /** A video handle's receiver, for the statistics of whichever group it is in. */
  receiver: (() => RTCRtpReceiver | null) | null = null;
  #status: TopicStatus = { phase: "connecting" };
  #settings: TopicSettings = { maxHz: 30 };
  #wait = deferred();
  #listeners = new Set<(status: TopicStatus) => void>();
  #settingsListeners = new Set<(settings: TopicSettings) => void>();
  constructor(
    readonly owner: TopicClient,
    readonly callback: (sample: TopicSample) => void,
    readonly exclusive = false,
  ) {}
  ready(): Promise<void> {
    return this.#wait.promise;
  }
  status(): TopicStatus {
    return this.#status;
  }
  settings(): TopicSettings {
    if (this.group !== null) this.#settings = this.group.settings;
    return this.#settings;
  }
  onSettings(callback: (settings: TopicSettings) => void): () => void {
    this.#settingsListeners.add(callback);
    return () => this.#settingsListeners.delete(callback);
  }
  applied(): void {
    notify(this.#settingsListeners, this.settings());
  }
  stats(): TopicStats | null {
    return this.owner.stats(this);
  }
  transition(status: TopicStatus): void {
    if (this.#status.phase === "closed") return;
    if ((status.phase === "connecting" || status.phase === "interrupted") && this.#wait.settled) {
      this.#wait = deferred();
    }
    this.#status = status;
    if (status.phase === "ready") {
      this.#wait.settled = true;
      this.#wait.resolve();
    }
    if (status.phase === "rejected" || status.phase === "closed") {
      this.#wait.settled = true;
      this.#wait.reject(status.error ?? new TopicError("closed", "rejected"));
    }
    notify(this.#listeners, status);
  }
  onStatus(callback: (status: TopicStatus) => void): () => void {
    this.#listeners.add(callback);
    notify([callback], this.#status);
    return () => this.#listeners.delete(callback);
  }
  update(options: TopicOptions): Promise<TopicSettings> {
    const group = this.group;
    if (group === null) return Promise.reject(new TopicError("closed", "rejected"));
    const unknown = Object.keys(options).find((key) => !OPTION_KEYS.includes(key));
    if (unknown !== undefined || !validAdaptation(options)) {
      return Promise.reject(new TopicError("unsupported_option", "rejected"));
    }
    const identity = group.identity;
    const nested = typeof options.adapt === "object" ? options.adapt : {};
    const policyOptions = {
      ...identity,
      ...options,
      priority: options.priority ?? nested.priority ?? identity.priority,
      keep: options.keep ?? nested.keep ?? identity.keep,
    };
    const policy = adaptationPolicy(
      policyOptions,
      options.profile ?? identity.profile,
      options.delivery ?? identity.delivery,
    );
    if (
      IDENTITY_KEYS.some((key) => options[key] !== undefined && options[key] !== identity[key]) ||
      policy.adapt !== identity.adapt || policy.priority !== identity.priority ||
      policy.keep !== identity.keep
    ) {
      // A different source, delivery or profile is another subscription.
      this.owner.release(this);
      this.transition({ phase: "connecting" });
      this.owner.acquire(this, group.keyExpr, {
        ...identity,
        ...(group.requested as TopicOptions),
        ...options,
        ...policy,
      });
      return this.ready().then(() => this.settings());
    }
    return this.owner.update(this, options);
  }
  close(): void {
    this.owner.release(this);
    this.transition({ phase: "closed" });
    this.#listeners.clear();
    this.#settingsListeners.clear();
  }
}

/** Per-message callbacks and routing live outside ChannelStore's UI tick. */
export class TopicClient implements Topics {
  readonly #adaptation = new TopicAdaptation();
  setAdaptation(enabled: boolean): void {
    this.#adaptation.setEnabled(enabled);
  }
  pong(aborted?: number): void {
    this.#adaptation.pong(aborted);
  }
  stats(handle: Handle): TopicStats | null {
    return handle.group === null ? null : this.#adaptation.stats(handle.group.adaptation);
  }
  /** The groups a new consumer may join, by signature. One with a change in
   * flight is left out: a joiner could not tell which settings it would get. */
  #groups = new Map<string, Group>();
  /** Every live group, by id. */
  #ids = new Map<string, Group>();
  #routes = new Map<string, Group>();
  #sources = new Map<string, TopicSourceInfo>();
  #sourceListeners = new Set<(sources: readonly TopicSourceInfo[]) => void>();
  #reconnectListeners = new Set<() => void>();
  #send: ((msg: Msg) => void) | null = null;
  #sendBinary: ((bytes: Uint8Array) => Promise<void>) | null = null;
  #disconnect: (() => void) | null = null;
  #negotiated = false;
  #closed = false;
  #handles = 0;
  #counter = 0;
  #prefix = crypto.randomUUID().slice(0, 8);
  #wasReady = false;
  #recovering = false;
  #credits = new Set<Group>();
  #creditTimer: ReturnType<typeof setTimeout> | null = null;
  /** Direct video handles by the id of their inner subscription. */
  #videoSinks = new Map<string, (msg: TopicRtcSignal) => void>();
  readonly #createPeerConnection: PeerConnectionFactory;
  #lists = new Map<string, {
    resolve: (page: TopicPage) => void;
    reject: (error: TopicError) => void;
    source: string;
    gen: number;
    limit: number;
    timer: ReturnType<typeof setTimeout>;
  }>();
  #publications = new Map<string, {
    source: string;
    gen: number;
    bytes: number;
    resolve: (receipt: TopicPublishReceipt) => void;
    reject: (error: TopicError) => void;
    timer: ReturnType<typeof setTimeout>;
  }>();
  #publicationBytes = 0;
  #sendGuard: ((bytes: Uint8Array) => Promise<void>) | null = null;
  #publishers = new TopicPublishers({
    id: () => this.#id(),
    source: (requested) => {
      try {
        const source = this.#source(requested);
        return source === undefined ? undefined : this.#sources.get(source);
      } catch {
        return undefined;
      }
    },
    send: (msg) => this.#send?.(msg),
    binary: (bytes) => this.#sendGuard?.(bytes) ?? Promise.reject(new Error("not connected")),
  });

  constructor(
    readonly defaultSource?: string,
    createPc: PeerConnectionFactory = createPeerConnection,
  ) {
    this.#createPeerConnection = createPc;
  }
  publisher(key: string, safeStop: Uint8Array, options: TopicPublisherOptions): TopicPublisher {
    return this.#publishers.publisher(key, safeStop, options);
  }
  video(
    keyExpr: string,
    options: TopicVideoOptions,
    onTrack: (track: MediaStreamTrack | null) => void,
  ): TopicVideo {
    return new TopicVideoHandle(this.#videoHost, keyExpr, options, onTrack);
  }
  readonly #videoHost: VideoHost = {
    subscribe: (keyExpr, options, onSample, exclusive) => {
      const handle = new Handle(this, onSample, exclusive);
      this.acquire(handle, keyExpr, options);
      return handle;
    },
    receiver: (handle, receiver) => {
      (handle as Handle).receiver = receiver;
    },
    routeOf: (handle) => {
      const group = (handle as Handle).group;
      return group === null || group.route === null || group.status.phase !== "ready"
        ? null
        : { id: group.id, route: group.route, gen: group.gen };
    },
    bind: (id, sink) => this.#videoSinks.set(id, sink),
    unbind: (id) => this.#videoSinks.delete(id),
    requestOf: (handle) => {
      // The source serving the camera, also when it was picked as the only one
      // connected: a fallback must not lose it to a source that joined since.
      const group = (handle as Handle).group;
      return group === null
        ? {}
        : { ...group.identity, source: group.source ?? group.identity.source, ...group.requested };
    },
    send: (msg) => {
      if (this.#send === null) return false;
      this.#send(msg);
      return true;
    },
    // Host candidates only: direct video needs the provider's UDP ports reachable.
    createPeerConnection: () => this.#createPeerConnection({ iceServers: [] }),
  };
  #id(): string {
    return `${this.#prefix}-${++this.#counter}`;
  }
  sources(): readonly TopicSourceInfo[] {
    return [...this.#sources.values()].map((s) => ({ ...s, profiles: [...s.profiles] }));
  }
  onSources(callback: (sources: readonly TopicSourceInfo[]) => void): () => void {
    this.#sourceListeners.add(callback);
    notify([callback], this.sources());
    return () => this.#sourceListeners.delete(callback);
  }
  onReconnect(callback: () => void): () => void {
    this.#reconnectListeners.add(callback);
    return () => this.#reconnectListeners.delete(callback);
  }
  subscribe(
    keyExpr: string,
    options: TopicOptions,
    callback: (sample: TopicSample) => void,
  ): TopicSubscription {
    const handle = new Handle(this, callback);
    this.acquire(handle, keyExpr, options);
    return handle;
  }
  /** The group for keyExpr and options, created and requested when new. */
  #obtain(keyExpr: string, options: TopicOptions, exclusive = false): Group | TopicError {
    if (this.#closed) return new TopicError("closed", "rejected");
    const identity: Identity = {
      ...adaptationPolicy(options, options.profile ?? "raw", options.delivery ?? "latest"),
      source: options.source ?? this.defaultSource,
      delivery: options.delivery ?? "latest",
      profile: options.profile ?? "raw",
    };
    const requested = {
      maxHz: options.maxHz ?? (identity.delivery === "reliable" ? 1000 : 30),
      ...settingsOf(options),
    };
    const { maxHz, ...settings } = requested;
    const candidate = {
      t: "topic_sub",
      id: "validate",
      source: identity.source ?? "auto",
      keyExpr,
      delivery: identity.delivery,
      profile: identity.profile,
      maxHz,
      ...(Object.keys(settings).length ? { settings } : {}),
    };
    if (
      !validAdaptation(options) || Object.keys(options).some((key) => !OPTION_KEYS.includes(key)) ||
      topicMsgFromUnknown(candidate) === null
    ) {
      return new TopicError("invalid_topic_options", "rejected");
    }
    let signature = signatureOf(keyExpr, identity, requested);
    let group = exclusive ? undefined : this.#groups.get(signature);
    if (group === undefined) {
      const id = this.#id();
      if (exclusive) signature = `${signature}#${id}`;
      group = {
        id,
        signature,
        exclusive,
        keyExpr,
        identity,
        requested,
        cap: {},
        adaptation: {
          profile: identity.profile,
          policy: identity,
          ready: () => group!.status.phase === "ready" && group!.pending === null,
          ceiling: () =>
            normalizeTopicSettings(this.#ranges(group!) ?? {}, group!.requested).settings ??
              group!.settings,
          settings: () => group!.settings,
          receiver: () => {
            for (const h of group!.handles) {
              if (h.group === group && h.receiver !== null) return h.receiver();
            }
            return null;
          },
          apply: (cap, applied) =>
            new Promise((resolve, reject) => {
              const handle = [...group!.handles].find((h) => h.group === group);
              if (!handle) {
                reject(new TopicError("closed", "rejected"));
                return;
              }
              const update: PendingUpdate = { handle, cap, applied, options: {}, resolve, reject };
              if (group!.pending === null) this.#change(group!, update);
              else group!.queued.push(update);
            }),
        },
        settings: { maxHz: requested.maxHz },
        handles: new Set(),
        sent: false,
        source: null,
        route: null,
        gen: 0,
        status: { phase: "connecting" },
        seq: null,
        keys: new Map(),
        frames: 0,
        bytes: 0,
        retry: null,
        failures: 0,
        revision: 0,
        pending: null,
        queued: [],
      };
      this.#groups.set(signature, group);
      this.#ids.set(group.id, group);
      this.#adaptation.add(group.adaptation);
    }
    // A new consumer or explicit update is a fresh attempt, including when
    // another handle still holds a previous rejection of this shared key.
    if (group.status.phase === "rejected") this.#restart(group);
    return group;
  }
  acquire(handle: Handle, keyExpr: string, options: TopicOptions): void {
    const group = this.#handles >= MAX_TOPIC_HANDLES
      ? new TopicError("topic_limit", "rejected")
      : this.#obtain(keyExpr, options, handle.exclusive);
    if (group instanceof TopicError) {
      handle.transition({ phase: "rejected", error: group });
      return;
    }
    group.handles.add(handle);
    handle.group = group;
    this.#handles++;
    handle.transition(group.status);
    this.#subscribe(group);
  }
  #leave(handle: Handle, group: Group): void {
    group.handles.delete(handle);
    if (group.handles.size > 0) return;
    if (group.retry !== null) clearTimeout(group.retry);
    if (group.sent) this.#send?.({ t: "topic_unsub", id: group.id });
    if (group.route !== null) this.#routes.delete(group.route);
    group.route = null;
    this.#credits.delete(group);
    if (this.#groups.get(group.signature) === group) this.#groups.delete(group.signature);
    this.#ids.delete(group.id);
    this.#adaptation.remove(group.adaptation);
    this.#failUpdates(group, new TopicError("closed", "rejected"));
  }
  release(handle: Handle): void {
    const move = handle.move;
    if (move !== null) {
      handle.move = null;
      this.#leave(handle, move.group);
      const error = new TopicError("closed", "rejected");
      move.reject(error);
      for (const update of move.queued) update.reject(error);
    }
    const group = handle.group;
    if (group === null) return;
    handle.group = null;
    this.#handles--;
    this.#leave(handle, group);
  }
  /** Settings change: in place when the handle owns its group and the relay
   * installed it, after any change still in flight; otherwise make before
   * break through another group. */
  update(handle: Handle, options: TopicOptions): Promise<TopicSettings> {
    const change: TopicSettingsUpdate = {};
    for (const name of TOPIC_SETTING_NAMES) {
      if (options[name] !== undefined) (change as Record<string, unknown>)[name] = options[name];
    }
    const candidate = { t: "topic_update", id: "validate", revision: 0, options: change };
    if (topicMsgFromUnknown(candidate) === null) {
      return Promise.reject(new TopicError("invalid_topic_options", "rejected"));
    }
    return new Promise((resolve, reject) => {
      this.#enqueue({ handle, options: change, resolve, reject });
    });
  }
  /** Follow the handle across moves, merging only after the preceding answer. */
  #enqueue(update: PendingUpdate): void {
    const { handle } = update;
    const group = handle.group;
    if (group === null) update.reject(new TopicError("closed", "rejected"));
    else if (handle.move !== null) handle.move.queued.push(update);
    else if (group.pending !== null) group.queued.push(update);
    else this.#change(group, update);
  }
  /** The acknowledged request with `options` applied. */
  #merge(group: Group, options: TopicSettingsUpdate): Record<string, unknown> {
    const requested: Record<string, unknown> = { ...group.requested };
    for (const [name, value] of Object.entries(options)) {
      if (value === null) delete requested[name];
      else requested[name] = value;
    }
    if (requested.maxHz === undefined) {
      // The rate is always requested; null asks the relay for the profile value.
      const range = this.#ranges(group)?.maxHz;
      requested.maxHz = typeof range?.default === "number" ? range.default : 1000;
    }
    return requested;
  }
  #change(group: Group, update: PendingUpdate): void {
    const { handle } = update;
    const requested = this.#merge(group, update.options);
    const cap = update.cap ?? group.cap;
    const effective = applyCap(requested, cap);
    const ranges = this.#ranges(group);
    if (ranges !== null) {
      // The page's request on its own: a cap would hide a bad value until it lifts.
      const result = normalizeTopicSettings(ranges, requested);
      if (result.error !== undefined) {
        update.reject(new TopicError(result.error, "rejected"));
        return;
      }
    }
    const existing = this.#groups.get(signatureOf(group.keyExpr, group.identity, requested));
    const inPlace = (update.cap !== undefined || group.handles.size === 1) &&
      group.status.phase === "ready" && group.sent &&
      this.#send !== null &&
      (update.cap !== undefined || group.exclusive || existing === undefined || existing === group);
    if (!inPlace && group.exclusive) {
      // An exclusive route never moves to a shared one; it changes once ready.
      update.reject(new TopicError("topic_not_ready", "rejected"));
      return;
    }
    if (inPlace) {
      if (this.#groups.get(group.signature) === group) this.#groups.delete(group.signature);
      const revision = ++group.revision;
      group.pending = {
        revision,
        requested,
        cap,
        applied: update.applied,
        resolve: update.resolve,
        reject: update.reject,
      };
      const options: TopicSettingsUpdate = { ...update.options };
      const previous = applyCap(group.requested, group.cap);
      // A disappearing cap resets the relay override, including null for a
      // setting the page never supplied. Ordinary page updates stay sparse.
      for (const name of TOPIC_SETTING_NAMES) {
        if (name in cap || name in group.cap) {
          if (
            name in options || JSON.stringify(previous[name]) !== JSON.stringify(effective[name])
          ) {
            (options as Record<string, unknown>)[name] = effective[name] ?? null;
          }
        }
      }
      this.#send!({ t: "topic_update", id: group.id, revision, options });
      return;
    }
    // A shared group, or settings another group already serves: move there.
    const target = this.#obtain(group.keyExpr, {
      ...group.identity,
      ...(requested as TopicOptions),
    });
    if (target instanceof TopicError) {
      update.reject(target);
      return;
    }
    if (target === group) {
      handle.ready().then(() => update.resolve(handle.settings()), update.reject);
      return;
    }
    if (!target.sent && target.handles.size === 0) {
      target.cap = { ...group.cap };
      this.#adaptation.inherit(target.adaptation, group.adaptation);
    }
    target.handles.add(handle);
    handle.move = { group: target, queued: [], resolve: update.resolve, reject: update.reject };
    this.#subscribe(target);
    this.#settle(target);
  }
  /** The relay answered: send the next change, or share the group again. */
  #next(group: Group): void {
    // A change can immediately move the handle and retire this group.
    for (const update of group.queued.splice(0)) this.#enqueue(update);
    this.#index(group);
  }
  /** Offer a live group to new consumers under its acknowledged settings,
   * unless a change is in flight or another group already serves them. */
  #index(group: Group): void {
    if (group.pending !== null || group.exclusive || this.#ids.get(group.id) !== group) return;
    if (!this.#groups.has(group.signature)) this.#groups.set(group.signature, group);
  }
  /** Handles waiting for `group` switch to it once ready; a rejection keeps
   * them where they are. Frames and status never reach a waiting handle. */
  #settle(group: Group): void {
    for (const handle of [...group.handles]) {
      const move = handle.move;
      if (move === null || move.group !== group) continue;
      if (group.status.phase === "ready") {
        handle.move = null;
        const previous = handle.group;
        handle.group = group;
        if (previous !== null) this.#leave(handle, previous);
        // A handle whose old group never got ready is ready through this one.
        if (handle.status().phase !== "ready") handle.transition(group.status);
        handle.applied();
        move.resolve(group.settings);
        for (const update of move.queued) this.#enqueue(update);
      } else if (group.status.phase === "rejected" || group.status.phase === "interrupted") {
        handle.move = null;
        this.#leave(handle, group);
        const error = group.status.error ?? new TopicError("rejected", "rejected");
        move.reject(error);
        for (const update of move.queued) {
          if (group.status.phase === "interrupted") update.reject(error);
          else this.#enqueue(update);
        }
      }
    }
  }
  #ranges(group: Group) {
    const source = group.source ?? group.identity.source;
    if (source === undefined) return null;
    const info = this.#sources.get(source);
    if (info === undefined) return null;
    return group.identity.profile === "raw"
      ? {}
      : info.profileOptions?.[group.identity.profile] ?? {};
  }
  #failUpdates(group: Group, error: TopicError): void {
    const pending = group.pending;
    group.pending = null;
    pending?.reject(error);
    for (const update of group.queued.splice(0)) update.reject(error);
    this.#index(group);
  }
  #transition(group: Group, status: TopicStatus): void {
    group.status = status;
    for (const handle of [...group.handles]) {
      if (handle.group === group) handle.transition(status);
    }
    this.#settle(group);
  }
  #restart(group: Group): void {
    if (this.#closed || this.#ids.get(group.id) !== group) return;
    if (group.retry !== null) clearTimeout(group.retry);
    group.retry = null;
    if (group.sent) this.#send?.({ t: "topic_unsub", id: group.id });
    if (group.route !== null) this.#routes.delete(group.route);
    this.#credits.delete(group);
    this.#ids.delete(group.id);
    group.id = this.#id(); // late errors/readiness from the previous attempt cannot poison this one
    this.#ids.set(group.id, group);
    group.route = null;
    group.sent = false;
    this.#failUpdates(group, new TopicError("connection_lost", "interrupted"));
    this.#transition(group, { phase: "connecting" });
    this.#subscribe(group);
  }
  #retry(group: Group): void {
    if (group.retry !== null || this.#send === null || this.#ids.get(group.id) !== group) return;
    const delay = Math.min(30_000, 1000 * 2 ** Math.min(group.failures++, 5));
    group.retry = setTimeout(() => this.#restart(group), delay);
  }
  #interrupt(group: Group, error: TopicError): void {
    if (group.route !== null) this.#routes.delete(group.route);
    group.route = null;
    this.#credits.delete(group);
    this.#recovering = this.#wasReady;
    this.#failUpdates(group, new TopicError(error.code, "interrupted", error.message));
    this.#transition(group, { phase: "interrupted", error });
    // source_offline keeps the desired subscription on the relay. Every other
    // fault drops just this route; recover it with bounded backoff on this session.
    if (error.code !== "source_offline") {
      if (group.sent) this.#send?.({ t: "topic_unsub", id: group.id });
      group.sent = false;
      this.#retry(group);
    }
  }
  #restored(): void {
    // A source emits one interruption per route before leaving the list. Do
    // not mistake the still-ready routes in that burst for a recovered app,
    // and never wait on routes whose source is not connected.
    const groups = [...this.#ids.values()].filter((group) =>
      group.source !== null && this.#sources.has(group.source)
    );
    const restored = groups.every((group) =>
      group.status.phase === "ready" || group.status.phase === "rejected"
    );
    if (!restored || !groups.some((group) => group.status.phase === "ready")) return;
    const recovering = this.#recovering;
    this.#wasReady = true;
    this.#recovering = false;
    if (recovering) notify(this.#reconnectListeners, undefined);
  }
  /** The requested or default source, else the only connected one; undefined
   * while none is connected. Several connected sources need an explicit name. */
  #source(requested?: string): string | undefined {
    const source = requested ?? this.defaultSource;
    if (source !== undefined) return source;
    if (this.#sources.size > 1) throw new TopicError("topic_source_required", "rejected");
    return this.#sources.keys().next().value;
  }
  /** The connected source a one-shot request goes to, or the rejection. */
  #target(requested: string | undefined, right: "read" | "write"): TopicSourceInfo {
    const source = this.#source(requested);
    if (source === undefined) throw new TopicError("topic_source_required", "rejected");
    const info = this.#sources.get(source);
    if (info === undefined) throw new TopicError("source_offline", "rejected");
    if (!info[right]) throw new TopicError(`topic_${right}_denied`, "rejected");
    return info;
  }
  #subscribe(group: Group): void {
    if (
      !this.#negotiated || group.sent || this.#send === null || group.status.phase === "rejected"
    ) return;
    let source: string | undefined;
    try {
      source = this.#source(group.identity.source);
    } catch (error) {
      this.#transition(group, { phase: "rejected", error: error as TopicError });
      return;
    }
    if (source === undefined) {
      // Nothing is connected yet; the next topic_sources announcement retries.
      this.#transition(group, {
        phase: "interrupted",
        error: new TopicError("source_offline", "interrupted"),
      });
      return;
    }
    // A source the relay has not announced is offline, not denied: the relay
    // keeps the desired subscription and attaches it once the source connects.
    if (this.#sources.get(source)?.read === false) {
      this.#transition(group, {
        phase: "rejected",
        error: new TopicError("topic_read_denied", "rejected"),
      });
      return;
    }
    group.source = source;
    // The relay refuses a profile the source does not offer before it looks at
    // the settings; the same order here keeps the reason the same when the
    // settings would also fail against the missing profile's empty ranges.
    const offered = this.#sources.get(source)?.profiles;
    if (offered !== undefined && !offered.includes(group.identity.profile)) {
      this.#transition(group, {
        phase: "rejected",
        error: new TopicError("unsupported_profile", "rejected"),
      });
      return;
    }
    // What the provider will apply, known from its advertised ranges; the
    // relay's acknowledgement replaces this once the route is ready.
    const ranges = this.#ranges(group);
    if (ranges !== null) {
      const result = normalizeTopicSettings(ranges, applyCap(group.requested, group.cap));
      if (result.error !== undefined) {
        this.#transition(group, {
          phase: "rejected",
          error: new TopicError(result.error, "rejected"),
        });
        return;
      }
      group.settings = result.settings;
    }
    group.sent = true;
    const { maxHz, ...settings } = applyCap(group.requested, group.cap);
    const message: TopicSubMsg = {
      t: "topic_sub",
      id: group.id,
      source,
      keyExpr: group.keyExpr,
      delivery: group.identity.delivery,
      profile: group.identity.profile,
      maxHz: maxHz as number,
      ...(Object.keys(settings).length ? { settings: settings as Partial<TopicSettings> } : {}),
    };
    this.#send(message);
  }
  connected(
    send: (msg: Msg) => void,
    disconnect: () => void,
    sendBinary?: (bytes: Uint8Array) => Promise<void>,
    sendGuard?: (bytes: Uint8Array) => Promise<void>,
  ): void {
    this.#send = send;
    this.#disconnect = disconnect;
    this.#sendBinary = sendBinary ?? null;
    this.#sendGuard = sendGuard ?? null;
  }
  disconnected(): void {
    this.#adaptation.disconnected();
    this.#publishers.stopAll("connection_lost");
    this.#sendGuard = null;
    if (this.#creditTimer !== null) clearTimeout(this.#creditTimer);
    this.#creditTimer = null;
    this.#credits.clear();
    this.#send = null;
    this.#sendBinary = null;
    this.#disconnect = null;
    this.#negotiated = false;
    this.#routes.clear();
    this.#sources.clear();
    this.#recovering = this.#wasReady;
    for (const group of this.#ids.values()) {
      if (group.retry !== null) clearTimeout(group.retry);
      group.retry = null;
      group.sent = false;
      group.route = null;
      group.keys.clear();
      const error = new TopicError("connection_lost", "interrupted");
      this.#failUpdates(group, error);
      this.#transition(group, { phase: "interrupted", error });
    }
    for (const id of [...this.#lists.keys()]) {
      this.#rejectList(id, new TopicError("connection_lost", "interrupted"));
    }
    for (const id of [...this.#publications.keys()]) {
      this.#publicationResult({
        t: "topic_pub_result",
        id,
        outcome: "unknown",
        code: "connection_lost",
        message: "Connection lost before provider acknowledgement",
        providerTs: 0,
      });
    }
    notify(this.#sourceListeners, this.sources());
  }
  close(): void {
    if (this.#closed) return;
    this.#closed = true;
    this.#publishers.close();
    this.#adaptation.close();
    for (const group of [...this.#ids.values()]) {
      for (const handle of [...group.handles]) handle.close();
    }
    this.disconnected();
    this.#sourceListeners.clear();
    this.#reconnectListeners.clear();
  }
  #rejectList(id: string, error: TopicError): void {
    const pending = this.#lists.get(id);
    if (pending === undefined) return;
    this.#lists.delete(id);
    clearTimeout(pending.timer);
    pending.reject(error);
  }
  list(options: TopicListOptions = {}): Promise<TopicPage> {
    if (!this.#negotiated || this.#send === null) {
      return Promise.reject(new TopicError("not_connected", "rejected"));
    }
    let info: TopicSourceInfo;
    try {
      info = this.#target(options.source, "read");
    } catch (error) {
      return Promise.reject(error);
    }
    if (this.#lists.size >= 16) {
      return Promise.reject(new TopicError("topic_request_limit", "rejected"));
    }
    const id = this.#id();
    const message = {
      t: "topic_list" as const,
      id,
      source: info.source,
      prefix: options.prefix ?? "",
      cursor: options.cursor ?? "",
      limit: options.limit ?? 32,
    };
    if (topicMsgFromUnknown(message) === null) {
      return Promise.reject(new TopicError("invalid_topic_options", "rejected"));
    }
    return new Promise((resolve, reject) => {
      const timer = setTimeout(
        () => this.#rejectList(id, new TopicError("topic_list_timeout", "unknown")),
        15_000,
      );
      this.#lists.set(id, {
        resolve,
        reject,
        source: info.source,
        gen: info.gen,
        limit: message.limit,
        timer,
      });
      this.#send!(message);
    });
  }
  #applied(msg: TopicAppliedMsg): void {
    const group = this.#ids.get(msg.id);
    const pending = group?.pending ?? null;
    if (group === undefined || pending === null || pending.revision !== msg.revision) return;
    group.pending = null;
    group.requested = pending.requested;
    group.cap = pending.cap;
    if (!group.exclusive) {
      group.signature = signatureOf(group.keyExpr, group.identity, pending.requested);
    }
    group.settings = msg.settings;
    // The controller's rung goes with the cap: a page update drained below
    // may split the group, and the new group inherits both.
    pending.applied?.();
    for (const handle of group.handles) if (handle.group === group) handle.applied();
    pending.resolve(msg.settings);
    this.#next(group);
  }
  onMessage(msg: TopicMsg): void {
    if (this.#closed) return;
    if (this.#publishers.message(msg)) return;
    if (
      msg.t === "topic_rtc_answer" || msg.t === "topic_rtc_candidate" || msg.t === "topic_rtc_close"
    ) {
      const group = this.#ids.get(msg.id);
      if (group?.route === msg.route && group.gen === msg.gen) this.#videoSinks.get(msg.id)?.(msg);
      return;
    }
    if (msg.t === "topic_sources") {
      this.#negotiated = true;
      this.#sources = new Map(msg.sources.map((s) => [s.source, s]));
      this.#publishers.sourcesChanged();
      for (const [id, pending] of this.#publications) {
        if (this.#sources.get(pending.source)?.gen !== pending.gen) {
          this.#publicationResult({
            t: "topic_pub_result",
            id,
            outcome: "unknown",
            code: "source_offline",
            message: "Source changed before provider acknowledgement",
            providerTs: 0,
          });
        }
      }
      notify(this.#sourceListeners, this.sources());
      for (const group of this.#ids.values()) this.#subscribe(group);
      this.#restored();
    } else if (msg.t === "topic_ready") {
      const group = this.#ids.get(msg.id);
      if (
        group === undefined || !group.sent || group.source !== msg.source ||
        this.#sources.get(msg.source)?.gen !== msg.gen
      ) return;
      if (group.route !== null) this.#routes.delete(group.route);
      group.route = msg.route;
      group.gen = msg.gen;
      group.seq = null;
      group.keys.clear();
      group.frames = 0;
      group.bytes = 0;
      if (msg.settings !== undefined) group.settings = msg.settings;
      if (group.retry !== null) clearTimeout(group.retry);
      group.retry = null;
      group.failures = 0;
      this.#routes.set(msg.route, group);
      // Install route before activation lets any data leave the relay.
      this.#send?.({ t: "topic_activate", route: msg.route, gen: msg.gen });
      if (group.status.phase === "interrupted") this.#recovering = this.#wasReady;
      this.#transition(group, { phase: "ready" });
      for (const handle of group.handles) if (handle.group === group) handle.applied();
      this.#restored();
    } else if (msg.t === "topic_applied") this.#applied(msg);
    else if (msg.t === "topic_error") {
      const error = new TopicError(msg.code, msg.outcome, msg.message);
      this.#rejectList(msg.id, error);
      const group = this.#ids.get(msg.id) ?? this.#routes.get(msg.id);
      if (group !== undefined && msg.revision !== undefined) {
        // One rejected change; the subscription and its settings stand.
        const pending = group.pending;
        if (pending !== null && pending.revision === msg.revision) {
          group.pending = null;
          pending.reject(error);
          this.#next(group);
        }
      } else if (group !== undefined) {
        if (msg.outcome === "interrupted") this.#interrupt(group, error);
        else {
          if (group.route !== null) this.#routes.delete(group.route);
          group.route = null;
          group.sent = false;
          this.#credits.delete(group);
          this.#failUpdates(group, error);
          this.#transition(group, { phase: "rejected", error });
          if (["topic_ready_timeout", "topic_limit", "topic_route_limit"].includes(error.code)) {
            this.#retry(group);
          }
        }
        this.#restored();
      }
    } else if (msg.t === "topic_page") this.#page(msg);
    else if (msg.t === "topic_pub_result") this.#publicationResult(msg);
  }

  publish(
    key: string,
    payload: Uint8Array,
    options: TopicPublishOptions = {},
  ): Promise<TopicPublishReceipt> {
    if (!this.#negotiated || this.#sendBinary === null) {
      return Promise.reject(new TopicError("not_connected", "rejected"));
    }
    let info: TopicSourceInfo;
    try {
      info = this.#target(options.source, "write");
    } catch (error) {
      return Promise.reject(error);
    }
    const id = this.#id();
    const meta = topicPublishMeta({
      id,
      source: info.source,
      gen: info.gen,
      key,
      encoding: options.encoding ?? "application/octet-stream",
    });
    if (
      meta === null || !(payload instanceof Uint8Array) ||
      payload.byteLength > MAX_TOPIC_PAYLOAD_BYTES
    ) {
      return Promise.reject(new TopicError("invalid_topic_publication", "rejected"));
    }
    if (
      this.#publications.size >= MAX_TOPIC_PUBLISH_PENDING ||
      this.#publicationBytes + payload.byteLength + 2048 > MAX_TOPIC_PUBLISH_BYTES
    ) {
      return Promise.reject(new TopicError("topic_publish_limit", "rejected"));
    }
    // Encode once before yielding; caller mutations cannot change a queued put.
    const frame = encodeDataFrame({
      ch: TOPIC_PUBLISH_CHANNEL,
      seq: this.#counter,
      ts: Date.now() / 1000,
      delivery: "reliable",
      meta,
    }, payload);
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        const disconnect = this.#disconnect;
        this.disconnected(); // reject every pending put as unknown; never replay it
        disconnect?.();
      }, 15_000);
      this.#publications.set(id, {
        source: info.source,
        gen: info.gen,
        bytes: frame.byteLength,
        resolve,
        reject,
        timer,
      });
      this.#publicationBytes += frame.byteLength;
      this.#sendBinary!(frame).catch(() => {
        const disconnect = this.#disconnect;
        this.disconnected();
        disconnect?.();
      });
    });
  }

  #publicationResult(msg: TopicPubResultMsg): void {
    const pending = this.#publications.get(msg.id);
    if (pending === undefined) return;
    this.#publications.delete(msg.id);
    this.#publicationBytes -= pending.bytes;
    clearTimeout(pending.timer);
    if (msg.outcome === "accepted") pending.resolve({ id: msg.id, providerTs: msg.providerTs });
    else pending.reject(new TopicError(msg.code, msg.outcome, msg.message));
  }
  #page(msg: TopicPageMsg): void {
    const pending = this.#lists.get(msg.id);
    if (
      pending === undefined || pending.source !== msg.source || pending.gen !== msg.gen ||
      msg.entries.length > pending.limit
    ) return;
    this.#lists.delete(msg.id);
    clearTimeout(pending.timer);
    pending.resolve({
      source: msg.source,
      generation: msg.gen,
      entries: msg.entries,
      cursor: msg.cursor,
    });
  }
  ingest(frame: DataFrame): void {
    const group = this.#routes.get(frame.header.ch);
    const meta = topicSampleMeta(frame.header.meta);
    if (
      group === undefined || group.status.phase !== "ready" || meta === null ||
      meta.gen !== group.gen ||
      frame.header.delivery !== group.identity.delivery ||
      frame.payload.byteLength > MAX_TOPIC_PAYLOAD_BYTES
    ) return;
    const previous = group.keys.get(meta.key);
    if (previous !== undefined && frame.header.seq <= previous) return;
    const keyLimit = !group.keys.has(meta.key) && group.keys.size >= MAX_TOPIC_KEYS;
    if (
      keyLimit ||
      (group.identity.delivery === "reliable" && group.seq !== null &&
        frame.header.seq !== group.seq + 1)
    ) {
      this.#interrupt(
        group,
        new TopicError(keyLimit ? "topic_key_limit" : "topic_order_lost", "interrupted"),
      );
      return;
    }
    group.keys.set(meta.key, frame.header.seq);
    group.seq = frame.header.seq;
    this.#adaptation.sample(group.adaptation, meta.ingressTs, frame.payload.byteLength);
    const sample: TopicSample = {
      payload: frame.payload,
      key: meta.key,
      encoding: meta.encoding,
      source: group.source!,
      generation: group.gen,
      sequence: frame.header.seq,
      ingressTs: meta.ingressTs,
      ...(meta.sourceTs === undefined ? {} : { sourceTs: meta.sourceTs }),
      sentTs: frame.header.ts,
      metadata: meta,
    };
    for (const handle of [...group.handles]) {
      if (handle.group === group) notify([handle.callback], sample);
    }
    if (group.identity.delivery === "reliable" && group.route !== null) {
      group.frames++;
      // Re-serializing a parsed header changes valid wire spellings (30.0,
      // whitespace, escaped Unicode) and would invent incorrect byte credits.
      // Only synthetic in-process frames need the canonical-size fallback.
      group.bytes += frame.byteLength ??
        (8 + new TextEncoder().encode(JSON.stringify(frame.header)).byteLength +
          frame.payload.byteLength);
      // Receipts are cumulative: one bounded timer covers an event burst,
      // while callbacks still run immediately for every sample.
      this.#credits.add(group);
      if (this.#creditTimer === null) {
        this.#creditTimer = setTimeout(() => {
          this.#creditTimer = null;
          for (const pending of this.#credits) {
            if (pending.route !== null) {
              this.#send?.({
                t: "topic_credit",
                route: pending.route,
                gen: pending.gen,
                frames: pending.frames,
                bytes: pending.bytes,
              });
            }
          }
          this.#credits.clear();
        }, 5);
      }
    }
  }

  streamClosed(route: string): void {
    const group = this.#routes.get(route);
    if (group?.identity.delivery !== "reliable" || group.status.phase !== "ready") return;
    this.#interrupt(group, new TopicError("topic_stream_closed", "interrupted"));
  }
}
