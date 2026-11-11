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
  TOPIC_PUBLISH_CHANNEL,
  type TopicDescriptor,
  type TopicMsg,
  topicMsgFromUnknown,
  type TopicPageMsg,
  topicPublishMeta,
  type TopicPubResultMsg,
  topicSampleMeta,
  type TopicSourceInfo,
  type TopicSubMsg,
} from "@dimos/shared";
import {
  type TopicPublisher,
  type TopicPublisherOptions,
  TopicPublishers,
} from "./topicPublishers.ts";

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
export interface TopicOptions {
  source?: string;
  delivery?: Delivery;
  profile?: string;
  /** Latest sampling rate. Reliable event streams preserve every sample. */
  maxHz?: number;
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
  update(options: TopicOptions): Promise<void>;
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
type ResolvedOptions = Required<Omit<TopicOptions, "source">> & { source?: string };
interface Group {
  id: string;
  signature: string;
  keyExpr: string;
  options: ResolvedOptions;
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
}

class Handle implements TopicSubscription {
  group: Group | null = null;
  #status: TopicStatus = { phase: "connecting" };
  #wait = deferred();
  #listeners = new Set<(status: TopicStatus) => void>();
  constructor(readonly owner: TopicClient, readonly callback: (sample: TopicSample) => void) {}
  ready(): Promise<void> {
    return this.#wait.promise;
  }
  status(): TopicStatus {
    return this.#status;
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
  update(options: TopicOptions): Promise<void> {
    if (this.group === null) return Promise.reject(new TopicError("closed", "rejected"));
    const old = this.group;
    this.owner.release(this);
    this.transition({ phase: "connecting" });
    this.owner.acquire(this, old.keyExpr, { ...old.options, ...options });
    return this.ready();
  }
  close(): void {
    this.owner.release(this);
    this.transition({ phase: "closed" });
    this.#listeners.clear();
  }
}

/** Per-message callbacks and routing live outside ChannelStore's UI tick. */
export class TopicClient implements Topics {
  #groups = new Map<string, Group>();
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

  constructor(readonly defaultSource?: string) {}
  publisher(key: string, safeStop: Uint8Array, options: TopicPublisherOptions): TopicPublisher {
    return this.#publishers.publisher(key, safeStop, options);
  }
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
  acquire(handle: Handle, keyExpr: string, options: TopicOptions): void {
    if (this.#closed || this.#handles >= MAX_TOPIC_HANDLES) {
      handle.transition({
        phase: "rejected",
        error: new TopicError(this.#closed ? "closed" : "topic_limit", "rejected"),
      });
      return;
    }
    const resolved: ResolvedOptions = {
      source: options.source ?? this.defaultSource,
      delivery: options.delivery ?? "latest",
      profile: options.profile ?? "raw",
      maxHz: options.maxHz ?? (options.delivery === "reliable" ? 1000 : 30),
    };
    const candidate = {
      t: "topic_sub",
      id: "validate",
      source: resolved.source ?? "auto",
      keyExpr,
      delivery: resolved.delivery,
      profile: resolved.profile,
      maxHz: resolved.maxHz,
    };
    if (
      Object.keys(options).some((key) =>
        !["source", "delivery", "profile", "maxHz"].includes(key)
      ) ||
      topicMsgFromUnknown(candidate) === null
    ) {
      handle.transition({
        phase: "rejected",
        error: new TopicError("invalid_topic_options", "rejected"),
      });
      return;
    }
    const signature = JSON.stringify([
      resolved.source,
      keyExpr,
      resolved.delivery,
      resolved.profile,
      resolved.maxHz,
    ]);
    let group = this.#groups.get(signature);
    if (group === undefined) {
      group = {
        id: this.#id(),
        signature,
        keyExpr,
        options: resolved,
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
      };
      this.#groups.set(signature, group);
      this.#ids.set(group.id, group);
    }
    group.handles.add(handle);
    handle.group = group;
    this.#handles++;
    // A new consumer or explicit update is a fresh attempt, including when
    // another handle still holds a previous rejection of this shared key.
    if (group.status.phase === "rejected") this.#restart(group);
    handle.transition(group.status);
    this.#subscribe(group);
  }
  release(handle: Handle): void {
    const group = handle.group;
    if (group === null) return;
    group.handles.delete(handle);
    handle.group = null;
    this.#handles--;
    if (group.handles.size > 0) return;
    if (group.retry !== null) clearTimeout(group.retry);
    if (group.sent) this.#send?.({ t: "topic_unsub", id: group.id });
    if (group.route !== null) this.#routes.delete(group.route);
    group.route = null;
    this.#credits.delete(group);
    this.#groups.delete(group.signature);
    this.#ids.delete(group.id);
  }
  #transition(group: Group, status: TopicStatus): void {
    group.status = status;
    for (const handle of [...group.handles]) handle.transition(status);
  }
  #restart(group: Group): void {
    if (this.#closed || this.#groups.get(group.signature) !== group) return;
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
    this.#transition(group, { phase: "connecting" });
    this.#subscribe(group);
  }
  #retry(group: Group): void {
    if (
      group.retry !== null || this.#send === null || this.#groups.get(group.signature) !== group
    ) return;
    const delay = Math.min(30_000, 1000 * 2 ** Math.min(group.failures++, 5));
    group.retry = setTimeout(() => this.#restart(group), delay);
  }
  #interrupt(group: Group, error: TopicError): void {
    if (group.route !== null) this.#routes.delete(group.route);
    group.route = null;
    this.#credits.delete(group);
    this.#recovering = this.#wasReady;
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
    const groups = [...this.#groups.values()].filter((group) =>
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
      source = this.#source(group.options.source);
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
    group.sent = true;
    const message: TopicSubMsg = {
      t: "topic_sub",
      id: group.id,
      source,
      keyExpr: group.keyExpr,
      delivery: group.options.delivery,
      profile: group.options.profile,
      maxHz: group.options.maxHz,
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
    for (const group of this.#groups.values()) {
      if (group.retry !== null) clearTimeout(group.retry);
      group.retry = null;
      group.sent = false;
      group.route = null;
      group.keys.clear();
      this.#transition(group, { phase: "interrupted" });
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
    for (const group of [...this.#groups.values()]) {
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
  onMessage(msg: TopicMsg): void {
    if (this.#closed) return;
    if (this.#publishers.message(msg)) return;
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
      for (const group of this.#groups.values()) this.#subscribe(group);
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
      if (group.retry !== null) clearTimeout(group.retry);
      group.retry = null;
      group.failures = 0;
      this.#routes.set(msg.route, group);
      // Install route before activation lets any data leave the relay.
      this.#send?.({ t: "topic_activate", route: msg.route, gen: msg.gen });
      if (group.status.phase === "interrupted") this.#recovering = this.#wasReady;
      this.#transition(group, { phase: "ready" });
      this.#restored();
    } else if (msg.t === "topic_error") {
      const error = new TopicError(msg.code, msg.outcome, msg.message);
      this.#rejectList(msg.id, error);
      const group = this.#ids.get(msg.id) ?? this.#routes.get(msg.id);
      if (group !== undefined) {
        if (msg.outcome === "interrupted") this.#interrupt(group, error);
        else {
          if (group.route !== null) this.#routes.delete(group.route);
          group.route = null;
          group.sent = false;
          this.#credits.delete(group);
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
      frame.header.delivery !== group.options.delivery ||
      frame.payload.byteLength > MAX_TOPIC_PAYLOAD_BYTES
    ) return;
    const previous = group.keys.get(meta.key);
    if (previous !== undefined && frame.header.seq <= previous) return;
    const keyLimit = !group.keys.has(meta.key) && group.keys.size >= MAX_TOPIC_KEYS;
    if (
      keyLimit ||
      (group.options.delivery === "reliable" && group.seq !== null &&
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
    if (group.options.delivery === "reliable" && group.route !== null) {
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
    if (group?.options.delivery !== "reliable" || group.status.phase !== "ready") return;
    this.#interrupt(group, new TopicError("topic_stream_closed", "interrupted"));
  }
}
