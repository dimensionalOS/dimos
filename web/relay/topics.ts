// Dynamic routing is deliberately independent of the robot registry and its
// manifest/teleop machinery. Every lookup starts from the authenticated peer.
// Access is relay-wide: with `open` every viewer reads and writes every
// connected source (within the provider's own capabilities); without it no
// viewer has topic access and session.ts turns providers away.
import {
  encodeDataFrame,
  type FrameHeader,
  MAX_TOPIC_HANDLES,
  MAX_TOPIC_KEYS,
  MAX_TOPIC_PAYLOAD_BYTES,
  MAX_TOPIC_PUBLISH_BYTES,
  MAX_TOPIC_PUBLISH_PENDING,
  MAX_TOPIC_ROUTES,
  MAX_TOPIC_SOURCES,
  type Msg,
  TOPIC_GUARD_CHANNEL,
  TOPIC_PUBLISH_CHANNEL,
  type TopicCapabilities,
  type TopicErrorMsg,
  type TopicMsg,
  topicPublishMeta,
  type TopicPubResultMsg,
  topicSampleMeta,
  type TopicSourcesMsg,
  type TopicSubMsg,
} from "@dimos/shared";
import type { ViewerSink } from "./forward.ts";
import { TOPIC_QUEUE_BYTES, TopicBudget, TopicEgress } from "./topic_forward.ts";
import { TopicPublishers } from "./topic_publishers.ts";

const MAX_DESIRED = 1024;
const REQUEST_TIMEOUT_MS = 10_000;

export interface TopicProvider {
  sendControl(msg: Msg): void;
  sendTopicPublication(bytes: Uint8Array): void;
  sendTopicGuard(bytes: Uint8Array): void;
  failTopics(reason: string): void;
}
export interface TopicViewer {
  readonly sink: ViewerSink;
  sendMsg(msg: Msg): void;
}
interface ViewerState {
  subs: Map<string, Subscription>;
  budget: TopicBudget;
  reply: (msg: Msg) => void;
}
interface Source {
  peer: TopicProvider;
  id: string;
  gen: number;
  groups: Map<string, Group>;
  closing: boolean;
  capabilities: TopicCapabilities;
}
interface Group {
  source: Source;
  route: string;
  signature: string;
  spec: TopicSubMsg;
  subs: Set<Subscription>;
  opened: boolean;
  active: boolean;
  keys: Map<string, number>;
  seq: number;
  stream: symbol | null;
  maxHz: number;
}
interface Subscription {
  viewer: TopicViewer;
  state: ViewerState;
  spec: TopicSubMsg;
  group: Group | null;
  route: string | null;
  egress: TopicEgress | null;
  deadline: number;
}
interface PendingList {
  viewer: TopicViewer;
  source: Source;
  id: string;
  limit: number;
  deadline: number;
}
interface PendingPublication {
  viewer: TopicViewer;
  source: Source;
  id: string;
  bytes: number;
  deadline: number;
}

export class TopicRegistry {
  #sources = new Map<string, Source>();
  #viewers = new Map<TopicViewer, ViewerState>();
  #groups = new Map<string, Group>();
  #routes = new Map<string, Subscription>();
  #lists = new Map<string, PendingList>();
  #publications = new Map<string, PendingPublication>();
  #generation = 0;
  #counter = 0;
  #desired = 0;
  #budget = new TopicBudget(128 * 1024 * 1024);
  #publishers: TopicPublishers;

  constructor(readonly now: () => number = Date.now, readonly open = true) {
    this.#publishers = new TopicPublishers((prefix) => this.#id(prefix), now);
  }

  #id(prefix: string): string {
    return `${prefix}${++this.#counter}`;
  }

  /** The connected sources with their providers' capabilities; a source that
   * disconnects leaves the list. Empty while the relay keeps topics closed. */
  sources(): TopicSourcesMsg {
    return {
      t: "topic_sources",
      v: 1,
      sources: this.open
        ? [...this.#sources.values()].map((s) => ({ source: s.id, gen: s.gen, ...s.capabilities }))
        : [],
    };
  }

  addViewer(viewer: TopicViewer, reply: (msg: Msg) => void): void {
    if (!this.#viewers.has(viewer)) {
      this.#viewers.set(viewer, {
        subs: new Map(),
        budget: new TopicBudget(TOPIC_QUEUE_BYTES),
        reply,
      });
    }
    reply(this.sources());
  }

  #announce(): void {
    const sources = this.sources();
    for (const state of this.#viewers.values()) state.reply(sources);
  }

  /** Called only after the session verified the provider (its source key,
   * when the relay has an auth file). Returns the rejection code, or null
   * once the source is registered. The count is capped at what one
   * topic_sources message can carry, so every announcement stays valid. */
  registerSource(
    peer: TopicProvider,
    id: string,
    capabilities: TopicCapabilities = { read: true, write: true, profiles: ["raw"] },
  ): "topic_source_conflict" | "topic_source_limit" | null {
    const previous = this.#sources.get(id);
    if (previous !== undefined) return previous.peer === peer ? null : "topic_source_conflict";
    if (this.#sources.size >= MAX_TOPIC_SOURCES) return "topic_source_limit";
    const source: Source = {
      id,
      peer,
      gen: ++this.#generation,
      groups: new Map(),
      closing: false,
      capabilities,
    };
    this.#sources.set(id, source);
    peer.sendControl({
      t: "topic_sources",
      v: 1,
      sources: [{ source: id, gen: source.gen, ...capabilities }],
    });
    this.#announce();
    for (const state of this.#viewers.values()) {
      for (const sub of [...state.subs.values()]) {
        if (sub.spec.source === id) this.#attach(sub, source);
      }
    }
    return null;
  }

  sourceClosed(peer: TopicProvider): void {
    this.#publishers.sourceClosed(peer);
    for (const source of this.#sources.values()) {
      if (source.peer !== peer) continue;
      this.#sources.delete(source.id);
      for (const [token, request] of this.#publications) {
        if (request.source === source) {
          this.#finishPublication(token, "unknown", "source_offline");
        }
      }
      for (const group of [...source.groups.values()]) {
        for (const sub of [...group.subs]) {
          this.#detach(sub);
          this.#error(sub.state.reply, sub.spec.id, "source_offline", "interrupted");
        }
      }
      for (const [token, request] of this.#lists) {
        if (request.source !== source) continue;
        this.#lists.delete(token);
        this.#error(request.viewer.sendMsg.bind(request.viewer), request.id, "source_offline");
      }
    }
    this.#announce();
  }

  viewerClosed(viewer: TopicViewer): void {
    this.#publishers.viewerClosed(viewer);
    const state = this.#viewers.get(viewer);
    if (state === undefined) return;
    for (const sub of [...state.subs.values()]) this.#drop(sub);
    for (const [token, request] of this.#lists) {
      if (request.viewer === viewer) this.#lists.delete(token);
    }
    // Keep sent writes charged until the provider acknowledges or disconnects.
    // Losing a browser cannot cancel a put already in flight.
    this.#viewers.delete(viewer);
  }

  #error(
    reply: (msg: Msg) => void,
    id: string,
    code: string,
    outcome: TopicErrorMsg["outcome"] = "rejected",
  ): void {
    reply({ t: "topic_error", id, code, message: code.replaceAll("_", " "), outcome });
  }

  /** A forged server/provider message has no effect. */
  onViewerMsg(viewer: TopicViewer, msg: TopicMsg, reply: (msg: Msg) => void): void {
    const state = this.#viewers.get(viewer);
    const requestId = "id" in msg ? msg.id : "route" in msg ? msg.route : "topics";
    if (state === undefined) {
      this.#error(reply, requestId, "topic_version_required");
      return;
    }
    if (this.#publishers.viewerMessage(viewer, msg)) return;
    switch (msg.t) {
      case "topic_sub": {
        if (!this.open) {
          this.#error(reply, msg.id, "topic_read_denied");
          return;
        }
        if (state.subs.has(msg.id)) {
          this.#error(reply, msg.id, "duplicate_topic_id");
          return;
        }
        if (state.subs.size >= MAX_TOPIC_HANDLES || this.#desired >= MAX_DESIRED) {
          this.#error(reply, msg.id, "topic_limit");
          return;
        }
        const sub: Subscription = {
          viewer,
          state,
          spec: msg,
          group: null,
          route: null,
          egress: null,
          deadline: 0,
        };
        state.subs.set(msg.id, sub);
        this.#desired++;
        const source = this.#sources.get(msg.source);
        if (source === undefined) this.#error(reply, msg.id, "source_offline", "interrupted");
        else this.#attach(sub, source);
        break;
      }
      case "topic_activate":
      case "topic_credit": {
        const sub = this.#routes.get(msg.route);
        if (sub?.viewer !== viewer || sub.group?.source.gen !== msg.gen || !sub.group.opened) {
          this.#error(reply, requestId, "unknown_topic_handle");
          return;
        }
        if (msg.t === "topic_credit") sub.egress?.credit(msg.frames, msg.bytes);
        else {
          sub.deadline = 0;
          sub.egress?.activate();
          if (!sub.group.active) {
            sub.group.active = true;
            sub.group.source.peer.sendControl({
              t: "topic_activate",
              route: sub.group.route,
              gen: sub.group.source.gen,
              maxHz: sub.group.maxHz,
            });
          }
        }
        break;
      }
      case "topic_unsub": {
        const sub = state.subs.get(msg.id);
        if (sub !== undefined) this.#drop(sub);
        break;
      }
      case "topic_list": {
        if (!this.open) {
          this.#error(reply, msg.id, "topic_read_denied");
          return;
        }
        const source = this.#sources.get(msg.source);
        if (source === undefined) {
          this.#error(reply, msg.id, "source_offline");
          return;
        }
        if (!source.capabilities.read) {
          this.#error(reply, msg.id, "topic_read_denied");
          return;
        }
        const own = [...this.#lists.values()].filter((r) => r.viewer === viewer);
        if (this.#lists.size >= 256 || own.length >= 16 || own.some((r) => r.id === msg.id)) {
          this.#error(reply, msg.id, "topic_request_limit");
          return;
        }
        const token = this.#id("l");
        this.#lists.set(token, {
          viewer,
          source,
          id: msg.id,
          limit: msg.limit,
          deadline: this.now() + REQUEST_TIMEOUT_MS,
        });
        source.peer.sendControl({ ...msg, id: token });
        break;
      }
      default:
        this.#error(reply, requestId, "invalid_topic_operation");
    }
  }

  #attach(sub: Subscription, source: Source): void {
    const spec = sub.spec;
    if (!source.capabilities.read || !source.capabilities.profiles.includes(spec.profile)) {
      this.#error(
        sub.state.reply,
        spec.id,
        source.capabilities.read ? "unsupported_profile" : "topic_read_denied",
      );
      this.#drop(sub);
      return;
    }
    const signature = JSON.stringify([
      spec.keyExpr,
      spec.delivery,
      spec.profile,
    ]);
    let group = source.groups.get(signature);
    const fresh = group === undefined;
    if (group === undefined) {
      if (this.#groups.size >= MAX_TOPIC_ROUTES) {
        this.#error(sub.state.reply, spec.id, "topic_route_limit");
        this.#drop(sub);
        return;
      }
      group = {
        source,
        signature,
        spec,
        route: this.#id("@t:p"),
        subs: new Set(),
        opened: false,
        active: false,
        keys: new Map(),
        seq: 0,
        stream: null,
        maxHz: spec.maxHz,
      };
      source.groups.set(signature, group);
      this.#groups.set(group.route, group);
    }
    sub.group = group;
    sub.route = this.#id("@t:v");
    sub.deadline = this.now() + REQUEST_TIMEOUT_MS;
    this.#routes.set(sub.route, sub);
    group.subs.add(sub);
    this.#updateRate(group);
    sub.egress = new TopicEgress(
      spec.delivery,
      sub.viewer.sink,
      [sub.state.budget, this.#budget],
      (code) => {
        this.#error(sub.state.reply, spec.id, code, "interrupted");
        this.#drop(sub);
      },
      this.now,
      spec.maxHz,
    );
    if (fresh) {
      source.peer.sendControl({
        t: "topic_open",
        route: group.route,
        gen: source.gen,
        keyExpr: spec.keyExpr,
        delivery: spec.delivery,
        profile: spec.profile,
        maxHz: spec.maxHz,
      });
    } else if (group.opened) this.#ready(sub);
  }

  #ready(sub: Subscription): void {
    sub.state.reply({
      t: "topic_ready",
      id: sub.spec.id,
      source: sub.spec.source,
      route: sub.route!,
      gen: sub.group!.source.gen,
    });
  }

  #updateRate(group: Group): void {
    const maxHz = Math.max(...[...group.subs].map((sub) => sub.spec.maxHz));
    if (maxHz === group.maxHz) return;
    group.maxHz = maxHz;
    if (group.active && this.#sources.get(group.source.id) === group.source) {
      group.source.peer.sendControl({
        t: "topic_activate",
        route: group.route,
        gen: group.source.gen,
        maxHz,
      });
    }
  }

  #detach(sub: Subscription): void {
    sub.egress?.dispose();
    sub.egress = null;
    if (sub.route !== null) this.#routes.delete(sub.route);
    sub.route = null;
    sub.deadline = 0;
    const group = sub.group;
    sub.group = null;
    if (group === null) return;
    group.subs.delete(sub);
    if (group.subs.size === 0) {
      this.#groups.delete(group.route);
      group.source.groups.delete(group.signature);
      if (this.#sources.get(group.source.id) === group.source) {
        group.source.peer.sendControl({
          t: "topic_close",
          route: group.route,
          gen: group.source.gen,
        });
      }
    } else this.#updateRate(group);
  }

  #drop(sub: Subscription): void {
    this.#detach(sub);
    if (sub.state.subs.delete(sub.spec.id)) this.#desired--;
  }

  #failGroup(group: Group, code: string): void {
    for (const sub of [...group.subs]) {
      this.#error(sub.state.reply, sub.spec.id, code, "interrupted");
      this.#drop(sub);
    }
  }

  onProviderMsg(peer: TopicProvider, msg: TopicMsg): void {
    this.#publishers.providerMessage(peer, msg);
    if (msg.t === "topic_pub_result") {
      const request = this.#publications.get(msg.id);
      if (request?.source.peer !== peer) return;
      this.#finishPublication(msg.id, msg.outcome, msg.code, msg.message, msg.providerTs);
    } else if (msg.t === "topic_opened") {
      const group = this.#groups.get(msg.route);
      if (group?.source.peer !== peer || group.source.gen !== msg.gen || group.opened) return;
      group.opened = true;
      for (const sub of group.subs) this.#ready(sub);
    } else if (msg.t === "topic_error") {
      const group = this.#groups.get(msg.id);
      if (group?.source.peer === peer) {
        for (const sub of [...group.subs]) {
          this.#error(sub.state.reply, sub.spec.id, msg.code, msg.outcome);
          this.#drop(sub);
        }
      }
      const request = this.#lists.get(msg.id);
      if (request?.source.peer === peer) {
        this.#lists.delete(msg.id);
        this.#error(request.viewer.sendMsg.bind(request.viewer), request.id, msg.code);
      }
    } else if (msg.t === "topic_page") {
      const request = this.#lists.get(msg.id);
      if (
        request?.source.peer !== peer || request.source.gen !== msg.gen ||
        request.source.id !== msg.source || msg.entries.length > request.limit
      ) return;
      this.#lists.delete(msg.id);
      request.viewer.sendMsg({ ...msg, id: request.id });
    }
  }

  #finishPublication(
    token: string,
    outcome: TopicPubResultMsg["outcome"],
    code: string,
    message = code.replaceAll("_", " "),
    providerTs = 0,
  ): void {
    const request = this.#publications.get(token);
    if (request === undefined) return;
    this.#publications.delete(token);
    if (this.#viewers.has(request.viewer)) {
      request.viewer.sendMsg({
        t: "topic_pub_result",
        id: request.id,
        outcome,
        code,
        message,
        providerTs,
      });
    }
  }

  /** Binary writes use a separate ordered stream. Receipts, not successful
   * stream.write(), release the queued plus in-flight publication budget. */
  publish(viewer: TopicViewer, header: FrameHeader, payload: Uint8Array): void {
    if (header.ch === TOPIC_GUARD_CHANNEL) {
      const source =
        this.open && typeof header.meta?.source === "string" && this.#viewers.has(viewer)
          ? this.#sources.get(header.meta.source)
          : undefined;
      this.#publishers.publish(viewer, source, header, payload);
      return;
    }
    const meta = topicPublishMeta(header.meta);
    if (meta === null || header.ch !== TOPIC_PUBLISH_CHANNEL || header.delivery !== "reliable") {
      viewer.sink.kick("invalid_topic_publication");
      return;
    }
    const reject = (code: string) =>
      viewer.sendMsg({
        t: "topic_pub_result",
        id: meta.id,
        outcome: "rejected",
        code,
        message: code.replaceAll("_", " "),
        providerTs: 0,
      });
    if (!this.open || !this.#viewers.has(viewer)) {
      reject("topic_write_denied");
      return;
    }
    const source = this.#sources.get(meta.source);
    if (source === undefined || source.closing) {
      reject("source_offline");
      return;
    }
    if (!source.capabilities.write) {
      reject("topic_write_denied");
      return;
    }
    if (meta.gen !== source.gen) {
      reject("stale_generation");
      return;
    }
    if (payload.byteLength > MAX_TOPIC_PAYLOAD_BYTES) {
      reject("topic_payload_too_large");
      return;
    }
    const pending = [...this.#publications.values()];
    const own = pending.filter((p) => p.viewer === viewer);
    const upstream = pending.filter((p) => p.source === source);
    const bytes = payload.byteLength + 8 +
      new TextEncoder().encode(JSON.stringify(header)).byteLength;
    const total = (requests: PendingPublication[]) =>
      requests.reduce((n, p) => n + p.bytes, 0) + bytes;
    if (own.some((p) => p.id === meta.id)) {
      viewer.sink.kick("duplicate_topic_publication");
      return;
    }
    if (
      own.length >= MAX_TOPIC_PUBLISH_PENDING || upstream.length >= 64 || pending.length >= 256 ||
      total(own) > MAX_TOPIC_PUBLISH_BYTES || total(upstream) > 32 * 1024 * 1024 ||
      total(pending) > 64 * 1024 * 1024
    ) {
      reject("topic_publish_limit");
      return;
    }
    const token = this.#id("p");
    this.#publications.set(token, {
      viewer,
      source,
      id: meta.id,
      bytes,
      deadline: this.now() + REQUEST_TIMEOUT_MS,
    });
    source.peer.sendTopicPublication(encodeDataFrame({
      ...header,
      meta: {
        id: token,
        source: source.id,
        gen: source.gen,
        key: meta.key,
        encoding: meta.encoding,
      },
    }, payload));
  }

  /** Called in arrival order from ONE persistent stream for reliable routes. */
  onFrame(peer: TopicProvider, header: FrameHeader, payload: Uint8Array, stream: symbol): void {
    const group = this.#groups.get(header.ch);
    const meta = topicSampleMeta(header.meta);
    if (
      group?.source.peer !== peer || !group.opened || !group.active || meta === null ||
      meta.gen !== group.source.gen || header.delivery !== group.spec.delivery ||
      !Number.isSafeInteger(header.seq) || header.seq < 1 || !Number.isFinite(header.ts) ||
      payload.byteLength > MAX_TOPIC_PAYLOAD_BYTES
    ) return;
    if (group.spec.delivery === "reliable") {
      if (group.stream !== null && group.stream !== stream || header.seq !== group.seq + 1) {
        this.#failGroup(group, "topic_order_lost");
        return;
      }
      group.stream = stream;
      group.seq = header.seq;
    }
    const previous = group.keys.get(meta.key);
    if (previous !== undefined && header.seq <= previous) return;
    if (previous === undefined && group.keys.size >= MAX_TOPIC_KEYS) {
      this.#failGroup(group, "topic_key_limit");
      return;
    }
    group.keys.set(meta.key, header.seq);
    for (const sub of [...group.subs]) {
      const bytes = encodeDataFrame({ ...header, ch: sub.route! }, payload);
      sub.egress?.offer(meta.key, bytes);
    }
  }

  streamClosed(peer: TopicProvider, stream: symbol): void {
    for (const group of [...this.#groups.values()]) {
      if (group.source.peer === peer && group.stream === stream) {
        this.#failGroup(group, "topic_stream_closed");
      }
    }
  }

  reap(nowMs: number): void {
    this.#publishers.reap(nowMs);
    for (const request of this.#publications.values()) {
      if (nowMs < request.deadline || request.source.closing) continue;
      // A timeout cannot release QUIC buffers while accepting more writes.
      // Tear down the provider connection; sourceClosed settles them unknown.
      request.source.closing = true;
      request.source.peer.failTopics("topic_publish_timeout");
    }
    for (const state of this.#viewers.values()) {
      for (const sub of [...state.subs.values()]) {
        sub.egress?.reap(nowMs);
        if (sub.deadline > 0 && nowMs >= sub.deadline) {
          this.#error(state.reply, sub.spec.id, "topic_ready_timeout");
          this.#drop(sub);
        }
      }
    }
    for (const [token, request] of this.#lists) {
      if (nowMs < request.deadline) continue;
      this.#lists.delete(token);
      this.#error(request.viewer.sendMsg.bind(request.viewer), request.id, "topic_list_timeout");
    }
  }
}
