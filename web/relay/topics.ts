// Dynamic routing is deliberately independent of the robot registry and its
// manifest/teleop machinery. Every lookup starts from the authenticated peer.
// Access is relay-wide: with `open` every viewer reads and writes every
// connected source (within the provider's own capabilities); without it no
// viewer has topic access and session.ts turns providers away.
import {
  encodeDataFrame,
  type FrameHeader,
  MAX_CONTROL_PAYLOAD_BYTES,
  MAX_TOPIC_HANDLES,
  MAX_TOPIC_KEYS,
  MAX_TOPIC_PAYLOAD_BYTES,
  MAX_TOPIC_PUBLISH_BYTES,
  MAX_TOPIC_PUBLISH_PENDING,
  MAX_TOPIC_ROUTES,
  MAX_TOPIC_RTC_CANDIDATES,
  MAX_TOPIC_SOURCES,
  MAX_TOPIC_VARIANTS,
  type Msg,
  normalizeTopicSettings,
  type OptionRanges,
  TOPIC_GUARD_CHANNEL,
  TOPIC_PUBLISH_CHANNEL,
  TOPIC_SETTING_NAMES,
  type TopicCapabilities,
  type TopicErrorMsg,
  type TopicMsg,
  topicPublishMeta,
  type TopicPubResultMsg,
  topicSampleMeta,
  type TopicSettings,
  type TopicSettingsUpdate,
  type TopicSourcesMsg,
  type TopicSubMsg,
} from "@dimos/shared";
import type { ViewerSink } from "./forward.ts";
import { TOPIC_QUEUE_BYTES, TopicBudget, TopicEgress } from "./topic_forward.ts";
import { TopicPublishers } from "./topic_publishers.ts";

const MAX_DESIRED = 1024;
const REQUEST_TIMEOUT_MS = 10_000;
/** A variant nobody uses stays open this long, so a preset toggle does not
 * churn encoders on the provider. */
export const VARIANT_GRACE_MS = 2_000;

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
/** One encoder configuration of a media route. Its identity within the
 * group is `variantKey`; the rate is the highest among the viewers it serves. */
interface Variant {
  id: string;
  key: string;
  settings: TopicSettings;
  ready: boolean;
  /** Set once no subscription uses or targets it; the grace close follows. */
  idleSince: number;
  keys: Map<string, number>;
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
  /** The settings this profile accepts, or null when the route is raw or the
   * provider is older: only the rate can change, and only this relay gates it. */
  ranges: OptionRanges | null;
  variants: Map<string, Variant> | null;
  /** The number in the newest variant id: ids are never reused on a route. */
  lastVariant: number;
  /** The variant opened with the route itself; topic_opened acknowledges it. */
  opening: Variant | null;
}
interface Move {
  variant: Variant;
  revision: number;
  requested: Record<string, unknown>;
  settings: TopicSettings;
  deadline: number;
}
/** One direct video peer: the provider knows it by `token`, the viewer by
 * its subscription id and route. Candidates are counted per direction. */
interface Peer {
  token: string;
  sub: Subscription;
  /** The variant the peer is fed from; a move to another closes it. */
  variant: Variant | null;
  /** Answer deadline; 0 once the provider answered. */
  deadline: number;
  fromViewer: number;
  fromProvider: number;
}
interface Subscription {
  viewer: TopicViewer;
  state: ViewerState;
  spec: TopicSubMsg;
  group: Group | null;
  route: string | null;
  egress: TopicEgress | null;
  deadline: number;
  /** The viewer's merged request; normalized against the provider's ranges. */
  requested: Record<string, unknown>;
  settings: TopicSettings;
  /** The variant whose frames this viewer receives; null until ready. */
  variant: Variant | null;
  /** A move in progress: the viewer keeps its variant until the target is ready. */
  target: Move | null;
  /** The direct video peer of a video route, one per subscription. */
  peer: Peer | null;
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

const isVideo = (profile: string) => profile.startsWith("video.");

/** What distinguishes variants of one route: the canonical encode settings.
 * The relay gates each viewer's rate on data frames, but a video peer takes
 * every packet of its encoder, and H.264 P-frames cannot be dropped per peer,
 * so there the rate is part of the encoder too. */
function variantKey(settings: TopicSettings, profile: string): string {
  const encode = settings as unknown as Record<string, unknown>;
  const names = isVideo(profile)
    ? TOPIC_SETTING_NAMES
    : TOPIC_SETTING_NAMES.filter((name) => name !== "maxHz");
  return JSON.stringify(names.map((name) => encode[name] ?? null));
}

export class TopicRegistry {
  #sources = new Map<string, Source>();
  #viewers = new Map<TopicViewer, ViewerState>();
  #groups = new Map<string, Group>();
  #routes = new Map<string, Subscription>();
  #lists = new Map<string, PendingList>();
  #publications = new Map<string, PendingPublication>();
  #peers = new Map<string, Peer>();
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
    return this.#announcement(this.open ? [...this.#sources.values()] : []);
  }

  #announcement(sources: Source[]): TopicSourcesMsg {
    return {
      t: "topic_sources",
      v: 1,
      sources: sources.map((s) => ({ source: s.id, gen: s.gen, ...s.capabilities })),
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

  /** Congestion resets for a viewer that negotiated topics. */
  load(viewer: TopicViewer): { aborted: number } | undefined {
    const state = this.#viewers.get(viewer);
    return state === undefined ? undefined : { aborted: state.budget.aborted };
  }

  #announce(): void {
    const sources = this.sources();
    for (const state of this.#viewers.values()) state.reply(sources);
  }

  /** Called only after the session verified the provider (its source key,
   * when the relay has an auth file). Returns the rejection code, or null
   * once the source is registered. Sources are capped at what one
   * topic_sources message can carry, in count and in control frame bytes,
   * so every announcement stays valid. */
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
      gen: this.#generation + 1,
      groups: new Map(),
      closing: false,
      capabilities,
    };
    const announced = this.#announcement([...this.#sources.values(), source]);
    if (
      new TextEncoder().encode(JSON.stringify(announced)).byteLength > MAX_CONTROL_PAYLOAD_BYTES
    ) return "topic_source_limit";
    this.#generation = source.gen;
    this.#sources.set(id, source);
    peer.sendControl(this.#announcement([source]));
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
    revision?: number,
  ): void {
    reply({
      t: "topic_error",
      id,
      ...(revision === undefined ? {} : { revision }),
      code,
      message: code.replaceAll("_", " "),
      outcome,
    });
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
        const requested = { maxHz: msg.maxHz, ...msg.settings };
        const sub: Subscription = {
          viewer,
          state,
          spec: msg,
          group: null,
          route: null,
          egress: null,
          deadline: 0,
          requested,
          settings: { maxHz: msg.maxHz },
          variant: null,
          target: null,
          peer: null,
        };
        state.subs.set(msg.id, sub);
        this.#desired++;
        const source = this.#sources.get(msg.source);
        if (source === undefined) this.#error(reply, msg.id, "source_offline", "interrupted");
        else this.#attach(sub, source);
        break;
      }
      case "topic_update": {
        const sub = state.subs.get(msg.id);
        if (sub === undefined) {
          this.#error(reply, msg.id, "unknown_topic_handle", "rejected", msg.revision);
          return;
        }
        this.#update(sub, msg.revision, msg.options);
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
              // Variants carry their own rates; a rate-only route takes the group's.
              ...(sub.group.variants === null ? { maxHz: sub.group.maxHz } : {}),
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
      case "topic_rtc_offer":
      case "topic_rtc_candidate":
      case "topic_rtc_close": {
        // The same binding as topic_activate: this viewer's route, on the
        // source generation it was readied with, plus the subscription id.
        const sub = this.#routes.get(msg.route);
        if (
          sub?.viewer !== viewer || sub.spec.id !== msg.id || sub.group?.source.gen !== msg.gen ||
          !sub.group.opened
        ) {
          this.#error(reply, msg.id, "unknown_topic_handle");
          return;
        }
        const group = sub.group;
        if (msg.t === "topic_rtc_offer") {
          if (sub.peer !== null && sub.peer.deadline > 0) {
            this.#error(reply, msg.id, "rtc_offer_pending");
            return;
          }
          // A new offer replaces a live peer: the SDK offers again after a failure.
          if (sub.peer !== null) this.#closePeer(sub.peer, "replaced", "provider");
          const peer: Peer = {
            token: this.#id("r"),
            sub,
            variant: sub.variant,
            deadline: this.now() + REQUEST_TIMEOUT_MS,
            fromViewer: 0,
            fromProvider: 0,
          };
          sub.peer = peer;
          this.#peers.set(peer.token, peer);
          group.source.peer.sendControl({
            t: "topic_rtc_offer",
            id: peer.token,
            route: group.route,
            gen: group.source.gen,
            sdp: msg.sdp,
            ...(sub.variant === null ? {} : { variant: sub.variant.id }),
          });
        } else if (msg.t === "topic_rtc_candidate") {
          const peer = sub.peer;
          if (peer === null) return; // late trickle after the peer ended
          if (++peer.fromViewer > MAX_TOPIC_RTC_CANDIDATES) {
            this.#closePeer(peer, "rtc_candidate_limit", "both");
            return;
          }
          group.source.peer.sendControl({
            t: "topic_rtc_candidate",
            id: peer.token,
            route: group.route,
            gen: group.source.gen,
            candidate: msg.candidate,
            mid: msg.mid,
          });
        } else if (sub.peer !== null) {
          this.#closePeer(sub.peer, msg.reason, "provider");
        }
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
    // Raw routes have no encoder, and an older provider advertises no ranges:
    // both keep the rate as their only setting, gated here per viewer.
    const ranges = spec.profile === "raw"
      ? null
      : source.capabilities.profileOptions?.[spec.profile] ?? null;
    const result = normalizeTopicSettings(ranges ?? {}, sub.requested);
    if (result.error !== undefined) {
      this.#error(sub.state.reply, spec.id, result.error);
      this.#drop(sub);
      return;
    }
    const settings = result.settings;
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
        maxHz: settings.maxHz,
        ranges,
        variants: ranges === null ? null : new Map(),
        lastVariant: 0,
        opening: null,
      };
      source.groups.set(signature, group);
      this.#groups.set(group.route, group);
    }
    sub.group = group;
    sub.route = this.#id("@t:v");
    sub.deadline = this.now() + REQUEST_TIMEOUT_MS;
    sub.settings = settings;
    this.#routes.set(sub.route, sub);
    group.subs.add(sub);
    sub.egress = new TopicEgress(
      spec.delivery,
      sub.viewer.sink,
      [sub.state.budget, this.#budget],
      (code) => {
        this.#error(sub.state.reply, spec.id, code, "interrupted");
        this.#drop(sub);
      },
      this.now,
      settings.maxHz,
    );
    let variant: Variant | null = null;
    if (group.variants !== null) {
      const found = this.#variantFor(group, settings);
      if (found === null) {
        this.#error(sub.state.reply, spec.id, "variant_limit");
        this.#drop(sub);
        return;
      }
      variant = found.variant;
      sub.target = { variant, revision: 0, requested: sub.requested, settings, deadline: 0 };
      if (fresh) group.opening = variant;
      else if (found.created) this.#announceVariant(group, variant);
    }
    if (fresh) {
      source.peer.sendControl({
        t: "topic_open",
        route: group.route,
        gen: source.gen,
        keyExpr: spec.keyExpr,
        delivery: spec.delivery,
        profile: spec.profile,
        maxHz: settings.maxHz,
        ...(variant === null ? {} : { settings: variant.settings }),
      });
    }
    this.#updateRate(group);
    if (!fresh && group.opened) {
      if (variant === null) this.#ready(sub);
      else if (variant.ready) this.#move(sub);
    }
  }

  /** The variant of `group` encoding `settings`, reusing an identical one,
   * which then is in use again. A full group first retires a variant in its
   * grace period. */
  #variantFor(
    group: Group,
    settings: TopicSettings,
  ): { variant: Variant; created: boolean } | null {
    const variants = group.variants!;
    const key = variantKey(settings, group.spec.profile);
    for (const variant of variants.values()) {
      if (variant.key === key) {
        variant.idleSince = 0;
        return { variant, created: false };
      }
    }
    if (variants.size >= MAX_TOPIC_VARIANTS) {
      const idle = [...variants.values()].find((variant) => variant.idleSince > 0);
      if (idle === undefined) return null;
      this.#closeVariant(group, idle);
    }
    const variant: Variant = {
      id: `v${++group.lastVariant}`,
      key,
      settings: { ...settings },
      ready: false,
      idleSince: 0,
      keys: new Map(),
    };
    variants.set(variant.id, variant);
    return { variant, created: true };
  }

  #announceVariant(group: Group, variant: Variant): void {
    if (this.#sources.get(group.source.id) !== group.source) return;
    group.source.peer.sendControl({
      t: "topic_variant",
      route: group.route,
      gen: group.source.gen,
      variant: variant.id,
      settings: variant.settings,
    });
  }

  #closeVariant(group: Group, variant: Variant): void {
    group.variants!.delete(variant.id);
    if (group.opening === variant) group.opening = null;
    if (this.#sources.get(group.source.id) === group.source) {
      group.source.peer.sendControl({
        t: "topic_variant_close",
        route: group.route,
        gen: group.source.gen,
        variant: variant.id,
      });
    }
  }

  /** Mark a variant idle once nothing uses or targets it. */
  #idle(group: Group, variant: Variant | null | undefined): void {
    if (variant === null || variant === undefined || variant.idleSince > 0) return;
    for (const sub of group.subs) {
      if (sub.variant === variant || sub.target?.variant === variant) return;
    }
    variant.idleSince = this.now();
  }

  #update(sub: Subscription, revision: number, options: TopicSettingsUpdate): void {
    const group = sub.group;
    const reject = (code: string) =>
      this.#error(sub.state.reply, sub.spec.id, code, "rejected", revision);
    if (group === null) {
      reject("source_offline");
      return;
    }
    // A change still waiting for its variant is the base: two changes to
    // different settings both apply.
    const requested = { ...(sub.target?.requested ?? sub.requested) };
    for (const [name, value] of Object.entries(options)) {
      if (value === null) delete requested[name];
      else requested[name] = value;
    }
    const result = normalizeTopicSettings(group.ranges ?? {}, requested);
    if (result.error !== undefined) {
      reject(result.error);
      return;
    }
    const settings = result.settings;
    if (group.variants === null) {
      // Rate-only: this relay owns the gate, so the change is in force now.
      sub.requested = requested;
      sub.settings = settings;
      sub.egress?.setRate(settings.maxHz);
      this.#updateRate(group);
      sub.state.reply({ t: "topic_applied", id: sub.spec.id, revision, settings });
      return;
    }
    const found = this.#variantFor(group, settings);
    if (found === null) {
      reject("variant_limit");
      return;
    }
    const previous = sub.target?.variant;
    sub.target = {
      variant: found.variant,
      revision,
      requested,
      settings,
      deadline: this.now() + REQUEST_TIMEOUT_MS,
    };
    if (previous !== found.variant) this.#idle(group, previous);
    if (found.created) this.#announceVariant(group, found.variant);
    this.#updateRate(group);
    // The viewer keeps its old variant until the new one encodes: no gap.
    if (found.variant.ready) this.#move(sub);
  }

  /** Switch a subscription to its target variant, now that it encodes. */
  #move(sub: Subscription): void {
    const group = sub.group!;
    const { variant, revision, requested, settings } = sub.target!;
    const previous = sub.variant;
    sub.target = null;
    sub.variant = variant;
    // A video peer is fed from one encoder; the viewer offers again for the new one.
    if (sub.peer !== null && sub.peer.variant !== variant) {
      this.#closePeer(sub.peer, "settings_changed", "both");
    }
    sub.requested = requested;
    sub.settings = settings;
    // Frames still queued were encoded by the old variant; the
    // acknowledgement promises the new settings from here on.
    if (previous !== null) sub.egress?.discard();
    sub.egress?.setRate(settings.maxHz);
    if (previous === null) this.#ready(sub);
    else this.#idle(group, previous);
    // The old variant now samples for whoever remains on it.
    this.#updateRate(group);
    if (previous !== null || revision > 0) {
      sub.state.reply({ t: "topic_applied", id: sub.spec.id, revision, settings });
    }
  }

  #ready(sub: Subscription): void {
    sub.state.reply({
      t: "topic_ready",
      id: sub.spec.id,
      source: sub.spec.source,
      route: sub.route!,
      gen: sub.group!.source.gen,
      settings: sub.settings,
    });
  }

  /** Each variant samples at the highest rate among the viewers it serves or
   * will serve; a rate-only route does the same for its one mailbox. */
  #updateRate(group: Group): void {
    const rateOf = (sub: Subscription, variant: Variant | null) =>
      sub.target !== null && sub.target.variant === variant
        ? sub.target.settings.maxHz
        : sub.settings.maxHz;
    if (group.variants === null) {
      const maxHz = Math.max(...[...group.subs].map((sub) => rateOf(sub, null)));
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
      return;
    }
    for (const variant of group.variants.values()) {
      const rates = [...group.subs]
        .filter((sub) => sub.variant === variant || sub.target?.variant === variant)
        .map((sub) => rateOf(sub, variant));
      if (rates.length === 0) continue;
      const maxHz = Math.max(...rates);
      if (maxHz === variant.settings.maxHz) continue;
      variant.settings = { ...variant.settings, maxHz };
      this.#announceVariant(group, variant);
    }
  }

  /** End a peer on the sides that do not already know. */
  #closePeer(peer: Peer, reason: string, notify: "viewer" | "provider" | "both"): void {
    this.#peers.delete(peer.token);
    if (peer.sub.peer === peer) peer.sub.peer = null;
    const group = peer.sub.group;
    if (group === null) return;
    const gen = group.source.gen;
    if (notify !== "viewer" && this.#sources.get(group.source.id) === group.source) {
      group.source.peer.sendControl({
        t: "topic_rtc_close",
        id: peer.token,
        route: group.route,
        gen,
        reason,
      });
    }
    if (notify !== "provider" && peer.sub.route !== null) {
      peer.sub.state.reply({
        t: "topic_rtc_close",
        id: peer.sub.spec.id,
        route: peer.sub.route,
        gen,
        reason,
      });
    }
  }

  #detach(sub: Subscription): void {
    if (sub.peer !== null) this.#closePeer(sub.peer, "route_closed", "provider");
    sub.egress?.dispose();
    sub.egress = null;
    if (sub.route !== null) this.#routes.delete(sub.route);
    sub.route = null;
    sub.deadline = 0;
    const group = sub.group;
    const variant = sub.variant;
    const target = sub.target?.variant;
    sub.group = null;
    sub.variant = null;
    sub.target = null;
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
    } else {
      this.#idle(group, variant);
      this.#idle(group, target);
      this.#updateRate(group);
    }
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
      const variant = group.opening;
      group.opening = null;
      if (variant !== null) variant.ready = true;
      for (const sub of [...group.subs]) {
        if (group.variants === null) this.#ready(sub);
        // The route's own variant, and any the provider acknowledged first.
        else if (sub.target?.variant.ready) this.#move(sub);
      }
    } else if (msg.t === "topic_variant_ready") {
      const group = this.#groups.get(msg.route);
      if (group?.source.peer !== peer || group.source.gen !== msg.gen) return;
      const variant = group.variants?.get(msg.variant);
      if (variant === undefined) return; // a late acknowledgement of a retired variant
      variant.ready = true;
      // Before topic_opened, viewers wait for the route itself.
      if (!group.opened) return;
      for (const sub of [...group.subs]) {
        if (sub.target?.variant === variant) this.#move(sub);
      }
    } else if (
      msg.t === "topic_rtc_answer" || msg.t === "topic_rtc_candidate" || msg.t === "topic_rtc_close"
    ) {
      const found = this.#peers.get(msg.id);
      const group = found?.sub.group;
      if (
        found === undefined || group === null || group === undefined ||
        group.source.peer !== peer ||
        group.source.gen !== msg.gen || group.route !== msg.route || found.sub.route === null
      ) return;
      if (msg.t === "topic_rtc_close") {
        this.#closePeer(found, msg.reason, "viewer");
        return;
      }
      if (msg.t === "topic_rtc_answer") found.deadline = 0;
      else if (++found.fromProvider > MAX_TOPIC_RTC_CANDIDATES) {
        this.#closePeer(found, "rtc_candidate_limit", "both");
        return;
      }
      found.sub.state.reply({ ...msg, id: found.sub.spec.id, route: found.sub.route });
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
    // A video route carries its media on direct peers, never as data frames.
    if (
      group?.source.peer !== peer || !group.opened || !group.active || meta === null ||
      isVideo(group.spec.profile) ||
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
    // A media route's frames belong to one variant each; one from a retired
    // variant is dropped. Sequence numbers are shared across variants, so the
    // per-key order check is kept per variant.
    const variant = group.variants === null
      ? null
      : meta.variant === undefined
      ? undefined
      : group.variants.get(meta.variant);
    if (variant === undefined) return;
    const keys = variant === null ? group.keys : variant.keys;
    const previous = keys.get(meta.key);
    if (previous !== undefined && header.seq <= previous) return;
    if (previous === undefined && keys.size >= MAX_TOPIC_KEYS) {
      this.#failGroup(group, "topic_key_limit");
      return;
    }
    keys.set(meta.key, header.seq);
    let forwarded = header;
    if (meta.variant !== undefined) {
      const { variant: _, ...rest } = meta;
      forwarded = { ...header, meta: rest };
    }
    for (const sub of [...group.subs]) {
      if (variant !== null && sub.variant !== variant) continue;
      const bytes = encodeDataFrame({ ...forwarded, ch: sub.route! }, payload);
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
        } else if (sub.target !== null && sub.target.deadline > 0 && nowMs >= sub.target.deadline) {
          // A move the provider never acknowledged: the viewer keeps what it has.
          const { variant, revision } = sub.target;
          sub.target = null;
          this.#error(state.reply, sub.spec.id, "topic_ready_timeout", "rejected", revision);
          this.#idle(sub.group!, variant);
          this.#updateRate(sub.group!);
        }
      }
    }
    for (const peer of [...this.#peers.values()]) {
      if (peer.deadline > 0 && nowMs >= peer.deadline) {
        this.#closePeer(peer, "rtc_answer_timeout", "both");
      }
    }
    for (const group of this.#groups.values()) {
      if (group.variants === null) continue;
      for (const variant of [...group.variants.values()]) {
        if (variant.idleSince > 0 && nowMs - variant.idleSince >= VARIANT_GRACE_MS) {
          this.#closeVariant(group, variant);
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
