// Viewer session: drives the control stream (hello, robots, watch, manifest,
// sub/unsub) and the incoming uni-stream data plane on top of
// ReconnectingTransport, writing everything into the stores. connect() is the
// entry point; one Session lives for the page.

import {
  type ChannelSpec,
  ControlFrameReader,
  type DataFrame,
  DataFrameStreamError,
  DataFrameStreamReader,
  encodeControlFrame,
  encodeDatagram,
  type JsonValue,
  MAX_PUB_DATA_BYTES,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import { type Manifest, ManifestError, parseManifest } from "@dimos/shared/manifest";
import { createDecoderRegistry, type DecoderRegistry } from "./decoders/index.ts";
import { PublishError, WatchRejectedError } from "./errors.ts";
import { registerTeleopHooks, type TeleopHooks } from "./internal/teleopMachine.ts";
import { type ChannelSnapshot, ChannelStore, StatusStore } from "./store.ts";
import {
  fetchRelayInfo,
  ReconnectingTransport,
  resolveInfoUrl,
  type TransportDeps,
  type WebTransportLike,
} from "./transport.ts";

const DEFAULT_UI_TICK_MS = 500;

// Local safety net over the relay's own 10 s publish timeout: a wedged-but-
// alive transport must not leak pending promises forever.
const PUBLISH_LOCAL_TIMEOUT_MS = 20_000;
// Local pending cap; the relay enforces its own (stricter) budgets.
const MAX_PENDING_PUBLISHES = 64;
// Publish values nesting deeper than this are rejected locally; the bridge
// enforces the same cap, so both ends agree on what "too deep" means.
const MAX_PUBLISH_DEPTH = 100;

// Correlated relay codes whose outcome is genuinely unknown: the bridge may
// have published even though no ack made it back.
const UNKNOWN_OUTCOME_CODES = new Set(["publish_timeout", "robot_disconnected"]);

export interface ConnectOptions {
  /** HTTP relay base to resolve /api/info against; default: same origin. */
  url?: string;
  /** Robot id to pin the watch to; default: auto-watch the lone robot. */
  robot?: string;
  /** Decoder registry, captured for the session's lifetime. */
  decoders?: DecoderRegistry;
  /** Period of the UI snapshot tick that feeds subscribe() callbacks. */
  uiTickMs?: number;
}

/** A resolved publish: the bridge decoded the value and called the DimOS
 * output stream. Timestamps are seconds since epoch (relay receive time and
 * bridge publish time). */
export interface PublishReceipt {
  ch: string;
  relayTs: number;
  bridgeTs: number;
}

export interface PublishOptions {
  /** Browser send time forwarded to the bridge's PublishContext. */
  clientTs?: number;
}

export interface Session {
  readonly status: StatusStore;
  readonly store: ChannelStore;
  /**
   * Pin the watch to `robotId`. Resolves once that robot's manifest has been
   * validated and adopted; stays pending while the robot is absent or its
   * manifest is invalid; rejects with WatchRejectedError when superseded by a
   * newer watch() or when the session closes.
   */
  watch(robotId: string): Promise<Manifest>;
  /**
   * Subscribe to an rx channel: one operation registers the UI-tick callback
   * and acquires wire interest (the first consumer subs, the last unsubs).
   * Allowed before a robot or manifest exists; once a manifest is adopted, a
   * missing or non-rx channel surfaces an unknown_channel session error while
   * staying desired for a later manifest.
   */
  subscribe(ch: string, cb: (snapshot: ChannelSnapshot) => void): () => void;
  /**
   * Publish one JSON value on a tx channel declared publish="shared".
   * Resolves once the bridge decoded the value and called the DimOS output
   * stream; rejects with PublishError - outcome "rejected" for definite
   * validation failures, "unknown" when the connection died (or nothing
   * answered) after the send. Never auto-resends: an "unknown" command may
   * well have been published.
   */
  publish(ch: string, value: JsonValue, options?: PublishOptions): Promise<PublishReceipt>;
  close(): void;
}

/** Local auto-select policy: watch the robot only when it is the only one. */
export function pickAutoWatch(robots: RobotInfo[]): RobotInfo | null {
  return robots.length === 1 ? robots[0] : null;
}

/** Random per-session publish-id prefix, so ids from independent sessions
 * (or a reloaded page) cannot collide at the relay's per-viewer dup check. */
function randomIdPrefix(): string {
  const bytes = new Uint8Array(6);
  crypto.getRandomValues(bytes);
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
}

/** First reason `v` is not a publishable JSON value, or null if it is one.
 * Stricter than JSON.stringify on purpose: anything stringify would silently
 * rewrite (non-finite numbers, undefined members, array holes, non-plain
 * objects) is rejected instead. The depth cap bounds recursion (cycles
 * included); messages are short constants that never echo the value. */
function findNonJson(v: unknown, depth: number): string | null {
  if (depth > MAX_PUBLISH_DEPTH) {
    return `value nests deeper than ${MAX_PUBLISH_DEPTH} levels`;
  }
  if (v === null || typeof v === "boolean" || typeof v === "string") return null;
  if (typeof v === "number") {
    return Number.isFinite(v) ? null : "value contains a non-finite number";
  }
  if (Array.isArray(v)) {
    for (const item of v) {
      const bad = findNonJson(item, depth + 1);
      if (bad !== null) return bad;
    }
    return null;
  }
  if (typeof v === "object") {
    const proto = Object.getPrototypeOf(v);
    if (proto !== Object.prototype && proto !== null) {
      return "value contains a non-plain object";
    }
    // Own enumerable string-keyed values: exactly what JSON.stringify emits.
    for (const item of Object.values(v)) {
      const bad = findNonJson(item, depth + 1);
      if (bad !== null) return bad;
    }
    return null;
  }
  return `value contains a non-JSON ${typeof v}`;
}

/** Structural equality over plain-JSON values (params, layout trees). Key
 * order is ignored so a re-serialized but identical manifest does not churn
 * the epoch. */
function jsonEqual(a: unknown, b: unknown): boolean {
  if (a === b) return true;
  if (Array.isArray(a) || Array.isArray(b)) {
    return (
      Array.isArray(a) &&
      Array.isArray(b) &&
      a.length === b.length &&
      a.every((v, i) => jsonEqual(v, b[i]))
    );
  }
  if (typeof a !== "object" || typeof b !== "object" || a === null || b === null) return false;
  const ra = a as Record<string, unknown>;
  const rb = b as Record<string, unknown>;
  const keys = Object.keys(ra);
  return keys.length === Object.keys(rb).length &&
    keys.every((k) => Object.hasOwn(rb, k) && jsonEqual(ra[k], rb[k]));
}

/** True when both manifests describe the same channels (order-insensitive),
 * the same panels (order-sensitive: panel order is display order), and the
 * same layout/pages. Layout/pages differences must compare unequal or a
 * layout-only blueprint edit would render a stale tree (the epoch remount
 * in the cockpit's App.tsx keys off this). */
export function manifestsEqual(a: Manifest, b: Manifest): boolean {
  if (
    a.version !== b.version ||
    a.channels.length !== b.channels.length ||
    a.panels.length !== b.panels.length
  ) {
    return false;
  }
  const key = (c: ChannelSpec) => c.ch;
  const sortedA = [...a.channels].sort((x, y) => key(x).localeCompare(key(y)));
  const sortedB = [...b.channels].sort((x, y) => key(x).localeCompare(key(y)));
  const channelsEqual = sortedA.every((c, i) => {
    const other = sortedB[i];
    return (
      c.ch === other.ch &&
      c.dir === other.dir &&
      c.encoding === other.encoding &&
      c.delivery === other.delivery &&
      c.maxHz === other.maxHz &&
      jsonEqual(c.params, other.params) &&
      c.publish === other.publish &&
      c.requiredScope === other.requiredScope
    );
  });
  const panelsEqual = a.panels.every((p, i) => {
    const other = b.panels[i];
    return (
      p.id === other.id &&
      p.kind === other.kind &&
      p.title === other.title &&
      p.channels.length === other.channels.length &&
      p.channels.every((ch, j) => ch === other.channels[j]) &&
      jsonEqual(p.params, other.params)
    );
  });
  return channelsEqual && panelsEqual && jsonEqual(a.layout, b.layout) &&
    a.pages.length === b.pages.length && a.pages.every((id, i) => id === b.pages[i]);
}

/** What a bare manifest reply (a robot that registered without a manifest)
 * adopts: valid, empty, renders nothing but the status bar. */
export const EMPTY_MANIFEST: Manifest = {
  version: 1,
  channels: [],
  panels: [],
  layout: null,
  pages: [],
};

class SessionImpl implements Session {
  readonly status = new StatusStore();
  readonly store = new ChannelStore();
  readonly transport: ReconnectingTransport;

  #registry: DecoderRegistry;
  #ticker: ReturnType<typeof setInterval>;
  #closed = false;

  // Bumped per connection; data-plane writes from a previous connection's
  // still-draining reader loops are dropped by comparing against it.
  #runId = 0;
  #manifest: Manifest | null = null;

  // Watch state. #pinned is the explicitly requested robot id (from
  // ConnectOptions.robot or watch()); null means auto (watch the lone
  // robot). #watchSentFor is the id the current connection last sent a watch
  // for: the relay drops this viewer's subscriptions when the watch moves to
  // a different robot, so the wire mirror clears with it.
  #pinned: string | null;
  #watchSentFor: string | null = null;
  #watchWaiter:
    | { id: string; resolve: (m: Manifest) => void; reject: (e: Error) => void }
    | null = null;

  // Pending publishes by request id; each settles exactly once - by pub_ack,
  // a correlated error, the local safety timer, its connection's death (the
  // run-loop sweep), or close(). Ids are session-random + counter, bounded.
  #pending = new Map<string, {
    runId: number;
    ch: string;
    resolve: (receipt: PublishReceipt) => void;
    reject: (error: PublishError) => void;
    timer: ReturnType<typeof setTimeout>;
  }>();
  #pubPrefix = randomIdPrefix();
  #pubN = 0;

  // Subscription refcounts. #desired counts live subscribe() consumers per
  // channel and survives reconnects and manifest changes. #wire mirrors the
  // relay-side viewer subscription set: cleared on a new connection and on a
  // watch switch, kept while a robot is merely gone (the relay keeps its
  // side too and force-resyncs a returning robot; see web/relay/registry.ts).
  // #wireRunId is the connection whose manifest adoption armed the wire:
  // subs may only be sent between an adoption and the connection's death.
  #desired = new Map<string, number>();
  #wire = new Set<string>();
  #wireRunId = -1;

  // Per-connection control sender, swapped in #runSession. Fire-and-forget:
  // a send racing a reconnect lands on the dead connection and is dropped
  // (desired subscriptions are replayed after the next adoption).
  #send: ((msg: Msg) => void) | null = null;

  // Per-connection teleop senders, swapped in #runSession. Fire-and-forget:
  // a send racing a reconnect lands on the dead connection and is dropped
  // (the machine repeats at publishHz and the bridge deadman covers loss).
  #teleopControl: ((msg: Msg) => void) | null = null;
  #teleopDatagram: ((msg: Msg) => void) | null = null;
  #teleopCbs = new Set<(msg: Msg) => void>();
  // Off the public Session on purpose (W1 is read-only): the cockpit reaches
  // these through teleopHooks() on the internal teleop entry until W9.
  readonly #teleop: TeleopHooks = {
    control: (msg) => this.#teleopControl?.(msg),
    datagram: (msg) => this.#teleopDatagram?.(msg),
    onMsg: (cb) => {
      this.#teleopCbs.add(cb);
      return () => this.#teleopCbs.delete(cb);
    },
    status: this.status,
  };

  constructor(options: ConnectOptions, deps: TransportDeps) {
    registerTeleopHooks(this, this.#teleop);
    this.#registry = options.decoders ?? createDecoderRegistry();
    this.#pinned = options.robot ?? null;
    const infoUrl = resolveInfoUrl(options.url);
    this.transport = new ReconnectingTransport(
      {
        onPhase: (phase) => this.status.update({ transport: phase }),
        onSession: (wt) => this.#runSession(wt),
      },
      {
        ...deps,
        fetchInfo: deps.fetchInfo ?? ((signal) => fetchRelayInfo(infoUrl, signal)),
      },
    );
    this.#ticker = setInterval(
      () => this.store.publishUi(),
      options.uiTickMs ?? DEFAULT_UI_TICK_MS,
    );
  }

  close(): void {
    if (this.#closed) return;
    this.#closed = true;
    this.#rejectWaiter(new WatchRejectedError("closed"));
    // A publish already sent may have been delivered; close cannot know.
    this.#sweepPendingPublishes(null, "closed", "the session was closed");
    clearInterval(this.#ticker);
    this.transport.stop();
  }

  watch(robotId: string): Promise<Manifest> {
    this.#rejectWaiter(new WatchRejectedError("superseded"));
    this.#pinned = robotId;
    if (this.#closed) return Promise.reject(new WatchRejectedError("closed"));
    const promise = new Promise<Manifest>((resolve, reject) => {
      this.#watchWaiter = { id: robotId, resolve, reject };
    });
    // watch() can arrive between robots pushes: re-resolve the target against
    // the last known list instead of waiting for the next push.
    this.#applyWatchTarget(this.status.get().robots);
    return promise;
  }

  subscribe(ch: string, cb: (snapshot: ChannelSnapshot) => void): () => void {
    // Throwing callbacks are isolated by the store's notification loops.
    const unsubscribeUi = this.store.subscribeUi(ch, () => cb(this.store.getUiSnapshot(ch)));
    const count = this.#desired.get(ch) ?? 0;
    this.#desired.set(ch, count + 1);
    if (count === 0) this.#syncChannel(ch);
    let released = false;
    return () => {
      if (released) return;
      released = true;
      unsubscribeUi();
      this.#release(ch);
    };
  }

  #release(ch: string): void {
    const count = this.#desired.get(ch) ?? 0;
    if (count > 1) {
      this.#desired.set(ch, count - 1);
      return;
    }
    this.#desired.delete(ch);
    if (this.#wire.has(ch)) {
      this.#wire.delete(ch);
      this.#send?.({ t: "unsub", ch });
    }
  }

  publish(ch: string, value: JsonValue, options?: PublishOptions): Promise<PublishReceipt> {
    const rejectLocal = (code: string, message: string) =>
      Promise.reject(new PublishError("rejected", code, message));
    if (this.#closed) return rejectLocal("closed", "the session is closed");
    // The subscription gate: a live connection whose manifest is adopted.
    if (this.#wireRunId !== this.#runId || this.#manifest === null || this.#send === null) {
      return rejectLocal("not_connected", "no adopted manifest on a live connection");
    }
    // Validate, then serialize exactly once: the bytes checked below are the
    // bytes sent (the message carries their parse, so a getter re-reading
    // differently cannot smuggle an unchecked value onto the wire).
    let data: string;
    try {
      const bad = findNonJson(value, 0);
      if (bad !== null) return rejectLocal("not_serializable", bad);
      data = JSON.stringify(value);
    } catch {
      // A property getter threw mid-traversal or mid-stringify.
      return rejectLocal("not_serializable", "value threw while serializing");
    }
    const clientTs = options?.clientTs;
    if (clientTs !== undefined && !Number.isFinite(clientTs)) {
      return rejectLocal("not_serializable", "clientTs must be a finite number");
    }
    const bytes = new TextEncoder().encode(data).byteLength;
    if (bytes > MAX_PUB_DATA_BYTES) {
      return rejectLocal(
        "too_large",
        `serialized data is ${bytes} B (limit ${MAX_PUB_DATA_BYTES})`,
      );
    }
    const spec = this.#manifest.channels.find((c) => c.ch === ch);
    if (spec === undefined) {
      return rejectLocal("unknown_channel", `no channel ${ch} in the adopted manifest`);
    }
    if (spec.publish === "exclusive") {
      return rejectLocal(
        "exclusive_unsupported",
        `channel ${ch} requires the exclusive publisher lease (not supported yet)`,
      );
    }
    if (spec.dir !== "tx" || spec.publish !== "shared") {
      return rejectLocal("not_publishable", `channel ${ch} does not accept generic publish`);
    }
    if (this.#pending.size >= MAX_PENDING_PUBLISHES) {
      return rejectLocal("pending_limit", "too many unacknowledged publishes");
    }
    const id = `${this.#pubPrefix}-${++this.#pubN}`;
    return new Promise<PublishReceipt>((resolve, reject) => {
      const timer = setTimeout(() => {
        // The relay's 10 s publish_timeout normally lands first; this only
        // fires on a wedged-but-alive transport, where nothing is provable.
        this.#settlePub(id)?.reject(
          new PublishError("unknown", "publish_timeout", "no acknowledgement (local timeout)"),
        );
      }, PUBLISH_LOCAL_TIMEOUT_MS);
      this.#pending.set(id, { runId: this.#runId, ch, resolve, reject, timer });
      this.#send!({
        t: "pub",
        id,
        ch,
        data: JSON.parse(data) as JsonValue,
        ...(clientTs !== undefined ? { clientTs } : {}),
      });
    });
  }

  /** Remove one pending publish, its timer cleared; null if already settled. */
  #settlePub(id: string) {
    const pending = this.#pending.get(id);
    if (pending === undefined) return null;
    this.#pending.delete(id);
    clearTimeout(pending.timer);
    return pending;
  }

  /** Reject pending publishes with outcome "unknown" (a sent command may
   * have been delivered): all of them (runId null, close) or one dead
   * connection's (its run-loop exit). Never resends anything. */
  #sweepPendingPublishes(runId: number | null, code: string, message: string): void {
    for (const [id, pending] of [...this.#pending]) {
      if (runId !== null && pending.runId !== runId) continue;
      this.#settlePub(id)?.reject(new PublishError("unknown", code, message));
    }
  }

  #rejectWaiter(error: WatchRejectedError): void {
    const waiter = this.#watchWaiter;
    this.#watchWaiter = null;
    waiter?.reject(error);
  }

  /**
   * First consumer of a channel (or a fresh adoption): send the sub when the
   * current connection has an adopted manifest carrying the channel as rx;
   * surface unknown_channel otherwise. Desired channels stay desired either
   * way, so a later manifest or connection can satisfy them.
   */
  #syncChannel(ch: string): void {
    if (this.#wireRunId !== this.#runId || this.#manifest === null) return;
    if (this.#wire.has(ch)) return;
    if (!this.#channelIsRx(ch)) {
      // One lastError slot: with several missing channels the last one wins.
      this.status.update({
        lastError: {
          code: "unknown_channel",
          ch,
          message: `channel ${ch} is not an rx channel of the adopted manifest`,
        },
      });
      return;
    }
    this.#wire.add(ch);
    this.#send?.({ t: "sub", ch });
  }

  #channelIsRx(ch: string): boolean {
    const spec = this.#manifest?.channels.find((c) => c.ch === ch);
    return spec !== undefined && spec.dir === "rx";
  }

  #syncDesired(): void {
    // A newly adopted manifest can invalidate wired channels (removed, or
    // flipped to tx): drop the relay-side interest while keeping the desire,
    // so the validation below surfaces unknown_channel and a later manifest
    // can re-subscribe them.
    for (const ch of [...this.#wire]) {
      if (!this.#channelIsRx(ch)) {
        this.#wire.delete(ch);
        this.#send?.({ t: "unsub", ch });
      }
    }
    for (const ch of this.#desired.keys()) this.#syncChannel(ch);
  }

  /**
   * Resolve the watch target against `robots`, publish robots + watched in
   * one status update, drop a producer being switched away from, and
   * (re-)send the watch. Robots pushes and watch() both funnel here.
   */
  #applyWatchTarget(robots: RobotInfo[]): void {
    const targetId = this.#pinned ?? pickAutoWatch(robots)?.id ?? null;
    const watched = robots.find((r) => r.id === targetId) ?? null;
    const previous = this.status.get().watchedRobot;
    this.status.update({ robots, watchedRobot: watched });
    if (watched === null) {
      this.#clearProducer();
      return;
    }
    if (previous !== null && previous.id !== watched.id) {
      // Switching producers: drop the old manifest and data before the new
      // robot's frames can land in the old view.
      this.#clearProducer();
    }
    // Unconditional (and idempotent on the relay): a same-id robot restart
    // is announced with the id we already watch, so only a fresh watch
    // re-confirms it, refreshes the manifest, and retries after an
    // unknown_robot race.
    this.#sendWatch(watched.id);
  }

  #sendWatch(id: string): void {
    if (this.#watchSentFor !== null && this.#watchSentFor !== id) {
      // The relay drops this viewer's subscriptions on a watch for a
      // different robot; mirror that so the next adoption re-subscribes the
      // desired set.
      this.#wire.clear();
    }
    this.#watchSentFor = id;
    this.#send?.({ t: "watch", robotId: id });
  }

  async #runSession(wt: WebTransportLike): Promise<void> {
    const runId = ++this.#runId;
    this.#wire.clear();
    this.#watchSentFor = null;
    const control = await wt.createBidirectionalStream();
    const writer = control.writable.getWriter();
    const send = async (msg: Msg) => {
      await writer.write(encodeControlFrame(msg));
    };
    this.#send = (msg) => {
      if (runId === this.#runId) void send(msg).catch(() => {});
    };
    const datagrams = wt.datagrams.writable.getWriter();
    this.#teleopControl = (msg) => {
      if (runId === this.#runId) void send(msg).catch(() => {});
    };
    this.#teleopDatagram = (msg) => {
      if (runId === this.#runId) void datagrams.write(encodeDatagram(msg)).catch(() => {});
    };
    await send({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
    void this.#readUniStreams(wt, runId);

    const reader = control.readable.getReader();
    const frames = new ControlFrameReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        for (const msg of frames.push(value)) {
          switch (msg.t) {
            case "welcome":
              // Session-level success: only now is the page usable, so only
              // now the transport may show "connected".
              this.transport.sessionReady();
              this.status.update({ lastError: null });
              break;
            case "robots":
              this.#applyWatchTarget(msg.robots);
              break;
            case "manifest": {
              // A reply to a watch that raced a robot change must not be
              // adopted: only the currently watched robot's manifest counts.
              if (msg.robotId !== this.status.get().watchedRobot?.id) break;
              let manifest: Manifest;
              if (msg.manifest === undefined) {
                manifest = EMPTY_MANIFEST;
              } else {
                try {
                  // Domain validation (version gate, duplicate/bogus ids,
                  // panel/layout refs) on top of the transport record check:
                  // the relay forwards the robot's manifest verbatim.
                  manifest = parseManifest(msg.manifest);
                } catch (e) {
                  if (e instanceof ManifestError && e.code === "unsupported_version") {
                    // A robot newer than this SDK build (stale cached dist
                    // behind a newer relay): drop anything stale and show
                    // the polite notice instead of a raw error.
                    this.#clearProducer();
                    this.status.update({ manifestUnsupported: true, lastError: null });
                  } else {
                    this.status.update({
                      lastError: {
                        code: "invalid_manifest",
                        message: `invalid manifest: ${(e as Error).message}`,
                      },
                    });
                  }
                  break;
                }
              }
              this.status.update({ lastError: null, manifestUnsupported: false });
              // Adopt before subscribing: a sub can trigger an immediate
              // frame (the bridge replays the cached costmap), and #ingest
              // drops everything while no manifest is adopted.
              this.#applyManifest(manifest);
              this.#wireRunId = runId;
              this.#syncDesired();
              if (this.#watchWaiter !== null && this.#watchWaiter.id === msg.robotId) {
                const waiter = this.#watchWaiter;
                this.#watchWaiter = null;
                waiter.resolve(manifest);
              }
              break;
            }
            case "teleop_started":
              for (const cb of this.#teleopCbs) cb(msg);
              break;
            case "pub_ack":
              this.#settlePub(msg.id)?.resolve({
                ch: msg.ch,
                relayTs: msg.relayTs,
                bridgeTs: msg.bridgeTs,
              });
              break;
            case "error": {
              if (msg.requestId !== undefined) {
                // A correlated publish failure settles its promise and never
                // reaches the session error banner; an unknown id (already
                // settled locally) is dropped.
                this.#settlePub(msg.requestId)?.reject(
                  new PublishError(
                    UNKNOWN_OUTCOME_CODES.has(msg.code) ? "unknown" : "rejected",
                    msg.code,
                    msg.message,
                  ),
                );
                break;
              }
              if (msg.code === "version_mismatch") {
                this.transport.fail(msg.message);
              } else if (msg.code === "teleop_held") {
                // A refused lease is the teleop panel's state, not a
                // session-level error banner.
                for (const cb of this.#teleopCbs) cb(msg);
              } else {
                this.status.update({
                  lastError: { code: "relay_error", message: `${msg.code}: ${msg.message}` },
                });
              }
              break;
            }
            default:
              // pong, robot-side messages: nothing to do
              break;
          }
        }
      }
    } catch {
      // control stream died with the connection; the transport reconnects
    } finally {
      // This connection can never answer its publishes; the commands may
      // have been delivered before it died, so the outcome is unknown and
      // nothing is ever resent.
      this.#sweepPendingPublishes(runId, "connection_lost", "the relay connection died");
    }
  }

  /**
   * Adopt a confirmed manifest. The producer behind it may differ from the
   * previous one (viewer reconnect, robot restart or replacement), so seq
   * tracking always rebaselines; a first adopt drops data left over from a
   * dead producer, and a changed manifest additionally remounts.
   */
  #applyManifest(manifest: Manifest): void {
    const prev = this.#manifest;
    this.#manifest = manifest;
    if (prev === null) {
      this.store.reset();
      this.status.update({ manifest });
    } else if (!manifestsEqual(prev, manifest)) {
      this.store.reset();
      this.status.update({ manifest, epoch: this.status.get().epoch + 1 });
    } else {
      this.status.update({ manifest });
    }
    this.store.rebaseline();
  }

  /**
   * Zero or ambiguous robots (or a watch switch): the watch is no longer
   * confirmed. Drop the manifest and all channel data and remount, so
   * nothing stale survives under whatever robot is confirmed next.
   */
  #clearProducer(): void {
    if (this.#manifest === null) return;
    this.#manifest = null;
    this.store.reset();
    this.status.update({
      manifest: null,
      manifestUnsupported: false,
      epoch: this.status.get().epoch + 1,
    });
  }

  async #readUniStreams(wt: WebTransportLike, runId: number): Promise<void> {
    const streams = wt.incomingUnidirectionalStreams.getReader();
    try {
      while (true) {
        const { value, done } = await streams.read();
        if (done) break;
        void this.#readStreamFrames(value, runId);
      }
    } catch {
      // connection died; the transport reconnects
    }
  }

  // A latest stream carries one frame; a reliable channel's persistent stream
  // carries them back to back. Frames dispatch on byte count (the relay's FIN
  // can be seconds late); the stream is never cancelled - reading to its end
  // costs nothing and a cancel would reset the persistent stream.
  async #readStreamFrames(stream: ReadableStream<Uint8Array>, runId: number): Promise<void> {
    const reader = stream.getReader();
    const frames = new DataFrameStreamReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        for (const frame of frames.push(value)) {
          if (runId === this.#runId) this.#ingest(frame);
        }
      }
    } catch (e) {
      // reset/aborted stream: a partial latest-wins frame is dropped by
      // design. Framing corruption still delivers the frames decoded before
      // it; only the corrupt stream is abandoned.
      if (e instanceof DataFrameStreamError && runId === this.#runId) {
        for (const frame of e.frames) this.#ingest(frame);
      }
    }
  }

  #ingest(frame: DataFrame): void {
    // No adopted manifest means no confirmed producer: anything arriving is
    // stale drain from a dead robot session and must not re-dirty the store.
    if (this.#manifest === null) return;
    const spec = this.#manifest.channels.find((c) => c.ch === frame.header.ch);
    const decoder = this.#registry.get(spec?.encoding);
    let value: unknown;
    let preview: string | undefined;
    let decodeOk = true;
    if (decoder !== undefined) {
      try {
        ({ value, preview } = decoder(frame.payload, frame.header));
      } catch {
        decodeOk = false;
      }
    }
    this.store.ingest(frame.header.ch, frame.header, value, decodeOk, preview);
  }
}

export function connect(options: ConnectOptions = {}, deps: TransportDeps = {}): Session {
  const session = new SessionImpl(options, deps);
  session.transport.start();
  return session;
}
