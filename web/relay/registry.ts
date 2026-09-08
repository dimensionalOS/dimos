// Robot registry + subscription bookkeeping: robotId -> session, per-viewer
// watch/sub state, and routing of robot frames to exactly the viewers that
// asked for them. Transport-blind (sessions are peers behind small
// interfaces) so every transition is unit-testable without QUIC.
//
// Subscription flow: viewers sub/unsub channels of their watched robot; after
// every mutation the registry recomputes the active set (union over watchers)
// and pushes a `subs` snapshot upstream when it changed. Snapshots ride the
// reliable robot control carrier (ordered, never lost on a live session);
// registration forces a baseline snapshot, and the monotonic `n` still guards
// against duplicates and reordering across reconnects.

import {
  type ChannelSpec,
  type Delivery,
  type FrameHeader,
  MAX_PUB_DATA_BYTES,
  type Msg,
  PROTOCOL_VERSION,
  type PubAckMsg,
  type PubNackMsg,
  RESERVED_CHANNEL_PREFIX,
  type RobotInfo,
  type RobotManifest,
} from "@dimos/shared";
import { MAX_MANIFEST_ID_LEN } from "@dimos/shared/manifest";

import type { CarrierStats } from "./carrier.ts";
import {
  type ChannelPolicy,
  LatestChannel,
  parseRobotFrameHeader,
  Rate,
  ReliableChannel,
  TokenBucket,
  type ViewerSink,
} from "./forward.ts";

// Publish forwarding bounds. The timeout settles pending routing state when
// a bridge never answers (SDK outcome "unknown": the bridge may still have
// published); the caps bound relay memory per viewer and per robot, and keep
// worst-case queued publish bytes far under the carrier's fail-fast caps.
export const PUBLISH_TIMEOUT_MS = 10_000;
const MAX_PENDING_PER_VIEWER = 16;
const MAX_PENDING_BYTES_PER_VIEWER = 256 * 1024;
const MAX_PENDING_PER_ROBOT = 64;
const MAX_PENDING_BYTES_PER_ROBOT = 1024 * 1024;

const enc = new TextEncoder();

/** What the registry needs from a robot session. */
export interface RobotPeer {
  /** Set by the session once a valid robot hello arrived. */
  readonly info: RobotInfo | null;
  /** Normalized channel specs (feeds the delivery map / sub validation). */
  readonly channels: ChannelSpec[];
  /** Raw manifest as received; forwarded verbatim in watch replies (the
   * relay never normalizes it and stays layout-blind). */
  readonly manifest: RobotManifest | null;
  /** Close reason once closed; a closed session must never (re)register. */
  readonly closed: string | null;
  /** Control message upstream to the bridge (datagram: lossy, never blocks).
   * Handshake and teleop only. */
  sendMsg(msg: Msg): void;
  /** Control message upstream on the reliable robot control carrier (ordered;
   * a write failure fails the whole robot session). Subs snapshots and future
   * robot-bound control. */
  sendControl(msg: Msg): void;
  /** One forwarded viewer publish, as a tx data frame on the carrier (meta
   * carries the relay provenance: token id, principal, relayTs, clientTs). */
  sendPub(ch: string, payload: Uint8Array, meta: Record<string, unknown>): void;
  /** Live carrier queue counters for stats(). */
  carrierStats(): CarrierStats;
}

/** What the registry needs from a viewer session (fakeable in tests). */
export interface ViewerPeer {
  readonly id: number;
  watched: string | null;
  readonly subs: Set<string>;
  readonly policies: Map<string, ChannelPolicy>;
  readonly sink: ViewerSink;
  /** True once a valid hello arrived; robots pushes skip un-greeted viewers. */
  greeted: boolean;
  /** Push channel chosen at hello time (bidi stream or datagrams). */
  sendMsg(msg: Msg): void;
}

/** Dispose and drop every delivery policy of `viewer` (queued frames from the
 * old watch must not reach the viewer, and persistent writers must release
 * their streams). */
function disposePolicies(viewer: ViewerPeer): void {
  for (const policy of viewer.policies.values()) policy.dispose();
  viewer.policies.clear();
}

interface ChannelInStats {
  delivery: Delivery;
  framesIn: number;
  bytesIn: number;
  rate: Rate;
}

/** One forwarded publish awaiting its bridge ack. Indexed twice: by relay
 * token on the robot entry (ack routing; dies with the robot) and by the
 * viewer's own request id in the per-viewer state (dup detection, caps,
 * disconnect release). */
interface PendingPub {
  viewer: ViewerPeer;
  /** The viewer's own request id, restored in the routed pub_ack/error. */
  viewerId: string;
  token: string;
  robotId: string;
  ch: string;
  bytes: number;
  deadlineMs: number;
}

/** Per-viewer publish state, keyed by the viewer object (viewer-supplied
 * request ids are untrusted and never namespace anything globally). */
interface ViewerPubState {
  pending: Map<string, PendingPub>;
  pendingBytes: number;
  /** Per-viewer accepted-rate buckets, cleared on watch switch and rebuilt
   * per channel when the spec's maxHz stops matching (a same-id robot
   * restart may change the manifest under a live viewer; the rebuild grants
   * one bounded fresh burst up to ceil(maxHz)). */
  buckets: Map<string, TokenBucket>;
}

interface RobotEntry {
  peer: RobotPeer;
  /** Normalized manifest spec per channel (delivery, publish policy, rate);
   * frame-header delivery is the undeclared-channel fallback. */
  specs: Map<string, ChannelSpec>;
  /** Viewer holding the exclusive teleop lease, or null. Dies with the
   * entry: a re-registered robot starts lease-free. */
  teleop: ViewerPeer | null;
  /** Lease generation, bumped on every fresh grant and stamped into
   * robot-bound teleop messages so the bridge can permanently void a
   * released lease's in-flight datagrams. Dies with the entry (safe: the
   * bridge resets its floor with the session). */
  teleopGen: number;
  /** Snapshot counter, monotonic for this robot session's registration. */
  n: number;
  /** Last snapshot content, for change detection and stats(). */
  lastChs: string[];
  /** Per-channel ingress stats, declared channels only. */
  channelsIn: Map<string, ChannelInStats>;
  /** One aggregate bucket for frames on undeclared channels: arbitrary
   * header ch strings must not grow channelsIn. A manifest-less robot's
   * traffic lands entirely here. */
  undeclared: { framesIn: number; bytesIn: number; rate: Rate };
  /** Relay-authored pub token counter (the id forwarded robot-ward). */
  pubN: number;
  /** Pending publishes by token; settled by ack/nack, timeout, or death. */
  pubPending: Map<string, PendingPub>;
  pubPendingBytes: number;
  /** Aggregate accepted-publish rate per channel (the manifest maxHz). */
  pubBuckets: Map<string, TokenBucket>;
}

export class Registry {
  #robots = new Map<string, RobotEntry>();
  #viewers = new Set<ViewerPeer>();
  #pubViewers = new Map<ViewerPeer, ViewerPubState>();
  #framesDropped = 0;
  #framesFromUnregistered = 0;
  #teleopForwarded = 0;
  #teleopDropped = 0;
  #carrierFailures = 0;
  #pubAccepted = 0;
  #pubAcked = 0;
  #pubTimedOut = 0;
  #pubLateAcks = 0;
  #pubRejected: Record<string, number> = {};
  readonly #now: () => number;

  /** Clock injected so tests fabricate time (buckets and pending deadlines
   * are the only time-dependent state beyond stats). */
  constructor(now: () => number = Date.now) {
    this.#now = now;
  }

  /** A robot control carrier overflowed or failed a write; the session is
   * being closed. Registry-level because the per-robot entry dies with it. */
  carrierFailed(): void {
    this.#carrierFailures++;
  }

  addViewer(viewer: ViewerPeer): void {
    this.#viewers.add(viewer);
  }

  viewerClosed(viewer: ViewerPeer): void {
    if (!this.#viewers.delete(viewer)) return;
    disposePolicies(viewer);
    // Release the viewer's pending publishes silently (there is nobody left
    // to route the ack to); the robot-side entries go with them.
    const state = this.#pubViewers.get(viewer);
    if (state !== undefined) {
      for (const pending of state.pending.values()) {
        const entry = this.#robots.get(pending.robotId);
        if (entry !== undefined && entry.pubPending.delete(pending.token)) {
          entry.pubPendingBytes -= pending.bytes;
        }
      }
      this.#pubViewers.delete(viewer);
    }
    if (viewer.watched !== null) {
      this.#releaseTeleop(viewer.watched, viewer);
      this.#syncSubs(viewer.watched);
    }
  }

  /**
   * Register a robot session after its valid hello. Idempotent under hello
   * resends. A different live peer with the same id is rejected so two bridges
   * cannot continually displace one another.
   */
  registerRobot(peer: RobotPeer): boolean {
    const info = peer.info;
    if (info === null || peer.closed !== null) return false;
    const entry = this.#robots.get(info.id);
    if (entry !== undefined && entry.peer === peer) return true; // hello resend
    if (entry !== undefined) {
      console.log(`[relay] rejecting duplicate live robot id ${info.id}`);
      return false;
    }
    this.#robots.set(info.id, {
      peer,
      specs: new Map(peer.channels.map((c) => [c.ch, c])),
      teleop: null,
      teleopGen: 0,
      n: 0,
      lastChs: [],
      channelsIn: new Map(),
      undeclared: { framesIn: 0, bytesIn: 0, rate: new Rate() },
      pubN: 0,
      pubPending: new Map(),
      pubPendingBytes: 0,
      pubBuckets: new Map(),
    });
    this.#pushRobots();
    // Forced: gives a fresh bridge its baseline and reattaches surviving
    // watchers after the old robot session has disconnected.
    this.#syncSubs(info.id, true);
    return true;
  }

  /** Session-closed hook; a no-op for an unregistered/rejected peer. */
  robotClosed(peer: RobotPeer): void {
    const id = peer.info?.id;
    if (id === undefined) return;
    const entry = this.#robots.get(id);
    if (entry === undefined || entry.peer !== peer) return;
    this.#robots.delete(id);
    // Pending publishes cannot be acked any more; their viewers get an
    // outcome-"unknown" error (the bridge may have published before dying).
    for (const pending of entry.pubPending.values()) {
      this.#settlePub(entry, pending);
      if (this.#viewers.has(pending.viewer)) {
        this.#countRejected("robot_disconnected");
        pending.viewer.sendMsg({
          t: "error",
          code: "robot_disconnected",
          message: `robot ${id} disconnected before acknowledging the publish`,
          requestId: pending.viewerId,
        });
      }
    }
    console.log(`[relay] robot ${id} disconnected`);
    // Viewers keep watched/subs: a returning robot reattaches seamlessly.
    this.#pushRobots();
  }

  /** Replies to viewer control messages; returns false if the session must close. */
  onViewerMsg(viewer: ViewerPeer, msg: Msg, reply: (msg: Msg) => void): boolean {
    if (msg.t !== "hello" && !viewer.greeted) {
      reply({
        t: "error",
        code: "hello_required",
        message: "send a valid role=viewer hello before viewer commands",
      });
      return false;
    }
    switch (msg.t) {
      case "hello": {
        if (msg.v !== PROTOCOL_VERSION) {
          reply({
            t: "error",
            code: "version_mismatch",
            message: `protocol v${PROTOCOL_VERSION} required, got v${msg.v}`,
          });
          return false;
        }
        if (msg.role !== "viewer") {
          reply({
            t: "error",
            code: "role_mismatch",
            message: "the /viewer endpoint requires role=viewer",
          });
          return false;
        }
        viewer.greeted = true;
        // Repeat hellos repeat both replies: the Python viewer's control
        // channel is datagrams, so this is its loss-healing path.
        reply({ t: "welcome", v: PROTOCOL_VERSION });
        reply(this.robotsMsg());
        break;
      }
      case "ping":
        reply({ t: "pong", n: msg.n, ts: msg.ts });
        break;
      case "watch": {
        const entry = this.#robots.get(msg.robotId);
        if (entry === undefined) {
          reply({ t: "error", code: "unknown_robot", message: `no robot ${msg.robotId}` });
          break;
        }
        const previous = viewer.watched;
        if (previous !== null && previous !== msg.robotId) {
          viewer.subs.clear();
          disposePolicies(viewer);
          this.#releaseTeleop(previous, viewer);
          // Publish buckets are per robot+channel; pending publishes stay
          // (the viewer is live, acks from the old robot still route).
          this.#pubViewers.get(viewer)?.buckets.clear();
        }
        viewer.watched = msg.robotId;
        if (previous !== null && previous !== msg.robotId) this.#syncSubs(previous);
        this.#syncSubs(msg.robotId);
        reply({
          t: "manifest",
          robotId: msg.robotId,
          // undefined for a manifest-less robot; JSON.stringify omits it.
          manifest: entry.peer.manifest ?? undefined,
        });
        break;
      }
      case "sub":
      case "unsub": {
        if (viewer.watched === null) {
          reply({ t: "error", code: "no_watch", message: "watch a robot before sub/unsub" });
          break;
        }
        if (msg.t === "sub") {
          // Reserved ids never enter subs (or snapshots), even on a
          // manifest-less robot: @-frames are control, not data.
          if (msg.ch.startsWith(RESERVED_CHANNEL_PREFIX)) {
            reply({
              t: "error",
              code: "unknown_channel",
              message: `channel ids beginning with ${RESERVED_CHANNEL_PREFIX} are reserved`,
            });
            break;
          }
          // Manifest ids are length-bounded; hold undeclared subs to the
          // same cap so a manifest-less robot (transport tests accept any
          // sub) cannot accumulate ch strings that push its subs snapshot
          // past the @control payload cap and fail the carrier.
          if (msg.ch.length > MAX_MANIFEST_ID_LEN) {
            reply({
              t: "error",
              code: "unknown_channel",
              message: `channel ids are at most ${MAX_MANIFEST_ID_LEN} chars`,
            });
            break;
          }
          // Manifest-validated: arbitrary ch strings must not grow relay and
          // bridge subscription state. A robot that declared no manifest at
          // all (transport tests) has nothing to validate against and
          // accepts any sub - production bridges always declare.
          const entry = this.#robots.get(viewer.watched);
          if (entry === undefined) {
            reply({ t: "error", code: "unknown_robot", message: `no robot ${viewer.watched}` });
            break;
          }
          if (entry.specs.size > 0 && !entry.specs.has(msg.ch)) {
            reply({
              t: "error",
              code: "unknown_channel",
              message: `no channel ${msg.ch.slice(0, 64)} on ${viewer.watched}`,
            });
            break;
          }
          viewer.subs.add(msg.ch);
        } else {
          viewer.subs.delete(msg.ch);
          viewer.policies.get(msg.ch)?.dispose();
          viewer.policies.delete(msg.ch);
        }
        this.#syncSubs(viewer.watched);
        break;
      }
      case "teleop_start": {
        if (viewer.watched === null) {
          reply({ t: "error", code: "no_watch", message: "watch a robot before teleop_start" });
          break;
        }
        const entry = this.#robots.get(viewer.watched);
        if (entry === undefined) {
          reply({ t: "error", code: "unknown_robot", message: `no robot ${viewer.watched}` });
          break;
        }
        if (entry.teleop !== null && entry.teleop !== viewer) {
          reply({
            t: "error",
            code: "teleop_held",
            message: `robot ${viewer.watched} teleop is held by another viewer`,
          });
          break;
        }
        if (entry.teleop === null) {
          entry.teleop = viewer;
          entry.teleopGen++;
          console.log(
            `[relay] robot ${viewer.watched} teleop lease -> viewer ${viewer.id} ` +
              `(gen ${entry.teleopGen})`,
          );
        }
        // Resent on an idempotent re-arm to heal datagram loss, like the
        // re-acked teleop_started; the bridge ignores duplicates.
        entry.peer.sendMsg({ t: "teleop_start", gen: entry.teleopGen });
        reply({ t: "teleop_started" });
        break;
      }
      case "teleop_stop": {
        // Idempotent release; no ack (disarm is local to the cockpit and the
        // bridge watchdog covers a lost robot-ward teleop_stop).
        if (viewer.watched !== null) this.#releaseTeleop(viewer.watched, viewer);
        break;
      }
      case "twist":
      case "stop": {
        // Lease-gated, silent on drop: gating runs at twist rate on lossy
        // datagrams, and the arm handshake (teleop_start) is where a viewer
        // learns it lacks the lease.
        const entry = viewer.watched === null ? undefined : this.#robots.get(viewer.watched);
        if (entry === undefined || entry.teleop !== viewer) {
          this.#teleopDropped++;
          break;
        }
        // The spread puts the relay's stamp last: a viewer-supplied gen is
        // overridden, the relay is the only generation authority.
        entry.peer.sendMsg({ ...msg, gen: entry.teleopGen });
        this.#teleopForwarded++;
        break;
      }
      case "pub": {
        // Every failure settles the request with a correlated error, never a
        // silent drop. Identity and shape first, then policy, then pending
        // caps, then the token buckets last: a rejected request never spends
        // rate tokens.
        const fail = (code: string, message: string) => {
          this.#countRejected(code);
          reply({ t: "error", code, message, requestId: msg.id });
        };
        const entry = viewer.watched === null ? undefined : this.#robots.get(viewer.watched);
        if (viewer.watched === null) {
          fail("no_watch", "watch a robot before publishing");
          break;
        }
        if (entry === undefined) {
          fail("unknown_robot", `no robot ${viewer.watched}`);
          break;
        }
        const state = this.#pubViewers.get(viewer) ?? {
          pending: new Map<string, PendingPub>(),
          pendingBytes: 0,
          buckets: new Map<string, TokenBucket>(),
        };
        this.#pubViewers.set(viewer, state);
        if (state.pending.has(msg.id)) {
          fail("duplicate_request", `request id ${msg.id} is already pending`);
          break;
        }
        // Independent of the SDK's own check: callers can bypass the SDK.
        const payload = enc.encode(JSON.stringify(msg.data));
        if (payload.byteLength > MAX_PUB_DATA_BYTES) {
          fail(
            "publish_too_large",
            `serialized data is ${payload.byteLength} B (limit ${MAX_PUB_DATA_BYTES})`,
          );
          break;
        }
        const spec = entry.specs.get(msg.ch);
        if (spec === undefined) {
          // A manifest-less robot (transport tests) declared no publishable
          // channels either: publishing requires a declared policy,
          // deliberately stricter than sub.
          fail("unknown_channel", `no channel ${msg.ch.slice(0, 64)} on ${viewer.watched}`);
          break;
        }
        if (spec.publish === "exclusive") {
          fail(
            "not_publishable",
            `channel ${msg.ch} requires the exclusive publisher lease (W8)`,
          );
          break;
        }
        if (spec.dir !== "tx" || spec.publish !== "shared" || spec.delivery !== "reliable") {
          fail("not_publishable", `channel ${msg.ch} does not accept generic publish`);
          break;
        }
        // Scope check: the local relay has no operator auth (W10); its
        // synthetic principal bypasses requiredScope by design. A remote
        // relay must check the authenticated principal's scopes here.
        const principal = "local";
        if (
          state.pending.size >= MAX_PENDING_PER_VIEWER ||
          state.pendingBytes + payload.byteLength > MAX_PENDING_BYTES_PER_VIEWER ||
          entry.pubPending.size >= MAX_PENDING_PER_ROBOT ||
          entry.pubPendingBytes + payload.byteLength > MAX_PENDING_BYTES_PER_ROBOT
        ) {
          fail("pending_limit", "too many unacknowledged publishes");
          break;
        }
        const now = this.#now();
        // Per-viewer bucket first (a rate-rejected attempt still costs its
        // viewer), then the aggregate at the advertised maxHz: more viewer
        // sessions must never multiply the accepted robot/channel rate. A
        // viewer bucket whose rate stopped matching the spec belongs to a
        // dead registration (a same-id robot restart may change maxHz) and
        // is rebuilt at the current rate.
        let viewerBucket = state.buckets.get(msg.ch);
        if (viewerBucket === undefined || viewerBucket.ratePerSec !== spec.maxHz) {
          viewerBucket = new TokenBucket(spec.maxHz);
          state.buckets.set(msg.ch, viewerBucket);
        }
        let aggregate = entry.pubBuckets.get(msg.ch);
        if (aggregate === undefined) {
          aggregate = new TokenBucket(spec.maxHz);
          entry.pubBuckets.set(msg.ch, aggregate);
        }
        if (!viewerBucket.take(now) || !aggregate.take(now)) {
          fail("rate_limited", `channel ${msg.ch} accepts at most ${spec.maxHz} publishes/s`);
          break;
        }
        const pending: PendingPub = {
          viewer,
          viewerId: msg.id,
          token: `p${++entry.pubN}`,
          robotId: viewer.watched,
          ch: msg.ch,
          bytes: payload.byteLength,
          deadlineMs: now + PUBLISH_TIMEOUT_MS,
        };
        entry.pubPending.set(pending.token, pending);
        entry.pubPendingBytes += payload.byteLength;
        state.pending.set(msg.id, pending);
        state.pendingBytes += payload.byteLength;
        entry.peer.sendPub(msg.ch, payload, {
          id: pending.token,
          principal,
          relayTs: now / 1000,
          ...(msg.clientTs !== undefined ? { clientTs: msg.clientTs } : {}),
        });
        this.#pubAccepted++;
        break;
      }
    }
    return true;
  }

  /** Route one bridge publish result to the originating viewer: settle the
   * pending entry (restoring the viewer's own request id) and forward the
   * ack, or map a nack to a correlated error. Late/duplicate results are
   * counted and dropped without growing state. */
  onRobotPubResult(peer: RobotPeer, msg: PubAckMsg | PubNackMsg): void {
    const id = peer.info?.id;
    const entry = id === undefined ? undefined : this.#robots.get(id);
    if (entry === undefined || entry.peer !== peer) {
      this.#pubLateAcks++;
      return;
    }
    const pending = entry.pubPending.get(msg.id);
    if (pending === undefined) {
      this.#pubLateAcks++;
      console.log(`[relay] dropping late/duplicate pub result ${msg.id.slice(0, 64)} from ${id}`);
      return;
    }
    this.#settlePub(entry, pending);
    if (!this.#viewers.has(pending.viewer)) return; // viewer left; nothing to route
    if (msg.t === "pub_ack") {
      this.#pubAcked++;
      pending.viewer.sendMsg({
        t: "pub_ack",
        id: pending.viewerId,
        ch: pending.ch,
        relayTs: msg.relayTs,
        bridgeTs: msg.bridgeTs,
      });
    } else {
      this.#countRejected(msg.code);
      pending.viewer.sendMsg({
        t: "error",
        code: msg.code.slice(0, 64),
        message: msg.message.slice(0, 256),
        requestId: pending.viewerId,
      });
    }
  }

  /** Remove one pending publish from both indexes (robot token map and the
   * viewer's request-id map), with byte accounting. */
  #settlePub(entry: RobotEntry, pending: PendingPub): void {
    if (entry.pubPending.delete(pending.token)) entry.pubPendingBytes -= pending.bytes;
    const state = this.#pubViewers.get(pending.viewer);
    if (state !== undefined && state.pending.delete(pending.viewerId)) {
      state.pendingBytes -= pending.bytes;
    }
  }

  #countRejected(code: string): void {
    this.#pubRejected[code] = (this.#pubRejected[code] ?? 0) + 1;
  }

  /** Release `viewer`'s teleop lease on `robotId`, if held: the bridge gets
   * a gen-stamped teleop_stop that permanently voids this lease's datagrams
   * (its deadman watchdog covers a lost stop) and the next teleop_start
   * wins the lease. */
  #releaseTeleop(robotId: string, viewer: ViewerPeer): void {
    const entry = this.#robots.get(robotId);
    if (entry === undefined || entry.teleop !== viewer) return;
    entry.teleop = null;
    // At release time teleopGen still names the released lease (it bumps
    // only on grant).
    entry.peer.sendMsg({ t: "teleop_stop", gen: entry.teleopGen });
    console.log(`[relay] robot ${robotId} teleop lease released (viewer ${viewer.id})`);
  }

  /**
   * Route one robot data frame (raw bytes, already length-complete) to the
   * viewers watching this robot and subscribed to its channel. The session
   * passes its already-parsed header (null = parsed and invalid); omitting
   * it parses here.
   */
  onRobotFrame(
    peer: RobotPeer,
    bytes: Uint8Array,
    header: FrameHeader | null = parseRobotFrameHeader(bytes),
  ): void {
    const id = peer.info?.id;
    const entry = id === undefined ? undefined : this.#robots.get(id);
    if (entry === undefined || entry.peer !== peer) {
      // Frames race registration (the stream loop starts at accept time).
      this.#framesFromUnregistered++;
      return;
    }
    if (header === null) {
      this.#framesDropped++;
      console.log("[relay] dropping robot frame with invalid header");
      return;
    }
    const ch = header.ch;
    const declared = entry.specs.get(ch)?.delivery;
    // Manifest delivery wins; the header's is the undeclared-channel fallback.
    const delivery = declared ?? header.delivery;

    if (declared === undefined) {
      // Aggregate bucket only: arbitrary header ch strings must not grow the
      // per-channel map. Routing below still forwards to subscribed viewers
      // (a robot that declared no manifest accepts any sub).
      entry.undeclared.framesIn++;
      entry.undeclared.bytesIn += bytes.byteLength;
      entry.undeclared.rate.push(bytes.byteLength, Date.now());
    } else {
      // delivery is fixed at creation: the manifest cannot change within a
      // registration.
      const stats = entry.channelsIn.get(ch) ??
        { delivery, framesIn: 0, bytesIn: 0, rate: new Rate() };
      stats.framesIn++;
      stats.bytesIn += bytes.byteLength;
      stats.rate.push(bytes.byteLength, Date.now());
      entry.channelsIn.set(ch, stats);
    }

    for (const viewer of this.#viewers) {
      if (viewer.watched !== id || !viewer.subs.has(ch)) continue;
      let policy = viewer.policies.get(ch);
      if (policy === undefined || policy.delivery !== delivery) {
        policy?.dispose();
        policy = delivery === "reliable"
          ? new ReliableChannel(viewer.sink)
          : new LatestChannel(viewer.sink);
        viewer.policies.set(ch, policy);
      }
      policy.offer(bytes);
    }
  }

  /** Reap stale accepted latest streams on every viewer, and settle expired
   * pending publishes with a `publish_timeout` error (outcome "unknown" at
   * the SDK: the bridge may still have published). Offers reap
   * opportunistically, but an idle input stops offering; server.ts drives
   * this on an interval. Clock passed in so tests fabricate time. */
  reapAll(nowMs: number): void {
    for (const viewer of this.#viewers) {
      for (const policy of viewer.policies.values()) policy.reap(nowMs);
    }
    for (const entry of this.#robots.values()) {
      for (const pending of entry.pubPending.values()) {
        if (nowMs < pending.deadlineMs) continue;
        this.#settlePub(entry, pending);
        this.#pubTimedOut++;
        if (this.#viewers.has(pending.viewer)) {
          pending.viewer.sendMsg({
            t: "error",
            code: "publish_timeout",
            message: `no bridge acknowledgement within ${PUBLISH_TIMEOUT_MS} ms`,
            requestId: pending.viewerId,
          });
        }
      }
    }
  }

  robotsMsg(): Msg {
    return { t: "robots", robots: this.#robotInfos() };
  }

  #robotInfos(): RobotInfo[] {
    const robots: RobotInfo[] = [];
    for (const entry of this.#robots.values()) {
      if (entry.peer.info !== null) robots.push(entry.peer.info);
    }
    return robots;
  }

  stats(): unknown {
    const now = this.#now();
    let pubPending = 0;
    for (const entry of this.#robots.values()) pubPending += entry.pubPending.size;
    return {
      robots: this.#robotInfos(),
      viewers: this.#viewers.size,
      framesDropped: this.#framesDropped,
      framesFromUnregistered: this.#framesFromUnregistered,
      teleopForwarded: this.#teleopForwarded,
      teleopDropped: this.#teleopDropped,
      carrierFailures: this.#carrierFailures,
      pub: {
        accepted: this.#pubAccepted,
        acked: this.#pubAcked,
        timedOut: this.#pubTimedOut,
        lateAcks: this.#pubLateAcks,
        pending: pubPending,
        rejected: { ...this.#pubRejected },
      },
      perRobot: Object.fromEntries(
        [...this.#robots].map(([id, e]) => [id, {
          subs: e.lastChs,
          teleop: e.teleop?.id ?? null,
          carrier: e.peer.carrierStats(),
          channels: Object.fromEntries(
            [...e.channelsIn].map(([ch, s]) => [ch, {
              delivery: s.delivery,
              framesIn: s.framesIn,
              bytesIn: s.bytesIn,
              ...s.rate.snapshot(now),
            }]),
          ),
          undeclared: {
            framesIn: e.undeclared.framesIn,
            bytesIn: e.undeclared.bytesIn,
            ...e.undeclared.rate.snapshot(now),
          },
        }]),
      ),
      perViewer: [...this.#viewers].map((v) => ({
        id: v.id,
        watched: v.watched,
        subs: [...v.subs].sort(),
        channels: Object.fromEntries(
          [...v.policies].map(([ch, p]) => [ch, {
            delivery: p.delivery,
            sent: p.sent,
            dropped: p.dropped,
            queued: p.queued(),
            aborted: p.aborted,
            expired: p.expired,
            inflight: p.inflight(),
            bytesOut: p.bytesOut,
            ...p.rate.snapshot(now),
          }]),
        ),
      })),
    };
  }

  /** Sorted union of the subs of every viewer watching `robotId`, kept to
   * the current manifest when one was declared (a reconnect can shrink the
   * manifest under surviving subs). */
  #activeChs(robotId: string, specs: Map<string, ChannelSpec>): string[] {
    const chs = new Set<string>();
    for (const viewer of this.#viewers) {
      if (viewer.watched !== robotId) continue;
      for (const ch of viewer.subs) {
        if (specs.size === 0 || specs.has(ch)) chs.add(ch);
      }
    }
    return [...chs].sort();
  }

  /**
   * Send a subs snapshot upstream iff the active set changed (or `force`).
   * Recomputed from scratch on every mutation: no incremental refcounts to
   * drift across watch switches, disconnects, and reconnects.
   */
  #syncSubs(robotId: string, force = false): void {
    const entry = this.#robots.get(robotId);
    if (entry === undefined) return;
    const chs = this.#activeChs(robotId, entry.specs);
    if (!force && chs.join("\n") === entry.lastChs.join("\n")) return;
    entry.lastChs = chs;
    entry.peer.sendControl({ t: "subs", chs, n: ++entry.n });
    console.log(`[relay] robot ${robotId} active channels: [${chs.join(", ")}]`);
  }

  #pushRobots(): void {
    const msg = this.robotsMsg();
    for (const viewer of this.#viewers) {
      if (viewer.greeted) viewer.sendMsg(msg);
    }
  }
}
