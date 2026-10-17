// Relay-side orchestration of Cloudflare SFU sessions for video.webrtc.v1
// channels: one SFU session per peer, and per viewer a serialized reconcile
// loop that recomputes from scratch what should be pulled, declares the
// robot's tracks on first need, pulls the missing ones and force-closes stale
// ones. A run that fails or ends pending (the robot encodes lazily, so its RTP
// can lag its declaration by seconds) arms a backoff re-run without blocking,
// so an event during the wait runs at once. Every allocation the SFU made is
// either negotiated and recorded or handed to the next run's close step, the
// one place that releases tracks. A track the SFU collected (30 s without
// media) is reported by the bridge when its media resumes (rtc_stalled): its
// pulls are closed and made again. Media never touches the relay.
import type { IceServer, Msg, RtcOfferMsg, RtcTrack } from "@dimos/shared";
import { TRACK_ENCODING } from "@dimos/shared/manifest";
import {
  type CloudflareClient,
  CloudflareError,
  SessionGoneError,
  STUN_ONLY,
  TRACK_GC_MS,
  type TrackResult,
  type TracksResponse,
  TURN_TTL_S,
} from "./cloudflare.ts";
import type { RobotPeer, RtcBroker, ViewerPeer } from "./registry.ts";

export const ANSWER_TIMEOUT_MS = 10_000;
/** Re-run delays after a failed or pending run. */
export const RECONCILE_BACKOFF_MS = [300, 600, 1000, 2000, 4000, 8000, 16000, 30000];
export const ICE_REFRESH_MS = (TURN_TTL_S * 1000) / 2;
/** A pull's force-close is tried this many times before its mid is given up
 * (an unfed track the SFU collects by itself; a fed one costs bandwidth until
 * the viewer's session dies). */
export const MAX_CLOSE_ATTEMPTS = 5;

/** What the hub needs from the Cloudflare client (fakeable in tests). */
export type SfuClient = Pick<
  CloudflareClient,
  "newSession" | "addTracks" | "closeTracks" | "renegotiate" | "iceServers"
>;

export interface RtcHubOptions {
  /** Test seams. */
  sleep?: (ms: number) => Promise<void>;
  now?: () => number;
  answerTimeoutMs?: number;
}

interface RobotRtc {
  peer: RobotPeer;
  /** Set once the SFU answered the robot's offer. */
  cfSessionId: string | null;
  /** Track channel -> the robot's transceiver mid, from its offer. */
  mids: Map<string, string>;
  /** Track channel -> when tracks/new (local) declared it on this session. */
  published: Map<string, number>;
}

interface Pull {
  mid: string;
  robotId: string;
  /** The robot SFU session the pull came from: a re-offered robot makes it stale. */
  robotSession: string;
}

interface ViewerRtc {
  /** The viewer's offer, kept so a failed sessions/new is retried with it. */
  offerSdp: string;
  cfSessionId: string | null;
  pulled: Map<string, Pull>;
  /** Pull mids to force-close -> failed tries so far; a mid stays until the
   * SFU confirmed it or MAX_CLOSE_ATTEMPTS passed. */
  closing: Map<string, number>;
  dirty: boolean;
  running: boolean;
  failures: number;
  pendingAnswer: {
    resolve: (sdp: string) => void;
    reject: (e: Error) => void;
    timer: number;
  } | null;
}

function describe(e: unknown): string {
  return (e as Error)?.message ?? String(e);
}

function defaultSleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export class RtcHub implements RtcBroker {
  readonly #cf: SfuClient;
  readonly #sleep: (ms: number) => Promise<void>;
  readonly #now: () => number;
  readonly #answerTimeoutMs: number;
  #robots = new Map<string, RobotRtc>();
  #viewers = new Map<ViewerPeer, ViewerRtc>();
  #ice: IceServer[] = STUN_ONLY;
  #iceTimer: number | null = null;
  #onRefresh: (() => void) | null = null;
  #disposed = false;
  #apiErrors = 0;
  #pullsCompleted = 0;

  constructor(cf: SfuClient, options: RtcHubOptions = {}) {
    this.#cf = cf;
    this.#sleep = options.sleep ?? defaultSleep;
    this.#now = options.now ?? Date.now;
    this.#answerTimeoutMs = options.answerTimeoutMs ?? ANSWER_TIMEOUT_MS;
  }

  /** TURN credentials are minted before any peer is served, then refreshed
   * at half-life; `onRefresh` runs after each later successful mint so the
   * registry hands every peer the new set (a peer's next peer connection
   * uses the latest it received). */
  async start(onRefresh?: () => void): Promise<void> {
    this.#onRefresh = onRefresh ?? null;
    await this.#mint();
    if (this.#disposed) return;
    this.#iceTimer = setInterval(() => void this.refreshIce(), ICE_REFRESH_MS);
    // A pending refresh must not keep the Deno process alive after shutdown().
    Deno.unrefTimer(this.#iceTimer);
  }

  /** The timer's job; a failed mint keeps the current set. */
  async refreshIce(): Promise<void> {
    if (await this.#mint()) this.#onRefresh?.();
  }

  dispose(): void {
    this.#disposed = true;
    if (this.#iceTimer !== null) clearInterval(this.#iceTimer);
    for (const entry of this.#viewers.values()) this.#rejectAnswer(entry, "relay shutdown");
  }

  async #mint(): Promise<boolean> {
    try {
      const servers = await this.#cf.iceServers();
      if (this.#disposed) return false;
      this.#ice = servers;
      return true;
    } catch (e) {
      this.#apiErrors++;
      console.log(`[relay] rtc: TURN credentials failed (the current set stays): ${describe(e)}`);
      return false;
    }
  }

  iceMsg(): Msg {
    return { t: "rtc_ice", iceServers: this.#ice };
  }

  /** A re-offer replaces the robot's SFU session; viewers' pulls from the
   * old one become stale. */
  robotOffer(peer: RobotPeer, msg: RtcOfferMsg): void {
    const id = peer.info?.id;
    if (id === undefined) return;
    const trackChs = new Set(
      peer.channels.filter((c) => c.encoding === TRACK_ENCODING).map((c) => c.ch),
    );
    const mids = new Map<string, string>();
    for (const t of msg.tracks ?? []) {
      if (trackChs.has(t.ch)) mids.set(t.ch, t.mid);
      else console.log(`[relay] rtc: robot ${id} offered a track for non-track channel ${t.ch}`);
    }
    const entry: RobotRtc = { peer, cfSessionId: null, mids, published: new Map() };
    this.#robots.set(id, entry);
    this.#scheduleAll();
    (async () => {
      const { sessionId, answerSdp } = await this.#cf.newSession(msg.sdp);
      if (this.#robots.get(id) !== entry) return; // superseded or closed meanwhile
      entry.cfSessionId = sessionId;
      peer.sendControl({ t: "rtc_answer", sdp: answerSdp });
      console.log(`[relay] rtc: robot ${id} SFU session ${sessionId} (${mids.size} tracks)`);
      this.#scheduleAll();
    })().catch((e) => {
      // No robot-ward error: the bridge re-offers after its own answer timeout.
      this.#apiErrors++;
      console.log(`[relay] rtc: robot ${id} SFU session failed: ${describe(e)}`);
      if (this.#robots.get(id) === entry) this.#robots.delete(id);
    });
  }

  robotClosed(peer: RobotPeer): void {
    const id = peer.info?.id;
    if (id === undefined) return;
    const entry = this.#robots.get(id);
    if (entry === undefined || entry.peer !== peer) return;
    this.#robots.delete(id);
    // Every pull from it is stale now; the SFU session itself is GC'd.
    this.#scheduleAll();
  }

  /** The bridge fed no media on `ch` for the SFU's track lifetime and does
   * again: the SFU collected the track, so every pull of it is dead and its
   * declaration stale. The pulls are closed and made again; the declaration
   * is aged so the first not_found re-declares at once, while a track the
   * SFU kept after all costs one extra pull. */
  robotStalled(peer: RobotPeer, ch: string): void {
    const id = peer.info?.id;
    if (id === undefined) return;
    const robot = this.#robots.get(id);
    if (robot === undefined || robot.peer !== peer || !robot.mids.has(ch)) return;
    if (robot.published.has(ch)) robot.published.set(ch, this.#now() - TRACK_GC_MS);
    let stale = 0;
    for (const entry of this.#viewers.values()) {
      const pull = entry.pulled.get(ch);
      if (pull === undefined || pull.robotId !== id) continue;
      entry.pulled.delete(ch);
      entry.closing.set(pull.mid, 0);
      stale++;
    }
    console.log(
      `[relay] rtc: robot ${id} track ${ch} stalled past the SFU's track lifetime; ` +
        `${stale} pull(s) made again`,
    );
    this.#scheduleAll();
  }

  /** The SFU session is created by the reconcile loop, so a failed
   * sessions/new is retried with the same offer. A re-offer replaces the
   * session; its pulls die with the old one. */
  viewerOffer(viewer: ViewerPeer, sdp: string): void {
    const previous = this.#viewers.get(viewer);
    if (previous !== undefined) this.#rejectAnswer(previous, "superseded by a new offer");
    const entry: ViewerRtc = {
      offerSdp: sdp,
      cfSessionId: null,
      pulled: new Map(),
      closing: new Map(),
      dirty: false,
      running: false,
      failures: 0,
      pendingAnswer: null,
    };
    this.#viewers.set(viewer, entry);
    this.#schedule(viewer, entry);
  }

  viewerAnswer(viewer: ViewerPeer, sdp: string): void {
    const entry = this.#viewers.get(viewer);
    const pending = entry?.pendingAnswer ?? null;
    if (entry === undefined || pending === null) {
      console.log(`[relay] rtc: viewer ${viewer.id} sent an unexpected rtc_answer; dropped`);
      return;
    }
    entry.pendingAnswer = null;
    clearTimeout(pending.timer);
    pending.resolve(sdp);
  }

  viewerChanged(viewer: ViewerPeer): void {
    const entry = this.#viewers.get(viewer);
    if (entry !== undefined) this.#schedule(viewer, entry);
  }

  viewerClosed(viewer: ViewerPeer): void {
    const entry = this.#viewers.get(viewer);
    if (entry === undefined) return;
    // No API call: the viewer's SFU session and its pulls die with its
    // PeerConnection (the SFU's 30 s GC).
    this.#viewers.delete(viewer);
    this.#rejectAnswer(entry, "viewer disconnected");
  }

  stats(): Record<string, number> {
    let pulls = 0;
    for (const entry of this.#viewers.values()) pulls += entry.pulled.size;
    return {
      robots: this.#robots.size,
      viewers: this.#viewers.size,
      pulls,
      pullsCompleted: this.#pullsCompleted,
      apiErrors: this.#apiErrors,
    };
  }

  #scheduleAll(): void {
    for (const [viewer, entry] of this.#viewers) this.#schedule(viewer, entry);
  }

  #schedule(viewer: ViewerPeer, entry: ViewerRtc): void {
    entry.dirty = true;
    if (!entry.running) void this.#run(viewer, entry);
  }

  async #run(viewer: ViewerPeer, entry: ViewerRtc): Promise<void> {
    entry.running = true;
    try {
      while (entry.dirty && !this.#disposed && this.#viewers.get(viewer) === entry) {
        entry.dirty = false;
        let why: string;
        try {
          if (await this.#reconcile(viewer, entry)) {
            entry.failures = 0;
            continue;
          }
          why = "waiting for the robot's tracks";
        } catch (e) {
          // Shutdown, a viewer that left, or a re-offer: not a failure.
          if (this.#disposed || this.#viewers.get(viewer) !== entry) return;
          if (e instanceof SessionGoneError && e.sessionId === entry.cfSessionId) {
            this.#viewerSessionGone(viewer, entry, e);
            return;
          }
          this.#apiErrors++;
          why = `setup failed: ${describe(e)}`;
          viewer.sendMsg({
            t: "error",
            code: "rtc_failed",
            message: `WebRTC setup failed: ${describe(e)}`,
          });
        }
        const delay = RECONCILE_BACKOFF_MS[
          Math.min(entry.failures, RECONCILE_BACKOFF_MS.length - 1)
        ];
        entry.failures++;
        console.log(`[relay] rtc: viewer ${viewer.id} ${why}; re-run in ${delay} ms`);
        // Armed, not awaited: an event during the wait re-runs at once.
        void this.#sleep(delay).then(() => this.#schedule(viewer, entry));
      }
    } finally {
      entry.running = false;
    }
  }

  /** The viewer's SFU session is gone for good: it must offer again. Its
   * pulls died with the session, so nothing is closed. */
  #viewerSessionGone(viewer: ViewerPeer, entry: ViewerRtc, e: SessionGoneError): void {
    this.#apiErrors++;
    this.#viewers.delete(viewer);
    this.#rejectAnswer(entry, "SFU session gone");
    console.log(`[relay] rtc: viewer ${viewer.id} SFU session gone (${describe(e)})`);
    viewer.sendMsg({
      t: "error",
      code: "rtc_session_gone",
      message: "the WebRTC session is gone at the SFU; send a new rtc_offer",
    });
  }

  /** True when settled, false when something is pending; throws on an API
   * failure. */
  async #reconcile(viewer: ViewerPeer, entry: ViewerRtc): Promise<boolean> {
    if (entry.cfSessionId === null) {
      const { sessionId, answerSdp } = await this.#cf.newSession(entry.offerSdp);
      if (this.#viewers.get(viewer) !== entry) return true;
      entry.cfSessionId = sessionId;
      viewer.sendMsg({ t: "rtc_answer", sdp: answerSdp });
      console.log(`[relay] rtc: viewer ${viewer.id} SFU session ${sessionId}`);
    }
    const viewerSession = entry.cfSessionId;
    const robotId = viewer.watched;
    const robot = robotId === null ? undefined : this.#robots.get(robotId);
    const robotSession = robot?.cfSessionId ?? null;
    const desired = new Set<string>();
    if (robot !== undefined && robotSession !== null) {
      for (const ch of viewer.subs) if (robot.mids.has(ch)) desired.add(ch);
    }
    for (const [ch, pull] of entry.pulled) {
      if (!desired.has(ch) || pull.robotSession !== robotSession) {
        entry.pulled.delete(ch);
        entry.closing.set(pull.mid, 0);
      }
    }
    if (entry.closing.size > 0) {
      await this.#close(viewer, entry, viewerSession);
      if (this.#viewers.get(viewer) !== entry) return true;
    }
    // A close the SFU has not confirmed keeps the run pending (retried with
    // backoff) without holding up the pulls.
    const closed = entry.closing.size === 0;
    if (robot === undefined || robotId === null || robotSession === null) return closed;
    const missing = [...desired].filter((ch) => !entry.pulled.has(ch));
    if (missing.length === 0) return closed;
    const undeclared = missing.filter((ch) => !robot.published.has(ch));
    if (undeclared.length > 0) {
      await this.#declare(robot, robotId, robotSession, undeclared);
      if (this.#viewers.get(viewer) !== entry || this.#robots.get(robotId) !== robot) return true;
    }
    const result = await this.#cf.addTracks(
      viewerSession,
      missing.map((ch) => ({
        location: "remote" as const,
        sessionId: robotSession,
        trackName: ch,
      })),
    );
    if (this.#viewers.get(viewer) !== entry) return true; // re-offered: session abandoned
    const allocated: RtcTrack[] = [];
    const waiting: string[] = [];
    const hard: { ch: string; code: string }[] = [];
    for (const ch of missing) {
      const t = result.tracks.find((t) => t.trackName === ch);
      if (t !== undefined && t.errorCode === undefined && typeof t.mid === "string") {
        allocated.push({ ch, mid: t.mid });
      } else if (t?.errorCode === "not_found_track_error") {
        waiting.push(ch);
      } else {
        hard.push({ ch, code: t?.errorCode ?? "no result" });
      }
    }
    if (viewer.watched !== robotId || this.#robots.get(robotId) !== robot) {
      // The target moved during the pull: the run it scheduled closes them.
      for (const t of allocated) entry.closing.set(t.mid, 0);
      return false;
    }
    if (allocated.length > 0) {
      if (result.sdp === null) {
        for (const t of allocated) entry.closing.set(t.mid, 0);
        throw new Error("pull: the SFU returned no offer");
      }
      try {
        const answer = await this.#exchange(viewer, entry, {
          t: "rtc_offer",
          sdp: result.sdp,
          robotId,
          tracks: allocated,
        });
        await this.#cf.renegotiate(viewerSession, answer);
      } catch (e) {
        for (const t of allocated) entry.closing.set(t.mid, 0);
        throw e;
      }
      if (this.#viewers.get(viewer) !== entry) return true;
      for (const t of allocated) entry.pulled.set(t.ch, { mid: t.mid, robotId, robotSession });
      this.#pullsCompleted += allocated.length;
      console.log(
        `[relay] rtc: viewer ${viewer.id} pulls [${
          allocated.map((t) => t.ch).join(", ")
        }] from ${robotId}`,
      );
    }
    // Not found right after the declaration is the RTP propagation race (the
    // bridge encodes lazily); past TRACK_GC_MS the SFU collected the track.
    const now = this.#now();
    for (const ch of waiting) {
      const declaredAt = robot.published.get(ch);
      if (declaredAt !== undefined && now - declaredAt >= TRACK_GC_MS) robot.published.delete(ch);
    }
    if (hard.length > 0) {
      throw new CloudflareError(200, hard[0].code, `pull ${hard[0].ch}: ${hard[0].code}`);
    }
    return waiting.length === 0 && closed;
  }

  /** Force-close the pending mids; each stays pending until the SFU
   * confirmed it. An HTTP failure counts one try for every mid and is thrown
   * (the run backs off); a failed mid in a 200 counts one for itself. */
  async #close(viewer: ViewerPeer, entry: ViewerRtc, viewerSession: string): Promise<void> {
    const mids = [...entry.closing.keys()];
    let results: TrackResult[];
    try {
      results = await this.#cf.closeTracks(viewerSession, mids);
    } catch (e) {
      if (!(e instanceof SessionGoneError)) {
        for (const mid of mids) this.#closeFailed(viewer, entry, mid, describe(e));
      }
      throw e;
    }
    if (this.#viewers.get(viewer) !== entry) return;
    for (const mid of mids) {
      const t = results.find((t) => t.mid === mid);
      if (t !== undefined && t.errorCode === undefined) entry.closing.delete(mid);
      else this.#closeFailed(viewer, entry, mid, t?.errorCode ?? "no result");
    }
  }

  #closeFailed(viewer: ViewerPeer, entry: ViewerRtc, mid: string, why: string): void {
    const tries = (entry.closing.get(mid) ?? 0) + 1;
    if (tries < MAX_CLOSE_ATTEMPTS) {
      entry.closing.set(mid, tries);
      return;
    }
    entry.closing.delete(mid);
    console.log(
      `[relay] rtc: viewer ${viewer.id} gives up closing pull ${mid} after ${tries} tries (${why})`,
    );
  }

  /** Right before the first pull: the SFU ignores tracks declared with
   * sessions/new. Marked before the call so a concurrent run for another
   * viewer does not declare them twice; a failed call unmarks them all, a
   * failed track itself (the whole response is read before the error). */
  async #declare(
    robot: RobotRtc,
    robotId: string,
    robotSession: string,
    chs: string[],
  ): Promise<void> {
    const now = this.#now();
    for (const ch of chs) robot.published.set(ch, now);
    const refs = [...robot.mids]
      .filter(([ch]) => chs.includes(ch))
      .map(([ch, mid]) => ({ location: "local" as const, mid, trackName: ch }));
    let result: TracksResponse;
    try {
      result = await this.#cf.addTracks(robotSession, refs);
    } catch (e) {
      for (const ch of chs) robot.published.delete(ch);
      if (e instanceof SessionGoneError && this.#robots.get(robotId) === robot) {
        // Its pulls are dead everywhere; the bridge offers again once its
        // own peer connection fails.
        this.#robots.delete(robotId);
        console.log(`[relay] rtc: robot ${robotId} SFU session gone (${describe(e)})`);
        this.#scheduleAll();
      }
      throw e;
    }
    const failed: { ch: string; code: string }[] = [];
    for (const { mid, trackName } of refs) {
      const t = result.tracks.find((t) => t.mid === mid || t.trackName === trackName);
      if (t !== undefined && t.errorCode === undefined) continue;
      robot.published.delete(trackName);
      failed.push({ ch: trackName, code: t?.errorCode ?? "no result" });
    }
    if (failed.length > 0) {
      throw new CloudflareError(
        200,
        failed[0].code,
        `publish ${failed.map((f) => f.ch).join(", ")}: ${failed[0].code}`,
      );
    }
  }

  /** Send an SFU offer to the viewer and wait for its rtc_answer. */
  #exchange(viewer: ViewerPeer, entry: ViewerRtc, offer: Msg): Promise<string> {
    return new Promise<string>((resolve, reject) => {
      const timer = setTimeout(() => {
        if (entry.pendingAnswer?.timer !== timer) return;
        entry.pendingAnswer = null;
        reject(new Error(`no rtc_answer within ${this.#answerTimeoutMs} ms`));
      }, this.#answerTimeoutMs);
      entry.pendingAnswer = { resolve, reject, timer };
      viewer.sendMsg(offer);
    });
  }

  #rejectAnswer(entry: ViewerRtc, reason: string): void {
    const pending = entry.pendingAnswer;
    if (pending === null) return;
    entry.pendingAnswer = null;
    clearTimeout(pending.timer);
    pending.reject(new Error(reason));
  }
}
