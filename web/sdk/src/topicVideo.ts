// Direct video for topics: a video.h264.v1 subscription whose media arrives
// on a MediaStreamTrack from a peer connection to the provider, signaled
// through the relay. One peer per handle; the provider encodes once per
// settings and packetizes per peer. Nothing here touches RTCPeerConnection at
// import time: tests run under node with a fake.
import {
  MAX_SDP_LEN,
  MAX_TOPIC_RTC_CANDIDATES,
  type Msg,
  type TopicRtcAnswerMsg,
  type TopicRtcCandidateMsg,
  type TopicRtcCloseMsg,
  type TopicSettings,
} from "@dimos/shared";
import type { PeerConnectionLike } from "./rtc.ts";
import {
  TopicError,
  type TopicOptions,
  type TopicSample,
  type TopicStatus,
  type TopicSubscription,
} from "./topics.ts";

export const VIDEO_PROFILE = "video.h264.v1";
export const VIDEO_FALLBACK_PROFILE = "image.jpeg.balanced.v1";
/** Mirrors the relay's answer deadline for an offer it forwarded. */
export const RTC_ANSWER_TIMEOUT_MS = 10_000;
/** An answered peer that has not connected by then counts as failed, without
 * waiting for however long the browser takes to report it. */
export const RTC_CONNECT_TIMEOUT_MS = 15_000;
/** Re-offer delays after the first and second failure; the third is final. */
export const RTC_RETRY_MS = [1_000, 2_000] as const;
/** Peer failures, with no media played since, before the handle stays failed
 * (or falls back). */
export const RTC_MAX_FAILURES = RTC_RETRY_MS.length + 1;

export type TopicVideoMode = "rtc" | "jpeg" | "none";
export interface TopicVideoOptions extends TopicOptions {
  /** Open image.jpeg.balanced.v1 instead when direct video is unavailable
   * or has failed for good; `mode()` then reports `jpeg`. */
  fallback?: "jpeg";
  /** The JPEG samples in `jpeg` mode. */
  onSample?: (sample: TopicSample) => void;
}
export interface TopicVideo {
  /** Resolves once a track plays, or once the JPEG fallback is ready. */
  ready(): Promise<void>;
  status(): TopicStatus;
  onStatus(callback: (status: TopicStatus) => void): () => void;
  /** `rtc` once media plays on the track, `jpeg` on the fallback, `none` otherwise. */
  mode(): TopicVideoMode;
  track(): MediaStreamTrack | null;
  /** The receiver behind the track, for jitterBufferTarget and stats. */
  receiver(): RTCRtpReceiver | null;
  settings(): TopicSettings;
  update(options: TopicOptions): Promise<TopicSettings>;
  close(): void;
}

export type TopicRtcSignal = TopicRtcAnswerMsg | TopicRtcCandidateMsg | TopicRtcCloseMsg;
export interface VideoRouteRef {
  id: string;
  route: string;
  gen: number;
}
/** What a video handle needs from the topic client. */
export interface VideoHost {
  /** An inner subscription; exclusive means it never shares a relay route. */
  subscribe(
    keyExpr: string,
    options: TopicOptions,
    onSample: (sample: TopicSample) => void,
    exclusive: boolean,
  ): TopicSubscription;
  /** The relay route an inner subscription is ready on, or null. */
  routeOf(handle: TopicSubscription): VideoRouteRef | null;
  /** Route incoming topic_rtc_* messages for subscription `id` to `sink`. */
  bind(id: string, sink: (msg: TopicRtcSignal) => void): void;
  unbind(id: string): void;
  /** The source and settings an inner subscription has in force. */
  requestOf(handle: TopicSubscription): TopicOptions;
  /** False while disconnected. */
  send(msg: Msg): boolean;
  createPeerConnection(): PeerConnectionLike;
}

const noop = () => {};
/** A relay or provider close reason, as the code the handle reports. */
function codeFor(reason: string): "rtc_unavailable" | "encoder_failed" | "rtc_failed" {
  if (reason === "rtc_limit" || reason === "rtc_unavailable") return "rtc_unavailable";
  if (reason === "encoder_failed") return "encoder_failed";
  return "rtc_failed";
}

interface Peer {
  ref: VideoRouteRef;
  pc: PeerConnectionLike;
  candidates: number;
  timer: ReturnType<typeof setTimeout> | null;
}

export class TopicVideoHandle implements TopicVideo {
  #inner: TopicSubscription;
  #unlisten: () => void;
  #peer: Peer | null = null;
  #track: MediaStreamTrack | null = null;
  #receiver: RTCRtpReceiver | null = null;
  #mode: TopicVideoMode = "none";
  #status: TopicStatus = { phase: "connecting" };
  #listeners = new Set<(status: TopicStatus) => void>();
  #failures = 0;
  #retry: ReturnType<typeof setTimeout> | null = null;
  #closed = false;
  #ready: { promise: Promise<void>; resolve: () => void; reject: (e: TopicError) => void };

  constructor(
    readonly host: VideoHost,
    readonly keyExpr: string,
    readonly options: TopicVideoOptions,
    readonly onTrack: (track: MediaStreamTrack | null) => void,
  ) {
    const { fallback: _fallback, onSample: _onSample, profile: _profile, ...settings } = options;
    this.#ready = deferred();
    this.#inner = host.subscribe(
      keyExpr,
      { ...settings, delivery: "latest", profile: VIDEO_PROFILE },
      noop,
      true,
    );
    this.#unlisten = this.#inner.onStatus((status) => this.#innerStatus(status));
  }

  ready(): Promise<void> {
    return this.#ready.promise;
  }
  status(): TopicStatus {
    return this.#status;
  }
  onStatus(callback: (status: TopicStatus) => void): () => void {
    this.#listeners.add(callback);
    callback(this.#status);
    return () => this.#listeners.delete(callback);
  }
  mode(): TopicVideoMode {
    return this.#mode;
  }
  track(): MediaStreamTrack | null {
    return this.#track;
  }
  receiver(): RTCRtpReceiver | null {
    return this.#receiver;
  }
  settings(): TopicSettings {
    return this.#inner.settings();
  }
  update(options: TopicOptions): Promise<TopicSettings> {
    if (options.profile !== undefined) {
      return Promise.reject(new TopicError("unsupported_option", "rejected"));
    }
    // A changed encoder closes the peer with settings_changed; the handle offers again.
    return this.#inner.update(options);
  }
  close(): void {
    if (this.#closed) return;
    this.#closed = true;
    if (this.#retry !== null) clearTimeout(this.#retry);
    this.#retry = null;
    this.#dropPeer();
    this.#inner.close();
    this.#transition({ phase: "closed" });
    this.#listeners.clear();
  }

  #transition(status: TopicStatus): void {
    if (this.#status.phase === "closed") return;
    if ((status.phase === "connecting" || status.phase === "interrupted") && this.#settled()) {
      this.#ready = deferred();
    }
    this.#status = status;
    if (status.phase === "ready") this.#ready.resolve();
    if (status.phase === "rejected" || status.phase === "closed") {
      this.#ready.reject(status.error ?? new TopicError("closed", "rejected"));
    }
    for (const listener of [...this.#listeners]) {
      try {
        listener(status);
      } catch (error) {
        console.error("topic video callback failed", error);
      }
    }
  }
  #settled(): boolean {
    const phase = this.#status.phase;
    return phase === "ready" || phase === "rejected";
  }

  /** The inner subscription moved: a ready route gets a peer, a lost one ends it. */
  #innerStatus(status: TopicStatus): void {
    if (this.#closed) return;
    if (this.#mode === "jpeg") {
      // The fallback carries the reason the direct path was given up.
      this.#transition({ ...status, error: this.#status.error ?? status.error });
      return;
    }
    if (status.phase === "connecting") {
      // Another route is on its way, as after a changed source; the old peer ended.
      this.#dropPeer();
      this.#transition(status);
    } else if (status.phase === "ready") {
      const ref = this.host.routeOf(this.#inner);
      if (ref === null) return;
      const current = this.#peer?.ref;
      if (current?.id === ref.id && current.route === ref.route && current.gen === ref.gen) return;
      this.#offer(ref);
    } else if (status.phase === "interrupted") {
      this.#dropPeer();
      this.#transition(status);
    } else if (status.phase === "rejected") {
      const code = status.error?.code;
      // A camera out of encoders is as unavailable as one without the profile.
      const error = code === "unsupported_profile"
        ? new TopicError("rtc_unavailable", "rejected", "source offers no direct video")
        : code === "variant_limit"
        ? new TopicError("rtc_unavailable", "rejected", code)
        : status.error ?? new TopicError("rejected", "rejected");
      this.#giveUp(error);
    } else if (status.phase === "closed") {
      this.close();
    }
  }

  #offer(ref: VideoRouteRef): void {
    this.#dropPeer();
    if (this.#retry !== null) clearTimeout(this.#retry);
    this.#retry = null;
    const pc = this.host.createPeerConnection();
    const peer: Peer = { ref, pc, candidates: 0, timer: null };
    this.#peer = peer;
    this.host.bind(ref.id, (msg) => this.#signal(peer, msg));
    pc.addEventListener("icecandidate", (event) => {
      if (this.#peer !== peer) return;
      const candidate = (event as RTCPeerConnectionIceEvent).candidate;
      // Host candidates only: the relay closes a peer past 64 of them.
      if (peer.candidates >= MAX_TOPIC_RTC_CANDIDATES) return;
      if (candidate !== null && candidate.candidate === "") return;
      peer.candidates++;
      this.host.send({
        t: "topic_rtc_candidate",
        ...ref,
        candidate: candidate?.candidate ?? "",
        mid: candidate?.sdpMid ?? "",
      });
    });
    pc.addEventListener("track", (event) => {
      if (this.#peer !== peer) return;
      const { track, receiver } = event as RTCTrackEvent;
      this.#track = track;
      this.#receiver = receiver ?? null;
      // This event only says the answer applied. The track plays, and the peer
      // has proven itself, when its first packet unmutes it.
      const playing = () => {
        if (this.#peer !== peer) return;
        this.#stopDeadline(peer);
        this.#mode = "rtc";
        this.#failures = 0;
        this.#transition({ phase: "ready" });
      };
      if (track.muted) track.addEventListener("unmute", playing, { once: true });
      else playing();
      this.onTrack(track);
    });
    pc.addEventListener("connectionstatechange", () => {
      if (this.#peer !== peer) return;
      // Connected with no media yet is a quiet camera, not a failure: no deadline.
      if (pc.connectionState === "connected") this.#stopDeadline(peer);
      else if (pc.connectionState === "failed") {
        this.#peerFailed(peer, new TopicError("rtc_failed", "interrupted", "no_direct_path"));
      }
    });
    pc.addTransceiver("video", { direction: "recvonly" });
    pc.createOffer().then(async (offer) => {
      await pc.setLocalDescription(offer);
      if (this.#peer !== peer) return;
      const sdp = pc.localDescription?.sdp ?? "";
      if (sdp.length === 0 || sdp.length > MAX_SDP_LEN) throw new Error("offer SDP out of bounds");
      if (!this.host.send({ t: "topic_rtc_offer", ...ref, sdp })) return; // the reconnect re-offers
      this.#startDeadline(peer, RTC_ANSWER_TIMEOUT_MS, "rtc_answer_timeout");
    }).catch((error) => {
      if (this.#peer !== peer) return;
      this.#peerFailed(peer, new TopicError("rtc_failed", "interrupted", String(error)));
    });
  }

  #signal(peer: Peer, msg: TopicRtcSignal): void {
    if (this.#peer !== peer) return;
    if (msg.t === "topic_rtc_answer") {
      this.#startDeadline(peer, RTC_CONNECT_TIMEOUT_MS, "no_direct_path");
      peer.pc.setRemoteDescription({ type: "answer", sdp: msg.sdp }).catch((error) => {
        if (this.#peer === peer) {
          this.#peerFailed(peer, new TopicError("rtc_failed", "interrupted", String(error)));
        }
      });
    } else if (msg.t === "topic_rtc_candidate") {
      const candidate = msg.candidate === ""
        ? null
        : { candidate: msg.candidate, sdpMid: msg.mid === "" ? null : msg.mid };
      peer.pc.addIceCandidate(candidate).catch(() => {}); // a stray late candidate is harmless
    } else if (msg.reason === "settings_changed" || msg.reason === "replaced") {
      // The provider feeds the peer from another encoder: offer again at once.
      this.#dropPeer();
      this.#transition({ phase: "connecting" });
      const ref = this.host.routeOf(this.#inner);
      if (ref !== null) this.#offer(ref);
    } else {
      const code = codeFor(msg.reason);
      if (code === "rtc_unavailable") {
        this.#dropPeer();
        this.#giveUp(new TopicError(code, "rejected", msg.reason));
      } else this.#peerFailed(peer, new TopicError(code, "interrupted", msg.reason));
    }
  }

  /** The peer fails unless it gets past this step in time. */
  #startDeadline(peer: Peer, ms: number, reason: string): void {
    this.#stopDeadline(peer);
    peer.timer = setTimeout(() => {
      this.#peerFailed(peer, new TopicError("rtc_failed", "interrupted", reason));
    }, ms);
  }
  #stopDeadline(peer: Peer): void {
    if (peer.timer !== null) clearTimeout(peer.timer);
    peer.timer = null;
  }

  /** A terminal failure of the peer: backoff and offer again, up to RTC_MAX_FAILURES. */
  #peerFailed(peer: Peer, error: TopicError): void {
    if (this.#peer !== peer) return;
    this.#dropPeer();
    this.#failures++;
    if (this.#failures >= RTC_MAX_FAILURES) {
      this.#giveUp(new TopicError(error.code, "rejected", error.message));
      return;
    }
    this.#transition({ phase: "interrupted", error });
    this.#retry = setTimeout(() => {
      this.#retry = null;
      const ref = this.host.routeOf(this.#inner);
      if (ref !== null && this.#peer === null && !this.#closed) this.#offer(ref);
    }, RTC_RETRY_MS[this.#failures - 1]);
  }

  /** Direct video is over for this handle: the fallback, or a rejection. */
  #giveUp(error: TopicError): void {
    if (this.#retry !== null) clearTimeout(this.#retry);
    this.#retry = null;
    const direct = ["rtc_unavailable", "rtc_failed", "encoder_failed"].includes(error.code);
    if (this.options.fallback === "jpeg" && direct) {
      // Not the options the handle opened with: a successful update() holds, and
      // the relay clamps the rest to the JPEG profile.
      const request = this.host.requestOf(this.#inner);
      this.#unlisten();
      this.#inner.close();
      this.#mode = "jpeg";
      this.#inner = this.host.subscribe(
        this.keyExpr,
        { ...request, delivery: "latest", profile: VIDEO_FALLBACK_PROFILE },
        this.options.onSample ?? noop,
        false,
      );
      this.#transition({ phase: "connecting", error });
      this.#unlisten = this.#inner.onStatus((status) => this.#innerStatus(status));
      return;
    }
    this.#mode = "none";
    this.#transition({ phase: "rejected", error });
  }

  #dropPeer(): void {
    const peer = this.#peer;
    if (peer === null) return;
    this.#peer = null;
    this.#stopDeadline(peer);
    this.host.unbind(peer.ref.id);
    peer.pc.close();
    if (this.#track !== null) {
      this.#track = null;
      this.#receiver = null;
      this.#mode = "none";
      this.onTrack(null);
    }
  }
}

function deferred() {
  let resolve!: () => void;
  let reject!: (error: TopicError) => void;
  const promise = new Promise<void>((yes, no) => {
    resolve = yes;
    reject = no;
  });
  promise.catch(() => {});
  return { promise, resolve, reject };
}
