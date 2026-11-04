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
/** Peer failures before the handle stays failed (or falls back). */
export const RTC_MAX_FAILURES = 3;
/** Re-offer delays after the first, second and third failure. */
export const RTC_RETRY_MS = [1_000, 2_000, 4_000] as const;

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
  /** `rtc` while a track is live, `jpeg` on the fallback, `none` otherwise. */
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
  answered: boolean;
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
  readonly #settings: TopicOptions;

  constructor(
    readonly host: VideoHost,
    readonly keyExpr: string,
    readonly options: TopicVideoOptions,
    readonly onTrack: (track: MediaStreamTrack | null) => void,
  ) {
    const { fallback: _fallback, onSample: _onSample, profile: _profile, ...settings } = options;
    this.#settings = settings;
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
    if (status.phase === "ready") {
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
      const error = code === "unsupported_profile"
        ? new TopicError("rtc_unavailable", "rejected", "source offers no direct video")
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
    const peer: Peer = { ref, pc, answered: false, candidates: 0, timer: null };
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
      this.#mode = "rtc";
      this.#failures = 0;
      this.#transition({ phase: "ready" });
      this.onTrack(track);
    });
    pc.addEventListener("connectionstatechange", () => {
      if (this.#peer === peer && pc.connectionState === "failed") {
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
      peer.timer = setTimeout(() => {
        if (this.#peer === peer && !peer.answered) {
          this.#peerFailed(peer, new TopicError("rtc_failed", "interrupted", "rtc_answer_timeout"));
        }
      }, RTC_ANSWER_TIMEOUT_MS);
    }).catch((error) => {
      if (this.#peer !== peer) return;
      this.#peerFailed(peer, new TopicError("rtc_failed", "interrupted", String(error)));
    });
  }

  #signal(peer: Peer, msg: TopicRtcSignal): void {
    if (this.#peer !== peer) return;
    if (msg.t === "topic_rtc_answer") {
      peer.answered = true;
      if (peer.timer !== null) clearTimeout(peer.timer);
      peer.timer = null;
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
    const delay = RTC_RETRY_MS[Math.min(this.#failures, RTC_RETRY_MS.length) - 1];
    this.#retry = setTimeout(() => {
      this.#retry = null;
      const ref = this.host.routeOf(this.#inner);
      if (ref !== null && this.#peer === null && !this.#closed) this.#offer(ref);
    }, delay);
  }

  /** Direct video is over for this handle: the fallback, or a rejection. */
  #giveUp(error: TopicError): void {
    const direct = ["rtc_unavailable", "rtc_failed", "encoder_failed"].includes(error.code);
    if (this.options.fallback === "jpeg" && direct) {
      this.#unlisten();
      this.#inner.close();
      this.#mode = "jpeg";
      this.#inner = this.host.subscribe(
        this.keyExpr,
        { ...this.#settings, delivery: "latest", profile: VIDEO_FALLBACK_PROFILE },
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
    if (peer.timer !== null) clearTimeout(peer.timer);
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
