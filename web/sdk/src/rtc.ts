// Viewer-side WebRTC: one RTCPeerConnection per relay connection toward the
// SFU, driven by the session. Nothing here touches RTCPeerConnection or
// MediaStream at import time: tests run under node.
import type { IceServer } from "@dimos/shared";

/** The subset of RTCPeerConnection the session needs, so tests can fake it. */
export interface PeerConnectionLike {
  readonly iceGatheringState: string;
  readonly connectionState: string;
  readonly localDescription: { sdp: string } | null;
  addTransceiver(kind: string, init?: { direction?: string }): unknown;
  createOffer(): Promise<RTCSessionDescriptionInit>;
  createAnswer(): Promise<RTCSessionDescriptionInit>;
  setLocalDescription(description?: RTCSessionDescriptionInit): Promise<void>;
  setRemoteDescription(description: RTCSessionDescriptionInit): Promise<void>;
  getTransceivers(): { mid: string | null; receiver: { track: MediaStreamTrack } }[];
  addEventListener(type: string, listener: (event: Event) => void): void;
  close(): void;
}

export type PeerConnectionFactory = (config: { iceServers: IceServer[] }) => PeerConnectionLike;

export function createPeerConnection(config: { iceServers: IceServer[] }): PeerConnectionLike {
  return new RTCPeerConnection(config);
}

// Non-trickle ICE: the SFU gets one complete offer. "complete" can lag a
// usable candidate set by ~10 s once TURN is configured, so the offer goes
// out after the first srflx/relay candidate plus a settle window; the cap
// covers networks that never produce one.
export const GATHER_TIMEOUT_MS = 10_000;
export const GATHER_SETTLE_MS = 400;

export class ViewerRtc {
  readonly #pc: PeerConnectionLike;

  /** `onFailed` fires when the connection reaches "failed", which is
   * terminal: the session drops the peer and offers again. "disconnected"
   * recovers by itself and "closed" only follows our own close(), so
   * neither is routed. */
  constructor(pc: PeerConnectionLike, onFailed: () => void) {
    this.#pc = pc;
    pc.addEventListener("connectionstatechange", () => {
      if (pc.connectionState === "failed") onFailed();
    });
  }

  /** One recvonly transceiver: the SFU reuses or adds transceivers per pull. */
  async offer(): Promise<string> {
    this.#pc.addTransceiver("video", { direction: "recvonly" });
    // Listeners first: setLocalDescription starts gathering.
    const gathered = this.#gather();
    await this.#pc.setLocalDescription(await this.#pc.createOffer());
    await gathered;
    return this.#localSdp();
  }

  accept(answerSdp: string): Promise<void> {
    return this.#pc.setRemoteDescription({ type: "answer", sdp: answerSdp });
  }

  async renegotiate(offerSdp: string): Promise<string> {
    await this.#pc.setRemoteDescription({ type: "offer", sdp: offerSdp });
    await this.#pc.setLocalDescription(await this.#pc.createAnswer());
    return this.#localSdp();
  }

  trackFor(mid: string): MediaStreamTrack | null {
    return this.#pc.getTransceivers().find((t) => t.mid === mid)?.receiver.track ?? null;
  }

  close(): void {
    this.#pc.close();
  }

  #localSdp(): string {
    const sdp = this.#pc.localDescription?.sdp;
    if (sdp === undefined) throw new Error("no local description");
    return sdp;
  }

  #gather(): Promise<void> {
    return new Promise<void>((resolve) => {
      let settleTimer: ReturnType<typeof setTimeout> | null = null;
      let done = false;
      const finish = (): void => {
        if (done) return;
        done = true;
        clearTimeout(cap);
        if (settleTimer !== null) clearTimeout(settleTimer);
        resolve();
      };
      const cap = setTimeout(finish, GATHER_TIMEOUT_MS);
      this.#pc.addEventListener("icegatheringstatechange", () => {
        if (this.#pc.iceGatheringState === "complete") finish();
      });
      this.#pc.addEventListener("icecandidate", (event) => {
        const candidate = (event as { candidate?: { type?: string } | null }).candidate;
        if (candidate === null || candidate === undefined) {
          finish(); // end-of-candidates
          return;
        }
        if ((candidate.type === "srflx" || candidate.type === "relay") && settleTimer === null) {
          settleTimer = setTimeout(finish, GATHER_SETTLE_MS);
        }
      });
    });
  }
}
