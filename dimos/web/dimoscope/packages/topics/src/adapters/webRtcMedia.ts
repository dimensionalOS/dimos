// WebRtcMedia — the browser receives the camera as a real WebRTC video track, encoded ONCE
// server-side by the Python gateway (aiortc, reusing quest_hosted's CameraVideoTrack) and
// decoded by the browser's <video> pipeline (GPU, ~zero JS/frame). ~20–40× less bandwidth than
// the JPEG-topic path, and the main thread stays free. Works on every browser.
//
// Signaling rides a thin WS to the gateway (precedent: ZenohTsTransport opens its own control
// WS too), next to teleop/goal. Non-trickle ICE (gather-then-send) — trivial + instant on a LAN.
// Slice scope: ONE active camera per channel (CameraView shows one); the multi-cam grid will
// hoist this to a shared multi-track PeerConnection.
import type { MediaCaps, MediaChannel, VideoMeta } from "../media";
import type { Status } from "../transport";

export class WebRtcMedia implements MediaChannel {
  readonly caps: MediaCaps = {
    output: "stream",
    codec: "h264",
    onDemand: true,
    multiStream: false,
    hardwareDecode: true,
  };
  label = "webrtc (aiortc gateway)";
  private ws?: WebSocket;
  private pc?: RTCPeerConnection;
  private active?: string; // the one subscribed camera topic
  private streamCb?: (id: string, s: MediaStream) => void;
  private statusCb?: (s: Status) => void;

  constructor(private gatewayUrl: string) {}

  connect(): Promise<void> {
    if (this.ws && this.ws.readyState <= WebSocket.OPEN) return Promise.resolve();
    return new Promise((resolve, reject) => {
      this.statusCb?.("connecting");
      const ws = new WebSocket(this.gatewayUrl);
      this.ws = ws;
      ws.onopen = () => {
        this.statusCb?.("open");
        resolve();
      };
      ws.onmessage = (e) => this.onSignal(e);
      ws.onerror = () => reject(new Error("webrtc signaling ws error"));
      ws.onclose = () => this.statusCb?.("closed");
    });
  }

  private onSignal(e: MessageEvent): void {
    if (typeof e.data !== "string") return; // ignore data-plane binary (we never subscribe topics)
    let m: any;
    try {
      m = JSON.parse(e.data);
    } catch {
      return;
    }
    if (m.op === "webrtc-answer" && this.pc) {
      this.pc.setRemoteDescription({ type: "answer", sdp: m.sdp }).catch(() => {});
    }
  }

  subscribe(streamId: string): void {
    if (this.active === streamId) return;
    this.teardownPc(); // single active camera — drop any prior one
    this.active = streamId;
    void this.negotiate(streamId);
  }

  private async negotiate(streamId: string): Promise<void> {
    const pc = new RTCPeerConnection();
    this.pc = pc;
    pc.addTransceiver("video", { direction: "recvonly" });
    pc.ontrack = (ev) => {
      if (this.active === streamId) this.streamCb?.(streamId, ev.streams[0]);
    };
    pc.onconnectionstatechange = () => {
      if (pc.connectionState === "failed" || pc.connectionState === "closed") this.statusCb?.("closed");
    };
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await this.iceComplete(pc);
    this.send({ op: "webrtc-offer", sdp: pc.localDescription?.sdp, topic: streamId });
  }

  /** Non-trickle: resolve once ICE candidate gathering finishes (instant on localhost). */
  private iceComplete(pc: RTCPeerConnection): Promise<void> {
    if (pc.iceGatheringState === "complete") return Promise.resolve();
    return new Promise((resolve) => {
      const check = () => {
        if (pc.iceGatheringState === "complete") {
          pc.removeEventListener("icegatheringstatechange", check);
          resolve();
        }
      };
      pc.addEventListener("icegatheringstatechange", check);
    });
  }

  unsubscribe(streamId: string): void {
    if (this.active !== streamId) return;
    this.send({ op: "webrtc-stop", topic: streamId });
    this.teardownPc();
    this.active = undefined;
  }

  private teardownPc(): void {
    this.pc?.close();
    this.pc = undefined;
  }
  private send(obj: unknown): void {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  onStream(cb: (id: string, s: MediaStream) => void): void {
    this.streamCb = cb;
  }
  onFrame(): void {} // n/a — this channel is "stream"
  onStatus(cb: (s: Status) => void): void {
    this.statusCb = cb;
  }

  close(): void {
    this.teardownPc();
    this.ws?.close();
    this.active = undefined;
  }
}
