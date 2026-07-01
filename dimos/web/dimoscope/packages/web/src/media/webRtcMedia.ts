// browser receives camera as a real WebRTC video track, encoded once server-side (aiortc),
// GPU-decoded by <video> (~20–40× less bandwidth than JPEG); signaling rides a thin WS to the
// gateway; non-trickle ICE (gather-then-send); currently ONE active camera per channel (multi-cam
// grid will share a PeerConnection).
import type { MediaCaps, MediaChannel } from "../media.ts";
import type { Status } from "../transport.ts";

export interface WebRtcMediaDeps {
  gatewayUrl: string; // media node WS for WebRTC signaling (e.g. ws://host:8092)
}

export const createWebRtcMedia = (deps: WebRtcMediaDeps): MediaChannel => {
  const { gatewayUrl } = deps;
  const caps: MediaCaps = {
    output: "stream",
    codec: "h264",
    onDemand: true,
    multiStream: false,
    hardwareDecode: true,
  };
  let ws: WebSocket | undefined;
  let pc: RTCPeerConnection | undefined;
  let active: string | undefined; // the one subscribed camera topic
  let streamCb: ((id: string, s: MediaStream) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;

  function connect(): Promise<void> {
    if (ws && ws.readyState <= WebSocket.OPEN) return Promise.resolve();
    return new Promise((resolve, reject) => {
      statusCb?.("connecting");
      const sock = new WebSocket(gatewayUrl);
      ws = sock;
      sock.onopen = () => {
        statusCb?.("open");
        resolve();
      };
      sock.onmessage = (e) => onSignal(e);
      sock.onerror = () => reject(new Error("webrtc signaling ws error"));
      sock.onclose = () => statusCb?.("closed");
    });
  }

  function onSignal(e: MessageEvent): void {
    if (typeof e.data !== "string") return; // ignore data-plane binary (we never subscribe topics)
    let m: any;
    try {
      m = JSON.parse(e.data);
    } catch {
      return;
    }
    if (m.op === "webrtc-answer" && pc) {
      pc.setRemoteDescription({ type: "answer", sdp: m.sdp }).catch(() => {});
    }
  }

  function subscribe(streamId: string): void {
    if (active === streamId) return;
    teardownPc(); // single active camera — drop any prior one
    active = streamId;
    void negotiate(streamId);
  }

  async function negotiate(streamId: string): Promise<void> {
    const peer = new RTCPeerConnection();
    pc = peer;
    peer.addTransceiver("video", { direction: "recvonly" });
    peer.ontrack = (ev) => {
      if (active === streamId) streamCb?.(streamId, ev.streams[0]);
    };
    peer.onconnectionstatechange = () => {
      if (peer.connectionState === "failed" || peer.connectionState === "closed") {
        statusCb?.("closed");
      }
    };
    const offer = await peer.createOffer();
    await peer.setLocalDescription(offer);
    await iceComplete(peer);
    send({ op: "webrtc-offer", sdp: peer.localDescription?.sdp, topic: streamId });
  }

  /** Non-trickle: resolve once ICE candidate gathering finishes (instant on localhost). */
  function iceComplete(peer: RTCPeerConnection): Promise<void> {
    if (peer.iceGatheringState === "complete") return Promise.resolve();
    return new Promise((resolve) => {
      const check = () => {
        if (peer.iceGatheringState === "complete") {
          peer.removeEventListener("icegatheringstatechange", check);
          resolve();
        }
      };
      peer.addEventListener("icegatheringstatechange", check);
    });
  }

  function unsubscribe(streamId: string): void {
    if (active !== streamId) return;
    send({ op: "webrtc-stop", topic: streamId });
    teardownPc();
    active = undefined;
  }

  function teardownPc(): void {
    pc?.close();
    pc = undefined;
  }
  function send(obj: unknown): void {
    if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }

  function close(): void {
    teardownPc();
    ws?.close();
    active = undefined;
  }

  return {
    caps,
    label: "webrtc (aiortc gateway)",
    connect,
    subscribe,
    unsubscribe,
    onStream(cb: (id: string, s: MediaStream) => void): void {
      streamCb = cb;
    },
    onFrame() {}, // n/a — this channel is "stream"
    onStatus(cb: (s: Status) => void): void {
      statusCb = cb;
    },
    close,
  };
};
