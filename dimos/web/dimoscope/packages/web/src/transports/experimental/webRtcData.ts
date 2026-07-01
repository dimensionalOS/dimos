/// <reference lib="dom" />
// Read-only WebRTC DataChannel (browser-only), via gateway/transports/webrtc.py re-transmitter;
// carries the same [f64 gateway-send-ms][LC02] frames as WS/SSE/poll (decoded via shared
// frameToSample); unordered + lossy by default ⇒ no TCP head-of-line blocking.
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../../transport.ts";
import type { TopicInfo } from "../../types.ts";
import { frameToSample } from "../frame.ts";

export interface WebRtcDataDeps {
  url: string; // signaling WS/HTTP, e.g. ws://localhost:8093
  ordered?: boolean; // default false (unordered → no head-of-line blocking)
  maxRetransmits?: number; // default 0 (unreliable/lossy — the UDP-like comparison)
}

export const createWebRtcDataTransport = (deps: WebRtcDataDeps): Transport => {
  // qos.maxHz "client": DataChannel forwards all subscribed topics (no per-channel downsample) ⇒ rate
  // cap client-side; ordered/maxRetransmits are set at channel creation, not per-subscription.
  const caps: TransportCaps = { onDemand: false, discovery: "passive", qos: { maxHz: "client" } };
  const sigUrl = deps.url.replace(/^http/, "ws");
  let sampleCb: ((s: RawSample) => void) | undefined;
  // passive discovery — stores but never pushes a topic list.
  let _topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let pc: RTCPeerConnection | undefined;
  let dc: RTCDataChannel | undefined;
  let sig: WebSocket | undefined;

  async function waitIce(p: RTCPeerConnection): Promise<void> {
    if (p.iceGatheringState === "complete") return;
    await new Promise<void>((res) => {
      const check = () => {
        if (p.iceGatheringState === "complete") {
          p.removeEventListener("icegatheringstatechange", check);
          res();
        }
      };
      p.addEventListener("icegatheringstatechange", check);
    });
  }

  function connect(): Promise<void> {
    statusCb?.("connecting");
    pc = new RTCPeerConnection();
    dc = pc.createDataChannel("bench", {
      ordered: deps.ordered ?? false,
      maxRetransmits: deps.maxRetransmits ?? 0,
    });
    dc.binaryType = "arraybuffer";
    dc.onmessage = (e: MessageEvent) => {
      const s = frameToSample(new Uint8Array(e.data as ArrayBuffer), Date.now());
      if (s) sampleCb?.(s);
    };
    dc.onclose = () => statusCb?.("closed");

    // Resolve only once the DataChannel is actually OPEN (not right after the offer) — else a caller
    // begins measuring before data arrives (the browser "0" bug); time out so an unreachable ICE/UDP
    // rejects instead of hanging.
    return new Promise<void>((resolve, reject) => {
      let settled = false;
      const finish = (fn: () => void) => {
        if (settled) return;
        settled = true;
        fn();
      };
      const timer = setTimeout(
        () => finish(() => reject(new Error("webrtc datachannel open timeout"))),
        15000,
      );
      dc!.onopen = () => {
        statusCb?.("open");
        finish(() => {
          clearTimeout(timer);
          resolve();
        });
      };
      const ws = new WebSocket(sigUrl);
      sig = ws;
      ws.onopen = async () => {
        await pc!.setLocalDescription(await pc!.createOffer());
        await waitIce(pc!);
        ws.send(JSON.stringify({ op: "offer", sdp: pc!.localDescription!.sdp }));
        // wait for dc.onopen above — do NOT resolve here
      };
      ws.onmessage = async (e: MessageEvent) => {
        const m = JSON.parse(e.data as string);
        if (m.op === "answer") await pc!.setRemoteDescription({ type: "answer", sdp: m.sdp });
      };
      ws.onerror = () =>
        finish(() => {
          clearTimeout(timer);
          reject(new Error("webrtc signaling failed"));
        });
    });
  }

  const transport: Transport = {
    caps,
    label: "WebRTC-data",
    get commands(): CommandInfo[] {
      return [];
    },
    connect,
    subscribe() {}, // server forwards all subscribed topics; client filters on delivery
    unsubscribe() {},
    publishTeleop() {}, // read-only channel
    publishGoal() {},
    rpc() {
      return Promise.reject(new Error("WebRTC-data transport is read-only"));
    },
    requestList() {},
    onSample(cb: (s: RawSample) => void) {
      sampleCb = cb;
    },
    onTopics(cb: (t: TopicInfo[]) => void) {
      _topicsCb = cb;
    },
    onStatus(cb: (s: Status) => void) {
      statusCb = cb;
    },
    close() {
      try {
        dc?.close();
        pc?.close();
        sig?.close();
      } catch {
        // already closed
      }
    },
  };
  return transport;
};
