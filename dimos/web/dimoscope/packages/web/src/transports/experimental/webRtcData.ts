/// <reference lib="dom" />
// WebRTC DataChannel (browser-only), served by the native sidecar (gateway/wt-sidecar/src/rtc.rs);
// the gateway's /rtc websocket only relays SDP. Three channels over one SCTP association mirror
// what WebTransport gets from QUIC, so the transports compare fairly:
//   ctl   ordered+reliable   the full control protocol — subscribe/QoS/list/ping AND teleop/goal/
//                            rpc (the sidecar forwards the write path to the gateway SafetyEgress)
//   pose  unordered+lossy    [f64 send-ms][LC02] frames ≤1100 B, one per message (datagram analogue)
//   bulk  ordered+reliable   [u32be len][frame] chunked server-side; reassembled here exactly like
//                            the WT bulk stream (concatenate messages, length-prefix parse)
import type {
  ClockSample,
  CommandInfo,
  Qos,
  RawSample,
  Status,
  TopicInfo,
  Transport,
  TransportCaps,
} from "../../types.ts";
import { GATEWAY_QOS } from "../../qos.ts";
import { frameToSample } from "../frame.ts";

export interface WebRtcDataDeps {
  url: string; // signaling WS/HTTP, e.g. ws://localhost:8080/rtc
  iceServers?: RTCIceServer[]; // default: one public STUN (no TURN — a relay would bench the relay)
}

const STUN: RTCIceServer[] = [{ urls: "stun:stun.l.google.com:19302" }];

export const createWebRtcDataTransport = (deps: WebRtcDataDeps): Transport => {
  // Same gateway read-path as /ws and WT: on-demand subscriptions + server-honored QoS fields.
  const caps: TransportCaps = { onDemand: true, discovery: "passive", qos: GATEWAY_QOS };
  const sigUrl = deps.url.replace(/^http/, "ws");
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let topics: TopicInfo[] = [];
  let pc: RTCPeerConnection | undefined;
  let ctl: RTCDataChannel | undefined;
  let sig: WebSocket | undefined;
  let pingId = 0;
  const pendingPings = new Map<number, (serverTs: number) => void>();
  let rpcId = 1;
  const pending = new Map<number, { resolve: (v: unknown) => void; reject: (e: Error) => void }>();
  let commands: CommandInfo[] = []; // @rpc commands the gateway advertises (from hello.rpc)
  let commandsCb: ((c: CommandInfo[]) => void) | undefined;

  function sendControl(obj: Record<string, unknown>) {
    if (ctl?.readyState === "open") ctl.send(JSON.stringify(obj));
  }

  function handleControl(
    m: {
      op?: string;
      topics?: TopicInfo[];
      topic?: string;
      type?: string;
      label?: string;
      rpc?: CommandInfo[];
      id?: number;
      res?: unknown;
      error?: string;
      serverTs?: number;
    },
  ) {
    if (m.op === "hello" || m.op === "topics") {
      if (m.label) transport.label = m.label;
      topics = m.topics ?? [];
      topicsCb?.(topics);
      if (Array.isArray(m.rpc)) {
        commands = m.rpc;
        commandsCb?.(commands);
      }
    } else if (m.op === "topic" && m.topic) {
      if (!topics.find((t) => t.topic === m.topic)) {
        topics.push({ topic: m.topic, type: m.type ?? "?" });
        topicsCb?.(topics);
      }
    } else if (m.op === "rpc-res" && m.id != null) {
      const p = pending.get(m.id);
      if (p) {
        pending.delete(m.id);
        if (m.error) p.reject(new Error(m.error));
        else p.resolve(m.res);
      }
    } else if (m.op === "pong" && m.id != null) {
      const f = pendingPings.get(m.id);
      if (f) {
        pendingPings.delete(m.id);
        f(m.serverTs ?? NaN);
      }
    }
  }

  // bulk = a byte stream in ≤64 KB messages: accumulate, then length-prefix parse (same framing as
  // the WebTransport bulk lane).
  let bulkBuf = new Uint8Array(0);
  function onBulkMessage(data: ArrayBuffer) {
    const next = new Uint8Array(bulkBuf.length + data.byteLength);
    next.set(bulkBuf);
    next.set(new Uint8Array(data), bulkBuf.length);
    bulkBuf = next;
    while (bulkBuf.length >= 4) {
      const len = new DataView(bulkBuf.buffer, bulkBuf.byteOffset).getUint32(0, false);
      if (bulkBuf.length < 4 + len) break;
      const s = frameToSample(bulkBuf.subarray(4, 4 + len), Date.now());
      if (s) sampleCb?.(s);
      bulkBuf = bulkBuf.subarray(4 + len);
    }
  }

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
    pc = new RTCPeerConnection({ iceServers: deps.iceServers ?? STUN });
    ctl = pc.createDataChannel("ctl", { ordered: true });
    const pose = pc.createDataChannel("pose", { ordered: false, maxRetransmits: 0 });
    const bulk = pc.createDataChannel("bulk", { ordered: true });
    pose.binaryType = "arraybuffer";
    bulk.binaryType = "arraybuffer";
    ctl.onmessage = (e: MessageEvent) => {
      try {
        handleControl(JSON.parse(e.data as string));
      } catch {
        // bad control JSON — drop, like the other transports
      }
    };
    pose.onmessage = (e: MessageEvent) => {
      const s = frameToSample(new Uint8Array(e.data as ArrayBuffer), Date.now());
      if (s) sampleCb?.(s);
    };
    bulk.onmessage = (e: MessageEvent) => onBulkMessage(e.data as ArrayBuffer);
    ctl.onclose = () => statusCb?.("closed");

    // Resolve only when all three channels are OPEN (measuring before data can flow is the browser
    // "0" bug); time out so an unreachable ICE/UDP path rejects instead of hanging.
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
      let open = 0;
      const onOpen = () => {
        if (++open === 3) {
          statusCb?.("open");
          finish(() => {
            clearTimeout(timer);
            resolve();
          });
        }
      };
      ctl!.onopen = onOpen;
      pose.onopen = onOpen;
      bulk.onopen = onOpen;

      const ws = new WebSocket(sigUrl);
      sig = ws;
      ws.onopen = async () => {
        await pc!.setLocalDescription(await pc!.createOffer());
        await waitIce(pc!);
        ws.send(JSON.stringify({ op: "offer", sdp: pc!.localDescription!.sdp }));
        // wait for the channels to open — do not resolve here
      };
      ws.onmessage = async (e: MessageEvent) => {
        const m = JSON.parse(e.data as string);
        if (m.op === "answer") await pc!.setRemoteDescription({ type: "answer", sdp: m.sdp });
        else if (m.op === "error") {
          // relay-level failure (sidecar down / answer timeout) — fail fast, don't wait 15 s
          finish(() => {
            clearTimeout(timer);
            reject(new Error(`webrtc signaling: ${m.error}`));
          });
        }
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
      return commands;
    },
    connect,
    subscribe(topic: string, qos?: Qos) {
      sendControl({
        op: "subscribe",
        topic,
        maxHz: qos?.maxHz,
        priority: qos?.priority,
        reliability: qos?.reliability,
        depth: qos?.depth,
      });
    },
    unsubscribe(topic: string) {
      sendControl({ op: "unsubscribe", topic });
    },
    // Write path over ctl → the sidecar forwards to the gateway's shared SafetyEgress (clamp+deadman).
    publishTeleop(linearX: number, angularZ: number, ttlMs?: number) {
      sendControl({ op: "teleop", linearX, angularZ, ttlMs });
    },
    publishGoal(x: number, y: number, z = 0) {
      sendControl({ op: "goal", x, y, z });
    },
    rpc(target: string, method: string, args: unknown[] = []): Promise<unknown> {
      const id = rpcId++;
      return new Promise((resolve, reject) => {
        pending.set(id, { resolve, reject });
        sendControl({ op: "rpc", id, target, method, args });
        // 20 s: the gateway's first RPC to a module waits for zenoh route establishment (>8 s on a
        // WAN box; instant once warm). Keep in sync with gatewayWs.ts / webTransport.ts.
        setTimeout(() => {
          if (pending.delete(id)) reject(new Error(`rpc ${target}/${method} timed out`));
        }, 20000);
      });
    },
    // NTP-style clock probe: offset = serverTs − (tSend + rtt/2), assuming a symmetric path.
    ping(): Promise<ClockSample> {
      const id = pingId++;
      const t0 = Date.now();
      return new Promise((resolve, reject) => {
        pendingPings.set(id, (serverTs) => {
          const rttMs = Date.now() - t0;
          resolve({ rttMs, offsetMs: serverTs - (t0 + rttMs / 2) });
        });
        sendControl({ op: "ping", id });
        setTimeout(() => {
          if (pendingPings.delete(id)) reject(new Error("ping timed out"));
        }, 3000);
      });
    },
    requestList() {
      sendControl({ op: "list" });
    },
    onSample(cb: (s: RawSample) => void) {
      sampleCb = cb;
    },
    onTopics(cb: (t: TopicInfo[]) => void) {
      topicsCb = cb;
    },
    onStatus(cb: (s: Status) => void) {
      statusCb = cb;
    },
    onCommands(cb: (c: CommandInfo[]) => void) {
      commandsCb = cb;
    },
    close() {
      try {
        ctl?.close();
        pc?.close();
        sig?.close();
      } catch {
        // already closed
      }
    },
  };
  return transport;
};
