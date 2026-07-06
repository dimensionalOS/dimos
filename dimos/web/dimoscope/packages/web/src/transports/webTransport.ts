/// <reference lib="dom" />
// Delivery over WebTransport (HTTP/3 / QUIC; browser-only). Talks to the native sidecar
// (gateway/wt-sidecar), which size-routes frames (small → datagrams, large → length-prefixed messages
// on one persistent uni stream). A bidirectional control stream
// carries the full WS control protocol (subscribe/QoS + teleop/goal/rpc via the shared SafetyEgress),
// so WT drives the robot alone. Cert: we fetch the server's self-signed SHA-256 for serverCertificateHashes.
import { GATEWAY_QOS } from "../qos.ts";
import type {
  ClockSample,
  CommandInfo,
  Qos,
  RawSample,
  Status,
  TopicInfo,
  Transport,
  TransportCaps,
} from "../types.ts";
import { frameToSample } from "./frame.ts";

export interface WebTransportDeps {
  url: string; // e.g. https://localhost:8443
  certHashUrl?: string; // where to GET the cert sha256 hex (default: the gateway's http://<host>:8080/cert)
}

function hexToBytes(hex: string): Uint8Array {
  const h = hex.trim();
  const out = new Uint8Array(h.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(h.substr(i * 2, 2), 16);
  return out;
}

/// Ack the sidecar every this many raw bulk-stream bytes consumed ({op:"bulk-ack", n:cumulative}).
/// The sidecar's bulk drain gates on written − acked (credit), so on a rate-capped link the
/// standing queue stays ~WT_BULK_TARGET_MS instead of send_window ÷ link-rate. Byte-granular
/// (headers + partial frames count) — the sidecar gates mid-frame, so frame-granular acks would
/// deadlock. ~40 B per ack; at 19 MB/s that is ~600 control lines/s.
export const BULK_ACK_QUANTUM = 32768;

/// Accumulates raw bulk-stream bytes and emits the cumulative total every BULK_ACK_QUANTUM.
export function createBulkAckCounter(send: (cumulative: number) => void): (n: number) => void {
  let read = 0;
  let acked = 0;
  return (n: number) => {
    read += n;
    if (read - acked >= BULK_ACK_QUANTUM) {
      acked = read;
      send(read);
    }
  };
}

// Large frames arrive as [u32be length][frame]… on one persistent unidirectional stream (the
// gateway opens it on the first big frame). Reassemble across read chunks without concatenating
// the whole backlog per chunk. `onBytes` sees every raw byte read off the stream (bulk-ack credit).
export async function readBulkStream(
  stream: ReadableStream<Uint8Array>,
  deliver: (frame: Uint8Array) => void,
  onBytes?: (n: number) => void,
) {
  const sr = stream.getReader();
  const chunks: Uint8Array[] = [];
  let total = 0;
  const take = (n: number): Uint8Array => { // caller guarantees total >= n
    const out = new Uint8Array(n);
    let off = 0;
    while (off < n) {
      const c = chunks[0];
      const want = n - off;
      if (c.length <= want) {
        out.set(c, off);
        off += c.length;
        chunks.shift();
      } else {
        out.set(c.subarray(0, want), off);
        chunks[0] = c.subarray(want);
        off = n;
      }
    }
    total -= n;
    return out;
  };
  let need = -1; // payload bytes of the frame being assembled; -1 = header not read yet
  for (;;) {
    const { value, done } = await sr.read();
    if (done) break;
    if (!value?.length) continue;
    chunks.push(value);
    total += value.length;
    onBytes?.(value.length);
    for (;;) {
      if (need < 0) {
        if (total < 4) break;
        const h = take(4);
        need = ((h[0] << 24) | (h[1] << 16) | (h[2] << 8) | h[3]) >>> 0;
      }
      if (total < need) break;
      deliver(take(need));
      need = -1;
    }
  }
}

export const createWebTransportTransport = (deps: WebTransportDeps): Transport => {
  // Same gateway as the WS adapter: server-side downsampling + per-client priority outbox.
  const caps: TransportCaps = {
    onDemand: true,
    discovery: "passive",
    qos: GATEWAY_QOS,
  };
  const url = deps.url.replace(/^ws/, "https");
  const host = new URL(url).hostname;
  const certHashUrl = deps.certHashUrl ?? `http://${host}:8080/cert`;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let wt: WebTransport | undefined;
  let ctlWriter: WritableStreamDefaultWriter<Uint8Array> | undefined;
  const wantSubs = new Set<string>(); // desired subs (re-sent when the control stream opens)
  const subQos = new Map<string, Qos>(); // declared QoS per topic
  let topics: TopicInfo[] = [];
  const enc = new TextEncoder();
  const pending = new Map<number, { resolve: (v: unknown) => void; reject: (e: Error) => void }>();
  const pendingPings = new Map<number, (serverTs: number) => void>();
  let rpcId = 1;
  let commands: CommandInfo[] = []; // @rpc commands the gateway advertises (from hello.rpc)
  let commandsCb: ((c: CommandInfo[]) => void) | undefined;

  function deliver(bytes: Uint8Array) {
    const s = frameToSample(bytes, Date.now());
    if (s) sampleCb?.(s);
  }

  async function readDatagrams(r: ReadableStreamDefaultReader<Uint8Array>) {
    for (;;) {
      const { value, done } = await r.read();
      if (done) break;
      if (value) deliver(value);
    }
  }

  // Cumulative bulk-ack credit for the sidecar's drain gate. Counted across all incoming uni
  // streams (the sidecar opens exactly one per session); recreated per connect() — the sidecar's
  // written counter starts at 0 for each new session.
  let onBulkBytes = createBulkAckCounter((n) => sendControl({ op: "bulk-ack", n }));

  async function readStreams(streams: ReadableStream<ReadableStream<Uint8Array>>) {
    const reader = streams.getReader();
    for (;;) {
      const { value: stream, done } = await reader.read();
      if (done) break;
      if (stream) readBulkStream(stream, deliver, onBulkBytes).catch(() => {});
    }
  }

  function sendControl(obj: unknown) {
    ctlWriter?.write(enc.encode(JSON.stringify(obj) + "\n")).catch(() => {});
  }

  function sendSub(topic: string, qos?: Qos) {
    sendControl({
      op: "subscribe",
      topic,
      maxHz: qos?.maxHz,
      priority: qos?.priority,
      reliability: qos?.reliability,
      depth: qos?.depth,
    });
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

  async function readControl(r: ReadableStreamDefaultReader<Uint8Array>) {
    const dec = new TextDecoder();
    let buf = "";
    for (;;) {
      const { value, done } = await r.read();
      if (done) break;
      buf += dec.decode(value, { stream: true });
      let nl: number;
      while ((nl = buf.indexOf("\n")) >= 0) {
        const line = buf.slice(0, nl);
        buf = buf.slice(nl + 1);
        if (!line.trim()) continue;
        try {
          handleControl(JSON.parse(line));
        } catch {
          // ignore malformed control line
        }
      }
    }
  }

  async function connect(): Promise<void> {
    statusCb?.("connecting");
    const hashHex = await (await fetch(certHashUrl)).text();
    wt = new WebTransport(url, {
      serverCertificateHashes: [{
        algorithm: "sha-256",
        value: hexToBytes(hashHex) as BufferSource,
      }],
    });
    await wt.ready;
    statusCb?.("open");
    onBulkBytes = createBulkAckCounter((n) => sendControl({ op: "bulk-ack", n }));
    // Bidirectional control stream: our subscribe/list ops out, the gateway's hello/topic in.
    const ctl = await wt.createBidirectionalStream();
    ctlWriter = ctl.writable.getWriter();
    readControl(ctl.readable.getReader()).catch(() => {}); // loops end on close; swallow the drop
    for (const t of wantSubs) sendSub(t, subQos.get(t)); // replay any pre-connect subs
    sendControl({ op: "list" }); // triggers hello (topic list) for discovery
    readDatagrams(wt.datagrams.readable.getReader()).catch(() => {});
    readStreams(wt.incomingUnidirectionalStreams).catch(() => {});
    wt.closed.then(() => statusCb?.("closed")).catch(() => statusCb?.("closed"));
  }

  const transport: Transport = {
    caps,
    label: "WebTransport",
    get commands(): CommandInfo[] {
      return commands;
    },
    connect,
    subscribe(topic: string, qos?: Qos) {
      wantSubs.add(topic);
      if (qos) subQos.set(topic, qos);
      sendSub(topic, qos);
    },
    unsubscribe(topic: string) {
      wantSubs.delete(topic);
      subQos.delete(topic);
      sendControl({ op: "unsubscribe", topic });
    },
    // Write-path control → the gateway's shared SafetyEgress over the same control stream (clamp+deadman).
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
        // WAN box; instant once warm). Keep in sync with gatewayWs.ts.
        setTimeout(() => {
          if (pending.delete(id)) reject(new Error(`rpc ${target}/${method} timed out`));
        }, 20000);
      });
    },
    // NTP-style clock probe: offset = serverTs − (tSend + rtt/2), assuming a symmetric path.
    ping(): Promise<ClockSample> {
      const id = rpcId++;
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
        wt?.close();
      } catch {
        // already closed
      }
    },
  };
  return transport;
};
