/// <reference lib="dom" />
// Delivery over WebTransport (HTTP/3 / QUIC; browser-only). Talks to gateway/transports/webtransport.py,
// which size-routes frames (small → datagrams, large → uni streams). A bidirectional control stream
// carries the full WS control protocol (subscribe/QoS + teleop/goal/rpc via the shared SafetyEgress),
// so WT drives the robot alone. Cert: we fetch the server's self-signed SHA-256 for serverCertificateHashes.
import { applyCaps } from "../qos.ts";
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../transport.ts";
import type { Qos, TopicInfo } from "../types.ts";
import { frameToSample } from "./frame.ts";

export interface WebTransportDeps {
  url: string; // e.g. https://localhost:8093
  certHashUrl?: string; // where to GET the cert sha256 hex (default: http://<host>:8094/cert-hash)
}

function hexToBytes(hex: string): Uint8Array {
  const h = hex.trim();
  const out = new Uint8Array(h.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(h.substr(i * 2, 2), 16);
  return out;
}

export const createWebTransportTransport = (deps: WebTransportDeps): Transport => {
  // maxHz "server" ⇒ gateway downsamples per subscriber+topic; priority/reliability/depth feed its
  // per-client priority outbox (same fields the WS adapter declares).
  const caps: TransportCaps = {
    onDemand: true,
    discovery: "passive",
    qos: { maxHz: "server", transport: ["priority", "reliability", "depth"] },
  };
  const url = deps.url.replace(/^ws/, "https");
  const host = new URL(url).hostname;
  const certHashUrl = deps.certHashUrl ?? `http://${host}:8094/cert-hash`;
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

  // Each large frame arrives as its own unidirectional stream; read it fully → one frame.
  async function readStreams(streams: ReadableStream<ReadableStream<Uint8Array>>) {
    const reader = streams.getReader();
    for (;;) {
      const { value: stream, done } = await reader.read();
      if (done) break;
      if (!stream) continue;
      (async () => {
        const sr = stream.getReader();
        const chunks: Uint8Array[] = [];
        for (;;) {
          const { value, done: d } = await sr.read();
          if (d) break;
          if (value) chunks.push(value);
        }
        let len = 0;
        for (const c of chunks) len += c.length;
        const buf = new Uint8Array(len);
        let off = 0;
        for (const c of chunks) {
          buf.set(c, off);
          off += c.length;
        }
        deliver(buf);
      })();
    }
  }

  function sendControl(obj: unknown) {
    ctlWriter?.write(enc.encode(JSON.stringify(obj) + "\n")).catch(() => {});
  }

  function sendSub(topic: string, qos?: Qos) {
    const q = qos ? applyCaps(qos, caps.qos!) : undefined;
    sendControl({
      op: "subscribe",
      topic,
      maxHz: q?.maxHz,
      priority: q?.priority,
      reliability: q?.reliability,
      depth: q?.depth,
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
        setTimeout(() => {
          if (pending.delete(id)) reject(new Error(`rpc ${target}/${method} timed out`));
        }, 8000);
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
