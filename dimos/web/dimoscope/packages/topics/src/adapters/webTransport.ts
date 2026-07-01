/// <reference lib="dom" />
// createWebTransportTransport — read-only delivery over WebTransport (HTTP/3 / QUIC; browser-only;
// Deno has no stable WebTransport client). Talks to servers/webtransport.py, which size-routes the
// same [f64 gateway-send-ms][LC02] frames: small → DATAGRAMS (unreliable/unordered, no TCP head-of-
// line blocking), large → unidirectional STREAMS. We decode both through the shared `frameToSample`,
// so the typed messages + latency accounting are identical to every other mechanism. Read-only (like
// SSE/poll/WebRTC); control would route to a gateway.
//
// Cert: localhost QUIC needs TLS. The server generates a short-lived self-signed ECDSA cert and serves
// its SHA-256 at `${certUrl}/cert-hash`; we fetch it and pass it as `serverCertificateHashes` (no CA).
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../transport.ts";
import type { TopicInfo } from "../types.ts";
import { frameToSample } from "./gatewayFrame.ts";

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
  const caps: TransportCaps = { onDemand: false, discovery: "passive" };
  const url = deps.url.replace(/^ws/, "https");
  const host = new URL(url).hostname;
  const certHashUrl = deps.certHashUrl ?? `http://${host}:8094/cert-hash`;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let wt: WebTransport | undefined;

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
    readDatagrams(wt.datagrams.readable.getReader());
    readStreams(wt.incomingUnidirectionalStreams);
    wt.closed.then(() => statusCb?.("closed")).catch(() => statusCb?.("closed"));
  }

  const transport: Transport = {
    caps,
    label: "WebTransport",
    get commands(): CommandInfo[] {
      return [];
    },
    connect,
    subscribe() {}, // server forwards all subscribed topics; client filters on delivery
    unsubscribe() {},
    publishTeleop() {}, // read-only channel
    publishGoal() {},
    rpc() {
      return Promise.reject(new Error("WebTransport transport is read-only"));
    },
    requestList() {},
    onSample(cb: (s: RawSample) => void) {
      sampleCb = cb;
    },
    onTopics(_cb: (t: TopicInfo[]) => void) {},
    onStatus(cb: (s: Status) => void) {
      statusCb = cb;
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
