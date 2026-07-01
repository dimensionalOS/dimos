/// <reference lib="dom" />
// webtransport() — the production "bad internet" transport. It combines TWO connections under ONE
// Transport: a WebSocket for CONTROL (teleop / goal / rpc / discovery — the duplex, low-rate plane)
// and WebTransport for DATA (QUIC datagrams → no TCP head-of-line blocking under loss). The combine is
// encapsulated here so the client stays transport-agnostic. When the browser lacks WebTransport
// (non-Chrome), it transparently falls back to a plain WebSocket transport that does everything.
import { createGatewayWsTransport } from "./gatewayWs.ts";
import { createWebTransportTransport } from "./webTransport.ts";
import type { TransportFactory } from "../client.ts";
import type { CommandInfo, RawSample, Transport, TransportCaps } from "../transport.ts";

export interface WebtransportOpts {
  /** WebSocket (control) port on the gateway host. Default 8080. */
  wsPort?: number;
  /** WebTransport (QUIC/UDP data) port on the gateway host. Default 8443. */
  wtPort?: number;
}

// Derive the control WS URL, data WT URL, and cert-hash URL from a base URL or bare host.
function derive(url: string, wsPort: number, wtPort: number) {
  const host = url.replace(/^\w+:\/\//, "").replace(/\/.*$/, "").split(":")[0] || "localhost";
  return {
    wsUrl: `ws://${host}:${wsPort}/ws`,
    wtUrl: `https://${host}:${wtPort}`,
    certUrl: `http://${host}:${wsPort}/cert`,
  };
}

export const webtransport = (opts?: WebtransportOpts): TransportFactory => (url) => {
  const { wsUrl, wtUrl, certUrl } = derive(url, opts?.wsPort ?? 8080, opts?.wtPort ?? 8443);
  const control = createGatewayWsTransport({ url: wsUrl, reconnect: true });

  // No WebTransport in this browser → the WS transport does everything (data + control).
  if (typeof (globalThis as { WebTransport?: unknown }).WebTransport === "undefined") return control;

  const data = createWebTransportTransport({ url: wtUrl, certHashUrl: certUrl });
  let sampleCb: ((s: RawSample) => void) | undefined;

  const t: Transport = {
    get caps(): TransportCaps {
      // Data over WT (no HoL, on-demand via its control stream); teleop/rpc + discovery over WS.
      return { onDemand: data.caps.onDemand, discovery: "live", qos: data.caps.qos };
    },
    label: "WebTransport+WS",
    async connect() {
      data.onSample((s) => sampleCb?.(s)); // data samples come from WT
      await Promise.all([control.connect(), data.connect()]);
    },
    close() {
      control.close();
      data.close();
    },
    // Read-path (subscribe/unsubscribe/QoS) goes to WT — only subscribed topics transit, over its own
    // control stream. Do NOT subscribe over the WS too (that would double-deliver). WS is control-only.
    subscribe: (topic, qos) => data.subscribe(topic, qos),
    unsubscribe: (topic) => data.unsubscribe(topic),
    // Control → WS.
    publishTeleop: (x, z, ttl) => control.publishTeleop(x, z, ttl),
    publishGoal: (x, y, z) => control.publishGoal(x, y, z),
    rpc: (target, method, args) => control.rpc(target, method, args),
    requestList: () => control.requestList(),
    onSample: (cb) => void (sampleCb = cb),
    onTopics: (cb) => control.onTopics(cb),
    onStatus: (cb) => control.onStatus(cb),
    onCommands: (cb) => control.onCommands?.(cb),
    get commands(): CommandInfo[] {
      return control.commands ?? [];
    },
  };
  return t;
};
