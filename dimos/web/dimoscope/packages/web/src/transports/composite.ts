/// <reference lib="dom" />
// webtransport() — one WebTransport connection for data + control, with a transparent WebSocket
// fallback when the browser lacks WT or UDP is blocked. WS always works; WT just adds no-HoL/lower jitter.
import { createGatewayWsTransport } from "./gatewayWs.ts";
import { createWebTransportTransport } from "./webTransport.ts";
import type { TransportFactory } from "../client.ts";
import type {
  CommandInfo,
  Qos,
  RawSample,
  Status,
  TopicInfo,
  Transport,
  TransportCaps,
} from "../types.ts";

export interface WebtransportOpts {
  /** WebSocket (control + fallback) port on the gateway host. Default 8080. */
  wsPort?: number;
  /** WebTransport (QUIC/UDP) port on the gateway host. Default 8443. */
  wtPort?: number;
  /** How long to wait for the WT connection before falling back to WS (ms). Default 4000. */
  timeoutMs?: number;
}

// Derive the WT data URL, WS fallback URL, and cert-hash URL from a base URL or bare host.
function derive(url: string, wsPort: number, wtPort: number) {
  const host = url.replace(/^\w+:\/\//, "").replace(/\/.*$/, "").split(":")[0] || "localhost";
  return {
    wsUrl: `ws://${host}:${wsPort}/ws`,
    wtUrl: `https://${host}:${wtPort}`,
    certUrl: `http://${host}:${wsPort}/cert`,
  };
}

function withTimeout<T>(p: Promise<T>, ms: number): Promise<T> {
  return Promise.race([
    p,
    new Promise<never>((_, reject) => setTimeout(() => reject(new Error("timeout")), ms)),
  ]);
}

/** One Transport that prefers `primary` and falls back to `mkFallback()` on connect failure/timeout.
 *  Callbacks registered before connect() are buffered and re-registered on whichever wire wins; the
 *  wrapper owns the connecting→open status so the WT→WS swap never surfaces a spurious "closed". */
function createAutoFallback(
  primary: Transport,
  mkFallback: () => Transport,
  timeoutMs: number,
): Transport {
  let active: Transport | undefined;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let commandsCb: ((c: CommandInfo[]) => void) | undefined;

  function wire(t: Transport) {
    if (sampleCb) t.onSample(sampleCb);
    if (topicsCb) t.onTopics(topicsCb);
    if (commandsCb) t.onCommands?.(commandsCb);
    t.onStatus((s) => {
      if (active === t) statusCb?.(s); // suppress a losing wire's status during selection
    });
  }

  async function connect(): Promise<void> {
    statusCb?.("connecting");
    wire(primary);
    try {
      await withTimeout(primary.connect(), timeoutMs);
      active = primary;
      statusCb?.("open");
    } catch {
      try {
        primary.close();
      } catch { /* already closing */ }
      const fb = mkFallback();
      wire(fb);
      active = fb;
      await fb.connect();
      statusCb?.("open");
    }
  }

  const caps: TransportCaps = {
    onDemand: true,
    discovery: "live",
    qos: { maxHz: "server", transport: ["priority", "reliability", "depth"] },
  };

  return {
    caps,
    get label(): string | undefined {
      return active?.label ?? "Auto (WT→WS)";
    },
    get commands(): CommandInfo[] {
      return active?.commands ?? [];
    },
    connect,
    close: () => active?.close(),
    subscribe: (topic: string, qos?: Qos) => active?.subscribe(topic, qos),
    unsubscribe: (topic: string) => active?.unsubscribe(topic),
    publishTeleop: (x: number, z: number, ttl?: number) => active?.publishTeleop(x, z, ttl),
    publishGoal: (x: number, y: number, z?: number) => active?.publishGoal(x, y, z),
    rpc: (target: string, method: string, args?: unknown[]) =>
      active ? active.rpc(target, method, args) : Promise.reject(new Error("not connected")),
    requestList: () => active?.requestList(),
    onSample: (cb) => void (sampleCb = cb),
    onTopics: (cb) => void (topicsCb = cb),
    onStatus: (cb) => void (statusCb = cb),
    onCommands: (cb) => void (commandsCb = cb),
  };
}

export const webtransport = (opts?: WebtransportOpts): TransportFactory => (url) => {
  const { wsUrl, wtUrl, certUrl } = derive(url, opts?.wsPort ?? 8080, opts?.wtPort ?? 8443);
  const mkWs = () => createGatewayWsTransport({ url: wsUrl, reconnect: true });
  // No WebTransport in this browser → WS does everything (data + control).
  if (typeof (globalThis as { WebTransport?: unknown }).WebTransport === "undefined") return mkWs();
  const wt = createWebTransportTransport({ url: wtUrl, certHashUrl: certUrl });
  return createAutoFallback(wt, mkWs, opts?.timeoutMs ?? 4000);
};
