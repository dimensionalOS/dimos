/// <reference lib="dom" />
// hybrid() — BOTH wires at once, subscriptions routed per topic by QoS lane:
//
//   sensor lane (pose/odom/imu/tf/joints…)  → WebTransport DATAGRAMS  (no HoL: ~3× flatter tail under loss)
//   command / default / bulk                → WebSocket               (reliable; real bulk throughput)
//   teleop / goal / rpc / ping              → WebSocket               (one control wire, one deadman)
//
// Why not one QUIC connection with both primitives? Measured on the real WAN (docs/benchmarks.md §3):
// the Python QUIC relay caps bulk at ~2.4 MB/s and bulk-on-QUIC starves same-connection datagrams,
// while WS carries 19.5 MB/s — so the best of each column is two connections. WS is required (control
// plane); WT is opportunistic — if it can't connect, or dies mid-session, its topics rebind to WS and
// the label reflects the degraded mode. Note the shared-bottleneck caveat: two connections still share
// one link queue, so gateway-side QoS (priority outbox + egress pacing) remains the first-line defense.
import { createGatewayWsTransport } from "./gatewayWs.ts";
import { createWebTransportTransport } from "./webTransport.ts";
import { defaultLane, type Lane } from "../qos.ts";
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

export interface HybridOpts {
  /** WebSocket (control + reliable lanes) port on the gateway host. Default 8080. */
  wsPort?: number;
  /** WebTransport (QUIC/UDP, sensor lanes) port on the gateway host. Default 8443. */
  wtPort?: number;
  /** How long to wait for the WT side before running WS-only (ms). Default 4000. */
  timeoutMs?: number;
  /** Test seam: build the two wires (defaults to the real transports). */
  _mkWires?: (url: string) => { ws: Transport; wt?: Transport };
}

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

/** Which wire a lane rides. Only `sensor` (small, latest-wins, loss-tolerant) goes to datagrams;
 *  command stays on the control wire, bulk frames don't fit datagrams, default plays it safe. */
export function laneWire(lane: Lane): "ws" | "wt" {
  return lane === "sensor" ? "wt" : "ws";
}

export const hybrid = (opts?: HybridOpts): TransportFactory => (url) => {
  const { wsUrl, wtUrl, certUrl } = derive(url, opts?.wsPort ?? 8080, opts?.wtPort ?? 8443);
  const wires = opts?._mkWires ? opts._mkWires(url) : {
    ws: createGatewayWsTransport({ url: wsUrl, reconnect: true }),
    wt: typeof (globalThis as { WebTransport?: unknown }).WebTransport === "undefined"
      ? undefined
      : createWebTransportTransport({ url: wtUrl, certHashUrl: certUrl }),
  };
  const ws = wires.ws;
  const wt = wires.wt;

  let wtOpen = false;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let commandsCb: ((c: CommandInfo[]) => void) | undefined;

  // topic → its current wire + last qos (for rebinds when WT dies / lane overrides change).
  const bound = new Map<string, { wire: "ws" | "wt"; qos?: Qos }>();
  // merged discovery (topic → type), so lane inference has the message type.
  const types = new Map<string, string>();

  function wireFor(topic: string, qos?: Qos): "ws" | "wt" {
    const lane = (qos as { lane?: Lane } | undefined)?.lane ??
      defaultLane(topic, types.get(topic) ?? "");
    return wtOpen ? laneWire(lane) : "ws";
  }

  function doSubscribe(topic: string, qos?: Qos) {
    const target = wireFor(topic, qos);
    const prev = bound.get(topic);
    if (prev && prev.wire !== target) {
      (prev.wire === "wt" ? wt : ws)?.unsubscribe(topic);
    }
    bound.set(topic, { wire: target, qos });
    (target === "wt" ? wt! : ws).subscribe(topic, qos);
  }

  /** WT died mid-session → carry its topics on WS (degrade, don't drop). */
  function rebindWtToWs() {
    for (const [topic, b] of bound) {
      if (b.wire === "wt") {
        b.wire = "ws";
        ws.subscribe(topic, b.qos);
      }
    }
  }

  function mergeTopics(list: TopicInfo[]) {
    for (const t of list) types.set(t.topic, t.type);
    topicsCb?.([...types].map(([topic, type]) => ({ topic, type })));
  }

  async function connect(): Promise<void> {
    statusCb?.("connecting");
    ws.onSample((s) => sampleCb?.(s));
    ws.onTopics(mergeTopics);
    ws.onCommands?.((c) => commandsCb?.(c));
    ws.onStatus((s) => statusCb?.(s)); // WS is the control plane: its status is THE status
    await ws.connect();
    if (wt) {
      wt.onSample((s) => sampleCb?.(s));
      wt.onTopics(mergeTopics);
      wt.onStatus((s) => {
        if (s === "closed" && wtOpen) {
          wtOpen = false;
          rebindWtToWs();
        }
      });
      try {
        await withTimeout(wt.connect(), opts?.timeoutMs ?? 4000);
        wtOpen = true;
        // topics subscribed before WT came up: move sensor lanes onto datagrams.
        for (const [topic, b] of [...bound]) {
          if (b.wire === "ws" && wireFor(topic, b.qos) === "wt") doSubscribe(topic, b.qos);
        }
      } catch {
        try {
          wt.close();
        } catch { /* never connected */ }
      }
    }
    statusCb?.("open");
  }

  const caps: TransportCaps = {
    onDemand: true,
    discovery: "live",
    qos: { maxHz: "server", transport: ["priority", "reliability", "depth"] },
  };

  return {
    caps,
    get label(): string | undefined {
      const base = ws.label ?? "gateway";
      return wtOpen ? `${base}/HYB(WS+WT)` : `${base}/HYB(WS-only)`;
    },
    get commands(): CommandInfo[] {
      return ws.commands ?? [];
    },
    connect,
    close: () => {
      try {
        wt?.close();
      } catch { /* already closed */ }
      ws.close();
    },
    subscribe: doSubscribe,
    unsubscribe: (topic: string) => {
      const b = bound.get(topic);
      bound.delete(topic);
      (b?.wire === "wt" ? wt : ws)?.unsubscribe(topic);
    },
    publishTeleop: (x, z, ttl) => ws.publishTeleop(x, z, ttl),
    publishGoal: (x, y, z) => ws.publishGoal(x, y, z),
    rpc: (target, method, args) => ws.rpc(target, method, args),
    ping: () => ws.ping ? ws.ping() : Promise.reject(new Error("ping unsupported")),
    requestList: () => {
      ws.requestList();
      if (wtOpen) wt?.requestList();
    },
    onSample: (cb) => void (sampleCb = cb),
    onTopics: (cb) => void (topicsCb = cb),
    onStatus: (cb) => void (statusCb = cb),
    onCommands: (cb) => void (commandsCb = cb),
  };
};
