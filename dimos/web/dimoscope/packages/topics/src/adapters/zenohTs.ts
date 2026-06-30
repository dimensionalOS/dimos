// createZenohTsTransport — the browser talks DIRECTLY to a Zenoh network via the official
// zenoh-ts client + a `zenoh-bridge-remote-api` WebSocket (default :10000). No gateway
// in the read path, so subscribe is TRUE per-client end-to-end on-demand: only the keys
// you declare ever transit to the browser (vs the LCM gateway, which receives every
// topic off multicast). The decode path is identical — the zenoh payload is the bare
// `lcm_encode` body (8-byte type hash + fields), exactly what the gateways relay, so the
// DimosClient decodes it with @dimos/msgs unchanged.
//
// SAFETY: teleop/goal do NOT go out as raw Zenoh `put`s (that would bypass the velocity
// clamp + deadman + stop-on-disconnect the gateways enforce). Instead this adapter holds
// a thin control WS to a gateway and forwards {op:teleop|goal|stop} — the trust boundary
// stays server-side. Pass `controlUrl` to enable driving.
//
// zenoh-ts is browser-only (no Bun/Node target) and the heavy client loads lazily inside
// connect(), so importing this module is cheap and a load failure can't break other paths.
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../transport.ts";
import type { TopicInfo } from "../types.ts";

const SCOUT_MS = 1800; // one-time discovery burst on the discovery key (then undeclared)

// zenoh key → {topic, type}; strips the `dimos/` namespace so the browser sees the SAME
// canonical names as the gateways + LCM (`/lidar`, not `/dimos/lidar`). bench/* keys are
// already un-prefixed and pass through unchanged.
//   "dimos/lidar/sensor_msgs.PointCloud2" → {/lidar,    sensor_msgs.PointCloud2}
//   "bench/p0/geometry_msgs.PoseStamped"  → {/bench/p0, geometry_msgs.PoseStamped}
function parseKey(key: string): { topic: string; type: string } | null {
  const slash = key.lastIndexOf("/");
  if (slash <= 0) return { topic: "/" + key, type: "?" };
  let topic = "/" + key.slice(0, slash);
  if (topic.startsWith("/dimos/")) topic = topic.slice("/dimos".length); // /dimos/lidar → /lidar
  return { topic, type: key.slice(slash + 1) };
}

export interface ZenohTsDeps {
  /** zenoh-bridge-remote-api WS (e.g. ws://localhost:10000) */
  remoteApiUrl: string;
  /** optional gateway WS for safe teleop/goal (e.g. ws://localhost:8088) */
  controlUrl?: string;
  /** wildcard scouted once for the topic list; "" disables discovery (e.g. the bench,
   *  which subscribes explicit keys). Defaults to "dimos/**". */
  discoveryKey?: string;
}

export const createZenohTsTransport = (deps: ZenohTsDeps): Transport => {
  const { remoteApiUrl, controlUrl } = deps;
  const discoveryKey = deps.discoveryKey ?? "dimos/**";

  const caps: TransportCaps = { onDemand: true, discovery: "passive" };
  let session: any;
  const subs = new Map<string, any>(); // topic -> zenoh Subscriber
  const rates = new Map<string, number>();
  const lastSent = new Map<string, number>();
  const topics = new Map<string, string>(); // topic -> type
  const keyBase = new Map<string, string>(); // canonical topic -> real zenoh key base ("/lidar" -> "dimos/lidar")
  // canonical browser topic → real zenoh subscribe key. parseKey() dropped the `dimos/`
  // namespace; re-apply it here (bench/* stays un-prefixed), preferring a base learned during
  // discovery so any non-standard namespace round-trips correctly.
  const topicToKey = (topic: string) =>
    (keyBase.get(topic) ?? (topic.startsWith("/bench/") ? topic.slice(1) : "dimos" + topic)) +
    "/**";
  let control: WebSocket | undefined;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  let commands: CommandInfo[] = [];
  let commandsCb: ((c: CommandInfo[]) => void) | undefined;
  const pending = new Map<number, { resolve: (v: unknown) => void; reject: (e: Error) => void }>();
  let rpcId = 1;

  async function connect(): Promise<void> {
    statusCb?.("connecting");
    const zenoh = await import("@eclipse-zenoh/zenoh-ts"); // lazy: only loads on this path
    session = await zenoh.Session.open(new zenoh.Config(remoteApiUrl));
    statusCb?.("open");
    // Discovery: Zenoh has no gateway "hello/list", so briefly scout the discovery key to
    // learn which (topic,type) keys exist, then undeclare it. Data subscriptions below are
    // the steady-state on-demand path; this is a one-time burst. discoveryKey "" disables it.
    if (discoveryKey) {
      try {
        const scout = await session.declareSubscriber(discoveryKey, {
          handler: (s: any) => note(String(s.keyexpr())),
        });
        setTimeout(() => scout.undeclare?.().catch(() => {}), SCOUT_MS);
      } catch {
        /* discovery is best-effort */
      }
    }
    ctrl(); // open the control WS now so the gateway's hello (rpc commands) arrives
  }

  function note(key: string) {
    const m = parseKey(key);
    if (!m || m.type === "?" || m.topic.includes("/rpc/")) return;
    const slash = key.lastIndexOf("/");
    if (slash > 0) keyBase.set(m.topic, key.slice(0, slash)); // remember real key base for subscribe()
    if (topics.get(m.topic) !== m.type) {
      topics.set(m.topic, m.type);
      topicsCb?.([...topics].map(([topic, type]) => ({ topic, type })));
    }
  }

  function subscribe(topic: string, maxHz?: number) {
    if (maxHz) rates.set(topic, maxHz);
    if (subs.has(topic) || !session) return;
    subs.set(topic, "pending"); // guard against double-declare races
    session
      .declareSubscriber(topicToKey(topic), { handler: (s: any) => onSampleZ(topic, s) })
      .then((sub: any) => {
        if (subs.get(topic) === "pending") subs.set(topic, sub);
        else sub.undeclare?.().catch(() => {}); // unsubscribed while declaring
      })
      .catch(() => subs.delete(topic));
  }

  function onSampleZ(topic: string, s: any) {
    const key = String(s.keyexpr());
    const m = parseKey(key);
    if (!m) return;
    note(key);
    const now = Date.now();
    const hz = rates.get(topic) ?? 0;
    if (hz > 0) {
      // Zenoh has no per-subscriber downsample, so throttle client-side.
      if (now - (lastSent.get(topic) ?? 0) < 1000 / hz) return;
      lastSent.set(topic, now);
    }
    const payload: Uint8Array = s.payload().toBytes(); // bare lcm_encode body (8-byte hash head)
    // No gatewaySendMs (no gateway hop) → DimosClient falls back to source-stamp latency.
    sampleCb?.({ topic: m.topic, type: m.type, payload, recvTs: now });
  }

  function unsubscribe(topic: string) {
    const sub = subs.get(topic);
    subs.delete(topic);
    rates.delete(topic);
    lastSent.delete(topic);
    if (sub && sub !== "pending") sub.undeclare?.().catch(() => {});
  }

  // ── teleop / goal: forwarded to a gateway so its clamp + deadman + stop-on-disconnect
  //    still apply (a browser doing raw Zenoh put would bypass all of that) ──────────────
  function ctrl(): WebSocket | undefined {
    if (!controlUrl) return undefined;
    const st = control?.readyState;
    if (control && (st === WebSocket.OPEN || st === WebSocket.CONNECTING)) return control;
    control = new WebSocket(controlUrl);
    control.onmessage = (e) => onControl(e); // rpc-res + the gateway hello (commands)
    return control;
  }
  function onControl(e: MessageEvent) {
    if (typeof e.data !== "string") return;
    let m: any;
    try {
      m = JSON.parse(e.data);
    } catch {
      return;
    }
    if (m.op === "hello" && Array.isArray(m.rpc)) {
      commands = m.rpc;
      commandsCb?.(commands);
    } else if (m.op === "rpc-res") {
      const p = pending.get(m.id);
      if (p) {
        pending.delete(m.id);
        if (m.error) p.reject(new Error(m.error));
        else p.resolve(m.res);
      }
    }
  }
  function ctrlSend(obj: unknown) {
    const ws = ctrl();
    if (!ws) return;
    const s = JSON.stringify(obj);
    if (ws.readyState === WebSocket.OPEN) ws.send(s);
    else ws.addEventListener("open", () => ws.send(s), { once: true });
  }
  function publishTeleop(linearX: number, angularZ: number, ttlMs?: number) {
    ctrlSend({ op: "teleop", linearX, angularZ, ttlMs });
  }
  function publishGoal(x: number, y: number, z = 0) {
    ctrlSend({ op: "goal", x, y, z });
  }
  function rpc(target: string, method: string, args: unknown[] = []): Promise<unknown> {
    const id = rpcId++;
    return new Promise((resolve, reject) => {
      pending.set(id, { resolve, reject });
      ctrlSend({ op: "rpc", id, target, method, args }); // routed to the gateway's RPC bridge
      setTimeout(() => {
        if (pending.delete(id)) reject(new Error(`rpc ${target}/${method} timed out`));
      }, 8000);
    });
  }

  function requestList() {} // discovery is the passive scout; nothing to request
  function onSample(cb: (s: RawSample) => void) {
    sampleCb = cb;
  }
  function onTopics(cb: (t: TopicInfo[]) => void) {
    topicsCb = cb;
  }
  function onStatus(cb: (s: Status) => void) {
    statusCb = cb;
  }
  function onCommands(cb: (c: CommandInfo[]) => void) {
    commandsCb = cb;
  }
  function close() {
    for (const sub of subs.values()) if (sub !== "pending") sub.undeclare?.().catch(() => {});
    subs.clear();
    control?.close();
    session?.close?.().catch?.(() => {});
    statusCb?.("closed");
  }

  return {
    caps,
    label: "zenoh-ts (direct)",
    get commands() {
      return commands;
    },
    connect,
    subscribe,
    unsubscribe,
    publishTeleop,
    publishGoal,
    rpc,
    requestList,
    onSample,
    onTopics,
    onStatus,
    onCommands,
    close,
  };
};
