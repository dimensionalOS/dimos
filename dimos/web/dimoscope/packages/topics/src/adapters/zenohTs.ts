// ZenohTsTransport — the browser talks DIRECTLY to a Zenoh network via the official
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
// stays server-side. Pass the gateway URL as the 2nd ctor arg to enable driving.
//
// zenoh-ts is browser-only (no Bun/Node target) and the heavy client loads lazily inside
// connect(), so importing this module is cheap and a load failure can't break other paths.
import type { RawSample, Status, Transport, TransportCaps } from "../transport";
import type { TopicInfo } from "../types";

const PREFIX = "dimos/"; // dimos Zenoh namespace (core/transport_factory.py:transport_topic)
const SCOUT_MS = 1800; // one-time discovery burst on dimos/** (then undeclared)

// zenoh key "dimos/lidar/sensor_msgs.PointCloud2" → {topic:"/dimos/lidar", type:"sensor_msgs.PointCloud2"}
// (the inverse of zenohpubsub._topic_to_key_expr; matches gateway_zenoh.py's mapping).
function parseKey(key: string): { topic: string; type: string } | null {
  if (!key.startsWith(PREFIX)) return null;
  const slash = key.lastIndexOf("/");
  if (slash < PREFIX.length) return { topic: "/" + key, type: "?" };
  return { topic: "/" + key.slice(0, slash), type: key.slice(slash + 1) };
}
// dimos topic "/dimos/lidar" → zenoh subscribe key "dimos/lidar/**" (any type suffix)
const topicToKey = (topic: string) => topic.replace(/^\//, "") + "/**";

export class ZenohTsTransport implements Transport {
  readonly caps: TransportCaps = { onDemand: true, discovery: "passive" };
  label = "zenoh-ts (direct)";
  private session: any;
  private subs = new Map<string, any>(); // topic -> zenoh Subscriber
  private rates = new Map<string, number>();
  private lastSent = new Map<string, number>();
  private topics = new Map<string, string>(); // topic -> type
  private control?: WebSocket;
  private sampleCb?: (s: RawSample) => void;
  private topicsCb?: (t: TopicInfo[]) => void;
  private statusCb?: (s: Status) => void;

  /** @param remoteApiUrl zenoh-bridge-remote-api WS (e.g. ws://localhost:10000)
   *  @param controlUrl   optional gateway WS for safe teleop/goal (e.g. ws://localhost:8088) */
  constructor(
    private remoteApiUrl: string,
    private controlUrl?: string,
  ) {}

  async connect(): Promise<void> {
    this.statusCb?.("connecting");
    const zenoh = await import("@eclipse-zenoh/zenoh-ts"); // lazy: only loads on this path
    this.session = await zenoh.Session.open(new zenoh.Config(this.remoteApiUrl));
    this.statusCb?.("open");
    // Discovery: Zenoh has no gateway "hello/list", so briefly scout dimos/** to learn
    // which (topic,type) keys exist, then undeclare it. Data subscriptions below are the
    // steady-state on-demand path; this is a one-time burst.
    try {
      const scout = await this.session.declareSubscriber(PREFIX + "**", {
        handler: (s: any) => this.note(String(s.keyexpr())),
      });
      setTimeout(() => scout.undeclare?.().catch(() => {}), SCOUT_MS);
    } catch {
      /* discovery is best-effort */
    }
  }

  private note(key: string) {
    const m = parseKey(key);
    if (!m || m.type === "?" || m.topic.includes("/rpc/")) return;
    if (this.topics.get(m.topic) !== m.type) {
      this.topics.set(m.topic, m.type);
      this.topicsCb?.([...this.topics].map(([topic, type]) => ({ topic, type })));
    }
  }

  subscribe(topic: string, maxHz?: number) {
    if (maxHz) this.rates.set(topic, maxHz);
    if (this.subs.has(topic) || !this.session) return;
    this.subs.set(topic, "pending"); // guard against double-declare races
    this.session
      .declareSubscriber(topicToKey(topic), { handler: (s: any) => this.onSampleZ(topic, s) })
      .then((sub: any) => {
        if (this.subs.get(topic) === "pending") this.subs.set(topic, sub);
        else sub.undeclare?.().catch(() => {}); // unsubscribed while declaring
      })
      .catch(() => this.subs.delete(topic));
  }

  private onSampleZ(topic: string, s: any) {
    const key = String(s.keyexpr());
    const m = parseKey(key);
    if (!m) return;
    this.note(key);
    const now = Date.now();
    const hz = this.rates.get(topic) ?? 0;
    if (hz > 0) {
      // Zenoh has no per-subscriber downsample, so throttle client-side.
      if (now - (this.lastSent.get(topic) ?? 0) < 1000 / hz) return;
      this.lastSent.set(topic, now);
    }
    const payload: Uint8Array = s.payload().toBytes(); // bare lcm_encode body (8-byte hash head)
    // No gatewaySendMs (no gateway hop) → DimosClient falls back to source-stamp latency.
    this.sampleCb?.({ topic: m.topic, type: m.type, payload, recvTs: now });
  }

  unsubscribe(topic: string) {
    const sub = this.subs.get(topic);
    this.subs.delete(topic);
    this.rates.delete(topic);
    this.lastSent.delete(topic);
    if (sub && sub !== "pending") sub.undeclare?.().catch(() => {});
  }

  // ── teleop / goal: forwarded to a gateway so its clamp + deadman + stop-on-disconnect
  //    still apply (a browser doing raw Zenoh put would bypass all of that) ──────────────
  private ctrl(): WebSocket | undefined {
    if (!this.controlUrl) return undefined;
    const st = this.control?.readyState;
    if (this.control && (st === WebSocket.OPEN || st === WebSocket.CONNECTING)) return this.control;
    this.control = new WebSocket(this.controlUrl);
    return this.control;
  }
  private ctrlSend(obj: unknown) {
    const ws = this.ctrl();
    if (!ws) return;
    const s = JSON.stringify(obj);
    if (ws.readyState === WebSocket.OPEN) ws.send(s);
    else ws.addEventListener("open", () => ws.send(s), { once: true });
  }
  publishTeleop(linearX: number, angularZ: number, ttlMs?: number) {
    this.ctrlSend({ op: "teleop", linearX, angularZ, ttlMs });
  }
  publishGoal(x: number, y: number, z = 0) {
    this.ctrlSend({ op: "goal", x, y, z });
  }

  requestList() {} // discovery is the passive scout; nothing to request
  onSample(cb: (s: RawSample) => void) {
    this.sampleCb = cb;
  }
  onTopics(cb: (t: TopicInfo[]) => void) {
    this.topicsCb = cb;
  }
  onStatus(cb: (s: Status) => void) {
    this.statusCb = cb;
  }
  close() {
    for (const sub of this.subs.values()) if (sub !== "pending") sub.undeclare?.().catch(() => {});
    this.subs.clear();
    this.control?.close();
    this.session?.close?.().catch?.(() => {});
    this.statusCb?.("closed");
  }
}
