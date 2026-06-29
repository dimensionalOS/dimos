// DimosClient — the stable, transport-agnostic browser API for DimOS topics.
//
//   const client = await connect({ url: "ws://localhost:8090" });
//   client.topic<PoseStamped>("/odom").subscribeLatest((p, meta) => ...);
//   client.teleop(0.5, 0.0);            // structured + safe (gateway clamps/watchdogs)
import { GatewayWsTransport } from "./adapters/gatewayWs";
import { decodeBody, srcTsMs } from "./decode";
import { Topic } from "./topic";
import type { RawSample, Status, Transport } from "./transport";
import type { TopicInfo } from "./types";

export interface ConnectOpts {
  url?: string;
  transport?: Transport;
  reconnect?: boolean;
}

export class DimosClient {
  status: Status = "connecting";
  private topicsMap = new Map<string, string>();
  private topicObjs = new Map<string, Topic<any>>();
  private topicsListeners = new Set<(t: TopicInfo[]) => void>();
  private statusListeners = new Set<(s: Status) => void>();

  constructor(private transport: Transport) {
    transport.onSample((s) => this.onSample(s));
    transport.onTopics((list) => {
      let changed = false;
      for (const ti of list)
        if (this.topicsMap.get(ti.topic) !== ti.type) {
          this.topicsMap.set(ti.topic, ti.type);
          changed = true;
        }
      if (changed) this.emitTopics();
    });
    transport.onStatus((s) => {
      this.status = s;
      this.statusListeners.forEach((f) => f(s));
    });
  }

  private onSample(s: RawSample) {
    if (!this.topicsMap.has(s.topic)) {
      this.topicsMap.set(s.topic, s.type);
      this.emitTopics();
    }
    const t = this.topicObjs.get(s.topic);
    if (!t) return; // discovered but nobody subscribed
    let data: unknown;
    try {
      data = decodeBody(s.payload);
    } catch {
      return; // unknown type / fragment
    }
    const srcTs = srcTsMs(data);
    t._deliver(data, {
      topic: s.topic,
      type: s.type,
      recvTs: s.recvTs,
      srcTs,
      latencyMs: srcTs != null ? s.recvTs - srcTs : undefined,
      sizeBytes: s.payload.length,
      dropped: 0,
    });
  }

  /** Get (or create) a typed handle to a topic. Subscribing is on-demand. */
  topic<T = unknown>(name: string): Topic<T> {
    let t = this.topicObjs.get(name);
    if (!t) {
      t = new Topic<T>(name, this.topicsMap.get(name) ?? "?", {
        subscribe: (topic, maxHz) => this.transport.subscribe(topic, maxHz),
        unsubscribe: (topic) => this.transport.unsubscribe(topic),
      });
      this.topicObjs.set(name, t);
    }
    return t as Topic<T>;
  }

  /** All discovered topics (real ones; skips untyped LCM internals). */
  listTopics(): TopicInfo[] {
    return [...this.topicsMap]
      .filter(([topic, type]) => type !== "?" && topic.startsWith("/"))
      .map(([topic, type]) => ({ topic, type }));
  }

  onTopics(cb: (t: TopicInfo[]) => void) {
    this.topicsListeners.add(cb);
    return () => this.topicsListeners.delete(cb);
  }
  onStatus(cb: (s: Status) => void) {
    this.statusListeners.add(cb);
    return () => this.statusListeners.delete(cb);
  }

  /** Safe teleop — structured velocity; the gateway clamps + runs a TTL watchdog. */
  teleop(linearX: number, angularZ: number, ttlMs?: number) {
    this.transport.publishTeleop(linearX, angularZ, ttlMs);
  }
  stop() {
    this.transport.publishTeleop(0, 0);
  }
  close() {
    this.transport.close();
  }

  private emitTopics() {
    const list = this.listTopics();
    this.topicsListeners.forEach((f) => f(list));
  }
}

export async function connect(opts: ConnectOpts = {}): Promise<DimosClient> {
  const transport =
    opts.transport ??
    new GatewayWsTransport(opts.url ?? "ws://localhost:8090", { reconnect: opts.reconnect });
  const client = new DimosClient(transport);
  await transport.connect();
  return client;
}
