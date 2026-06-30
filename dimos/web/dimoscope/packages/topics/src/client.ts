// DimosClient — the stable, transport-agnostic browser API for DimOS topics.
//
//   const client = await connect({ url: "ws://localhost:8090" });
//   client.topic<PoseStamped>("/odom").subscribeLatest((p, meta) => ...);
//   client.teleop(0.5, 0.0);            // structured + safe (gateway clamps/watchdogs)
import { createGatewayWsTransport } from "./adapters/gatewayWs.ts";
import { decodeBody, seqFrom, srcTsMs } from "./decode.ts";
import { createTopic, type Topic } from "./topic.ts";
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "./transport.ts";
import type { TopicInfo } from "./types.ts";

export interface ConnectOpts {
  url?: string;
  transport?: Transport;
  reconnect?: boolean;
}

export interface DimosClient {
  status: Status;
  /** Get (or create) a typed handle to a topic. Subscribing is on-demand. */
  topic<T = unknown>(name: string): Topic<T>;
  /** All discovered topics (real ones; skips untyped LCM internals). */
  listTopics(): TopicInfo[];
  /** The label the connected gateway reports (e.g. "Bun↔LCM", "Python↔Zenoh"). */
  readonly gatewayLabel: string | undefined;
  /** The active transport's capabilities (on-demand, discovery, qos) — for QoS-aware UI. */
  readonly caps: TransportCaps;
  onTopics(cb: (t: TopicInfo[]) => void): () => void;
  onStatus(cb: (s: Status) => void): () => void;
  /** Safe teleop — structured velocity; the gateway clamps + runs a TTL watchdog. */
  teleop(linearX: number, angularZ: number, ttlMs?: number): void;
  stop(): void;
  /** Send a nav goal in world metres (x,y[,z]). Gateway → PointStamped on clicked_point. */
  navigate(x: number, y: number, z?: number): void;
  /** Invoke a whitelisted dimos `@rpc` command, e.g. client.call("GO2Connection", "standup"). */
  call<T = unknown>(target: string, method: string, ...args: unknown[]): Promise<T>;
  /** Commands the connected gateway advertises as browser-callable (from its hello). */
  readonly commands: CommandInfo[];
  onCommands(cb: (c: CommandInfo[]) => void): () => void;
  close(): void;
}

export interface DimosClientDeps {
  transport: Transport;
}

export const createDimosClient = (deps: DimosClientDeps): DimosClient => {
  const { transport } = deps;
  const topicsMap = new Map<string, string>();
  const topicObjs = new Map<string, Topic<any>>();
  const topicsListeners = new Set<(t: TopicInfo[]) => void>();
  const statusListeners = new Set<(s: Status) => void>();
  let commandsList: CommandInfo[] = [];
  const commandsListeners = new Set<(c: CommandInfo[]) => void>();

  function emitTopics() {
    const list = listTopics();
    topicsListeners.forEach((f) => f(list));
  }

  function onSample(s: RawSample) {
    if (!topicsMap.has(s.topic)) {
      topicsMap.set(s.topic, s.type);
      emitTopics();
    }
    const t = topicObjs.get(s.topic);
    if (!t) return; // discovered but nobody subscribed
    let data: unknown;
    if (s.decoded !== undefined) {
      data = s.decoded; // server already decoded it (server-json mode) — the decode-location axis
    } else {
      try {
        data = decodeBody(s.payload);
      } catch {
        return; // unknown type / fragment
      }
    }
    const srcTs = srcTsMs(data);
    // Prefer true transport latency (gateway-send → browser-recv); fall back to
    // source-stamp age. Gateway-send avoids the huge "age" of replay/stale stamps.
    // Gateway-send → browser-recv on the same host: sub-ms deltas can round
    // slightly negative (Date.now() is integer ms; the gateway stamps fractional
    // ms), so floor at 0. Falls back to source-stamp age when no gateway stamp.
    const latencyMs = s.gatewaySendMs != null
      ? Math.max(0, s.recvTs - s.gatewaySendMs)
      : srcTs != null
      ? s.recvTs - srcTs
      : undefined;
    t._deliver(data, {
      topic: s.topic,
      type: s.type,
      recvTs: s.recvTs,
      srcTs,
      latencyMs,
      sizeBytes: s.payload.length,
      dropped: 0,
      seq: seqFrom(data),
    });
  }

  function topic<T = unknown>(name: string): Topic<T> {
    let t = topicObjs.get(name);
    if (!t) {
      t = createTopic<T>({
        name,
        type: topicsMap.get(name) ?? "?",
        wiring: {
          subscribe: (topicName, maxHz) => transport.subscribe(topicName, maxHz),
          unsubscribe: (topicName) => transport.unsubscribe(topicName),
        },
      });
      topicObjs.set(name, t);
    }
    return t as Topic<T>;
  }

  function listTopics(): TopicInfo[] {
    return [...topicsMap]
      .filter(([topic, type]) => type !== "?" && topic.startsWith("/") && !topic.includes("/rpc/"))
      .map(([topic, type]) => ({ topic, type }));
  }

  // ── wiring: subscribe to the transport once at construction ──────────────────
  transport.onSample((s) => onSample(s));
  transport.onCommands?.((c) => {
    commandsList = c;
    commandsListeners.forEach((f) => f(c));
  });
  transport.onTopics((list) => {
    let changed = false;
    for (const ti of list) {
      if (topicsMap.get(ti.topic) !== ti.type) {
        topicsMap.set(ti.topic, ti.type);
        changed = true;
      }
    }
    if (changed) emitTopics();
  });
  transport.onStatus((s) => {
    client.status = s;
    statusListeners.forEach((f) => f(s));
  });

  const client: DimosClient = {
    status: "connecting",
    topic,
    listTopics,
    get gatewayLabel() {
      return transport.label;
    },
    get caps() {
      return transport.caps;
    },
    onTopics(cb: (t: TopicInfo[]) => void) {
      topicsListeners.add(cb);
      return () => topicsListeners.delete(cb);
    },
    onStatus(cb: (s: Status) => void) {
      statusListeners.add(cb);
      return () => statusListeners.delete(cb);
    },
    teleop(linearX: number, angularZ: number, ttlMs?: number) {
      transport.publishTeleop(linearX, angularZ, ttlMs);
    },
    stop() {
      transport.publishTeleop(0, 0);
    },
    navigate(x: number, y: number, z = 0) {
      transport.publishGoal(x, y, z);
    },
    call<T = unknown>(target: string, method: string, ...args: unknown[]): Promise<T> {
      return transport.rpc(target, method, args) as Promise<T>;
    },
    get commands() {
      return commandsList;
    },
    onCommands(cb: (c: CommandInfo[]) => void) {
      commandsListeners.add(cb);
      if (commandsList.length) cb(commandsList);
      return () => commandsListeners.delete(cb);
    },
    close() {
      transport.close();
    },
  };

  return client;
};

export async function connect(opts: ConnectOpts = {}): Promise<DimosClient> {
  const transport = opts.transport ??
    createGatewayWsTransport({ url: opts.url ?? "ws://localhost:8090", reconnect: opts.reconnect });
  const client = createDimosClient({ transport });
  await transport.connect();
  return client;
}
