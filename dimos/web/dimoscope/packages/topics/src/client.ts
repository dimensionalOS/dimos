// DimosClient — the transport-agnostic browser API for DimOS topics.
//
//   const dimos = createDimosClient();                 // default transport = ws()
//   await dimos.connect("ws://localhost:8080");
//   dimos.subscribe("/odom", (m) => use(m.data, m.ts)); // one { data, ts, meta } envelope everywhere
//   dimos.topic("/odom").getLatest();                   // rich per-topic handle
//   dimos.teleop(0.5, 0.0);                             // structured + safe (gateway clamps/watchdogs)
//   await dimos.modules.GO2Connection.standup();        // typed Proxy over @rpc
//
// The transport lives IN createDimosClient (default `ws()`), `connect(url)` is a METHOD, and the
// combine (e.g. `webtransport()` = WS control + WT data) is encapsulated inside the adapter — the
// client stays transport-agnostic. For typed topics + modules pass the generated maps:
//   const dimos = createDimosClient<DimosTopics, DimosCommands>();
import { createGatewayWsTransport } from "./adapters/gatewayWs.ts";
import { decodeBody, seqFrom, srcTsMs } from "./decode.ts";
import { createTopic, type Topic } from "./topic.ts";
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "./transport.ts";
import type { Message, MessageMeta, Qos, Subscription, TopicInfo, TopicStats } from "./types.ts";

/** A transport built lazily from the connect() URL. `ws()` / `webtransport()` return one of these, so
 *  the transport is chosen at `createDimosClient({ transport })` time but wired at `connect(url)`. */
export type TransportFactory = (url: string) => Transport;

/** The default transport: one WebSocket for control + data. `connect("ws://host:8080")` (the `/ws`
 *  path is appended if missing). The universal production backbone + the duplex control plane. */
export const ws = (opts?: { reconnect?: boolean }): TransportFactory => (url) =>
  createGatewayWsTransport({
    url: /\/ws$/.test(url) ? url : url.replace(/\/+$/, "") + "/ws",
    reconnect: opts?.reconnect,
  });

/** Like `ws()`, but asks the gateway to decode + ship JSON (rosbridge-style) instead of the binary
 *  self-describing frame — so the client renders ANY type (incl. user messages not in @dimos/msgs) with
 *  zero rebuild. Costs ~2.6× the bytes on the wire vs the client-decode default (see `bench:decode`). */
export const wsServerJson = (opts?: { reconnect?: boolean }): TransportFactory => (url) =>
  createGatewayWsTransport({
    url: /\/ws$/.test(url) ? url : url.replace(/\/+$/, "") + "/ws",
    reconnect: opts?.reconnect,
    decode: "server-json",
  });

/** Default topic map for an untyped client: no known keys, so `keyof … & string` is `never` and the
 *  key-based overloads drop out — an untyped client accepts any string. */
export type EmptyTopicMap = Record<never, never>;
/** Shape of a codegen module/RPC map: `{ ModuleName: { method: (...args) => Promise<ret> } }`. */
export type ModuleMap = Record<string, Record<string, (...args: any[]) => Promise<any>>>;
export type EmptyModuleMap = Record<never, never>;

export interface DimosClient<TMap = EmptyTopicMap, TCmds = EmptyModuleMap> {
  status: Status;
  /** Connect (or reconnect) the transport. Pass the gateway URL (`ws://host:8080` for `ws()`; a bare
   *  `host` for composite adapters that derive their own ports). */
  connect(url: string): Promise<void>;
  /** Rich per-topic handle (subscribe/getLatest/setQos/stats/pause). */
  topic<K extends (keyof TMap & string) | (string & Record<never, never>)>(
    name: K,
  ): Topic<K extends keyof TMap ? TMap[K] : unknown>;
  /** Flat subscribe — the easy path. Delivers the `{ data, ts, meta }` envelope. */
  subscribe<K extends (keyof TMap & string) | (string & Record<never, never>)>(
    name: K,
    handler: (m: Message<K extends keyof TMap ? TMap[K] : unknown>) => void,
    qos?: Qos,
  ): Subscription;
  /** Firehose — one callback per update across ALL topics (asks the gateway for the `*` wildcard). */
  subscribeAll(handler: (m: Message) => void): Subscription;
  /** One-shot: resolve with the next message on `name` (or reject after `timeoutMs`). */
  peek<K extends (keyof TMap & string) | (string & Record<never, never>)>(
    name: K,
    opts?: { timeoutMs?: number },
  ): Promise<Message<K extends keyof TMap ? TMap[K] : unknown>>;
  /** Sync last value seen on a topic (undefined if none yet). */
  latest<K extends (keyof TMap & string) | (string & Record<never, never>)>(
    name: K,
  ): (K extends keyof TMap ? TMap[K] : unknown) | undefined;
  /** Rolling 1s stats for a topic. */
  stats(name: string): TopicStats;
  /** Update QoS for a topic (rate/priority/reliability/depth, where the transport honors it). */
  setQos(name: string, qos: Qos): void;
  /** All discovered topics (real ones; skips untyped LCM internals). */
  listTopics(): TopicInfo[];
  readonly gatewayLabel: string | undefined;
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
  /** Typed Proxy over `call`: `client.modules.GO2Connection.standup(args)`. Types come from the
   *  generated command map (second generic); untyped it accepts any module/method. */
  readonly modules: TCmds extends EmptyModuleMap ? ModuleMap : TCmds;
  readonly commands: CommandInfo[];
  onCommands(cb: (c: CommandInfo[]) => void): () => void;
  close(): void;
}

export interface DimosClientDeps {
  /** The transport factory (`ws()` default, `webtransport()`, an experimental one, …), wired at
   *  `connect(url)`. */
  transport?: TransportFactory;
}

export const createDimosClient = <TMap = EmptyTopicMap, TCmds = EmptyModuleMap>(
  deps: DimosClientDeps = {},
): DimosClient<TMap, TCmds> => {
  const factory: TransportFactory = deps.transport ?? ws();
  let transport: Transport | undefined;
  const topicsMap = new Map<string, string>();
  const topicObjs = new Map<string, Topic<any>>();
  // Last value per topic across ALL samples — incl. firehose/subscribeAll topics that have no `topic()`
  // handle. `latest(name)` reads this; `topic(name).getLatest()` only tracks explicitly-subscribed ones.
  const latestData = new Map<string, unknown>();
  const topicsListeners = new Set<(t: TopicInfo[]) => void>();
  const statusListeners = new Set<(s: Status) => void>();
  const firehose = new Set<(m: Message) => void>();
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
    if (!t && firehose.size === 0) return; // discovered but nobody listening
    let data: unknown;
    if (s.decoded !== undefined) {
      data = s.decoded; // server already decoded it (server-json mode)
    } else {
      try {
        data = decodeBody(s.payload);
      } catch {
        return; // unknown type / fragment
      }
    }
    const srcTs = srcTsMs(data);
    const latencyMs = s.gatewaySendMs != null
      ? Math.max(0, s.recvTs - s.gatewaySendMs)
      : srcTs != null
      ? s.recvTs - srcTs
      : undefined;
    const meta: MessageMeta = {
      topic: s.topic,
      type: s.type,
      recvTs: s.recvTs,
      srcTs,
      latencyMs,
      sizeBytes: s.payload.length,
      dropped: 0,
      seq: seqFrom(data),
    };
    latestData.set(s.topic, data);
    t?._deliver(data, meta);
    if (firehose.size) {
      const m: Message = { data, ts: srcTs ?? s.recvTs, meta };
      firehose.forEach((f) => f(m));
    }
  }

  function topic<T = unknown>(name: string): Topic<T> {
    let t = topicObjs.get(name);
    if (!t) {
      t = createTopic<T>({
        name,
        type: topicsMap.get(name) ?? "?",
        wiring: {
          subscribe: (topicName, qos) => transport?.subscribe(topicName, qos),
          unsubscribe: (topicName) => transport?.unsubscribe(topicName),
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

  function wire(t: Transport) {
    transport = t;
    t.onSample(onSample);
    t.onCommands?.((c) => {
      commandsList = c;
      commandsListeners.forEach((f) => f(c));
    });
    t.onTopics((list) => {
      let changed = false;
      for (const ti of list) {
        if (topicsMap.get(ti.topic) !== ti.type) {
          topicsMap.set(ti.topic, ti.type);
          changed = true;
        }
      }
      if (changed) emitTopics();
    });
    t.onStatus((s) => {
      client.status = s;
      statusListeners.forEach((f) => f(s));
    });
  }

  const modulesProxy = new Proxy({} as ModuleMap, {
    get: (_t, target: string) =>
      new Proxy({} as Record<string, (...a: unknown[]) => Promise<unknown>>, {
        get: (_t2, method: string) => (...args: unknown[]) => client.call(target, method, ...args),
      }),
  });

  const client = {
    status: "connecting" as Status,
    async connect(url: string) {
      if (!transport) wire(factory(url));
      await transport!.connect();
    },
    topic,
    subscribe(name: string, handler: (m: Message) => void, qos?: Qos): Subscription {
      const t = topic(name);
      if (qos) t.setQos(qos);
      return t.subscribe(handler); // topic._deliver builds the { data, ts, meta } envelope
    },
    subscribeAll(handler: (m: Message) => void): Subscription {
      firehose.add(handler);
      transport?.subscribe("*");
      return {
        unsubscribe() {
          firehose.delete(handler);
          if (firehose.size === 0) transport?.unsubscribe("*");
        },
      };
    },
    peek(name: string, opts?: { timeoutMs?: number }): Promise<Message> {
      return new Promise((resolve, reject) => {
        let timer: ReturnType<typeof setTimeout> | undefined;
        const sub = client.subscribe(name, (m) => {
          if (timer) clearTimeout(timer);
          sub.unsubscribe();
          resolve(m);
        });
        if (opts?.timeoutMs) {
          timer = setTimeout(() => {
            sub.unsubscribe();
            reject(new Error(`peek("${name}") timed out after ${opts.timeoutMs}ms`));
          }, opts.timeoutMs);
        }
      });
    },
    latest(name: string) {
      return latestData.get(name);
    },
    stats(name: string): TopicStats {
      return topic(name).stats();
    },
    setQos(name: string, qos: Qos) {
      topic(name).setQos(qos);
    },
    listTopics,
    get gatewayLabel() {
      return transport?.label;
    },
    get caps() {
      return (transport?.caps ?? { onDemand: false, discovery: "passive" }) as TransportCaps;
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
      transport?.publishTeleop(linearX, angularZ, ttlMs);
    },
    stop() {
      transport?.publishTeleop(0, 0);
    },
    navigate(x: number, y: number, z = 0) {
      transport?.publishGoal(x, y, z);
    },
    call<T = unknown>(target: string, method: string, ...args: unknown[]): Promise<T> {
      if (!transport) return Promise.reject(new Error("not connected"));
      return transport.rpc(target, method, args) as Promise<T>;
    },
    get modules() {
      return modulesProxy;
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
      transport?.close();
    },
  };

  // TMap/TCmds are compile-time phantoms — the runtime map is `Map<string, Topic<any>>`; typed and
  // untyped views share one implementation. Cast once here.
  return client as unknown as DimosClient<TMap, TCmds>;
};
