// Transport-agnostic browser API for DimOS topics. Transport is chosen at
// createDimosClient({transport}) (default ws()) and wired at connect(url); pass the generated
// <DimosTopics, DimosCommands> maps for typed topics + modules. Usage: see README.
import { createGatewayWsTransport } from "./transports/gatewayWs.ts";
import { decode as msgsDecode } from "@dimos/msgs";
import { createTopic, type Topic } from "./topic.ts";
import type {
  ClockSample,
  CommandInfo,
  Message,
  MessageMeta,
  Qos,
  RawSample,
  Status,
  Subscription,
  TopicInfo,
  TopicStats,
  Transport,
  TransportCaps,
} from "./types.ts";

/** Best-effort source timestamp (ms) from a std_msgs/Header — the data's origin-publish time,
 *  surfaced as `meta.srcTs` and used by the bench's end-to-end latency mode (`recvTs − srcTs`). The
 *  heuristics exist because there's no single canonical stamp field/epoch across message types (ns vs
 *  ms vs s, `stamp` vs `ts`, `{sec,nsec}` variants). */
export function srcTsMs(data: unknown): number | undefined {
  const stamp = (data as any)?.header?.stamp ?? (data as any)?.header?.ts ?? (data as any)?.ts;
  if (typeof stamp === "number") {
    // Heuristic: ns (>1e15) | ms (>1e12) | s (else).
    if (stamp > 1e15) return stamp / 1e6;
    if (stamp > 1e12) return stamp;
    return stamp * 1000;
  }
  if (stamp && typeof stamp === "object") {
    const sec = (stamp as any).sec ?? (stamp as any).secs ?? 0;
    const nsec = (stamp as any).nsec ?? (stamp as any).nanosec ?? (stamp as any).nanos ?? 0;
    return sec * 1000 + nsec / 1e6;
  }
  return undefined;
}

/** Best-effort per-topic sequence number, for wire drop/gap detection. The bench source stamps
 *  `frame_id = str(seq)`, so a purely-numeric frame_id is read as a counter; real frames
 *  ("base_link", "odom") are not numeric and yield undefined. Falls back to a `header.seq` field. */
export function seqFrom(data: unknown): number | undefined {
  const fid = (data as any)?.frame_id ?? (data as any)?.header?.frame_id;
  if (typeof fid === "string" && fid.length > 0 && fid.length < 16 && /^\d+$/.test(fid)) {
    return Number(fid);
  }
  const seq = (data as any)?.header?.seq;
  return typeof seq === "number" ? seq : undefined;
}

/** A transport built lazily from the connect() URL. */
export type TransportFactory = (url: string) => Transport;

/** Default transport: one WebSocket for control + data (appends `/ws` if missing). */
export const ws = (opts?: { reconnect?: boolean }): TransportFactory => (url) =>
  createGatewayWsTransport({
    url: /\/ws$/.test(url) ? url : url.replace(/\/+$/, "") + "/ws",
    reconnect: opts?.reconnect,
  });

/** Untyped client topic map: no known keys, so the key-based overloads accept any string. */
export type EmptyTopicMap = Record<never, never>;
/** Shape of an untyped module/RPC surface: any module, any method, any args. */
export type ModuleMap = Record<string, Record<string, (...args: any[]) => Promise<any>>>;
export type EmptyModuleMap = Record<never, never>;

/** A codegen RPC descriptor (`{ args, ret }`) → the callable it denotes. args/ret are `unknown` today
 *  (only target + method are typed) but tighten for free if a descriptor ever carries real types. */
type RpcFn<C> = C extends { args: infer A extends unknown[]; ret: infer R }
  ? (...args: A) => Promise<R>
  : (...args: unknown[]) => Promise<unknown>;
/** Lift a generated `DimosCommands` descriptor map into the callable `modules` surface. */
type TypedModules<TCmds> = { [T in keyof TCmds]: { [M in keyof TCmds[T]]: RpcFn<TCmds[T][M]> } };

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
  /** Firehose — one callback per update across all topics (asks the gateway for the `*` wildcard). */
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
  /** Estimate (gateway clock − client clock) from `samples` ping round-trips, keeping the
   *  lowest-RTT sample (NTP discipline: least queueing → tightest offset bound). Lets the bench
   *  correct cross-machine `recvTs − srcTs` latency over WAN, where raw clock skew swamps the
   *  signal. Resolves undefined when the transport has no ping (read-only mechanisms). */
  estimateClockOffset(samples?: number): Promise<ClockSample | undefined>;
  /** Typed Proxy over `call`: `client.modules.GO2Connection.standup(args)`. Pass the generated
   *  `DimosCommands` (second generic) → target + method are autocompleted + typo-checked (arg/return
   *  types stay `unknown` until `@rpc` introspection lands); untyped it accepts any module/method.
   *  The `[keyof …]` tuple-wrap disables distribution so an empty map (keyof = never) → the permissive
   *  `ModuleMap` and a real map → the typed callable surface. */
  readonly modules: [keyof TCmds] extends [never] ? ModuleMap : TypedModules<TCmds>;
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
  // Last value per topic across all samples — incl. firehose/subscribeAll topics that have no `topic()`
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
      data = s.decoded; // pre-decoded injection (unit tests); no transport sets this
    } else {
      try {
        data = msgsDecode(s.payload);
      } catch {
        return; // unknown type / fragment
      }
    }
    const srcTs = srcTsMs(data); // for meta.srcTs (the bench's end-to-end latency)
    const latencyMs = s.gatewaySendMs != null ? Math.max(0, s.recvTs - s.gatewaySendMs) : undefined;
    const meta: MessageMeta = {
      topic: s.topic,
      type: s.type,
      recvTs: s.recvTs,
      srcTs,
      latencyMs,
      sizeBytes: s.payload.length,
      seq: seqFrom(data),
    };
    latestData.set(s.topic, data);
    t?._deliver(data, meta);
    if (firehose.size) {
      const m: Message = { data, ts: s.recvTs, meta };
      firehose.forEach((f) => {
        try {
          f(m);
        } catch (e) {
          console.error("[@dimos/web] subscribeAll handler threw", e);
        }
      });
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
    async estimateClockOffset(samples = 5): Promise<ClockSample | undefined> {
      if (!transport?.ping) return undefined;
      let best: ClockSample | undefined;
      for (let i = 0; i < samples; i++) {
        try {
          const s = await transport.ping();
          if (Number.isFinite(s.offsetMs) && (!best || s.rttMs < best.rttMs)) best = s;
        } catch {
          // timed-out probe — keep whatever samples we have
        }
      }
      return best;
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
