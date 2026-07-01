// DimosClient — the stable, transport-agnostic browser API for DimOS topics.
//
//   const client = await connect({ url: "ws://localhost:8090" });
//   client.topic("/odom").subscribeLatest((p, meta) => ...);   // Topic<unknown> (untyped client)
//   client.teleop(0.5, 0.0);            // structured + safe (gateway clamps/watchdogs)
//
// For typed topics + autocomplete, pass the generated map (`dtop gen-types`):
//   import type { DimosTopics } from "./dimos.topics.gen.ts";
//   const client = await connect<DimosTopics>({ url: "ws://localhost:8090" });
//   client.topic("/odom");             // ← autocompletes names, inferred Topic<PoseStamped>
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

/** Default topic map for an untyped client: no known keys, so `keyof … & string` is `never` and the
 *  key-based `topic` overload drops out — an untyped `connect()` behaves exactly as before.
 *  (Must be `Record<never, never>`, NOT `Record<string, never>`: the latter's `keyof` is `string`,
 *  which would resolve the key overload and hand back `Topic<never>`.) */
export type EmptyTopicMap = Record<never, never>;

export interface DimosClient<TMap = EmptyTopicMap> {
  status: Status;
  /** Get (or create) a typed handle to a topic. On a `connect<DimosTopics>()` client a KNOWN name
   *  autocompletes and infers its mapped message type; any other (runtime-discovered) name is
   *  accepted and yields `Topic<unknown>`. Types come from the generated map — there is no
   *  hand-typed `topic<Msg>()` form. The `string & Record<never, never>` in the key union keeps the
   *  known-name autocomplete alive while still accepting arbitrary strings. */
  topic<K extends (keyof TMap & string) | (string & Record<never, never>)>(
    name: K,
  ): Topic<K extends keyof TMap ? TMap[K] : unknown>;
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

export const createDimosClient = <TMap = EmptyTopicMap>(
  deps: DimosClientDeps,
): DimosClient<TMap> => {
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
          subscribe: (topicName, qos) => transport.subscribe(topicName, qos),
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
    // These are getters, not plain fields, because their backing transport state is filled in
    // asynchronously after construction: `label` + `commands` arrive on the gateway's `hello`,
    // and `caps` is the (provider-specific) live transport capability set. A captured field would
    // freeze the connecting-time values (undefined / []); a getter always reads the current one.
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

  // TMap is a compile-time phantom — the runtime map is `Map<string, Topic<any>>`, so the typed
  // (key-based) and untyped views share one implementation. Cast once here.
  return client as unknown as DimosClient<TMap>;
};

export async function connect<TMap = EmptyTopicMap>(
  opts: ConnectOpts = {},
): Promise<DimosClient<TMap>> {
  const transport = opts.transport ??
    createGatewayWsTransport({ url: opts.url ?? "ws://localhost:8090", reconnect: opts.reconnect });
  const client = createDimosClient<TMap>({ transport });
  await transport.connect();
  return client;
}
