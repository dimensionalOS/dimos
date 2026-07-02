// createGatewayWsTransport — talks to the dimoscope gateway (Python, LCM+Zenoh tap)
// over WebSocket. Binary frames are raw LCM packets; JSON frames are control.
// Runs unchanged in any JS runtime with a global WebSocket.
import { frameToSample } from "./frame.ts";
import { applyCaps, GATEWAY_QOS } from "../qos.ts";
import type {
  ClockSample,
  CommandInfo,
  Qos,
  RawSample,
  Status,
  TopicInfo,
  Transport,
  TransportCaps,
} from "../types.ts";

export interface GatewayWsDeps {
  url: string;
  reconnect?: boolean;
  /** connect() rejects if no socket has opened within this window (ms). With reconnect on,
   *  retries continue in the background, so a gateway that comes up later still connects. Default 10000. */
  connectTimeoutMs?: number;
  /** Liveness ping cadence (ms). A beat that finds the previous ping unanswered — or a ping that
   *  times out — closes the socket (half-open TCP) and lets reconnect take over. 0 disables. Default 5000. */
  heartbeatMs?: number;
  /** Test seam: base reconnect backoff (ms). Doubles per failed attempt (cap 10 s), ±30% jitter. */
  _retryBaseMs?: number;
}

export const createGatewayWsTransport = (deps: GatewayWsDeps): Transport => {
  // The gateway downsamples per subscriber+topic (op:"subscribe" maxHz), so a rate cap removes
  // bytes from the wire (not just client-side filtering); its per-client priority outbox honors
  // the scheduler fields.
  const caps: TransportCaps = {
    onDemand: true,
    discovery: "live",
    qos: GATEWAY_QOS,
  };
  let ws: WebSocket | undefined;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  const wantSubs = new Set<string>(); // desired subs, re-sent on reconnect
  const subQos = new Map<string, Qos>(); // declared QoS per topic, re-sent on reconnect
  let topics: TopicInfo[] = [];
  let commands: CommandInfo[] = [];
  let commandsCb: ((c: CommandInfo[]) => void) | undefined;
  const pending = new Map<number, { resolve: (v: unknown) => void; reject: (e: Error) => void }>();
  const pendingPings = new Map<
    number,
    { resolve: (serverTs: number) => void; reject: (e: Error) => void }
  >();
  let rpcId = 1;
  let reconnect = deps.reconnect ?? true;
  const connectTimeoutMs = deps.connectTimeoutMs ?? 10_000;
  const heartbeatMs = deps.heartbeatMs ?? 5_000;
  const retryBaseMs = deps._retryBaseMs ?? 500;

  let started = false; // first attempt launched
  let isOpen = false;
  let failedAttempts = 0; // consecutive, resets on open — drives the backoff
  let retryTimer: ReturnType<typeof setTimeout> | undefined;
  let hbTimer: ReturnType<typeof setInterval> | undefined;
  let hbAwaitingPong = false;
  const openWaiters = new Set<
    { resolve: () => void; reject: (e: Error) => void; deadline: ReturnType<typeof setTimeout> }
  >();

  const transport: Transport = {
    caps,
    label: undefined,
    get commands() {
      return commands;
    },
    connect,
    subscribe,
    unsubscribe,
    publishTeleop,
    publishGoal,
    rpc,
    ping,
    requestList,
    onSample,
    onTopics,
    onStatus,
    onCommands,
    close,
  };

  /** Resolves on the first open; rejects after `connectTimeoutMs`. Reconnect retries keep running
   *  in the background either way — a late gateway still flips status to "open". */
  function connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      if (isOpen) return resolve();
      const waiter = {
        resolve,
        reject,
        deadline: setTimeout(() => {
          openWaiters.delete(waiter);
          reject(new Error(`connect() to ${deps.url} timed out after ${connectTimeoutMs}ms`));
        }, connectTimeoutMs),
      };
      openWaiters.add(waiter);
      if (!started) {
        started = true;
        attempt();
      }
    });
  }

  function settleOpen() {
    for (const w of openWaiters) {
      clearTimeout(w.deadline);
      w.resolve();
    }
    openWaiters.clear();
  }

  function settleFailed(err: Error) {
    for (const w of openWaiters) {
      clearTimeout(w.deadline);
      w.reject(err);
    }
    openWaiters.clear();
  }

  /** One socket attempt. Never throws/rejects — failures land in onclose, which schedules the retry. */
  function attempt() {
    retryTimer = undefined;
    const sock = new WebSocket(deps.url);
    sock.binaryType = "arraybuffer";
    ws = sock;
    statusCb?.("connecting");
    sock.onopen = () => {
      isOpen = true;
      failedAttempts = 0;
      statusCb?.("open");
      for (const t of wantSubs) sendSub(t, subQos.get(t));
      send({ op: "list" });
      startHeartbeat();
      settleOpen();
    };
    sock.onclose = () => {
      isOpen = false;
      stopHeartbeat();
      failInflight(new Error("connection closed"));
      statusCb?.("closed");
      if (reconnect) scheduleRetry();
      else settleFailed(new Error(`connection to ${deps.url} closed`));
    };
    sock.onmessage = (e) => onMessage(e);
    sock.onerror = () => console.debug(`[@dimos/web] ws error on ${deps.url}`);
  }

  function scheduleRetry() {
    if (retryTimer !== undefined) return;
    const backoff = Math.min(10_000, retryBaseMs * 2 ** Math.min(failedAttempts, 8));
    failedAttempts++;
    const jitter = backoff * 0.3 * (Math.random() * 2 - 1);
    retryTimer = setTimeout(attempt, Math.max(0, backoff + jitter));
  }

  function startHeartbeat() {
    if (heartbeatMs <= 0) return;
    stopHeartbeat();
    hbTimer = setInterval(() => {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      if (hbAwaitingPong) {
        ws.close(); // previous beat never answered — half-open link
        return;
      }
      hbAwaitingPong = true;
      ping().then(
        () => (hbAwaitingPong = false),
        () => {
          hbAwaitingPong = false;
          if (ws && ws.readyState === WebSocket.OPEN) ws.close(); // ping timed out — treat as dead
        },
      );
    }, heartbeatMs);
  }

  function stopHeartbeat() {
    if (hbTimer !== undefined) clearInterval(hbTimer);
    hbTimer = undefined;
    hbAwaitingPong = false;
  }

  /** Reject everything awaiting a reply on the dropped socket — callers see the drop now,
   *  not an 8 s timeout later; nothing survives a reconnect silently. */
  function failInflight(err: Error) {
    for (const p of pending.values()) p.reject(err);
    pending.clear();
    for (const p of pendingPings.values()) p.reject(err);
    pendingPings.clear();
  }

  function onMessage(e: MessageEvent) {
    if (typeof e.data === "string") {
      let m: any;
      try {
        m = JSON.parse(e.data);
      } catch {
        return;
      }
      if (m.op === "hello" || m.op === "topics") {
        if (m.label) transport.label = m.label;
        topics = m.topics ?? [];
        topicsCb?.(topics);
        if (Array.isArray(m.rpc)) {
          commands = m.rpc;
          commandsCb?.(commands);
        }
      } else if (m.op === "topic") {
        if (!topics.find((t) => t.topic === m.topic)) {
          topics.push({ topic: m.topic, type: m.type });
          topicsCb?.(topics);
        }
      } else if (m.op === "rpc-res") {
        const p = pending.get(m.id);
        if (p) {
          pending.delete(m.id);
          if (m.error) p.reject(new Error(m.error));
          else p.resolve(m.res);
        }
      } else if (m.op === "pong") {
        const f = pendingPings.get(m.id);
        if (f) {
          pendingPings.delete(m.id);
          f.resolve(m.serverTs);
        }
      }
      return;
    }
    // Frame = [f64 BE gateway-send-ms][LC02 packet] — same wire format frameToSample decodes.
    const s = frameToSample(new Uint8Array(e.data as ArrayBuffer), Date.now());
    if (s) sampleCb?.(s);
  }

  function send(obj: unknown) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }

  // Re-sending subscribe with new qos updates the server's per-client override — how setQos propagates.
  function sendSub(topic: string, qos?: Qos) {
    const q = qos ? applyCaps(qos, caps.qos) : undefined;
    send({
      op: "subscribe",
      topic,
      maxHz: q?.maxHz,
      priority: q?.priority,
      reliability: q?.reliability,
      depth: q?.depth,
    });
  }
  function subscribe(topic: string, qos?: Qos) {
    wantSubs.add(topic);
    if (qos) subQos.set(topic, qos);
    sendSub(topic, qos);
  }
  function unsubscribe(topic: string) {
    wantSubs.delete(topic);
    subQos.delete(topic);
    send({ op: "unsubscribe", topic });
  }
  function publishTeleop(linearX: number, angularZ: number, ttlMs?: number) {
    send({ op: "teleop", linearX, angularZ, ttlMs });
  }
  function publishGoal(x: number, y: number, z = 0) {
    send({ op: "goal", x, y, z });
  }
  function rpc(target: string, method: string, args: unknown[] = []): Promise<unknown> {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error("not connected"));
    }
    const id = rpcId++;
    return new Promise((resolve, reject) => {
      pending.set(id, { resolve, reject });
      send({ op: "rpc", id, target, method, args });
      setTimeout(() => {
        const p = pending.get(id);
        if (p) {
          pending.delete(id);
          p.reject(new Error(`rpc ${target}/${method} timed out`));
        }
      }, 8000);
    });
  }
  // NTP-style clock probe: offset = serverTs − (tSend + rtt/2), assuming a symmetric path.
  function ping(): Promise<ClockSample> {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error("not connected"));
    }
    const id = rpcId++;
    const t0 = Date.now();
    return new Promise((resolve, reject) => {
      pendingPings.set(id, {
        resolve: (serverTs) => {
          const rttMs = Date.now() - t0;
          resolve({ rttMs, offsetMs: serverTs - (t0 + rttMs / 2) });
        },
        reject,
      });
      send({ op: "ping", id });
      setTimeout(() => {
        const p = pendingPings.get(id);
        if (p) {
          pendingPings.delete(id);
          p.reject(new Error("ping timed out"));
        }
      }, 3000);
    });
  }
  function requestList() {
    send({ op: "list" });
  }
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
    reconnect = false;
    if (retryTimer !== undefined) clearTimeout(retryTimer);
    retryTimer = undefined;
    stopHeartbeat();
    settleFailed(new Error("transport closed"));
    failInflight(new Error("transport closed"));
    ws?.close();
  }

  return transport;
};
