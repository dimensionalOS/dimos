// createGatewayWsTransport — talks to the dimos-web-gateway (Bun/LCM or Python/Zenoh)
// over WebSocket. Binary frames are raw LCM packets; JSON frames are control.
// Runs unchanged in the browser and in Bun (both have a global WebSocket).
import { frameToSample } from "./frame.ts";
import { applyCaps } from "../qos.ts";
import type {
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
}

export const createGatewayWsTransport = (deps: GatewayWsDeps): Transport => {
  // qos.maxHz "server" ⇒ the gateway downsamples per subscriber+topic (op:"subscribe" maxHz), so a
  // rate cap removes bytes from the wire (not just client-side filtering); its per-client priority
  // outbox honors declared QoS.
  const caps: TransportCaps = {
    onDemand: true,
    discovery: "live",
    qos: { maxHz: "server", transport: ["priority", "reliability", "depth"] },
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
  let rpcId = 1;
  let reconnect = deps.reconnect ?? true;

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
    requestList,
    onSample,
    onTopics,
    onStatus,
    onCommands,
    close,
  };

  function connect(): Promise<void> {
    return new Promise((resolve) => {
      const sock = new WebSocket(deps.url);
      sock.binaryType = "arraybuffer";
      ws = sock;
      statusCb?.("connecting");
      sock.onopen = () => {
        statusCb?.("open");
        for (const t of wantSubs) sendSub(t, subQos.get(t));
        send({ op: "list" });
        resolve();
      };
      sock.onclose = () => {
        statusCb?.("closed");
        if (reconnect) setTimeout(() => connect(), 1000);
      };
      sock.onmessage = (e) => onMessage(e);
      sock.onerror = () => {};
    });
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
    const q = qos ? applyCaps(qos, caps.qos!) : undefined;
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
    const id = rpcId++;
    return new Promise((resolve, reject) => {
      pending.set(id, { resolve, reject });
      send({ op: "rpc", id, target, method, args });
      setTimeout(() => {
        if (pending.delete(id)) reject(new Error(`rpc ${target}/${method} timed out`));
      }, 8000);
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
    ws?.close();
  }

  return transport;
};
