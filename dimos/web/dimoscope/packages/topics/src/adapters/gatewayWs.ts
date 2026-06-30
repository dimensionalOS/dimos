// createGatewayWsTransport — talks to the dimos-web-gateway (Bun/LCM or Python/Zenoh)
// over WebSocket. Binary frames are raw LCM packets; JSON frames are control.
// Runs unchanged in the browser and in Bun (both have a global WebSocket).
import { decodeChannel } from "@dimos/msgs";
import { splitChannel } from "../decode.ts";
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../transport.ts";
import type { TopicInfo } from "../types.ts";

export interface GatewayWsDeps {
  url: string;
  reconnect?: boolean;
  /** "server-json": ask the gateway to decode and send JSON (rosbridge-style) instead of the
   *  binary self-describing frame. Default "client" (the browser decodes via @dimos/msgs). */
  decode?: "client" | "server-json";
}

export const createGatewayWsTransport = (deps: GatewayWsDeps): Transport => {
  // qos.maxHz "server": the gateway downsamples per subscriber+topic (op:"subscribe" maxHz),
  // so a rate cap actually removes bytes from the wire — not just client-side filtering.
  const caps: TransportCaps = { onDemand: true, discovery: "live", qos: { maxHz: "server" } };
  let ws: WebSocket | undefined;
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  const wantSubs = new Set<string>(); // desired subs, re-sent on reconnect
  const rates = new Map<string, number>();
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
        for (const t of wantSubs) {
          send({ op: "subscribe", topic: t, maxHz: rates.get(t), decode: deps.decode });
        }
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
      } else if (m.op === "sample") {
        // server-json mode: the gateway decoded the message; deliver it directly. payload carries
        // the JSON wire bytes so sizeBytes (bandwidth) reflects the JSON cost vs the binary frame.
        sampleCb?.({
          topic: m.topic,
          type: m.type,
          payload: new TextEncoder().encode(e.data),
          recvTs: Date.now(),
          gatewaySendMs: m.gatewaySendMs,
          decoded: m.data,
        });
      }
      return;
    }
    // Frame = [f64 BE gateway-send-ms][LC02 packet]
    const buf = e.data as ArrayBuffer;
    if (buf.byteLength < 8) return;
    const gatewaySendMs = new DataView(buf).getFloat64(0, false);
    const packet = new Uint8Array(buf, 8);
    let channel: string, payload: Uint8Array;
    try {
      ({ channel, payload } = decodeChannel(packet));
    } catch {
      return; // not an LC02 packet we can split
    }
    const { topic, type } = splitChannel(channel);
    sampleCb?.({ topic, type, payload, recvTs: Date.now(), gatewaySendMs });
  }

  function send(obj: unknown) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }

  function subscribe(topic: string, maxHz?: number) {
    wantSubs.add(topic);
    if (maxHz) rates.set(topic, maxHz);
    send({ op: "subscribe", topic, maxHz, decode: deps.decode });
  }
  function unsubscribe(topic: string) {
    wantSubs.delete(topic);
    rates.delete(topic);
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
