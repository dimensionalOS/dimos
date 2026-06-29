// GatewayWsTransport — talks to the dimos-web-gateway (Bun/LCM or Python/Zenoh)
// over WebSocket. Binary frames are raw LCM packets; JSON frames are control.
// Runs unchanged in the browser and in Bun (both have a global WebSocket).
import { decodeChannel } from "@dimos/msgs";
import { splitChannel } from "../decode";
import type { Transport, RawSample, Status, TransportCaps } from "../transport";
import type { TopicInfo } from "../types";

export interface GatewayWsOptions {
  reconnect?: boolean;
}

export class GatewayWsTransport implements Transport {
  readonly caps: TransportCaps = { onDemand: true, discovery: "live" };
  label?: string;
  private ws?: WebSocket;
  private sampleCb?: (s: RawSample) => void;
  private topicsCb?: (t: TopicInfo[]) => void;
  private statusCb?: (s: Status) => void;
  private wantSubs = new Set<string>(); // desired subs, re-sent on reconnect
  private rates = new Map<string, number>();
  private topics: TopicInfo[] = [];
  private reconnect: boolean;

  constructor(private url: string, opts: GatewayWsOptions = {}) {
    this.reconnect = opts.reconnect ?? true;
  }

  connect(): Promise<void> {
    return new Promise((resolve) => {
      const ws = new WebSocket(this.url);
      ws.binaryType = "arraybuffer";
      this.ws = ws;
      this.statusCb?.("connecting");
      ws.onopen = () => {
        this.statusCb?.("open");
        for (const t of this.wantSubs) this.send({ op: "subscribe", topic: t, maxHz: this.rates.get(t) });
        this.send({ op: "list" });
        resolve();
      };
      ws.onclose = () => {
        this.statusCb?.("closed");
        if (this.reconnect) setTimeout(() => this.connect(), 1000);
      };
      ws.onmessage = (e) => this.onMessage(e);
      ws.onerror = () => {};
    });
  }

  private onMessage(e: MessageEvent) {
    if (typeof e.data === "string") {
      let m: any;
      try {
        m = JSON.parse(e.data);
      } catch {
        return;
      }
      if (m.op === "hello" || m.op === "topics") {
        if (m.label) this.label = m.label;
        this.topics = m.topics ?? [];
        this.topicsCb?.(this.topics);
      } else if (m.op === "topic") {
        if (!this.topics.find((t) => t.topic === m.topic)) {
          this.topics.push({ topic: m.topic, type: m.type });
          this.topicsCb?.(this.topics);
        }
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
    this.sampleCb?.({ topic, type, payload, recvTs: Date.now(), gatewaySendMs });
  }

  private send(obj: unknown) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  subscribe(topic: string, maxHz?: number) {
    this.wantSubs.add(topic);
    if (maxHz) this.rates.set(topic, maxHz);
    this.send({ op: "subscribe", topic, maxHz });
  }
  unsubscribe(topic: string) {
    this.wantSubs.delete(topic);
    this.rates.delete(topic);
    this.send({ op: "unsubscribe", topic });
  }
  publishTeleop(linearX: number, angularZ: number, ttlMs?: number) {
    this.send({ op: "teleop", linearX, angularZ, ttlMs });
  }
  publishGoal(x: number, y: number, z = 0) {
    this.send({ op: "goal", x, y, z });
  }
  requestList() {
    this.send({ op: "list" });
  }
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
    this.reconnect = false;
    this.ws?.close();
  }
}
