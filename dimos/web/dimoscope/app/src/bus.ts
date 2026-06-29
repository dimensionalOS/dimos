// ─────────────────────────────────────────────────────────────────────────
// DimosBus — the framework core (Rungs 2-4)
//
// One WebSocket to the bridge. For every inbound packet it:
//   1. decodes it with @dimos/msgs (type auto-resolved from the 8-byte hash),
//   2. learns the topic (discovery — no config needed),
//   3. routes the decoded message to whoever subscribed to that topic.
// It can also publish typed messages back (teleop).
//
// This is deliberately framework-agnostic (no React) so it could back a Svelte/
// Vue/vanilla UI too. The React glue lives in useBus.tsx.
// ─────────────────────────────────────────────────────────────────────────
import { decodePacket, encodePacket, geometry_msgs } from "@dimos/msgs";

export type Decoded = { topic: string; type: string; data: any; ts: number };
type Handler = (d: Decoded) => void;
export type Status = "connecting" | "open" | "closed";

export class DimosBus {
  /** Discovered topics: topic name -> message type (e.g. "/odom" -> "geometry_msgs.PoseStamped"). */
  topics = new Map<string, string>();
  status: Status = "connecting";

  private ws!: WebSocket;
  private subs = new Map<string, Set<Handler>>();
  private listeners = new Set<() => void>(); // notified on topic-list / status change

  constructor(public url: string) {
    this.connect();
  }

  private connect() {
    const ws = new WebSocket(this.url);
    ws.binaryType = "arraybuffer";
    this.ws = ws;
    this.status = "connecting";
    this.emit();

    ws.onopen = () => {
      this.status = "open";
      this.emit();
    };
    ws.onclose = () => {
      this.status = "closed";
      this.emit();
      setTimeout(() => this.connect(), 1000); // auto-reconnect
    };
    ws.onmessage = (e) => {
      if (!(e.data instanceof ArrayBuffer)) return;
      let channel: string, data: unknown;
      try {
        ({ channel, data } = decodePacket(new Uint8Array(e.data)));
      } catch {
        return; // fragmented (>64KB) or unknown type — skipped until Rung 7
      }
      // Channel convention: "<topic>#<pkg>.<Type>"
      const h = channel.indexOf("#");
      const topic = h >= 0 ? channel.slice(0, h) : channel;
      const type = h >= 0 ? channel.slice(h + 1) : "?";
      if (!this.topics.has(topic)) {
        this.topics.set(topic, type);
        this.emit(); // a new topic appeared → UI updates its sidebar
      }
      this.subs.get(topic)?.forEach((fn) => fn({ topic, type, data, ts: Date.now() }));
    };
  }

  /** Subscribe to decoded messages on a topic. Returns an unsubscribe fn. */
  subscribe(topic: string, handler: Handler): () => void {
    let set = this.subs.get(topic);
    if (!set) {
      set = new Set();
      this.subs.set(topic, set);
    }
    set.add(handler);
    return () => set!.delete(handler);
  }

  /** Publish a pre-encoded LCM packet (raw bytes) back onto the bus. */
  publishPacket(packet: Uint8Array) {
    if (this.ws.readyState === WebSocket.OPEN) this.ws.send(packet);
  }

  /** Convenience: publish a Twist on /cmd_vel (teleop). */
  publishTwist(linX: number, angZ: number) {
    const t = new geometry_msgs.Twist({
      linear: new geometry_msgs.Vector3({ x: linX, y: 0, z: 0 }),
      angular: new geometry_msgs.Vector3({ x: 0, y: 0, z: angZ }),
    });
    this.publishPacket(encodePacket("/cmd_vel#geometry_msgs.Twist", t));
  }

  /** React glue: be notified when the topic list or status changes. */
  onChange(fn: () => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }
  private emit() {
    this.listeners.forEach((fn) => fn());
  }
}
