// Read-only Server-Sent Events; gateway streams `data: <base64(frame)>` on GET /sse?topics=<csv|*>;
// parsed with fetch+stream reader (not EventSource) so the same adapter runs in browser and headless
// (Deno/Bun); base64 ≈ 33% overhead; no per-subscriber downsample.
import type {
  CommandInfo,
  RawSample,
  Status,
  TopicInfo,
  Transport,
  TransportCaps,
} from "../../types.ts";
import { b64ToBytes, frameToSample } from "../frame.ts";

export interface SseDeps {
  url: string; // gateway base, e.g. http://localhost:8080
  reconnect?: boolean;
}

export const createSseTransport = (deps: SseDeps): Transport => {
  // No caps.qos: /sse filters by topic set only (no per-subscriber downsample — that's on the
  // WS control channel). QoS is unsupported here — subscribes deliver at full wire rate.
  const caps: TransportCaps = { onDemand: true, discovery: "passive" };
  const base = deps.url.replace(/\/$/, "").replace(/^ws/, "http");
  let sampleCb: ((s: RawSample) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  const wantSubs = new Set<string>();
  let abort: AbortController | undefined;
  let reopenTimer: ReturnType<typeof setTimeout> | undefined;
  let connected = false;
  let reconnect = deps.reconnect ?? true;

  function streamUrl(): string {
    const topics = wantSubs.size ? [...wantSubs].join(",") : "*";
    return `${base}/sse?topics=${encodeURIComponent(topics)}`;
  }

  function handleEvent(evt: string) {
    let data = "";
    let isHello = false;
    for (const line of evt.split("\n")) {
      if (line.startsWith("data:")) data += line.slice(5).trim();
      else if (line.startsWith("event:") && line.includes("hello")) isHello = true;
    }
    if (!data) return;
    if (isHello) {
      try {
        const j = JSON.parse(data);
        if (j.label) transport.label = j.label;
        if (Array.isArray(j.topics)) topicsCb?.(j.topics);
      } catch {
        // ignore malformed hello
      }
      return;
    }
    const s = frameToSample(b64ToBytes(data), Date.now());
    if (s) sampleCb?.(s);
  }

  async function open() {
    abort?.abort();
    const ac = new AbortController();
    abort = ac;
    statusCb?.("connecting");
    try {
      const res = await fetch(streamUrl(), {
        signal: ac.signal,
        headers: { accept: "text/event-stream" },
      });
      if (!res.body) throw new Error("no body");
      statusCb?.("open");
      const reader = res.body.getReader();
      const td = new TextDecoder();
      let buf = "";
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        buf += td.decode(value, { stream: true });
        let nl: number;
        while ((nl = buf.indexOf("\n\n")) >= 0) {
          handleEvent(buf.slice(0, nl));
          buf = buf.slice(nl + 2);
        }
      }
    } catch {
      // stream ended/aborted → fall through to reconnect
    }
    statusCb?.("closed");
    if (reconnect && connected) reopenTimer = setTimeout(open, 1000);
  }

  function reopenSoon() {
    if (!connected) return;
    if (reopenTimer) clearTimeout(reopenTimer);
    reopenTimer = setTimeout(open, 30); // debounce rapid subscribe/unsubscribe
  }

  const transport: Transport = {
    caps,
    label: "SSE",
    get commands(): CommandInfo[] {
      return [];
    },
    connect() {
      connected = true;
      if (wantSubs.size) open();
      return Promise.resolve();
    },
    subscribe(topic: string) {
      wantSubs.add(topic);
      reopenSoon();
    },
    unsubscribe(topic: string) {
      wantSubs.delete(topic);
      if (wantSubs.size) reopenSoon();
      else abort?.abort(); // nothing subscribed → drop the stream
    },
    publishTeleop() {}, // read-only channel (control would route to a gateway)
    publishGoal() {},
    rpc() {
      return Promise.reject(new Error("SSE transport is read-only"));
    },
    requestList() {},
    onSample(cb: (s: RawSample) => void) {
      sampleCb = cb;
    },
    onTopics(cb: (t: TopicInfo[]) => void) {
      topicsCb = cb;
    },
    onStatus(cb: (s: Status) => void) {
      statusCb = cb;
    },
    close() {
      reconnect = false;
      connected = false;
      if (reopenTimer) clearTimeout(reopenTimer);
      abort?.abort();
    },
  };
  return transport;
};
