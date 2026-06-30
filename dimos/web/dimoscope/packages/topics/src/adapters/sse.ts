// createSseTransport — read-only delivery over Server-Sent Events.
//
// The gateway streams `data: <base64(frame)>\n\n` on GET /sse?topics=<csv|*>. We parse
// the event-stream with fetch + a stream reader (NOT EventSource), so the exact same
// adapter runs in the browser AND headless under Deno/Bun. SSE is server→client only and
// text-framed, so: (1) teleop/goal/rpc are not on this channel — like zenoh-ts, control
// would route to a gateway; (2) binary frames ride as base64 (~33% larger) — a real cost
// this benchmark is meant to expose. Subscriptions are the URL topic set; changing them
// reopens the stream (the gateway then filters → true per-stream on-demand).
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../transport.ts";
import type { TopicInfo } from "../types.ts";
import { b64ToBytes, frameToSample } from "./gatewayFrame.ts";

export interface SseDeps {
  url: string; // gateway base, e.g. http://localhost:8090
  reconnect?: boolean;
}

export const createSseTransport = (deps: SseDeps): Transport => {
  // qos.maxHz "client": the /sse endpoint filters by topic set only — it has no per-subscriber
  // downsample (that lives on the WS control channel), so a rate cap is enforced client-side.
  const caps: TransportCaps = { onDemand: true, discovery: "passive", qos: { maxHz: "client" } };
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
