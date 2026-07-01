// Read-only HTTP long-poll baseline; loops GET /poll?since=<cursor>&topics=<csv|*>&max=<n>; gateway
// holds until frames arrive (or timeout) then returns a binary batch; ring-buffer cursor detects
// loss; server→client only.
import type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "../../transport.ts";
import type { TopicInfo } from "../../types.ts";
import { frameToSample } from "../frame.ts";

export interface HttpPollDeps {
  url: string; // gateway base, e.g. http://localhost:8090
  maxBatch?: number;
}

export const createHttpPollTransport = (deps: HttpPollDeps): Transport => {
  // qos.maxHz "client": /poll filters by topic set only (no per-subscriber downsample) → client-side cap.
  const caps: TransportCaps = { onDemand: true, discovery: "passive", qos: { maxHz: "client" } };
  const base = deps.url.replace(/\/$/, "").replace(/^ws/, "http");
  const maxBatch = deps.maxBatch ?? 512;
  let sampleCb: ((s: RawSample) => void) | undefined;
  // httpPoll does passive discovery, so it stores but never pushes a topic list.
  let _topicsCb: ((t: TopicInfo[]) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;
  const wantSubs = new Set<string>();
  let cursor = -1; // first poll asks for "head" so we only get frames from now on
  let running = false;
  let connected = false;

  async function loop() {
    if (running) return;
    running = true;
    statusCb?.("open");
    while (running) {
      const topics = wantSubs.size ? [...wantSubs].join(",") : "*";
      const url = `${base}/poll?since=${cursor}&topics=${
        encodeURIComponent(topics)
      }&max=${maxBatch}`;
      try {
        const res = await fetch(url);
        const buf = new Uint8Array(await res.arrayBuffer());
        if (buf.byteLength >= 8) {
          const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
          const newCursor = dv.getUint32(0, false);
          const count = dv.getUint32(4, false);
          let off = 8;
          const now = Date.now();
          for (let i = 0; i < count && off + 4 <= buf.byteLength; i++) {
            const len = dv.getUint32(off, false);
            off += 4;
            const s = frameToSample(buf.subarray(off, off + len), now);
            off += len;
            if (s) sampleCb?.(s);
          }
          cursor = newCursor;
        }
      } catch {
        statusCb?.("closed");
        await new Promise((r) => setTimeout(r, 500)); // back off, then retry
      }
    }
  }

  const transport: Transport = {
    caps,
    label: "HTTP-poll",
    get commands(): CommandInfo[] {
      return [];
    },
    connect() {
      connected = true;
      if (wantSubs.size) loop();
      return Promise.resolve();
    },
    subscribe(topic: string) {
      wantSubs.add(topic);
      if (connected) loop();
    },
    unsubscribe(topic: string) {
      wantSubs.delete(topic);
    },
    publishTeleop() {},
    publishGoal() {},
    rpc() {
      return Promise.reject(new Error("HTTP-poll transport is read-only"));
    },
    requestList() {},
    onSample(cb: (s: RawSample) => void) {
      sampleCb = cb;
    },
    onTopics(cb: (t: TopicInfo[]) => void) {
      _topicsCb = cb;
    },
    onStatus(cb: (s: Status) => void) {
      statusCb = cb;
    },
    close() {
      running = false;
      connected = false;
    },
  };
  return transport;
};
