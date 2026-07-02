// Unit tests for Topic — on-demand subscribe lifecycle + verbatim delivery (QoS is a server
// request; the client never drops locally).
import assert from "node:assert/strict";

import { createTopic } from "./topic.ts";
import type { MessageMeta, Qos } from "./types.ts";

const meta = (recvTs: number): MessageMeta => ({
  topic: "/t",
  type: "x",
  recvTs,
  sizeBytes: 10,
});

Deno.test("on-demand: wire subscribe on first handler, unsubscribe on last", () => {
  const calls: string[] = [];
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: () => calls.push("sub"), unsubscribe: () => calls.push("unsub") },
  });
  const a = t.subscribe(() => {});
  const b = t.subscribe(() => {});
  assert.deepEqual(calls, ["sub"]); // wired once, not per-handler
  a.unsubscribe();
  assert.deepEqual(calls, ["sub"]); // b still active → stays subscribed
  b.unsubscribe();
  assert.deepEqual(calls, ["sub", "unsub"]); // last handler gone → unsubscribed
});

Deno.test("delivery is verbatim: every received message reaches handlers", () => {
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: () => {}, unsubscribe: () => {} },
  });
  let delivered = 0;
  t.subscribe(() => delivered++);
  t.setQos({ maxHz: 10 }); // a server request — no local gate
  t._deliver({}, meta(1000));
  t._deliver({}, meta(1010)); // 100 Hz burst still delivers (the gateway is the rate limiter)
  t._deliver({}, meta(1020));
  assert.equal(delivered, 3);
  assert.equal(t.stats().count, 3);
});

Deno.test("qos: setQos passes the request verbatim to the wire; re-set re-subscribes", () => {
  const wired: (Qos | undefined)[] = [];
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: (_n, qos) => wired.push(qos), unsubscribe: () => {} },
  });
  t.subscribe(() => {}); // first handler → wired with the default (empty) request
  t.setQos({ maxHz: 5 }); // ask the gateway to downsample
  t.setQos({ maxHz: 5, priority: "high" });
  assert.deepEqual(wired, [{}, { maxHz: 5 }, { maxHz: 5, priority: "high" }]);
});

Deno.test("qos: declared priority/reliability flow to the transport (client override)", () => {
  const seen: (Qos | undefined)[] = [];
  const t = createTopic({
    name: "/lidar",
    type: "sensor_msgs.PointCloud2",
    wiring: { subscribe: (_n, qos) => seen.push(qos), unsubscribe: () => {} },
  });
  t.subscribe(() => {});
  t.setQos({ priority: "critical", reliability: "reliable" }); // override the server default
  const last = seen.at(-1);
  assert.equal(last?.priority, "critical");
  assert.equal(last?.reliability, "reliable");
});

Deno.test("a throwing subscriber doesn't break delivery to later subscribers", () => {
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: () => {}, unsubscribe: () => {} },
  });
  let delivered = 0;
  t.subscribe(() => {
    throw new Error("boom");
  });
  t.subscribe(() => delivered++);
  const origError = console.error;
  console.error = () => {}; // the isolation logs — keep test output clean
  try {
    t._deliver({}, meta(1000));
  } finally {
    console.error = origError;
  }
  assert.equal(delivered, 1);
});
