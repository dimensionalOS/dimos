// Unit tests for Topic — on-demand subscribe lifecycle + rate-limit coalescing.
import assert from "node:assert/strict";

import { createTopic } from "./topic.ts";
import type { MessageMeta, Qos } from "./types.ts";

const meta = (recvTs: number): MessageMeta => ({
  topic: "/t",
  type: "x",
  recvTs,
  sizeBytes: 10,
  dropped: 0,
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

Deno.test("rate-limit: coalesces within the window, counts drops", () => {
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: () => {}, unsubscribe: () => {} },
  });
  let delivered = 0;
  t.subscribe(() => delivered++);
  t.setQos({ maxHz: 10 }); // ≤ 1 per 100 ms
  t._deliver({}, meta(1000)); // first → delivered
  t._deliver({}, meta(1010)); // +10 ms → dropped
  t._deliver({}, meta(1050)); // +50 ms → dropped
  t._deliver({}, meta(1200)); // +200 ms → delivered
  assert.equal(delivered, 2);
  assert.equal(t.stats().dropped, 2);
});

Deno.test("qos: rateLimit 'server' passes maxHz to the wire; 'client' withholds it", () => {
  const wired: (number | undefined)[] = [];
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: (_n, qos) => wired.push(qos?.maxHz), unsubscribe: () => {} },
  });
  t.subscribe(() => {}); // first handler → wired with no cap yet
  t.setQos({ maxHz: 5, rateLimit: "server" }); // ask the gateway to downsample
  t.setQos({ maxHz: 5, rateLimit: "client" }); // gateway keeps sending; drop locally
  assert.deepEqual(wired, [undefined, 5, undefined]);
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

Deno.test("qos: client-side rateLimit still drops in _deliver (bytes flow, delivery capped)", () => {
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: () => {}, unsubscribe: () => {} },
  });
  let delivered = 0;
  t.subscribe(() => delivered++);
  t.setQos({ maxHz: 10, rateLimit: "client" }); // ≤ 1 per 100 ms, enforced client-side
  t._deliver({}, meta(1000)); // delivered
  t._deliver({}, meta(1010)); // dropped (within window)
  t._deliver({}, meta(1200)); // delivered
  assert.equal(delivered, 2);
  assert.equal(t.stats().dropped, 1);
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

Deno.test("qos: conflation 'all' forces unlimited delivery (overrides maxHz)", () => {
  const t = createTopic({
    name: "/t",
    type: "x",
    wiring: { subscribe: () => {}, unsubscribe: () => {} },
  });
  let delivered = 0;
  t.subscribe(() => delivered++);
  t.setQos({ maxHz: 10, conflation: "all" }); // "all" ⇒ deliver every message
  t._deliver({}, meta(1000));
  t._deliver({}, meta(1010));
  t._deliver({}, meta(1020));
  assert.equal(delivered, 3);
  assert.equal(t.stats().dropped, 0);
});
