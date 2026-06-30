// Unit tests for Topic — on-demand subscribe lifecycle + rate-limit coalescing.
import assert from "node:assert/strict";

import { createTopic } from "./topic.ts";
import type { MessageMeta } from "./types.ts";

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
  t.setRateLimit(10); // ≤ 1 per 100 ms
  t._deliver({}, meta(1000)); // first → delivered
  t._deliver({}, meta(1010)); // +10 ms → dropped
  t._deliver({}, meta(1050)); // +50 ms → dropped
  t._deliver({}, meta(1200)); // +200 ms → delivered
  assert.equal(delivered, 2);
  assert.equal(t.stats().dropped, 2);
});
