// Unit tests for the bench measurement core — percentiles, stddev, seq-loss, and
// on-demand savings. Uses a fake client so the math is deterministic (no transport).
import assert from "node:assert/strict";

import { type BenchRow, measureScenario, onDemandSaving } from "./bench.ts";
import type { Message, MessageMeta } from "./types.ts";

// Records per-topic handlers so a test can push synthetic samples during the
// measure window. measureScenario registers its subscribers synchronously (before
// its first await), so delivering right after the call lands inside the window.
function fakeClient() {
  const handlers = new Map<string, Array<(message: Message<unknown>) => void>>();
  const client = {
    topic(name: string) {
      return {
        subscribe(h: (message: Message<unknown>) => void) {
          const arr = handlers.get(name) ?? [];
          arr.push(h);
          handlers.set(name, arr);
          return { unsubscribe() {} };
        },
      };
    },
  };
  const deliver = (name: string, over: Partial<MessageMeta>) => {
    const meta: MessageMeta = { topic: name, type: "x", recvTs: 0, sizeBytes: 100, dropped: 0, ...over };
    (handlers.get(name) ?? []).forEach((h) => h({ data: {}, ts: meta.srcTs ?? meta.recvTs, meta }));
  };
  return { client: client as never, deliver };
}

const row = (over: Partial<BenchRow>): BenchRow => ({
  scenario: "s",
  topics: 1,
  msgs: 0,
  hz: 0,
  kbps: 0,
  latP50: 0,
  latP95: 0,
  latP99: 0,
  latMax: 0,
  latStd: 0,
  lossPct: 0,
  ...over,
});

Deno.test("measureScenario: percentiles, stddev, seq-loss, hz, kB/s", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 30);
  const lats = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
  const seqs = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10]; // seq 8 dropped → span 11, recv 10
  for (let i = 0; i < lats.length; i++) {
    deliver("/a", { latencyMs: lats[i], seq: seqs[i], sizeBytes: 100 });
  }
  const r = await p;
  assert.equal(r.msgs, 10);
  assert.equal(r.latP50, 6);
  assert.equal(r.latP95, 10);
  assert.equal(r.latP99, 10);
  assert.equal(r.latMax, 10);
  assert.equal(r.latStd, 2.87); // sqrt(8.25) rounded
  assert.equal(r.lossPct, 9.09); // 1 - 10/11
  assert.equal(r.hz, 333.33); // 10 / (30/1000)
  assert.equal(r.kbps, 32.55); // 1000B / 1024 / 0.03s
});

Deno.test("measureScenario: no seq stamp → lossPct is NaN", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20);
  deliver("/a", { latencyMs: 5 });
  const r = await p;
  assert.ok(Number.isNaN(r.lossPct));
});

Deno.test("measureScenario: qos arg is optional-chained (fake topic without setQos doesn't throw)", async () => {
  const { client, deliver } = fakeClient();
  // fakeClient.topic() returns an object WITHOUT setQos — passing qos must not throw.
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20, true, {
    maxHz: 5,
    rateLimit: "server",
  });
  deliver("/a", { srcTs: 0, recvTs: 3, seq: 0 });
  const r = await p;
  assert.equal(r.msgs, 1);
});

Deno.test("onDemandSaving: 1-of-4 vs all-4 kB/s", () => {
  const rows = [
    row({ scenario: "4x (throughput)", topics: 4, kbps: 40 }),
    row({ scenario: "1x (on-demand)", topics: 1, kbps: 10 }),
  ];
  assert.equal(onDemandSaving(rows), 75);
});
