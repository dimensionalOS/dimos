// Unit tests for the bench measurement core — percentiles, stddev, seq-loss vs late,
// warmup discard, and on-demand savings. Uses a fake client so the math is deterministic
// (no transport). Timing-derived rates (hz/kB/s) are asserted as ranges because the
// divisor is the actual elapsed wall clock, not the nominal window.
import assert from "node:assert/strict";

import {
  type BenchRow,
  formatMarkdown,
  measureScenario,
  onDemandSaving,
  STREAM_PROFILES,
} from "./bench.ts";
import type { Message, MessageMeta } from "./types.ts";

const NO_PHASES = { warmupMs: 0, graceMs: 0 };

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
    const meta: MessageMeta = {
      topic: name,
      type: "x",
      recvTs: 0,
      sizeBytes: 100,
      ...over,
    };
    (handlers.get(name) ?? []).forEach((h) => h({ data: {}, ts: meta.srcTs ?? meta.recvTs, meta }));
  };
  return { client: client as never, deliver };
}

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

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
  latePct: 0,
  ...over,
});

Deno.test("measureScenario: percentiles, stddev, seq-loss, hz, kB/s", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 30, false, undefined, NO_PHASES);
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
  assert.equal(r.lossPct, 9.09); // 1 - 10/11, no grace arrival
  assert.equal(r.latePct, 0);
  // hz/kB/s divide by actual elapsed (≥ nominal 30ms, plus timer slop) — assert a sane band.
  assert.ok(r.hz > 100 && r.hz <= 333.34, `hz out of band: ${r.hz}`);
  assert.ok(r.kbps > 9.7 && r.kbps <= 32.56, `kbps out of band: ${r.kbps}`);
});

Deno.test("measureScenario: no seq stamp → lossPct/latePct are NaN", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20, false, undefined, NO_PHASES);
  deliver("/a", { latencyMs: 5 });
  const r = await p;
  assert.ok(Number.isNaN(r.lossPct));
  assert.ok(Number.isNaN(r.latePct));
});

Deno.test("measureScenario: warmup discards ramp-up samples", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 40, false, undefined, {
    warmupMs: 20,
    graceMs: 0,
  });
  deliver("/a", { latencyMs: 999, seq: 0 }); // lands in warmup → discarded
  await sleep(30);
  deliver("/a", { latencyMs: 5, seq: 1 }); // lands in the measure window
  const r = await p;
  assert.equal(r.msgs, 1);
  assert.equal(r.latMax, 5); // the warmup sample's latency never entered the stats
});

Deno.test("measureScenario: frame arriving in grace is late, not lost", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 30, false, undefined, {
    warmupMs: 0,
    graceMs: 60,
  });
  // Window sees seqs 0,1,3 → span 4, seq 2 missing at window close.
  for (const seq of [0, 1, 3]) deliver("/a", { latencyMs: 1, seq });
  await sleep(45); // past the 30ms window, inside the grace drain
  deliver("/a", { latencyMs: 500, seq: 2 }); // the delayed frame
  deliver("/a", { latencyMs: 1, seq: 9 }); // outside the measured span → ignored
  const r = await p;
  assert.equal(r.msgs, 3); // grace arrivals don't count toward rate
  assert.equal(r.lossPct, 0); // seq 2 was delayed, not dropped
  assert.equal(r.latePct, 25); // 1 of the 4-frame span
});

Deno.test("measureScenario: offsetMs corrects cross-machine end-to-end latency", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20, true, undefined, {
    ...NO_PHASES,
    offsetMs: 5000, // source clock runs 5s ahead of the client clock
  });
  deliver("/a", { srcTs: 10_000, recvTs: 5_010 }); // raw recvTs−srcTs = −4990; corrected = 10
  deliver("/a", { srcTs: 11_000, recvTs: 6_002 }); // corrected = 2
  const r = await p;
  assert.equal(r.msgs, 2);
  assert.equal(r.latMax, 10);
  assert.equal(r.latP50, 10); // floor(0.5*2)=1 → sorted[1]
});

Deno.test("measureScenario: rate-limited run reports NaN loss (gaps are intentional)", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20, true, {
    maxHz: 5,
  }, NO_PHASES);
  deliver("/a", { srcTs: 0, recvTs: 3, seq: 0 });
  deliver("/a", { srcTs: 0, recvTs: 3, seq: 4 }); // gap = downsampling, not loss
  const r = await p;
  assert.equal(r.msgs, 2);
  assert.ok(Number.isNaN(r.lossPct));
  assert.ok(Number.isNaN(r.latePct));
});

Deno.test("measureScenario: qos arg is optional-chained (fake topic without setQos doesn't throw)", async () => {
  const { client, deliver } = fakeClient();
  // fakeClient.topic() returns an object WITHOUT setQos — passing qos must not throw.
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20, true, {
    maxHz: 5,
  }, NO_PHASES);
  deliver("/a", { srcTs: 0, recvTs: 3, seq: 0 });
  const r = await p;
  assert.equal(r.msgs, 1);
});

Deno.test("onDemandSaving: on-demand vs all-lanes kB/s", () => {
  const rows = [
    row({ scenario: "all-lanes", topics: 4, kbps: 40 }),
    row({ scenario: "on-demand", topics: 1, kbps: 10 }),
  ];
  assert.equal(onDemandSaving(rows), 75);
});

Deno.test("formatMarkdown: on-demand footer renders only when the pair ran", () => {
  const pair = [
    row({ scenario: "all-lanes", topics: 4, kbps: 40 }),
    row({ scenario: "on-demand", topics: 1, kbps: 10 }),
  ];
  const md = formatMarkdown("ws", "ws://x", 1000, "2026-07-02", pair);
  assert.ok(md.includes("75% reduction"), md);
  assert.ok(md.includes("1 of 4 topics"), md);
  const solo = formatMarkdown("ws", "ws://x", 1000, "2026-07-02", [
    row({ scenario: "pose", topics: 3, kbps: 5 }),
  ]);
  assert.ok(!solo.includes("On-demand bandwidth"), solo);
});

Deno.test("STREAM_PROFILES carries the pair the footer matches on (all-lanes + on-demand)", () => {
  const ids = STREAM_PROFILES.map((p) => p.id);
  assert.ok(ids.includes("all-lanes"), "all-lanes profile missing");
  assert.ok(ids.includes("on-demand"), "on-demand profile missing");
});
