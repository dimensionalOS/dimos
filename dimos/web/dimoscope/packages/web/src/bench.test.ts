// Unit tests for the bench measurement core — percentiles, stddev, seq-loss vs late,
// warmup discard, per-lane split, time-series buckets, offered-vs-delivered, and on-demand
// savings. Uses a fake client so the math is deterministic (no transport). Timing-derived
// rates (hz/kB/s) are asserted as ranges because the divisor is the actual elapsed wall
// clock, not the nominal window; ratio metrics (deliveryPct) cancel elapsed and are exact.
import assert from "node:assert/strict";

import {
  type BenchBucket,
  type BenchRow,
  measureScenario,
  ON_DEMAND_PAIR,
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
  elapsedMs: 0,
  offeredHz: NaN,
  offeredKbps: NaN,
  deliveryPct: NaN,
  seqResets: 0,
  lanes: [],
  buckets: [],
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
  // The single-topic row's lane slice IS the row.
  assert.equal(r.lanes.length, 1);
  assert.equal(r.lanes[0].topic, "/a");
  assert.equal(r.lanes[0].msgs, 10);
  assert.equal(r.lanes[0].latP50, 6);
  assert.equal(r.lanes[0].lossPct, 9.09);
});

Deno.test("measureScenario: no seq stamp → lossPct/latePct/offered are NaN", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 20, false, undefined, NO_PHASES);
  deliver("/a", { latencyMs: 5 });
  const r = await p;
  assert.ok(Number.isNaN(r.lossPct));
  assert.ok(Number.isNaN(r.latePct));
  assert.ok(Number.isNaN(r.offeredHz));
  assert.ok(Number.isNaN(r.deliveryPct));
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

Deno.test("measureScenario: frame arriving in grace is late, not lost (pooled + lane)", async () => {
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
  assert.equal(r.lanes[0].latePct, 25);
  assert.equal(r.lanes[0].lossPct, 0);
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

Deno.test("measureScenario: rate-limited run reports NaN loss/offered (gaps are intentional)", async () => {
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
  assert.ok(Number.isNaN(r.offeredHz));
  assert.ok(Number.isNaN(r.deliveryPct));
  assert.ok(Number.isNaN(r.lanes[0].offeredHz));
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

Deno.test("measureScenario: per-lane split — each lane keeps its own stats, pooled derives", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(
    client,
    { name: "t", topics: ["/fast", "/img"] },
    30,
    false,
    undefined,
    NO_PHASES,
  );
  // /fast: 4 msgs, low latency, small. /img: 2 msgs, high latency, big.
  for (const [lat, seq] of [[1, 0], [2, 1], [3, 2], [4, 3]] as const) {
    deliver("/fast", { latencyMs: lat, seq, sizeBytes: 100 });
  }
  for (const [lat, seq] of [[100, 0], [200, 1]] as const) {
    deliver("/img", { latencyMs: lat, seq, sizeBytes: 10_000 });
  }
  const r = await p;
  assert.equal(r.lanes.length, 2);
  assert.equal(r.lanes[0].topic, "/fast"); // scenario order
  assert.equal(r.lanes[1].topic, "/img");
  assert.equal(r.lanes[0].msgs, 4);
  assert.equal(r.lanes[1].msgs, 2);
  assert.equal(r.lanes[0].latMax, 4); // /img's 200ms never leaks into /fast
  assert.equal(r.lanes[1].latP50, 200); // floor(0.5*2)=1 → sorted[1]
  assert.equal(r.lanes[1].latMax, 200);
  assert.equal(r.msgs, 6); // pooled = Σ lanes
  assert.equal(r.latMax, 200); // pooled max = max over lanes
  assert.equal(r.topics, 2);
  // Both lanes gapless → pooled offered is the sum, delivery 100.
  assert.equal(r.deliveryPct, 100);
  assert.equal(r.lanes[0].deliveryPct, 100);
});

Deno.test("measureScenario: per-lane loss isolates the lossy lane (the fair-lane story)", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(
    client,
    { name: "t", topics: ["/a", "/b"] },
    30,
    false,
    undefined,
    NO_PHASES,
  );
  for (const seq of [0, 1, 3]) deliver("/a", { latencyMs: 1, seq }); // span 4, recv 3 → 25%
  for (const seq of [0, 1, 2, 3]) deliver("/b", { latencyMs: 1, seq }); // clean
  const r = await p;
  assert.equal(r.lanes[0].lossPct, 25);
  assert.equal(r.lanes[1].lossPct, 0);
  assert.equal(r.lossPct, 12.5); // pooled: 1 missing of 8 expected
});

Deno.test("measureScenario: silent lane is visible, pooled offered goes strict-NaN", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(
    client,
    { name: "t", topics: ["/a", "/b"] },
    20,
    false,
    undefined,
    NO_PHASES,
  );
  for (const seq of [0, 1, 2]) deliver("/a", { latencyMs: 1, seq });
  const r = await p;
  const b = r.lanes[1];
  assert.equal(b.topic, "/b");
  assert.equal(b.msgs, 0);
  assert.equal(b.hz, 0);
  assert.ok(Number.isNaN(b.latP50));
  assert.ok(Number.isNaN(b.lossPct));
  assert.ok(Number.isNaN(b.offeredHz));
  assert.ok(Number.isFinite(r.lanes[0].offeredHz)); // the live lane still reports
  // Pooled offered refuses to sum a partial picture — the dead lane IS the finding.
  assert.ok(Number.isNaN(r.offeredHz));
  assert.ok(Number.isNaN(r.deliveryPct));
});

Deno.test("measureScenario: offered/delivered ratios are exact (elapsed cancels)", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 25, false, undefined, NO_PHASES);
  // Span 0..10 (expected 11), seq 8 missing → 10 delivered.
  for (const seq of [0, 1, 2, 3, 4, 5, 6, 7, 9, 10]) deliver("/a", { latencyMs: 1, seq });
  const r = await p;
  assert.equal(r.deliveryPct, 90.91); // r2(10/11 × 100)
  assert.ok(r.offeredHz > r.hz);
  assert.equal(r.lanes[0].deliveryPct, 90.91);
  // offeredKbps ≈ offeredHz × meanSize: 100 B payloads → ratio matches hz ratio.
  assert.ok(Math.abs(r.offeredKbps / r.kbps - 11 / 10) < 0.01);
});

Deno.test("measureScenario: gapless run → offered ≡ delivered", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 25, false, undefined, NO_PHASES);
  for (const seq of [0, 1, 2, 3, 4]) deliver("/a", { latencyMs: 1, seq });
  const r = await p;
  assert.equal(r.deliveryPct, 100);
  assert.equal(r.offeredHz, r.hz);
  assert.equal(r.offeredKbps, r.kbps);
});

Deno.test("measureScenario: offered NaN on single message / mixed seq-less lane", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(
    client,
    { name: "t", topics: ["/one", "/noseq"] },
    20,
    false,
    undefined,
    NO_PHASES,
  );
  deliver("/one", { latencyMs: 1, seq: 0 }); // expected 1 < 2 → meaningless rate
  deliver("/noseq", { latencyMs: 1 });
  const r = await p;
  assert.ok(Number.isNaN(r.lanes[0].offeredHz));
  assert.ok(Number.isNaN(r.lanes[1].offeredHz));
  assert.ok(Number.isNaN(r.offeredHz));
});

Deno.test("measureScenario: seq reset banks the segment instead of fabricating loss", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 25, false, undefined, NO_PHASES);
  // Source restarts mid-run: 5000..5002, then 0..1. Without detection the span would be
  // 0..5002 → ~99.9% loss; with banking, both segments are complete → 0%.
  for (const seq of [5000, 5001, 5002, 0, 1]) deliver("/a", { latencyMs: 1, seq });
  const r = await p;
  assert.equal(r.lanes[0].seqResets, 1);
  assert.equal(r.seqResets, 1);
  assert.equal(r.lossPct, 0);
  assert.equal(r.deliveryPct, 100);
});

Deno.test("measureScenario: buckets — conservation, gap-free tMs, partial tail", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 120, false, undefined, {
    ...NO_PHASES,
    bucketMs: 25,
  });
  // Synchronous adjacency to the measure start → guaranteed bucket 0.
  for (let i = 0; i < 5; i++) deliver("/a", { latencyMs: 2 + i, seq: i, sizeBytes: 200 });
  const r = await p;
  assert.ok(r.buckets.length >= 5, `expected ≥5 buckets, got ${r.buckets.length}`);
  assert.equal(r.buckets[0].msgs, 5);
  assert.equal(r.buckets[0].latP50, 4); // floor(0.5*5)=2 → sorted [2,3,4,5,6]
  assert.equal(
    r.buckets.reduce((a, b) => a + b.msgs, 0),
    r.msgs,
  );
  r.buckets.forEach((b, i) => assert.equal(b.tMs, i * 25)); // gap-free, zeros included
  assert.ok(r.buckets[1].msgs === 0 && Number.isNaN(r.buckets[1].latP50));
  assert.ok(r.buckets[r.buckets.length - 1].durMs <= 25.01);
  const sumDur = r.buckets.reduce((a, b) => a + b.durMs, 0);
  assert.ok(Math.abs(sumDur - r.elapsedMs) < 1, `Σ durMs ${sumDur} vs elapsed ${r.elapsedMs}`);
});

Deno.test("measureScenario: bucket lane sub-stats", async () => {
  const { client, deliver } = fakeClient();
  const p = measureScenario(
    client,
    { name: "t", topics: ["/a", "/b"] },
    60,
    false,
    undefined,
    { ...NO_PHASES, bucketMs: 25 },
  );
  deliver("/a", { latencyMs: 1, seq: 0, sizeBytes: 100 });
  deliver("/a", { latencyMs: 2, seq: 1, sizeBytes: 100 });
  deliver("/b", { latencyMs: 50, seq: 0, sizeBytes: 1000 });
  const r = await p;
  const b0 = r.buckets[0];
  assert.ok(b0.lanes["/a"] && b0.lanes["/b"]);
  assert.ok(b0.lanes["/a"].kbps < b0.lanes["/b"].kbps);
  assert.equal(b0.lanes["/a"].latP95, 2);
  assert.deepEqual(r.buckets[1].lanes, {}); // empty bucket → no lane keys
});

Deno.test("measureScenario: onBucket live stream ≡ row.buckets", async () => {
  const { client, deliver } = fakeClient();
  const live: BenchBucket[] = [];
  const p = measureScenario(client, { name: "t", topics: ["/a"] }, 80, false, undefined, {
    ...NO_PHASES,
    bucketMs: 20,
    onBucket: (b) => live.push(b),
  });
  for (let i = 0; i < 3; i++) deliver("/a", { latencyMs: 1, seq: i });
  const r = await p;
  assert.deepEqual(live, r.buckets); // end-flush guarantees completeness, order included
});

Deno.test("onDemandSaving: on-demand vs all-lanes kB/s", () => {
  const rows = [
    row({ scenario: "all-lanes", topics: 4, kbps: 40 }),
    row({ scenario: "on-demand", topics: 1, kbps: 10 }),
  ];
  assert.equal(onDemandSaving(rows), 75);
});

Deno.test("STREAM_PROFILES: on-demand pair present; heavy profiles carry their gen config", () => {
  const ids = STREAM_PROFILES.map((p) => p.id);
  assert.ok(ids.includes(ON_DEMAND_PAIR.all), "all-lanes profile missing");
  assert.ok(ids.includes(ON_DEMAND_PAIR.one), "on-demand profile missing");
  for (const id of ["lidar", "camera", "dense", "depth-hd", "raw-1080p", "firehose", "mixed"]) {
    const p = STREAM_PROFILES.find((x) => x.id === id);
    assert.ok(p?.gen, `${id} missing gen`);
    assert.ok(p!.gen!.hz > 0 && p!.gen!.bytes > 0);
  }
  const dense = STREAM_PROFILES.find((p) => p.id === "dense")!;
  assert.deepEqual({ hz: dense.gen!.hz, bytes: dense.gen!.bytes }, { hz: 20, bytes: 1_000_000 });
  assert.ok(dense.hint.includes("20 MB/s")); // hint derives from gen
});
