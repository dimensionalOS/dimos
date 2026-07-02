// Unit tests for the run-record domain — serializer round-trip (NaN ⇄ null), validation,
// the persistence trim policy, and the Markdown export shape.
import assert from "node:assert/strict";

import type { BenchRow } from "./bench.ts";
import {
  formatMarkdown,
  parseRun,
  reviveRun,
  RUN_SCHEMA,
  RUN_VERSION,
  type RunCell,
  type RunRecord,
  serializeRun,
  stripBuckets,
  trimRuns,
} from "./benchRun.ts";

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
  elapsedMs: 1000,
  offeredHz: NaN,
  offeredKbps: NaN,
  deliveryPct: NaN,
  seqResets: 0,
  lanes: [],
  buckets: [],
  ...over,
});

const cell = (over: Partial<RunCell>): RunCell => ({
  scenario: over.row?.scenario ?? "pose",
  netem: "live",
  maxHz: 0,
  transport: "WebSocket",
  rep: 1,
  row: row({}),
  ...over,
});

const run = (cells: RunCell[], over?: Partial<RunRecord>): RunRecord => ({
  schema: RUN_SCHEMA,
  version: RUN_VERSION,
  id: "r1",
  stamp: "2026-07-02T12:00",
  meta: {
    transportLabel: "Auto (WT→WS)",
    gatewayUrl: "host:8080",
    durMs: 4000,
    warmupMs: 500,
    graceMs: 750,
  },
  cells,
  ...over,
});

Deno.test("serialize/parse round-trip preserves NaN via null (never the string NaN)", () => {
  const r = run([
    cell({
      row: row({
        scenario: "pose",
        lossPct: NaN,
        offeredHz: NaN,
        lanes: [{
          topic: "/a",
          msgs: 0,
          hz: 0,
          kbps: 0,
          latP50: NaN,
          latP95: NaN,
          latP99: NaN,
          latMax: NaN,
          latStd: NaN,
          lossPct: NaN,
          latePct: NaN,
          offeredHz: NaN,
          offeredKbps: NaN,
          deliveryPct: NaN,
          seqResets: 0,
        }],
        buckets: [{
          tMs: 0,
          durMs: 1000,
          msgs: 0,
          hz: 0,
          kbps: 0,
          latP50: NaN,
          latP95: NaN,
          lanes: { "/a": { hz: 0, kbps: 0, latP95: NaN } },
        }],
      }),
    }),
  ], {
    meta: {
      transportLabel: "ws",
      gatewayUrl: "g",
      durMs: 1,
      warmupMs: 0,
      graceMs: 0,
      clock: { offsetMs: 1.2, rttMs: 0.6 },
    },
  });
  const json = serializeRun(r);
  assert.ok(!json.includes("NaN"), "NaN must serialize as null");
  assert.ok(json.includes("null"));
  const back = parseRun(json);
  assert.ok(back);
  assert.deepEqual(back, r); // deepStrictEqual: NaN ≡ NaN (SameValue)
});

Deno.test("reviveRun rejects wrong schema / future version / missing cells", () => {
  assert.equal(parseRun("not json"), undefined);
  assert.equal(reviveRun({ schema: "other", version: 1, cells: [] }), undefined);
  assert.equal(reviveRun({ schema: RUN_SCHEMA, version: RUN_VERSION + 1, cells: [] }), undefined);
  assert.equal(reviveRun({ schema: RUN_SCHEMA, version: 1 }), undefined);
});

Deno.test("reviveRun tolerates trimmed rows (lanes/buckets default []) and drops row-less cells", () => {
  const raw = {
    schema: RUN_SCHEMA,
    version: 1,
    id: "x",
    stamp: "s",
    meta: {},
    cells: [
      {
        scenario: "pose",
        netem: "live",
        maxHz: 0,
        transport: "ws",
        rep: 1,
        row: { scenario: "pose", lossPct: null },
      },
      { scenario: "broken", netem: "live", maxHz: 0, transport: "ws", rep: 1 }, // no row
    ],
  };
  const r = reviveRun(JSON.parse(JSON.stringify(raw)))!;
  assert.equal(r.cells.length, 1);
  assert.deepEqual(r.cells[0].row.lanes, []);
  assert.deepEqual(r.cells[0].row.buckets, []);
  assert.ok(Number.isNaN(r.cells[0].row.lossPct)); // null → NaN on the known field
});

Deno.test("trimRuns: maxRuns keeps newest + pinned baseline", () => {
  const runs = [1, 2, 3, 4, 5].map((i) => run([cell({})], { id: `r${i}` })); // r1 newest
  const out = trimRuns(runs, { maxRuns: 2, keepId: "r5" });
  const ids = out.map((r) => r.id);
  assert.ok(ids.includes("r1"), "newest must survive");
  assert.ok(ids.includes("r5"), "baseline must survive");
  assert.ok(out.length <= 3);
});

Deno.test("trimRuns: byte cap strips buckets from the oldest first, keeps the newest intact", () => {
  const fat = () =>
    row({
      buckets: Array.from({ length: 60 }, (_, i) => ({
        tMs: i * 1000,
        durMs: 1000,
        msgs: 10,
        hz: 10,
        kbps: 1,
        latP50: 1,
        latP95: 2,
        lanes: {},
      })),
    });
  const runs = [
    run([cell({ row: fat() })], { id: "new" }),
    run([cell({ row: fat() })], { id: "old" }),
  ];
  const budget = serializeRun(runs[0]).length + serializeRun(stripBuckets(runs[1])).length + 10;
  const out = trimRuns(runs, { maxBytes: budget });
  assert.equal(out.length, 2);
  assert.ok(out[0].cells[0].row.buckets.length > 0, "newest keeps its time-series");
  assert.equal(out[1].cells[0].row.buckets.length, 0, "oldest stripped first");
});

Deno.test("trimRuns never returns empty", () => {
  const only = run([cell({})], { id: "solo" });
  const out = trimRuns([only], { maxBytes: 10 });
  assert.equal(out.length, 1);
});

Deno.test("formatMarkdown: groups by condition, offered/deliv columns, lane sub-rows, NaN as –", () => {
  const lanes = [
    {
      topic: "/load/fast",
      msgs: 400,
      hz: 100,
      kbps: 8,
      latP50: 1,
      latP95: 3,
      latP99: 5,
      latMax: 9,
      latStd: 1,
      lossPct: 0,
      latePct: 0,
      offeredHz: 100,
      offeredKbps: 8,
      deliveryPct: 100,
      seqResets: 0,
    },
    {
      topic: "/load/img",
      msgs: 10,
      hz: 2.5,
      kbps: 2500,
      latP50: 300,
      latP95: 800,
      latP99: 900,
      latMax: 950,
      latStd: 200,
      lossPct: 87.5,
      latePct: 0,
      offeredHz: 20,
      offeredKbps: 20000,
      deliveryPct: 12.5,
      seqResets: 0,
    },
  ];
  const r = run([
    cell({
      scenario: "mixed",
      netem: "loss-5",
      maxHz: 0,
      row: row({ scenario: "mixed", topics: 2, lanes }),
    }),
    cell({
      scenario: "pose",
      netem: "loss-5",
      maxHz: 60,
      row: row({ scenario: "pose", lossPct: NaN }),
    }),
    cell({
      scenario: "dense",
      netem: "loss-5",
      maxHz: 0,
      error: "transport closed",
      row: row({ scenario: "dense" }),
    }),
  ], {
    meta: {
      transportLabel: "Auto",
      gatewayUrl: "g:8080",
      durMs: 4000,
      warmupMs: 500,
      graceMs: 750,
      reproUrl: "http://x/?run=1",
    },
  });
  const md = formatMarkdown(r);
  assert.ok(md.includes("net:loss-5 · maxHz=∞"));
  assert.ok(md.includes("net:loss-5 · maxHz=60Hz")); // second group
  assert.ok(md.includes("offered kB/s | deliv%"));
  assert.ok(md.includes("↳ `/load/fast`"));
  assert.ok(md.includes("↳ `/load/img`"));
  assert.ok(md.includes(" – |")); // the NaN lossPct cell
  assert.ok(md.includes("✗ dense"));
  assert.ok(md.includes("repro: http://x/?run=1"));
});

Deno.test("formatMarkdown: single-lane rows skip sub-rows; on-demand footer only with the pair", () => {
  const pair = run([
    cell({ scenario: "all-lanes", row: row({ scenario: "all-lanes", topics: 4, kbps: 40 }) }),
    cell({ scenario: "on-demand", row: row({ scenario: "on-demand", topics: 1, kbps: 10 }) }),
  ]);
  const md = formatMarkdown(pair);
  assert.ok(md.includes("75% reduction"), md);
  assert.ok(md.includes("1 of 4 topics"), md);
  assert.ok(!md.includes("↳"));
  const solo = formatMarkdown(run([cell({ scenario: "pose", row: row({ scenario: "pose" }) })]));
  assert.ok(!solo.includes("On-demand bandwidth"), solo);
});

Deno.test("formatMarkdown: seq-reset rows get the † footnote", () => {
  const md = formatMarkdown(run([cell({ row: row({ scenario: "pose", seqResets: 1 }) })]));
  assert.ok(md.includes("pose†"));
  assert.ok(md.includes("† source restarted"));
});
