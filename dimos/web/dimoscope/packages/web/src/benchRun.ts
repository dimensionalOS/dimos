// Run-record domain: the versioned, self-describing artifact one sweep produces — per-cell
// condition stamps (netem × maxHz × workload × rep), JSON serialization (NaN ⇄ null), the
// persistence trim policy, and the Markdown export. Measurement itself lives in bench.ts.
import type { ClockSample } from "./types.ts";
import {
  type BenchRow,
  BULK_LANES,
  FAST_LANE,
  type LaneStats,
  ON_DEMAND_PAIR,
  onDemandSaving,
} from "./bench.ts";

export const RUN_SCHEMA = "dimoscope.bench.run" as const;
export const RUN_VERSION = 1;

/** Load-source config when the sweep drove `start_bench` itself. */
export interface GeneratorConfig {
  kind: string;
  hz: number;
  bytes: number;
}

export interface RunMeta {
  /** UI transport selection ("Auto (WT→WS)", "WebSocket", …). */
  transportLabel: string;
  gatewayUrl: string;
  origin?: string;
  userAgent?: string;
  durMs: number;
  warmupMs: number;
  graceMs: number;
  /** The probe applied as opts.offsetMs — absent means latency was raw. */
  clock?: ClockSample;
  /** Paste-to-reproduce URL (config + run=1), stamped by the app. */
  reproUrl?: string;
}

/** One sweep cell: a single measureScenario run under one asserted condition. */
export interface RunCell {
  scenario: string;
  /** netem profile id, or "live" when /netem is absent/untouched. */
  netem: string;
  /** 0 = ∞ (no client rate request). */
  maxHz: number;
  /** Selected server label — cells append across transport switches. */
  transport: string;
  /** The wire that actually carried it; "WT-rs→WS" when it changed mid-cell. */
  wire?: string;
  /** 1-based repeat index. */
  rep: number;
  /** What drove the generator for this cell; null = not driven (measured whatever flowed). */
  gen?: GeneratorConfig | null;
  /** The cell failed (transport died mid-measure) — kept as data, row may be empty. */
  error?: string;
  row: BenchRow;
}

/** Everything needed to render, compare, and reproduce one sweep. */
export interface RunRecord {
  schema: typeof RUN_SCHEMA;
  version: number;
  id: string;
  /** ISO 8601 sweep start. */
  stamp: string;
  name?: string;
  aborted?: boolean;
  meta: RunMeta;
  cells: RunCell[];
}

// NaN survives to JSON as null (JSON.stringify does this natively); reviveRun restores it via
// these explicit field lists — never a deep null-walk, so a legitimately-null field can't be
// corrupted. Keep in sync with LaneStats/BenchRow/BenchBucket.
const ROW_NUMS = [
  "hz",
  "kbps",
  "latP50",
  "latP95",
  "latP99",
  "latMax",
  "latStd",
  "lossPct",
  "latePct",
  "offeredHz",
  "offeredKbps",
  "deliveryPct",
] as const;
const BUCKET_NUMS = ["latP50", "latP95"] as const;

// deno-lint-ignore no-explicit-any
const restoreNums = (obj: any, keys: readonly string[]) => {
  for (const k of keys) if (obj[k] === null) obj[k] = NaN;
};

export function serializeRun(run: RunRecord): string {
  return JSON.stringify(run); // NaN → null
}

/** Validate + restore a parsed (JSON) value into a RunRecord; undefined on wrong schema,
 *  a future version, or a missing cells array. Takes ownership of the value (mutates).
 *  Trimmed/older rows revive with `lanes`/`buckets` defaulted to []. */
export function reviveRun(v: unknown): RunRecord | undefined {
  // deno-lint-ignore no-explicit-any
  const run = v as any;
  if (!run || run.schema !== RUN_SCHEMA) return undefined;
  if (typeof run.version !== "number" || run.version > RUN_VERSION) return undefined;
  if (!Array.isArray(run.cells)) return undefined;
  run.meta = run.meta ?? {};
  run.cells = run.cells.filter((c: unknown) => {
    // deno-lint-ignore no-explicit-any
    const cell = c as any;
    if (!cell || typeof cell.row !== "object" || cell.row === null) return false;
    const row = cell.row;
    restoreNums(row, ROW_NUMS);
    row.lanes = Array.isArray(row.lanes) ? row.lanes : [];
    for (const l of row.lanes) restoreNums(l, ROW_NUMS);
    row.buckets = Array.isArray(row.buckets) ? row.buckets : [];
    for (const b of row.buckets) {
      restoreNums(b, BUCKET_NUMS);
      b.lanes = b.lanes ?? {};
      for (const bl of Object.values(b.lanes)) restoreNums(bl, ["latP95"]);
    }
    return true;
  });
  return run as RunRecord;
}

export function parseRun(json: string): RunRecord | undefined {
  try {
    return reviveRun(JSON.parse(json));
  } catch {
    return undefined;
  }
}

/** Drop the time-series (the heavy part) — how runs are persisted; lanes are the per-lane
 *  story and stay. */
export function stripBuckets(run: RunRecord): RunRecord {
  return {
    ...run,
    cells: run.cells.map((c) => ({ ...c, row: { ...c.row, buckets: [] } })),
  };
}

/** Persistence trim policy (pure — the app owns the actual localStorage calls). `runs` is
 *  newest-first; the newest run and `keepId` (the pinned baseline) are never dropped. Over the
 *  byte cap, oldest runs lose `buckets` first, then whole oldest runs go. The re-serialize
 *  sizing is O(n²) at ~20 × 100 KB — fine for a persist-on-sweep-end call. */
export function trimRuns(
  runs: RunRecord[],
  opts?: { maxRuns?: number; maxBytes?: number; keepId?: string },
): RunRecord[] {
  const maxRuns = opts?.maxRuns ?? 20;
  const maxBytes = opts?.maxBytes ?? 1_500_000;
  const keepId = opts?.keepId;
  const protectedRun = (r: RunRecord, i: number) => i === 0 || r.id === keepId;

  let out = runs.slice();
  if (out.length > maxRuns) {
    const kept = out.filter(protectedRun);
    const rest = out.filter((r, i) => !protectedRun(r, i));
    out = [...kept, ...rest.slice(0, Math.max(0, maxRuns - kept.length))]
      .sort((a, b) => runs.indexOf(a) - runs.indexOf(b));
  }
  const size = () => out.reduce((a, r) => a + serializeRun(r).length, 0);
  // Oldest-first bucket strip.
  for (let i = out.length - 1; i >= 0 && size() > maxBytes; i--) {
    out[i] = stripBuckets(out[i]);
  }
  // Then drop whole oldest non-protected runs.
  while (out.length > 1 && size() > maxBytes) {
    let dropped = false;
    for (let i = out.length - 1; i > 0; i--) {
      if (out[i].id !== keepId) {
        out.splice(i, 1);
        dropped = true;
        break;
      }
    }
    if (!dropped) break; // only protected runs left — never return empty
  }
  return out;
}

const cell = (n: number) => (Number.isFinite(n) ? String(n) : "–");
const hzLabel = (maxHz: number) => (maxHz ? `${maxHz}Hz` : "∞");

/** The `/load/fast` slice of a row — the interference signal lane. */
export const fastLane = (row: BenchRow): LaneStats | undefined =>
  row.lanes.find((l) => l.topic === FAST_LANE);

/** A coexistence row measures the fast lane BESIDE a flood lane. Lane presence, not profile
 *  id — so `mixed`, `<tier>+pose`, manual-flood cells, and old saved runs all qualify. */
export const isCoexRow = (row: BenchRow): boolean =>
  !!fastLane(row) &&
  row.lanes.some((l) => (BULK_LANES as readonly string[]).includes(l.topic));

/** Interference match key — transport IS in the key (opposite of the baseline `cellKey`):
 *  the comparison is flood-on vs flood-off on ONE wire, not across wires. */
export const floodOffKey = (c: Pick<RunCell, "transport" | "netem" | "maxHz">): string =>
  [c.transport, c.netem, c.maxHz].join("\u0000");

/** The fast lane's flood-off reference under one condition. */
export interface FloodOffStat {
  p95: number;
  deliveryPct: number;
}

const median = (ns: number[]): number => {
  const f = ns.filter(Number.isFinite).sort((a, b) => a - b);
  return f.length ? f[Math.floor((f.length - 1) / 2)] : NaN;
};

/** key → the fast lane's median p95/deliv% over the flood-OFF cells (fast lane present, no
 *  bulk lane, no error) — the "alone" side `interferenceDelta` compares against. */
export function buildFloodOffIndex(cells: RunCell[]): Map<string, FloodOffStat> {
  const byKey = new Map<string, { p95: number[]; deliv: number[] }>();
  for (const c of cells) {
    if (c.error || isCoexRow(c.row)) continue;
    const fl = fastLane(c.row);
    if (!fl) continue;
    const k = floodOffKey(c);
    const e = byKey.get(k) ?? byKey.set(k, { p95: [], deliv: [] }).get(k)!;
    e.p95.push(fl.latP95);
    e.deliv.push(fl.deliveryPct);
  }
  const out = new Map<string, FloodOffStat>();
  for (const [k, v] of byKey) out.set(k, { p95: median(v.p95), deliveryPct: median(v.deliv) });
  return out;
}

/** How much the flood hurts the fast lane in this cell vs its flood-off reference. On /ws
 *  this is inter-stream HOL blocking; on WT/WebRTC (independent lanes) what remains is
 *  congestion-control coupling + drop policy — same number, different mechanism. */
export interface InterferenceDelta {
  /** underP95 / baseP95 — ×1 = the flood didn't touch the fast lane. */
  ratio: number;
  baseP95: number;
  underP95: number;
  baseDeliv: number;
  underDeliv: number;
  /** deliv% drop in percentage points (NaN when either side is NaN). */
  delivDropPp: number;
}

export function interferenceDelta(
  c: RunCell,
  index: Map<string, FloodOffStat>,
): InterferenceDelta | undefined {
  if (c.error || !isCoexRow(c.row)) return undefined;
  const fl = fastLane(c.row);
  const base = index.get(floodOffKey(c));
  if (!fl || !base) return undefined;
  if (!Number.isFinite(base.p95) || base.p95 <= 0 || !Number.isFinite(fl.latP95)) return undefined;
  return {
    ratio: fl.latP95 / base.p95,
    baseP95: base.p95,
    underP95: fl.latP95,
    baseDeliv: base.deliveryPct,
    underDeliv: fl.deliveryPct,
    delivDropPp: base.deliveryPct - fl.deliveryPct,
  };
}

/** Render a whole run as Markdown: header + context, one table per condition group
 *  (transport·wire·netem·maxHz), per-lane sub-rows, offered/deliv% columns, the on-demand
 *  footer per group, and the repro URL. */
export function formatMarkdown(run: RunRecord): string {
  const m = run.meta;
  const clockLine = m.clock
    ? `clock offset ${m.clock.offsetMs.toFixed(1)}ms (rtt ${m.clock.rttMs.toFixed(1)}ms) corrected`
    : "no clock sync (latency raw)";
  const lines: string[] = [
    `# Transport benchmark — ${m.transportLabel}${run.name ? ` · ${run.name}` : ""}${
      run.aborted ? " · (aborted)" : ""
    }`,
    ``,
    `_${m.durMs}ms/scenario (warmup ${Math.round(m.warmupMs)} · grace ${
      Math.round(m.graceMs)
    }) · ${m.gatewayUrl} · ${run.stamp} · ${clockLine}_`,
  ];
  if (m.reproUrl) lines.push(`_repro: ${m.reproUrl}_`);
  if (m.origin || m.userAgent) lines.push(`_${m.origin ?? "?"} · ${m.userAgent ?? "?"}_`);
  lines.push(``);

  // Group by condition — one table per (transport, wire, netem, maxHz).
  const groups = new Map<string, RunCell[]>();
  for (const c of run.cells) {
    const k = [c.transport, c.wire ?? "", c.netem, c.maxHz].join("\u0000");
    (groups.get(k) ?? groups.set(k, []).get(k)!).push(c);
  }
  let anyReset = false;
  const ifIndex = buildFloodOffIndex(run.cells);
  for (const cells of groups.values()) {
    const first = cells[0];
    const wireTag = first.wire && first.wire !== first.transport ? ` (wire: ${first.wire})` : "";
    lines.push(
      `## ${first.transport}${wireTag} · net:${first.netem} · maxHz=${hzLabel(first.maxHz)}`,
      ``,
    );
    const reps = cells.some((c) => c.rep > 1);
    const head = ["scenario"];
    if (reps) head.push("rep");
    head.push(
      "topics",
      "msgs",
      "hz",
      "kB/s",
      "offered kB/s",
      "deliv%",
      "p50",
      "p95",
      "p99",
      "fast p95",
      "max",
      "std",
      "loss%",
      "late%",
    );
    lines.push(
      `| ${head.join(" | ")} |`,
      `|---${"|--:".repeat(head.length - 1)}|`,
    );
    const ok = cells.filter((c) => !c.error);
    for (const c of ok) {
      const r = c.row;
      const mark = r.seqResets > 0 ? "†" : "";
      if (r.seqResets > 0) anyReset = true;
      const cols = [`${c.scenario}${mark}`];
      if (reps) cols.push(String(c.rep));
      cols.push(
        String(r.topics),
        String(r.msgs),
        cell(r.hz),
        cell(r.kbps),
        cell(r.offeredKbps),
        cell(r.deliveryPct),
        cell(r.latP50),
        cell(r.latP95),
        cell(r.latP99),
        cell(fastLane(r)?.latP95 ?? NaN),
        cell(r.latMax),
        cell(r.latStd),
        cell(r.lossPct),
        cell(r.latePct),
      );
      lines.push(`| ${cols.join(" | ")} |`);
      if (r.lanes.length > 1) {
        for (const l of r.lanes) {
          const lc = [`↳ \`${l.topic}\``];
          if (reps) lc.push("");
          lc.push(
            "",
            String(l.msgs),
            cell(l.hz),
            cell(l.kbps),
            cell(l.offeredKbps),
            cell(l.deliveryPct),
            cell(l.latP50),
            cell(l.latP95),
            cell(l.latP99),
            "",
            cell(l.latMax),
            cell(l.latStd),
            cell(l.lossPct),
            cell(l.latePct),
          );
          lines.push(`| ${lc.join(" | ")} |`);
        }
      }
    }
    lines.push(``);
    for (const c of cells.filter((c) => c.error)) {
      lines.push(`- ✗ ${c.scenario} (rep ${c.rep}): ${c.error}`);
    }
    // Only meaningful when this group ran the on-demand pair (all-lanes + on-demand).
    const rows = ok.map((c) => c.row);
    const all = rows.find((r) => r.scenario === ON_DEMAND_PAIR.all);
    const one = rows.find((r) => r.scenario === ON_DEMAND_PAIR.one);
    if (one && all) {
      lines.push(
        `**On-demand bandwidth:** subscribing 1 of ${all.topics} topics delivered **${one.kbps} kB/s** vs **${all.kbps} kB/s** for all ${all.topics} — a **${
          onDemandSaving(rows)
        }% reduction** on the WS hop.`,
        ``,
      );
    }
    // Interference footer — each coex scenario's fast lane vs its flood-off twin (one line
    // per scenario, first rep; per-rep values are in the rows above).
    const coexSeen = new Set<string>();
    let coexNoRef = false;
    for (const c of ok) {
      if (!isCoexRow(c.row) || coexSeen.has(c.scenario)) continue;
      coexSeen.add(c.scenario);
      const d = interferenceDelta(c, ifIndex);
      if (!d) {
        coexNoRef = true;
        continue;
      }
      const dlv = Number.isFinite(d.delivDropPp)
        ? `, deliv ${cell(d.baseDeliv)} → ${cell(d.underDeliv)}%`
        : "";
      lines.push(
        `**Interference (${c.scenario}):** \`/load/fast\` p95 ${cell(d.baseP95)} → ${
          cell(d.underP95)
        } ms (**×${
          d.ratio.toFixed(1)
        }**)${dlv} beside the flood — vs the flood-off cell at the same net·maxHz.`,
        ``,
      );
    }
    if (coexNoRef) {
      lines.push(`- interference Δ unavailable — add the \`pose\` profile to the sweep`, ``);
    }
  }
  if (anyReset) {
    lines.push(
      `† source restarted mid-run (seq reset) — treat that row's loss/offered with care.`,
      ``,
    );
  }
  return lines.join("\n");
}
