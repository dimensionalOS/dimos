// Pure sweep-domain logic — plan building (cell order), time estimates, baseline matching,
// Δ rendering rules. No React, no IO; the runner and table consume this.
import {
  coexProfile,
  defaultGrace,
  defaultWarmup,
  type FloodOffStat,
  type GenSpec,
  interferenceDelta,
  type RunCell,
  type RunMeta,
  type RunRecord,
  RUN_SCHEMA,
  RUN_VERSION,
  STREAM_PROFILES,
  type StreamProfile,
} from "@dimos/web";
import type { NetemState } from "../../netem";

/** What the runner must ensure the generator is doing before a cell measures. */
export type GenWant =
  | { kind: "set"; spec: GenSpec }
  | { kind: "off" }
  | { kind: "keep" }; // auto-drive off / no RPC — measure whatever flows

export interface PlanCell {
  /** Netem profile to assert for this cell; null = leave the path as-is ("live"). */
  netem: string | null;
  profile: StreamProfile;
  maxHz: number;
  rep: number;
  want: GenWant;
}

export interface SweepPlan {
  cells: PlanCell[];
  /** Whether the sweep drives the generator (autoDrive && RPC advertised). */
  driveGen: boolean;
  /** ?net ids requested but not available on this gateway (absent/disabled/unknown). */
  netIgnored: string[];
  axes: { nets: number; workloads: number; hzs: number; reps: number };
}

/**
 * Cell order: netem → workload → maxHz → repeat (innermost). Netem is the only global
 * server-state transition (POST + tc + settle) so it moves at most #netems times; each
 * workload group shares one generator config; repeats run back-to-back to measure variance
 * under fixed conditions. Netems keep the server's profile order (clean → disaster: if the
 * tab dies under load it dies late, with the cheap cells banked); workloads keep
 * STREAM_PROFILES order (small lanes first, floods ascending — the documented ladder).
 */
export function buildPlan(
  cfg: {
    profiles: string[];
    maxHz: number[];
    net: string[];
    reps: number;
    autoDrive: boolean;
    coex: boolean;
  },
  netem: NetemState | null,
  hasRpc: boolean,
): SweepPlan {
  const serverIds = netem?.enabled ? netem.profiles.map((p) => p.id) : [];
  const nets = serverIds.filter((id) => cfg.net.includes(id));
  const netIgnored = cfg.net.filter((id) => !serverIds.includes(id));
  const netAxis: (string | null)[] = nets.length ? nets : [null];
  // coex derives `<tier>+pose` scenarios (pose lanes beside each flood) — cell count, gen
  // handling, and the scenario-keyed machinery downstream all inherit the derived profile.
  const profiles = STREAM_PROFILES.filter((p) => cfg.profiles.includes(p.id))
    .map((p) => (cfg.coex ? coexProfile(p) : p));
  const hzs = [...cfg.maxHz].sort((a, b) => a - b);
  const reps = Math.max(1, cfg.reps);
  const driveGen = cfg.autoDrive && hasRpc;

  const cells: PlanCell[] = [];
  for (const net of netAxis) {
    for (const profile of profiles) {
      const want: GenWant = !driveGen
        ? { kind: "keep" }
        : profile.gen
        ? { kind: "set", spec: profile.gen }
        : { kind: "off" };
      for (const maxHz of hzs) {
        for (let rep = 1; rep <= reps; rep++) {
          cells.push({ netem: net, profile, maxHz, rep, want });
        }
      }
    }
  }
  return {
    cells,
    driveGen,
    netIgnored,
    axes: { nets: netAxis.length, workloads: profiles.length, hzs: hzs.length, reps },
  };
}

const NETEM_SETTLE_MS = 1000;
const GEN_SETTLE_MS = 300;
const CLOCK_MS = 1500;

/** Honest wall-clock estimate: per-cell window (dur+warmup+grace) + the transitions. */
export function estimateMs(plan: SweepPlan, durMs: number): number {
  const perCell = durMs + defaultWarmup(durMs) + defaultGrace(durMs);
  let netMoves = 0;
  let genMoves = 0;
  let curNet: string | null = null;
  let curGen = "keep";
  for (const c of plan.cells) {
    if (c.netem !== curNet) {
      if (c.netem !== null) netMoves++;
      curNet = c.netem;
    }
    const g = c.want.kind === "set" ? `${c.want.spec.hz}×${c.want.spec.bytes}×${c.want.spec.kind}` : c.want.kind;
    if (g !== curGen && c.want.kind !== "keep") {
      genMoves++;
      curGen = g;
    }
  }
  return CLOCK_MS + plan.cells.length * perCell + netMoves * NETEM_SETTLE_MS + genMoves * GEN_SETTLE_MS;
}

export function fmtDuration(ms: number): string {
  const s = Math.round(ms / 1000);
  return s < 60 ? `${s}s` : `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
}

/** Baseline match key — transport/wire deliberately excluded: comparing wires across the
 *  same (scenario, condition) is the point. */
export const cellKey = (scenario: string, netem: string, maxHz: number) =>
  `${scenario}\u0000${netem}\u0000${maxHz}`;

const median = (ns: number[]): number => {
  const f = ns.filter(Number.isFinite).sort((a, b) => a - b);
  return f.length ? f[Math.floor((f.length - 1) / 2)] : NaN;
};

export interface BaselineStat {
  p95: number;
  loss: number;
}

/** key → median over that key's cells (repeats collapse robustly; error cells skipped). */
export function buildBaselineIndex(rec: RunRecord): Map<string, BaselineStat> {
  const byKey = new Map<string, { p95: number[]; loss: number[] }>();
  for (const c of rec.cells) {
    if (c.error) continue;
    const k = cellKey(c.scenario, c.netem, c.maxHz);
    const e = byKey.get(k) ?? byKey.set(k, { p95: [], loss: [] }).get(k)!;
    e.p95.push(c.row.latP95);
    e.loss.push(c.row.lossPct);
  }
  const out = new Map<string, BaselineStat>();
  for (const [k, v] of byKey) out.set(k, { p95: median(v.p95), loss: median(v.loss) });
  return out;
}

export interface DeltaChip {
  text: string;
  /** Increase (worse) — both metrics are lower-is-better. */
  bad: boolean;
  /** Below the noise floor — render plain muted. */
  faint: boolean;
}
export interface CellDeltas {
  p95?: DeltaChip;
  loss?: DeltaChip;
}

/** Δ vs the pinned baseline for one displayed cell; undefined when no matching key. p95 is
 *  relative (Δ%, absolute ms when the base is 0), loss is percentage-POINT (no div-by-zero
 *  at 0% baselines). Noise floors: 5% on p95, 0.5pp on loss. */
export function deltas(c: RunCell, index: Map<string, BaselineStat>): CellDeltas | undefined {
  const base = index.get(cellKey(c.scenario, c.netem, c.maxHz));
  if (!base) return undefined;
  const out: CellDeltas = {};
  // Zero (after rounding) is "no signal", not "small signal" — no chip at all; the faint
  // tier stays for small-but-real deltas.
  if (Number.isFinite(base.p95) && Number.isFinite(c.row.latP95)) {
    if (base.p95 > 0) {
      const pct = ((c.row.latP95 - base.p95) / base.p95) * 100;
      if (Math.abs(pct) >= 0.05) {
        out.p95 = {
          text: `${pct > 0 ? "▲" : "▼"}${Math.abs(pct).toFixed(0)}%`,
          bad: pct > 0,
          faint: Math.abs(pct) < 5,
        };
      }
    } else {
      const abs = c.row.latP95 - base.p95;
      if (Math.abs(abs) >= 0.05) {
        out.p95 = {
          text: `${abs > 0 ? "▲" : "▼"}${Math.abs(abs).toFixed(1)}ms`,
          bad: abs > 0,
          faint: Math.abs(abs) < 1,
        };
      }
    }
  }
  if (Number.isFinite(base.loss) && Number.isFinite(c.row.lossPct)) {
    const pp = c.row.lossPct - base.loss;
    if (Math.abs(pp) >= 0.05) {
      out.loss = {
        text: `${pp > 0 ? "▲" : "▼"}${Math.abs(pp).toFixed(1)}pp`,
        bad: pp > 0,
        faint: Math.abs(pp) < 0.5,
      };
    }
  }
  return out.p95 || out.loss ? out : undefined;
}

/** ×N chip for the fast-p95 column: this coex cell's `/load/fast` p95 vs its flood-off twin
 *  (same transport·net·maxHz, from `buildFloodOffIndex`). Noise floor mirrors `deltas`:
 *  under ×1.2 renders faint. On /ws the ratio is inter-stream HOL blocking; on WT/WebRTC
 *  (independent lanes) it measures congestion-control coupling + drop policy instead. */
export function interferenceChip(
  c: RunCell,
  index: Map<string, FloodOffStat>,
): { chip: DeltaChip; title: string } | undefined {
  const d = interferenceDelta(c, index);
  if (!d) return undefined;
  const dlv = Number.isFinite(d.delivDropPp)
    ? ` · deliv ${d.baseDeliv} → ${d.underDeliv}%`
    : "";
  return {
    chip: { text: `×${d.ratio.toFixed(1)}`, bad: d.ratio > 1, faint: d.ratio < 1.2 },
    title:
      `/load/fast p95 ${d.baseP95}ms alone → ${d.underP95}ms beside the flood${dlv} — HOL on a single TCP pipe; CC coupling + drop policy on WT/RTC`,
  };
}

export const newRunId = () => crypto.randomUUID().slice(0, 8);

/** Wrap the displayed cells as a RunRecord (for copy Markdown / copy JSON of the current
 *  table, which may span several sweeps and transports). */
export function synthRecord(cells: RunCell[], meta: RunMeta, over?: Partial<RunRecord>): RunRecord {
  return {
    schema: RUN_SCHEMA,
    version: RUN_VERSION,
    id: over?.id ?? newRunId(),
    stamp: over?.stamp ?? new Date().toISOString().slice(0, 16).replace("T", " "),
    ...over,
    meta,
    cells,
  };
}
