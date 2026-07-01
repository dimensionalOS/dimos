// Transport benchmark core — env-free measurement logic shared by the headless Bun
// runner (bench/bench.ts) and the in-browser bench page (app/src/bench.tsx), so both
// measure identically. Latency (p50/p95/max), throughput (hz), bandwidth (kB/s), and
// on-demand WS-hop savings (subscribe 1 of N vs all N). No Bun/Node/file/DOM APIs here.
import type { DimosClient } from "./client.ts";
import type { Qos } from "./types.ts";
// Not a test: this is the shared measurement *core*, imported by the app runtime (app/src/bench.tsx +
// panels/BenchTab.tsx), the headless CLI runners (bench/*, cli/dtop.ts), and exported from the public
// package API. Its own unit tests live beside it in bench.test.ts.
export interface BenchScenario {
  name: string;
  topics: string[];
}

/**
 * A named workload class — mirrors the headless matrix's STREAM axis
 * (bench/matrix_run.ts + bench/matrix.sh) so in-browser results label-match the
 * RESULTS-mechanisms-<id>.md tables. The heavy classes (lidar/camera/dense) all
 * ride the SAME /bench/img topic; the class is a *publisher* config (hz × bytes),
 * not a distinct topic — so the browser measures whatever the source is emitting
 * and the `hint` is just what bench/matrix.sh would set the generator to.
 */
export interface StreamProfile {
  id: string;
  /** topics to subscribe + measure */
  topics: string[];
  /** human note on the publisher config / expected load (for the UI). */
  hint: string;
}

/** Canonical workload classes — keep `id` + `topics` in sync with bench/matrix_run.ts STREAMS. */
export const STREAM_PROFILES: StreamProfile[] = [
  {
    id: "pose",
    topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"],
    hint: "4×Pose@100Hz — small, high-rate",
  },
  { id: "lidar", topics: ["/bench/img"], hint: "img@10Hz × 200KB ≈ 2 MB/s" },
  { id: "camera", topics: ["/bench/img"], hint: "img@20Hz × 550KB ≈ 11 MB/s" },
  { id: "dense", topics: ["/bench/img"], hint: "img@20Hz × 1MB ≈ 20 MB/s" },
  {
    id: "mixed",
    topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3", "/bench/grid", "/bench/img"],
    hint: "pose@100 + grid@20 + img@10×200KB",
  },
];
export interface BenchRow {
  scenario: string;
  topics: number;
  msgs: number;
  hz: number;
  kbps: number;
  latP50: number;
  latP95: number;
  latP99: number;
  latMax: number;
  /** Latency standard deviation (ms) — the jitter the tail (p95/p99) hints at. */
  latStd: number;
  /** Wire loss % from sequence gaps (received vs the seq span seen). NaN if the
   *  source doesn't stamp a numeric seq. Rate-limited scenarios are excluded. */
  lossPct: number;
}

/** The standard load fed by bench/bench_publisher.py (topic names match every transport). */
export const BENCH_SCENARIOS: BenchScenario[] = [
  {
    name: "4x PoseStamped (throughput)",
    topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"],
  },
  { name: "1x PoseStamped (on-demand)", topics: ["/bench/p0"] },
  { name: "1x OccupancyGrid (large)", topics: ["/bench/grid"] },
];

const r2 = (n: number) => Math.round(n * 100) / 100;

/**
 * Subscribe `scenario.topics` on `client`, collect for `durMs`, return aggregated stats.
 * `endToEnd`: measure latency as source-publish → browser-recv (`recvTs - srcTs`, available
 * on every transport) instead of the SDK default (gateways stamp the WS hop, zenoh-ts the
 * full path) — use it for a fair cross-transport browser compare.
 */
export async function measureScenario(
  client: DimosClient,
  scenario: BenchScenario,
  durMs: number,
  endToEnd = false,
  qos?: Qos,
): Promise<BenchRow> {
  const lat: number[] = [];
  let count = 0;
  let bytes = 0;
  // Per-topic seq span (first/last seen) + received count → wire loss. Each topic
  // carries its own counter, so loss is computed per topic then pooled.
  const seqStat = new Map<string, { min: number; max: number; recv: number }>();
  const subs = scenario.topics.map((t) => {
    const topic = client.topic(t);
    // Apply QoS before subscribing so the gateway downsample (rateLimit:"server") is requested
    // on the first subscribe. Optional-chained so test fakes without setQos don't throw.
    if (qos) topic.setQos?.(qos);
    return topic.subscribe((_d, m) => {
      count++;
      bytes += m.sizeBytes;
      const l = endToEnd ? (m.srcTs != null ? m.recvTs - m.srcTs : undefined) : m.latencyMs;
      if (l != null && l >= 0 && l < 10000) lat.push(l);
      if (m.seq != null) {
        const s = seqStat.get(m.topic);
        if (!s) seqStat.set(m.topic, { min: m.seq, max: m.seq, recv: 1 });
        else {
          if (m.seq < s.min) s.min = m.seq;
          if (m.seq > s.max) s.max = m.seq;
          s.recv++;
        }
      }
    });
  });
  await new Promise((r) => setTimeout(r, durMs));
  subs.forEach((s) => s.unsubscribe());
  lat.sort((a, b) => a - b);
  const q = (
    p: number,
  ) => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : NaN);
  const mean = lat.length ? lat.reduce((a, b) => a + b, 0) / lat.length : NaN;
  const std = lat.length
    ? Math.sqrt(lat.reduce((a, b) => a + (b - mean) * (b - mean), 0) / lat.length)
    : NaN;
  // Pool the per-topic spans: expected = Σ(max-min+1), received = Σrecv.
  let expected = 0;
  let received = 0;
  for (const s of seqStat.values()) {
    expected += s.max - s.min + 1;
    received += s.recv;
  }
  const lossPct = expected > 0 ? r2(Math.max(0, (1 - received / expected) * 100)) : NaN;
  return {
    scenario: scenario.name,
    topics: scenario.topics.length,
    msgs: count,
    hz: r2(count / (durMs / 1000)),
    kbps: r2(bytes / 1024 / (durMs / 1000)),
    latP50: r2(q(0.5)),
    latP95: r2(q(0.95)),
    latP99: r2(q(0.99)),
    latMax: r2(lat[lat.length - 1] ?? NaN),
    latStd: r2(std),
    lossPct,
  };
}

/** On-demand WS-hop saving %: 1-of-4 kB/s vs all-4 kB/s, from the standard rows. */
export function onDemandSaving(rows: BenchRow[]): number {
  const all4 = rows.find((r) => r.topics === 4);
  const one = rows.find((r) => r.scenario.includes("on-demand"));
  if (!all4 || !one || all4.kbps <= 0) return 0;
  return Math.round((1 - one.kbps / all4.kbps) * 100);
}

/** Render one transport's rows as a Markdown section (the format the CLI bench writes). */
export function formatMarkdown(
  label: string,
  url: string,
  durMs: number,
  stamp: string,
  rows: BenchRow[],
): string {
  const all4 = rows.find((r) => r.topics === 4);
  const one = rows.find((r) => r.scenario.includes("on-demand"));
  return [
    `# Transport benchmark — ${label}`,
    ``,
    `_${durMs}ms per scenario · ${url} · ${stamp}_`,
    ``,
    `| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | max | std | loss% |`,
    `|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|`,
    ...rows.map(
      (r) =>
        `| ${r.scenario} | ${r.topics} | ${r.msgs} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latP99} | ${r.latMax} | ${r.latStd} | ${r.lossPct} |`,
    ),
    ``,
    `**On-demand bandwidth:** subscribing 1 of 4 topics delivered **${one?.kbps ?? 0} kB/s** vs **${
      all4?.kbps ?? 0
    } kB/s** for all 4 — a **${onDemandSaving(rows)}% reduction** on the WS hop.`,
    ``,
  ].join("\n");
}
