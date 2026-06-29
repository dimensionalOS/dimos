// Transport benchmark core — env-free measurement logic shared by the headless Bun
// runner (bench/bench.ts) and the in-browser bench page (app/src/bench.tsx), so both
// measure identically. Latency (p50/p95/max), throughput (hz), bandwidth (kB/s), and
// on-demand WS-hop savings (subscribe 1 of N vs all N). No Bun/Node/file/DOM APIs here.
import type { DimosClient } from "./client";

export interface BenchScenario {
  name: string;
  topics: string[];
}
export interface BenchRow {
  scenario: string;
  topics: number;
  msgs: number;
  hz: number;
  kbps: number;
  latP50: number;
  latP95: number;
  latMax: number;
}

/** The standard load fed by bench/bench_publisher.py (topic names match every transport). */
export const BENCH_SCENARIOS: BenchScenario[] = [
  { name: "4x PoseStamped (throughput)", topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"] },
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
): Promise<BenchRow> {
  const lat: number[] = [];
  let count = 0;
  let bytes = 0;
  const subs = scenario.topics.map((t) =>
    client.topic<unknown>(t).subscribe((_d, m) => {
      count++;
      bytes += m.sizeBytes;
      const l = endToEnd ? (m.srcTs != null ? m.recvTs - m.srcTs : undefined) : m.latencyMs;
      if (l != null && l >= 0 && l < 10000) lat.push(l);
    }),
  );
  await new Promise((r) => setTimeout(r, durMs));
  subs.forEach((s) => s.unsubscribe());
  lat.sort((a, b) => a - b);
  const q = (p: number) => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : NaN);
  return {
    scenario: scenario.name,
    topics: scenario.topics.length,
    msgs: count,
    hz: r2(count / (durMs / 1000)),
    kbps: r2(bytes / 1024 / (durMs / 1000)),
    latP50: r2(q(0.5)),
    latP95: r2(q(0.95)),
    latMax: r2(lat[lat.length - 1] ?? NaN),
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
    `| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |`,
    `|---|--:|--:|--:|--:|--:|--:|--:|`,
    ...rows.map(
      (r) =>
        `| ${r.scenario} | ${r.topics} | ${r.msgs} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latMax} |`,
    ),
    ``,
    `**On-demand bandwidth:** subscribing 1 of 4 topics delivered **${one?.kbps ?? 0} kB/s** vs **${all4?.kbps ?? 0} kB/s** for all 4 — a **${onDemandSaving(rows)}% reduction** on the WS hop.`,
    ``,
  ].join("\n");
}
