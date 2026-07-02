// Transport benchmark core: measures latency (p50/p95/max), throughput (hz), bandwidth (kB/s), and
// on-demand WS-hop savings (subscribe 1 of N vs all N). Platform-agnostic — runs identically in the
// browser and in tests. Unit tests in bench.test.ts.
import type { DimosClient } from "./client.ts";
import type { Qos } from "./types.ts";
export interface BenchScenario {
  name: string;
  topics: string[];
}

/**
 * A named workload class. The heavy classes (lidar/camera/dense/…) all ride the SAME
 * /load/img topic; the class is a *publisher* config (hz × bytes), not a distinct
 * topic — so the browser measures whatever the source is emitting and the `hint` is
 * just what the GO2Load flood (start_bench) would be set to.
 */
export interface StreamProfile {
  id: string;
  /** topics to subscribe + measure */
  topics: string[];
  /** human note on the publisher config / expected load (for the UI). */
  hint: string;
}

/** Canonical workload classes — keep `id` + `topics` in sync with the GO2Load `/load/*` ports. */
export const STREAM_PROFILES: StreamProfile[] = [
  {
    id: "pose",
    topics: ["/load/fast", "/load/mid", "/load/slow"],
    hint: "fast@100 + mid@20 + slow@2 Hz — small, high-rate lanes",
  },
  {
    id: "all-lanes",
    topics: ["/load/fast", "/load/mid", "/load/slow", "/load/grid"],
    hint: "fast@100 + mid@20 + slow@2 + grid@5 Hz — every small lane",
  },
  {
    id: "on-demand",
    topics: ["/load/fast"],
    hint: "fast@100 only — run with all-lanes: the export prints the WS-hop cut",
  },
  { id: "lidar", topics: ["/load/img"], hint: "img@10Hz × 200KB ≈ 2 MB/s" },
  { id: "camera", topics: ["/load/img"], hint: "img@20Hz × 550KB ≈ 11 MB/s" },
  { id: "dense", topics: ["/load/img"], hint: "img@20Hz × 1MB ≈ 20 MB/s" },
  // Overload tiers — mirror the BenchDrawer generator ladder (STREAM_TIERS) so a sweep *records*
  // the big-data cases that stress/crash the tab, not just up to ~20 MB/s. Run light→heavy: the top
  // tiers may degrade or kill the tab (expected — that's the case being measured).
  { id: "depth-hd", topics: ["/load/img"], hint: "img@20Hz × 2.5MB ≈ 50 MB/s (RealSense/Ouster)" },
  { id: "raw-1080p", topics: ["/load/img"], hint: "img@30Hz × 6MB ≈ 180 MB/s (raw RGB)" },
  {
    id: "firehose",
    topics: ["/load/img"],
    hint: "img@30Hz × 10MB ≈ 300 MB/s (raw 4K / multi-cam)",
  },
  {
    id: "mixed",
    topics: ["/load/fast", "/load/mid", "/load/slow", "/load/grid", "/load/img"],
    hint: "fast@100 + mid@20 + slow@2 + grid@5 + img flood",
  },
];

/** The measured on-demand pair: run both profiles in one sweep and formatMarkdown
 *  prints the WS-hop saving (subscribe 1 lane vs all of them). Ids must match
 *  STREAM_PROFILES — the sync is unit-tested. */
const ON_DEMAND_PAIR = { all: "all-lanes", one: "on-demand" } as const;

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
  /** Wire loss % from sequence gaps: frames in the measured seq span that never arrived,
   *  even after the grace drain. NaN if the source doesn't stamp a numeric seq, or if the
   *  run is client-rate-limited (`qos.maxHz` > 0 — gaps are then intentional downsampling). */
  lossPct: number;
  /** % of the seq span delivered only during the grace drain (delayed past the window but
   *  not dropped — e.g. a backlog of reliable-stream frames). Same NaN rules as lossPct. */
  latePct: number;
}

export interface BenchOpts {
  /** Discard samples for this long after subscribe, so ramp-up (QoS negotiation, first-frame
   *  setup) doesn't pollute the window. Default min(500, durMs/5). */
  warmupMs?: number;
  /** Keep listening after the window so in-flight frames count as "late", not "lost".
   *  Default min(750, durMs/2). */
  graceMs?: number;
  /** Clock correction for end-to-end latency across machines: (source clock − client clock) in ms,
   *  e.g. from `client.estimateClockOffset()` (the gateway is assumed co-located with the source).
   *  The raw `recvTs − srcTs` measures (latency − offset), so the offset is added back; without
   *  it, cross-machine skew swamps WAN latency. */
  offsetMs?: number;
}

const r2 = (n: number) => Math.round(n * 100) / 100;
const now = () => performance.now();

/**
 * Subscribe `scenario.topics` on `client`, warm up, collect for `durMs`, then drain a grace
 * period to split delayed frames ("late") from dropped ones ("lost").
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
  opts?: BenchOpts,
): Promise<BenchRow> {
  const warmupMs = opts?.warmupMs ?? Math.min(500, durMs / 5);
  const graceMs = opts?.graceMs ?? Math.min(750, durMs / 2);
  const offsetMs = opts?.offsetMs ?? 0;
  const lat: number[] = [];
  let count = 0;
  let bytes = 0;
  // Per-topic seq span + the set of seqs received in-window; grace arrivals inside the span
  // are "late". Per topic, then pooled — each topic carries its own counter.
  const span = new Map<string, { min: number; max: number }>();
  const seqs = new Map<string, Set<number>>();
  const lateSeqs = new Map<string, Set<number>>();
  let phase: "warmup" | "measure" | "grace" = warmupMs > 0 ? "warmup" : "measure";
  const subs = scenario.topics.map((t) => {
    const topic = client.topic(t);
    // Apply QoS before subscribing so the gateway downsample (maxHz) is requested on the
    // first subscribe. Optional-chained so test fakes without setQos don't throw.
    if (qos) topic.setQos?.(qos);
    return topic.subscribe(({ meta: m }) => {
      if (phase === "warmup") return;
      if (phase === "grace") {
        // Only reclassify frames the window already expected (inside the measured span).
        if (m.seq == null) return;
        const sp = span.get(m.topic);
        if (!sp || m.seq < sp.min || m.seq > sp.max || seqs.get(m.topic)?.has(m.seq)) return;
        let late = lateSeqs.get(m.topic);
        if (!late) lateSeqs.set(m.topic, late = new Set());
        late.add(m.seq);
        return;
      }
      count++;
      bytes += m.sizeBytes;
      const l = endToEnd
        ? (m.srcTs != null ? m.recvTs - m.srcTs + offsetMs : undefined)
        : m.latencyMs;
      // Small negatives are residual offset-estimation error (clamp to 0); large ones are
      // uncorrected skew and would poison the stats (drop, as before).
      if (l != null && l >= -50 && l < 10000) lat.push(Math.max(0, l));
      if (m.seq != null) {
        const sp = span.get(m.topic);
        if (!sp) span.set(m.topic, { min: m.seq, max: m.seq });
        else {
          if (m.seq < sp.min) sp.min = m.seq;
          if (m.seq > sp.max) sp.max = m.seq;
        }
        let set = seqs.get(m.topic);
        if (!set) seqs.set(m.topic, set = new Set());
        set.add(m.seq);
      }
    });
  });
  if (warmupMs > 0) await new Promise((r) => setTimeout(r, warmupMs));
  phase = "measure";
  const t0 = now();
  await new Promise((r) => setTimeout(r, durMs));
  // Divide rates by the wall clock actually elapsed — a GC pause or tab throttle stretches
  // the window, and dividing by nominal durMs would over-report.
  const elapsed = Math.max(now() - t0, 1);
  phase = "grace";
  if (graceMs > 0) await new Promise((r) => setTimeout(r, graceMs));
  subs.forEach((s) => s.unsubscribe());
  lat.sort((a, b) => a - b);
  const q = (
    p: number,
  ) => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : NaN);
  const mean = lat.length ? lat.reduce((a, b) => a + b, 0) / lat.length : NaN;
  const std = lat.length
    ? Math.sqrt(lat.reduce((a, b) => a + (b - mean) * (b - mean), 0) / lat.length)
    : NaN;
  // Pool the per-topic spans: expected = Σ(max-min+1); anything not received in-window or
  // during grace is lost. A client-rate-limited run gaps by design → NaN, not loss.
  let expected = 0;
  let received = 0;
  let late = 0;
  for (const [t, sp] of span) {
    expected += sp.max - sp.min + 1;
    received += seqs.get(t)?.size ?? 0;
    late += lateSeqs.get(t)?.size ?? 0;
  }
  const rateLimited = !!qos?.maxHz;
  const lossPct = expected > 0 && !rateLimited
    ? r2(Math.max(0, (1 - (received + late) / expected) * 100))
    : NaN;
  const latePct = expected > 0 && !rateLimited ? r2((late / expected) * 100) : NaN;
  return {
    scenario: scenario.name,
    topics: scenario.topics.length,
    msgs: count,
    hz: r2(count / (elapsed / 1000)),
    kbps: r2(bytes / 1024 / (elapsed / 1000)),
    latP50: r2(q(0.5)),
    latP95: r2(q(0.95)),
    latP99: r2(q(0.99)),
    latMax: r2(lat[lat.length - 1] ?? NaN),
    latStd: r2(std),
    lossPct,
    latePct,
  };
}

/** On-demand WS-hop saving %: the `on-demand` profile's kB/s vs `all-lanes`'s, from the rows. */
export function onDemandSaving(rows: BenchRow[]): number {
  const all = rows.find((r) => r.scenario === ON_DEMAND_PAIR.all);
  const one = rows.find((r) => r.scenario === ON_DEMAND_PAIR.one);
  if (!all || !one || all.kbps <= 0) return 0;
  return Math.round((1 - one.kbps / all.kbps) * 100);
}

/** Render one transport's rows as a Markdown section (the in-app bench's copy-Markdown export). */
export function formatMarkdown(
  label: string,
  url: string,
  durMs: number,
  stamp: string,
  rows: BenchRow[],
): string {
  const all = rows.find((r) => r.scenario === ON_DEMAND_PAIR.all);
  const one = rows.find((r) => r.scenario === ON_DEMAND_PAIR.one);
  const lines = [
    `# Transport benchmark — ${label}`,
    ``,
    `_${durMs}ms per scenario · ${url} · ${stamp}_`,
    ``,
    `| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | max | std | loss% | late% |`,
    `|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|`,
    ...rows.map(
      (r) =>
        `| ${r.scenario} | ${r.topics} | ${r.msgs} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latP99} | ${r.latMax} | ${r.latStd} | ${r.lossPct} | ${r.latePct} |`,
    ),
    ``,
  ];
  // Only meaningful when the sweep actually ran the on-demand pair (all-lanes + on-demand).
  if (one && all) {
    lines.push(
      `**On-demand bandwidth:** subscribing 1 of ${all.topics} topics delivered **${one.kbps} kB/s** vs **${all.kbps} kB/s** for all ${all.topics} — a **${
        onDemandSaving(rows)
      }% reduction** on the WS hop.`,
      ``,
    );
  }
  return lines.join("\n");
}
