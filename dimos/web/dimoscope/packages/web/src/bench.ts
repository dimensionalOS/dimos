// Transport benchmark core: measures latency (p50/p95/p99/max/std), throughput (hz), bandwidth
// (kB/s), per-lane breakdowns, 1 s time-series buckets, and offered-vs-delivered goodput derived
// from the source seq span. Platform-agnostic — runs identically in the browser and in tests.
// Unit tests in bench.test.ts; the run-record/export layer lives in benchRun.ts.
import type { DimosClient } from "./client.ts";
import type { Qos } from "./types.ts";
export interface BenchScenario {
  name: string;
  topics: string[];
}

/** Publisher config a heavy profile expects on `/load/img` — the sweep drives GO2Load
 *  `start_bench(hz, bytes, kind)` from this, so the profile name means what it says. */
export interface GenSpec {
  hz: number;
  bytes: number;
  kind: "image" | "cloud";
  note?: string;
}

/**
 * A named workload class. The heavy classes (lidar/camera/dense/…) all ride the SAME
 * /load/img topic; `gen` is the *publisher* config (hz × bytes) that makes the class real —
 * the bench runner applies it via the GO2Load RPC when available. Without the RPC the
 * browser measures whatever the source is emitting and `hint` is just the intent.
 */
export interface StreamProfile {
  id: string;
  /** topics to subscribe + measure */
  topics: string[];
  /** human note on the publisher config / expected load (for the UI). */
  hint: string;
  /** publisher config for the flood classes — the single source of truth for tier hz×bytes. */
  gen?: GenSpec;
}

const fmtBytes = (b: number) => (b >= 1e6 ? `${b / 1e6}MB` : `${b / 1e3}KB`);
const genHint = (g: GenSpec) =>
  `img@${g.hz}Hz × ${fmtBytes(g.bytes)} ≈ ${Math.round((g.hz * g.bytes) / 1e6)} MB/s${
    g.note ? ` (${g.note})` : ""
  }`;
const heavy = (id: string, hz: number, bytes: number, note?: string): StreamProfile => {
  const gen: GenSpec = { hz, bytes, kind: "image", note };
  return { id, topics: ["/load/img"], hint: genHint(gen), gen };
};

/** The small-lane trio the coex variants ride beside a flood — the `pose` profile's topics. */
export const POSE_LANES = ["/load/fast", "/load/mid", "/load/slow"] as const;
/** The interference signal lane: 100 Hz, tiny — the first casualty of a congested wire. */
export const FAST_LANE = "/load/fast";
/** Flood-carrying topics — a row with one of these beside FAST_LANE is a coexistence cell. */
export const BULK_LANES = ["/load/img", "/load/cloud"] as const;

/**
 * Coexistence variant of a flood profile: the pose lanes measured BESIDE its flood, so the
 * per-lane split answers "does the bulk stream delay /load/fast?". Identity for profiles
 * that already carry the fast lane (pose/all-lanes/on-demand/mixed) — and thus idempotent.
 */
export const coexProfile = (p: StreamProfile): StreamProfile =>
  p.topics.includes(FAST_LANE) ? p : {
    ...p,
    id: `${p.id}+pose`,
    topics: [...POSE_LANES, ...p.topics],
    hint: `pose lanes beside ${p.hint}`,
  };

/** Canonical workload classes — keep `id` + `topics` in sync with the GO2Load `/load/*` ports. */
export const STREAM_PROFILES: StreamProfile[] = [
  {
    id: "pose",
    topics: [...POSE_LANES],
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
  heavy("lidar", 10, 200_000, "2D lidar"),
  heavy("camera", 20, 550_000, "1080p"),
  heavy("dense", 20, 1_000_000, "depth"),
  // Overload tiers — run light→heavy: the top tiers may degrade or kill the tab (expected —
  // that's the case being measured).
  heavy("depth-hd", 20, 2_500_000, "RealSense/Ouster"),
  heavy("raw-1080p", 30, 6_000_000, "raw RGB"),
  heavy("firehose", 30, 10_000_000, "raw 4K / multi-cam"),
  {
    id: "mixed",
    topics: ["/load/fast", "/load/mid", "/load/slow", "/load/grid", "/load/img"],
    hint: "fast@100 + mid@20 + slow@2 + grid@5 + img flood",
    gen: { hz: 20, bytes: 1_000_000, kind: "image", note: "the beside-bulk coexistence case" },
  },
];

/** The measured on-demand pair: run both profiles in one sweep and the export prints the
 *  WS-hop saving (subscribe 1 lane vs all of them). Ids must match STREAM_PROFILES —
 *  the sync is unit-tested. */
export const ON_DEMAND_PAIR = { all: "all-lanes", one: "on-demand" } as const;

/** Per-topic slice of a BenchRow — same stat set and NaN rules, scoped to one lane. */
export interface LaneStats {
  topic: string;
  msgs: number;
  hz: number;
  kbps: number;
  latP50: number;
  latP95: number;
  latP99: number;
  latMax: number;
  latStd: number;
  lossPct: number;
  latePct: number;
  /** Source publish rate estimated from the seq span: (max−min+1)/elapsed. Under-counts by
   *  ≤~2 frames at the window edges (frames published before the first / after the last
   *  received seq). NaN if the topic is seq-less, saw <2 expected frames, or the run is
   *  client-rate-limited (qos.maxHz — gaps are then intentional downsampling). */
  offeredHz: number;
  /** offeredHz × mean delivered msg size (the bench lanes have fixed payloads). */
  offeredKbps: number;
  /** In-window goodput vs offered: msgs/expected × 100 (≡ kbps/offeredKbps). >100 ⇒ duplicate
   *  delivery. Shares the source-side caveat of lossPct: a publisher that skips seqs under
   *  load is indistinguishable from wire loss receiver-side. */
  deliveryPct: number;
  /** Backward seq jumps > RESET_GAP (source restarts) absorbed mid-run; >0 ⇒ distrust this lane. */
  seqResets: number;
}

/** Per-topic slice of one bucket (sparkline unit). */
export interface BucketLane {
  hz: number;
  kbps: number;
  latP95: number;
}

/** One wall-clock slice of the measure window (default 1 s). */
export interface BenchBucket {
  /** Start offset from the measure-window start (ms): 0, bucketMs, 2·bucketMs, … gap-free. */
  tMs: number;
  /** Actual width — the trailing bucket is cut at the window edge (may be partial). */
  durMs: number;
  msgs: number;
  hz: number;
  kbps: number;
  /** NaN when the bucket has no latency samples. */
  latP50: number;
  latP95: number;
  /** Only topics that delivered in this bucket appear (missing key ⇒ 0). */
  lanes: Record<string, BucketLane>;
}

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
  /** Actual measured wall clock (ms) the rates divided by; Σ buckets[].durMs ≈ elapsedMs. */
  elapsedMs: number;
  /** Σ lane offered — NaN unless EVERY lane has a valid span (summing only the known lanes
   *  would overstate deliveryPct exactly when a lane collapses to zero, the case being
   *  measured). Per-lane values keep what is known. */
  offeredHz: number;
  offeredKbps: number;
  deliveryPct: number;
  /** Σ lane seqResets — >0 ⇒ annotate the row (source restarted mid-run). */
  seqResets: number;
  /** Per-topic breakdown, scenario.topics order. */
  lanes: LaneStats[];
  /** Gap-free slices of the measure window; buckets.length === ceil(elapsedMs/bucketMs). */
  buckets: BenchBucket[];
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
  /** Time-series bucket width for row.buckets (ms). Default 1000; min 1. */
  bucketMs?: number;
  /** Live progress: fired once per completed bucket, in order, gap buckets included. When set,
   *  a bucketMs interval also ticks during the window so a stalled stream still emits 0-buckets. */
  onBucket?: (b: BenchBucket) => void;
}

/** Default phase lengths — exported so the app can stamp actual values into a RunRecord
 *  and compute honest sweep ETAs. */
export const defaultWarmup = (durMs: number): number => Math.min(500, durMs / 5);
export const defaultGrace = (durMs: number): number => Math.min(750, durMs / 2);

/** A backward seq jump larger than this is a source restart, not a reorder — the active
 *  segment is banked so loss/offered stay correct across it. */
const RESET_GAP = 1000;

const r2 = (n: number) => Math.round(n * 100) / 100;
const now = () => performance.now();

const sortedStats = (lat: number[]) => {
  lat.sort((a, b) => a - b);
  const q = (
    p: number,
  ) => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : NaN);
  const mean = lat.length ? lat.reduce((a, b) => a + b, 0) / lat.length : NaN;
  const std = lat.length
    ? Math.sqrt(lat.reduce((a, b) => a + (b - mean) * (b - mean), 0) / lat.length)
    : NaN;
  return { p50: q(0.5), p95: q(0.95), p99: q(0.99), max: lat[lat.length - 1] ?? NaN, std };
};

interface LaneAcc {
  lat: number[];
  msgs: number;
  bytes: number;
  span?: { min: number; max: number };
  seqs: Set<number>;
  late: Set<number>;
  seqResets: number;
  // Σ over segments closed by a detected source restart — expected/received keep counting across it.
  closedExpected: number;
  closedReceived: number;
}
const freshLane = (): LaneAcc => ({
  lat: [],
  msgs: 0,
  bytes: 0,
  seqs: new Set(),
  late: new Set(),
  seqResets: 0,
  closedExpected: 0,
  closedReceived: 0,
});

interface BucketAcc {
  msgs: number;
  bytes: number;
  lat: number[];
  perLane: Map<string, { msgs: number; bytes: number; lat: number[] }>;
}
const freshBucket = (): BucketAcc => ({ msgs: 0, bytes: 0, lat: [], perLane: new Map() });

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
  const warmupMs = opts?.warmupMs ?? defaultWarmup(durMs);
  const graceMs = opts?.graceMs ?? defaultGrace(durMs);
  const offsetMs = opts?.offsetMs ?? 0;
  const bucketMs = Math.max(1, opts?.bucketMs ?? 1000);

  const lanes = new Map<string, LaneAcc>();
  const laneOrder: string[] = [];
  const lane = (topic: string): LaneAcc => {
    let acc = lanes.get(topic);
    if (!acc) {
      lanes.set(topic, acc = freshLane());
      laneOrder.push(topic);
    }
    return acc;
  };
  for (const t of scenario.topics) lane(t);

  // Bucket state — only the open bucket keeps latency arrays; closed ones keep scalars.
  const buckets: BenchBucket[] = [];
  let curIdx = 0;
  let cur = freshBucket();
  // Provisional until the measure phase starts — a message delivered synchronously on
  // subscribe (warmupMs 0) must not compute a bucket index against an unset clock.
  let t0 = now();
  const finalize = (acc: BucketAcc, tMs: number, width: number): BenchBucket => {
    const s = (width || 1) / 1000;
    const laneOut: Record<string, BucketLane> = {};
    for (const [t, l] of acc.perLane) {
      laneOut[t] = {
        hz: r2(l.msgs / s),
        kbps: r2(l.bytes / 1024 / s),
        latP95: r2(sortedStats(l.lat).p95),
      };
    }
    const st = sortedStats(acc.lat);
    return {
      tMs: r2(tMs),
      durMs: r2(width),
      msgs: acc.msgs,
      hz: r2(acc.msgs / s),
      kbps: r2(acc.bytes / 1024 / s),
      latP50: r2(st.p50),
      latP95: r2(st.p95),
      lanes: laneOut,
    };
  };
  const emit = (b: BenchBucket) => {
    buckets.push(b);
    opts?.onBucket?.(b);
  };
  // The single bucket-emission path — message arrival, the optional live tick, and the
  // end-of-window flush all funnel through here, so gap buckets emit as zeros in order.
  const advanceTo = (idx: number) => {
    while (curIdx < idx) {
      emit(finalize(cur, curIdx * bucketMs, bucketMs));
      curIdx++;
      cur = freshBucket();
    }
  };

  let phase: "warmup" | "measure" | "grace" = warmupMs > 0 ? "warmup" : "measure";
  const subs = scenario.topics.map((t) => {
    const topic = client.topic(t);
    // Apply QoS before subscribing so the gateway downsample (maxHz) is requested on the
    // first subscribe. Optional-chained so test fakes without setQos don't throw.
    if (qos) topic.setQos?.(qos);
    return topic.subscribe(({ meta: m }) => {
      if (phase === "warmup") return;
      const acc = lane(m.topic);
      if (phase === "grace") {
        // Only reclassify frames the window already expected (inside the active span).
        if (m.seq == null || !acc.span) return;
        if (m.seq < acc.span.min || m.seq > acc.span.max || acc.seqs.has(m.seq)) return;
        acc.late.add(m.seq);
        return;
      }
      acc.msgs++;
      acc.bytes += m.sizeBytes;
      const l = endToEnd
        ? (m.srcTs != null ? m.recvTs - m.srcTs + offsetMs : undefined)
        : m.latencyMs;
      // Small negatives are residual offset-estimation error (clamp to 0); large ones are
      // uncorrected skew and would poison the stats (drop, as before).
      const lv = l != null && l >= -50 && l < 10000 ? Math.max(0, l) : undefined;
      if (lv != null) acc.lat.push(lv);
      if (m.seq != null) {
        if (acc.span && m.seq < acc.span.min - RESET_GAP) {
          // Source restart: bank the finished segment so expected/received span both.
          acc.closedExpected += acc.span.max - acc.span.min + 1;
          acc.closedReceived += acc.seqs.size;
          acc.seqs = new Set();
          acc.span = undefined;
          acc.seqResets++;
        }
        if (!acc.span) acc.span = { min: m.seq, max: m.seq };
        else {
          if (m.seq < acc.span.min) acc.span.min = m.seq;
          if (m.seq > acc.span.max) acc.span.max = m.seq;
        }
        acc.seqs.add(m.seq);
      }
      advanceTo(Math.floor((now() - t0) / bucketMs));
      cur.msgs++;
      cur.bytes += m.sizeBytes;
      if (lv != null) cur.lat.push(lv);
      let bl = cur.perLane.get(m.topic);
      if (!bl) cur.perLane.set(m.topic, bl = { msgs: 0, bytes: 0, lat: [] });
      bl.msgs++;
      bl.bytes += m.sizeBytes;
      if (lv != null) bl.lat.push(lv);
    });
  });

  if (warmupMs > 0) await new Promise((r) => setTimeout(r, warmupMs));
  phase = "measure";
  t0 = now();
  // The interval exists only for live consumers: a stalled stream still ticks zero buckets.
  const iv = opts?.onBucket
    ? setInterval(() => advanceTo(Math.floor((now() - t0) / bucketMs)), bucketMs)
    : undefined;
  await new Promise((r) => setTimeout(r, durMs));
  // Divide rates by the wall clock actually elapsed — a GC pause or tab throttle stretches
  // the window, and dividing by nominal durMs would over-report.
  const elapsed = Math.max(now() - t0, 1);
  if (iv !== undefined) clearInterval(iv);
  phase = "grace";
  advanceTo(Math.floor(elapsed / bucketMs)); // close all full buckets (gaps emit as zeros)
  const tail = elapsed - curIdx * bucketMs;
  if (tail > 0) emit(finalize(cur, curIdx * bucketMs, tail));
  if (graceMs > 0) await new Promise((r) => setTimeout(r, graceMs));
  subs.forEach((s) => s.unsubscribe());

  const rateLimited = !!qos?.maxHz;
  const es = elapsed / 1000;
  const laneStats: LaneStats[] = laneOrder.map((topic) => {
    const acc = lanes.get(topic)!;
    const st = sortedStats(acc.lat);
    const expected = acc.closedExpected + (acc.span ? acc.span.max - acc.span.min + 1 : 0);
    const received = acc.closedReceived + acc.seqs.size;
    const late = acc.late.size;
    const kbps = acc.bytes / 1024 / es;
    const seqValid = expected > 0 && !rateLimited;
    // deliveryPct = msgs/expected (≡ kbps/offeredKbps under the mean-size estimator) — the
    // ratio cancels elapsed, so it's exact and timing-independent.
    const offValid = expected >= 2 && !rateLimited && acc.msgs > 0;
    return {
      topic,
      msgs: acc.msgs,
      hz: r2(acc.msgs / es),
      kbps: r2(kbps),
      latP50: r2(st.p50),
      latP95: r2(st.p95),
      latP99: r2(st.p99),
      latMax: r2(st.max),
      latStd: r2(st.std),
      lossPct: seqValid ? r2(Math.max(0, (1 - (received + late) / expected) * 100)) : NaN,
      latePct: seqValid ? r2((late / expected) * 100) : NaN,
      offeredHz: offValid ? r2(expected / es) : NaN,
      offeredKbps: offValid ? r2((expected * (acc.bytes / acc.msgs)) / 1024 / es) : NaN,
      deliveryPct: offValid ? r2((acc.msgs / expected) * 100) : NaN,
      seqResets: acc.seqResets,
    };
  });

  // Pooled stats derive from the concatenated lane arrays — the identical sample set the
  // pre-lane implementation collected in one array.
  const allLat: number[] = [];
  let msgs = 0;
  let bytes = 0;
  let expected = 0;
  let received = 0;
  let late = 0;
  let seqResets = 0;
  for (const t of laneOrder) {
    const acc = lanes.get(t)!;
    allLat.push(...acc.lat);
    msgs += acc.msgs;
    bytes += acc.bytes;
    expected += acc.closedExpected + (acc.span ? acc.span.max - acc.span.min + 1 : 0);
    received += acc.closedReceived + acc.seqs.size;
    late += acc.late.size;
    seqResets += acc.seqResets;
  }
  const st = sortedStats(allLat);
  const allOffered = laneStats.length > 0 && laneStats.every((l) => Number.isFinite(l.offeredHz));
  return {
    scenario: scenario.name,
    topics: scenario.topics.length,
    msgs,
    hz: r2(msgs / es),
    kbps: r2(bytes / 1024 / es),
    latP50: r2(st.p50),
    latP95: r2(st.p95),
    latP99: r2(st.p99),
    latMax: r2(st.max),
    latStd: r2(st.std),
    lossPct: expected > 0 && !rateLimited
      ? r2(Math.max(0, (1 - (received + late) / expected) * 100))
      : NaN,
    latePct: expected > 0 && !rateLimited ? r2((late / expected) * 100) : NaN,
    elapsedMs: r2(elapsed),
    offeredHz: allOffered ? r2(laneStats.reduce((a, l) => a + l.offeredHz, 0)) : NaN,
    offeredKbps: allOffered ? r2(laneStats.reduce((a, l) => a + l.offeredKbps, 0)) : NaN,
    deliveryPct: allOffered && expected > 0 ? r2((msgs / expected) * 100) : NaN,
    seqResets,
    lanes: laneStats,
    buckets,
  };
}

/** On-demand WS-hop saving %: the `on-demand` profile's kB/s vs `all-lanes`'s, from the rows. */
export function onDemandSaving(rows: BenchRow[]): number {
  const all = rows.find((r) => r.scenario === ON_DEMAND_PAIR.all);
  const one = rows.find((r) => r.scenario === ON_DEMAND_PAIR.one);
  if (!all || !one || all.kbps <= 0) return 0;
  return Math.round((1 - one.kbps / all.kbps) * 100);
}
