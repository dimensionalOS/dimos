// Headless transport benchmark — measures the @dimos/topics SDK against a gateway:
// latency (p50/p95/max), throughput, bandwidth, and ON-DEMAND bandwidth savings
// (subscribe 1 of N vs all N). Writes bench/RESULTS.md + results.json.
//
// RUN (with bench_publisher.py + a gateway running):
//   GATEWAY_URL=ws://localhost:8090 BENCH_LABEL="Bun↔LCM" bun run bench/bench.ts
import { connect } from "../packages/topics/src/index";

const WS_URL = process.env.GATEWAY_URL ?? "ws://localhost:8090";
const LABEL = process.env.BENCH_LABEL ?? "Bun↔LCM gateway";
const DUR = Number(process.env.BENCH_DUR_MS ?? 5000);

interface Row {
  scenario: string;
  topics: number;
  msgs: number;
  hz: number;
  kbps: number;
  latP50: number;
  latP95: number;
  latMax: number;
}

const rows: Row[] = [];
const r2 = (n: number) => Math.round(n * 100) / 100;

async function meas(scenario: string, topics: string[]): Promise<Row> {
  const client = await connect({ url: WS_URL, reconnect: false });
  await new Promise((r) => setTimeout(r, 200));
  const lat: number[] = [];
  let count = 0;
  let bytes = 0;
  const subs = topics.map((t) =>
    client.topic<any>(t).subscribe((_d, m) => {
      count++;
      bytes += m.sizeBytes;
      if (m.latencyMs != null && m.latencyMs >= 0 && m.latencyMs < 10000) lat.push(m.latencyMs);
    }),
  );
  await new Promise((r) => setTimeout(r, DUR));
  subs.forEach((s) => s.unsubscribe());
  client.close();
  lat.sort((a, b) => a - b);
  const q = (p: number) => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : NaN);
  const row: Row = {
    scenario,
    topics: topics.length,
    msgs: count,
    hz: r2(count / (DUR / 1000)),
    kbps: r2(bytes / 1024 / (DUR / 1000)),
    latP50: r2(q(0.5)),
    latP95: r2(q(0.95)),
    latMax: r2(lat[lat.length - 1] ?? NaN),
  };
  rows.push(row);
  console.log(
    `  ${scenario.padEnd(32)} topics=${row.topics} hz=${row.hz} kB/s=${row.kbps} ` +
      `lat(ms) p50=${row.latP50} p95=${row.latP95} max=${row.latMax}`,
  );
  return row;
}

console.log(`\n=== Benchmark: ${LABEL}  (${DUR}ms/scenario, ${WS_URL}) ===`);
const all4 = await meas("4x PoseStamped (throughput)", ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"]);
const one = await meas("1x PoseStamped (on-demand)", ["/bench/p0"]);
const grid = await meas("1x OccupancyGrid (large)", ["/bench/grid"]);
void grid;

const ondemandSaving = all4.kbps > 0 ? Math.round((1 - one.kbps / all4.kbps) * 100) : 0;
const stamp = process.env.BENCH_STAMP ?? "(unstamped)";

const md = [
  `# Transport benchmark — ${LABEL}`,
  ``,
  `_${DUR}ms per scenario · ${WS_URL} · ${stamp}_`,
  ``,
  `| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |`,
  `|---|--:|--:|--:|--:|--:|--:|--:|`,
  ...rows.map(
    (r) => `| ${r.scenario} | ${r.topics} | ${r.msgs} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latMax} |`,
  ),
  ``,
  `**On-demand bandwidth:** subscribing 1 of 4 topics delivered **${one.kbps} kB/s** vs **${all4.kbps} kB/s** for all 4 — a **${ondemandSaving}% reduction** on the WS hop (per-client gateway filtering).`,
  ``,
  `> Over LCM the gateway still *receives* every topic (UDP multicast), so this saves the browser hop only. True end-to-end on-demand (robot→gateway) needs the Zenoh gateway (\`declareSubscriber\`/\`undeclare\`).`,
  ``,
].join("\n");

await Bun.write(new URL("./last_run.md", import.meta.url).pathname, md);
await Bun.write(
  new URL("./results.json", import.meta.url).pathname,
  JSON.stringify({ label: LABEL, url: WS_URL, durMs: DUR, stamp, rows, ondemandSaving }, null, 2),
);
console.log(`\n  on-demand WS-hop saving: ${ondemandSaving}%  → wrote bench/last_run.md\n`);
process.exit(rows[0]?.msgs > 0 ? 0 : 1);
