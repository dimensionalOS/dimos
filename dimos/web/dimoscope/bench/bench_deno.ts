// Headless zenoh-ts benchmark under DENO. zenoh-ts won't run in Bun — its wasm-bindgen
// bundler-target WASM fails there (`wasm.__wbindgen_start is not a function`) — but Deno
// instantiates it (zenoh-ts's own examples are Deno-based). Same scenarios + measurement
// as the Bun runner (bench/bench.ts) via @dimos/topics/bench, so it's apples-to-apples.
//
// RUN (with the :10000 bridge + a zenoh bench_publisher up — see bench/serve-bench.sh):
//   deno run -A --node-modules-dir bench/bench_deno.ts
import { connect, createZenohTsTransport } from "../packages/topics/src/index.ts";
import {
  BENCH_SCENARIOS,
  type BenchRow,
  formatMarkdown,
  measureScenario,
  onDemandSaving,
} from "../packages/topics/src/bench.ts";

const TS_URL = Deno.env.get("ZENOH_TS_URL") ?? "ws://localhost:10000";
const DUR = Number(Deno.env.get("BENCH_DUR_MS") ?? 4000);
const LABEL = "zenoh-ts (direct, Deno)";

console.log(`\n=== Benchmark: ${LABEL}  (${DUR}ms/scenario, ${TS_URL}) ===`);
const rows: BenchRow[] = [];
for (const scenario of BENCH_SCENARIOS) {
  // discoveryKey "" → no scout; subscribe explicit /bench/* keys.
  const client = await connect({
    transport: createZenohTsTransport({ remoteApiUrl: TS_URL, discoveryKey: "" }),
  });
  await new Promise((r) => setTimeout(r, 250));
  const row = await measureScenario(client, scenario, DUR, true); // endToEnd (publish→client)
  client.close();
  rows.push(row);
  console.log(
    `  ${row.scenario.padEnd(32)} hz=${row.hz} kB/s=${row.kbps} ` +
      `lat(ms) p50=${row.latP50} p95=${row.latP95} max=${row.latMax}`,
  );
}

const stamp = Deno.env.get("BENCH_STAMP") ?? "(deno)";
await Deno.writeTextFile(
  new URL("./last_run.md", import.meta.url).pathname,
  formatMarkdown(LABEL, TS_URL, DUR, stamp, rows),
);
console.log(`\n  on-demand saving: ${onDemandSaving(rows)}%  → wrote bench/last_run.md\n`);
