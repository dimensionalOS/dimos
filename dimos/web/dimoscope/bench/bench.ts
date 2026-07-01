// Headless transport benchmark (Deno runner) — measures the @dimos/topics SDK against a
// gateway, OR the zenoh-ts direct transport (BENCH_TRANSPORT=ts). Scenarios + measurement
// live in @dimos/topics/bench so the in-browser bench page (app/src/bench.tsx) is identical.
//
// RUN (with bench_publisher.py + the relevant server running):
//   GATEWAY_URL=ws://localhost:8090 BENCH_LABEL="Deno↔LCM" deno run -A bench/bench.ts
//   BENCH_TRANSPORT=ts deno run -A bench/bench.ts        # zenoh-ts direct (needs the :10000 bridge)
import { createDimosClient, ws } from "../packages/topics/src/index.ts";
import {
  BENCH_SCENARIOS,
  type BenchRow,
  formatMarkdown,
  measureScenario,
  onDemandSaving,
} from "../packages/topics/src/bench.ts";

const WS_URL = Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8080/ws";
const LABEL = Deno.env.get("BENCH_LABEL") ?? "Deno↔LCM gateway";
const DUR = Number(Deno.env.get("BENCH_DUR_MS") ?? 5000);
const TRANSPORT = Deno.env.get("BENCH_TRANSPORT"); // "ts" → zenoh-ts direct
const TS_URL = Deno.env.get("ZENOH_TS_URL") ?? "ws://localhost:10000";
const url = TRANSPORT === "ts" ? TS_URL : WS_URL;
// BENCH_E2E=1 → end-to-end latency (publish→client) instead of the gateway WS-hop, so a
// combined all-transport table (incl. The gateway-less zenoh-ts) compares the same thing.
const E2E = Deno.env.get("BENCH_E2E") === "1";

async function buildClient() {
  if (TRANSPORT === "ts") {
    const { zenohTs } = await import("../packages/topics/src/experimental.ts");
    // read via the remote-api bridge; no scout (bench subscribes explicit /bench/* topics).
    const c = createDimosClient({ transport: zenohTs({ discoveryKey: "" }) });
    await c.connect(TS_URL);
    return c;
  }
  const c = createDimosClient({ transport: ws({ reconnect: false }) });
  await c.connect(WS_URL);
  return c;
}

console.log(`\n=== Benchmark: ${LABEL}  (${DUR}ms/scenario, ${url}) ===`);
const rows: BenchRow[] = [];
for (const scenario of BENCH_SCENARIOS) {
  const client = await buildClient();
  await new Promise((r) => setTimeout(r, 200));
  const row = await measureScenario(client, scenario, DUR, E2E);
  client.close();
  rows.push(row);
  console.log(
    `  ${row.scenario.padEnd(32)} topics=${row.topics} hz=${row.hz} kB/s=${row.kbps} ` +
      `lat(ms) p50=${row.latP50} p95=${row.latP95} max=${row.latMax}`,
  );
}

const stamp = Deno.env.get("BENCH_STAMP") ?? "(unstamped)";
const saving = onDemandSaving(rows);
await Deno.writeTextFile(
  new URL("./last_run.md", import.meta.url).pathname,
  formatMarkdown(LABEL, url, DUR, stamp, rows),
);
await Deno.writeTextFile(
  new URL("./results.json", import.meta.url).pathname,
  JSON.stringify({ label: LABEL, url, durMs: DUR, stamp, rows, ondemandSaving: saving }, null, 2),
);
console.log(`\n  on-demand WS-hop saving: ${saving}%  → wrote bench/last_run.md\n`);
Deno.exit(rows[0]?.msgs > 0 ? 0 : 1);
