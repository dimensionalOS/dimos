// Mechanism smoke/compare: drive the SAME gateway bus tap over WebSocket, SSE and
// HTTP long-poll, measure each with the shared bench core, print one row per mechanism.
// Run the gateway + bench_source first (see bench/run.sh / matrix). A building block
// for the full matrix runner.
//   GATEWAY_URL=ws://localhost:8090 DUR=2000 deno run -A bench/mech_smoke.ts
import { measureScenario } from "../packages/topics/src/bench.ts";
import { createDimosClient, type TransportFactory, ws } from "../packages/topics/src/client.ts";
import { poll, sse } from "../packages/topics/src/experimental.ts";

// the gateway serves WS at /ws, SSE at /sse, poll at /poll on one origin. BASE is the origin; WS_URL adds /ws.
const BASE = (Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8080").replace(/\/ws$/, "");
const WS_URL = `${BASE}/ws`;
const DUR = Number(Deno.env.get("DUR") ?? 2000);
const WARMUP_MS = Number(Deno.env.get("WARMUP_MS") ?? 15000);
const SCENARIO = {
  name: "4x PoseStamped",
  topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"],
};

// Wait until the publisher+gateway are actually flowing (or give up), so the first
// measured mechanism isn't penalised by cold-start import lag.
async function warmup() {
  const client = createDimosClient({ transport: ws({ reconnect: false }) });
  await client.connect(WS_URL);
  await new Promise<void>((res) => {
    let done = false;
    const fin = () => {
      if (done) return;
      done = true;
      res();
    };
    const to = setTimeout(fin, WARMUP_MS);
    const sub = client.topic("/bench/p0").subscribe(() => {
      clearTimeout(to);
      sub.unsubscribe();
      fin();
    });
  });
  client.close();
}

async function run(label: string, transport: TransportFactory, url: string) {
  const client = createDimosClient({ transport });
  await client.connect(url);
  const r = await measureScenario(client, SCENARIO, DUR, true); // end-to-end (publish→recv)
  console.log(
    `${
      label.padEnd(10)
    } hz=${r.hz}  kB/s=${r.kbps}  p50=${r.latP50}  p95=${r.latP95}  p99=${r.latP99}  max=${r.latMax}  std=${r.latStd}  loss%=${r.lossPct}`,
  );
  client.close();
}

console.log(`[mech_smoke] warming up (≤${WARMUP_MS}ms)…`);
await warmup();
await run("ws", ws({ reconnect: false }), WS_URL);
await run("sse", sse({ reconnect: false }), BASE);
await run("poll", poll(), BASE);
Deno.exit(0);
