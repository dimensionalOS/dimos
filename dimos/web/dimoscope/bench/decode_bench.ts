// decode_bench.ts — the decode-LOCATION axis: client-side binary decode (gateway = thin self-
// describing byte-relay, dimos today) vs server-side decode→JSON (the rosbridge model). Same gateway,
// same scenario, two decode modes — prints the bandwidth + latency cost of moving decode to the server.
//   GATEWAY_URL=ws://localhost:8090 DUR=3000 deno run -A --unstable-sloppy-imports bench/decode_bench.ts
import { measureScenario } from "../packages/topics/src/bench.ts";
import { createDimosClient, ws, wsServerJson } from "../packages/topics/src/client.ts";
import type { BenchRow } from "../packages/topics/src/bench.ts";

const URL = Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8080/ws";
const DUR = Number(Deno.env.get("DUR") ?? 3000);
const SCEN = {
  name: "4x PoseStamped + grid",
  topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3", "/bench/grid"],
};

async function run(mode: "client" | "server-json"): Promise<BenchRow> {
  const client = createDimosClient({
    transport: mode === "server-json" ? wsServerJson({ reconnect: false }) : ws({ reconnect: false }),
  });
  await client.connect(URL);
  const r = await measureScenario(client, SCEN, DUR, true); // end-to-end (publish→recv)
  client.close();
  return r;
}

// Warm up (wait for the publisher) so neither mode eats the cold start.
{
  const c = createDimosClient({ transport: ws({ reconnect: false }) });
  await c.connect(URL);
  await new Promise<void>((res) => {
    const to = setTimeout(res, 15000);
    const s = c.topic("/bench/p0").subscribe(() => {
      clearTimeout(to);
      s.unsubscribe();
      res();
    });
  });
  c.close();
}

const bin = await run("client");
const json = await run("server-json");
const f = (n: number) => n.toFixed(2);
console.log(`decode-mode    hz     kB/s    p50    p95    p99   loss%`);
console.log(
  `client       ${f(bin.hz)}  ${f(bin.kbps)}  ${f(bin.latP50)}  ${f(bin.latP95)}  ${
    f(bin.latP99)
  }  ${f(bin.lossPct)}`,
);
console.log(
  `server-json  ${f(json.hz)}  ${f(json.kbps)}  ${f(json.latP50)}  ${f(json.latP95)}  ${
    f(json.latP99)
  }  ${f(json.lossPct)}`,
);
console.log(
  `→ server-JSON moves ${
    f(json.kbps / bin.kbps)
  }× the bytes of the binary self-describing relay (the rosbridge tax).`,
);
Deno.exit(0);
