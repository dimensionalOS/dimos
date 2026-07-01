// matrix_run.ts — measure each delivery mechanism (ws/sse/poll) for ONE network
// profile and print one Markdown row per mechanism. Driven by bench/matrix.sh, which
// starts the gateway + publisher once and a netsim proxy per profile in front of them.
//   GATEWAY_URL=ws://localhost:8099 PROFILE=3g DUR=3000 deno run -A bench/matrix_run.ts
import { measureScenario } from "../packages/topics/src/bench.ts";
import { createDimosClient, type TransportFactory, ws } from "../packages/topics/src/client.ts";
import { poll, sse } from "../packages/topics/src/experimental.ts";

// the gateway serves WS at /ws, SSE at /sse, poll at /poll (all one origin). Strip a trailing /ws so
// the SSE/poll adapters get the base; build the WS url with /ws.
const URL = (Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8099").replace(/\/ws$/, "");
const WS = `${URL}/ws`;
const PROFILE = Deno.env.get("PROFILE") ?? "lan";
const STREAM = Deno.env.get("STREAM") ?? "pose";
const DUR = Number(Deno.env.get("DUR") ?? 3000);
const WARMUP_MS = Number(Deno.env.get("WARMUP_MS") ?? 8000);
// Stream profiles: which topics to subscribe/measure. bench/matrix.sh tells the publisher the
// matching rates/sizes (lidar ~2MB/s, dense ~20MB/s, camera ~11MB/s, mixed = pose+grid+lidar).
const POSE = ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"];
const STREAMS: Record<string, { topics: string[]; warm: string }> = {
  pose: { topics: POSE, warm: "/bench/p0" },
  lidar: { topics: ["/bench/img"], warm: "/bench/img" },
  dense: { topics: ["/bench/img"], warm: "/bench/img" },
  camera: { topics: ["/bench/img"], warm: "/bench/img" },
  mixed: { topics: [...POSE, "/bench/grid", "/bench/img"], warm: "/bench/p0" },
};
const S = STREAMS[STREAM] ?? STREAMS.pose;
const SCENARIO = { name: STREAM, topics: S.topics };

const mk: Record<string, TransportFactory> = {
  ws: ws({ reconnect: false }), // ws() appends /ws to URL
  sse: sse({ reconnect: false }),
  poll: poll(),
};

// Wait (through the proxy) until data is flowing, so the first measured mechanism
// isn't charged the publisher cold-start. Subsequent profiles find it already warm.
async function warmup() {
  const client = createDimosClient({ transport: mk.ws });
  await client.connect(URL);
  await new Promise<void>((res) => {
    let done = false;
    const fin = () => {
      if (done) return;
      done = true;
      res();
    };
    const to = setTimeout(fin, WARMUP_MS);
    const sub = client.topic(S.warm).subscribe(() => {
      clearTimeout(to);
      sub.unsubscribe();
      fin();
    });
  });
  client.close();
}

async function measure(mech: string): Promise<string> {
  const client = createDimosClient({ transport: mk[mech] });
  await client.connect(URL);
  const r = await measureScenario(client, SCENARIO, DUR, true); // end-to-end (publish→recv)
  client.close();
  return `| ${STREAM} | ${PROFILE} | ${mech} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latP99} | ${r.lossPct} |`;
}

await warmup();
for (const mech of ["ws", "sse", "poll"]) {
  console.log(await measure(mech));
  await new Promise((r) => setTimeout(r, 200)); // let the proxy/link drain between runs
}
Deno.exit(0);
