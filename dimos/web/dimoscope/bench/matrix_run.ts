// matrix_run.ts — measure each delivery mechanism (ws/sse/poll) for ONE network
// profile and print one Markdown row per mechanism. Driven by bench/matrix.sh, which
// starts the gateway + publisher once and a netsim proxy per profile in front of them.
//   GATEWAY_URL=ws://localhost:8099 PROFILE=3g DUR=3000 deno run -A bench/matrix_run.ts
import { createGatewayWsTransport } from "../packages/topics/src/adapters/gatewayWs.ts";
import { createHttpPollTransport } from "../packages/topics/src/adapters/httpPoll.ts";
import { createSseTransport } from "../packages/topics/src/adapters/sse.ts";
import { measureScenario } from "../packages/topics/src/bench.ts";
import { createDimosClient } from "../packages/topics/src/client.ts";
import type { Transport } from "../packages/topics/src/transport.ts";

const URL = Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8099";
const PROFILE = Deno.env.get("PROFILE") ?? "lan";
const DUR = Number(Deno.env.get("DUR") ?? 3000);
const WARMUP_MS = Number(Deno.env.get("WARMUP_MS") ?? 8000);
const SCENARIO = {
  name: "4x PoseStamped",
  topics: ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"],
};

const mk: Record<string, () => Transport> = {
  ws: () => createGatewayWsTransport({ url: URL, reconnect: false }),
  sse: () => createSseTransport({ url: URL, reconnect: false }),
  poll: () => createHttpPollTransport({ url: URL }),
};

// Wait (through the proxy) until data is flowing, so the first measured mechanism
// isn't charged the publisher cold-start. Subsequent profiles find it already warm.
async function warmup() {
  const t = mk.ws();
  const client = createDimosClient({ transport: t });
  await t.connect();
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
  t.close();
}

async function measure(mech: string): Promise<string> {
  const t = mk[mech]();
  const client = createDimosClient({ transport: t });
  await t.connect();
  const r = await measureScenario(client, SCENARIO, DUR, true); // end-to-end (publish→recv)
  t.close();
  return `| ${PROFILE} | ${mech} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latP99} | ${r.latMax} | ${r.latStd} | ${r.lossPct} |`;
}

await warmup();
for (const mech of ["ws", "sse", "poll"]) {
  console.log(await measure(mech));
  await new Promise((r) => setTimeout(r, 200)); // let the proxy/link drain between runs
}
Deno.exit(0);
