// qos_demo_run.ts — the QoS A/B measurement client. Connects to the single service (serve.py `/ws`),
// subscribes to a LIGHT (sensor-lane) and a HEAVY (bulk-lane) topic **simultaneously** so they contend
// for the link, and reports per-topic hz + latency. Run once with the scheduler OFF (FIFO) and once ON
// (priority outbox) — under a constrained link the light topic should stay crisp ON while it starves OFF.
//
//   GW_URL=ws://localhost:8099/ws LIGHT_TOPIC=/pose HEAVY_TOPIC=/lidar MODE=on \
//     deno run -A bench/qos_demo_run.ts
import { connect } from "../packages/topics/src/index.ts";
import { measureScenario } from "../packages/topics/src/bench.ts";
import { resolveQos } from "../packages/topics/src/qos.ts";

const GW = Deno.env.get("GW_URL") ?? "ws://localhost:8080/ws";
const LIGHT = Deno.env.get("LIGHT_TOPIC") ?? "/bench/p0";
const HEAVY = Deno.env.get("HEAVY_TOPIC") ?? "/bench/img";
const DUR = Number(Deno.env.get("DUR_MS") ?? 4000);
const MODE = Deno.env.get("MODE") ?? "?";
// Declare the lanes (sensor for the light topic, bulk for the heavy) — what a real client would do; the
// gateway also derives this server-side via defaultLane, so either path enforces priority.
const lightLane = Deno.env.get("LIGHT_LANE") as "sensor" | "command" | undefined;
const heavyLane = Deno.env.get("HEAVY_LANE") as "bulk" | undefined;

const client = await connect({ url: GW, reconnect: false });
await new Promise((r) => setTimeout(r, 300));
// both at once → they share (and contend for) the one link
const [light, heavy] = await Promise.all([
  measureScenario(client, { name: "light", topics: [LIGHT] }, DUR, true, resolveQos(LIGHT, "", lightLane ? { lane: lightLane } : undefined)),
  measureScenario(client, { name: "heavy", topics: [HEAVY] }, DUR, true, resolveQos(HEAVY, "", heavyLane ? { lane: heavyLane } : undefined)),
]);
client.close();
console.log(
  `QOS_DEMO mode=${MODE} | LIGHT ${LIGHT} hz=${light.hz} p50=${light.latP50} p95=${light.latP95} ` +
    `loss=${light.lossPct}% | HEAVY ${HEAVY} hz=${heavy.hz} p50=${heavy.latP50} p95=${heavy.latP95}`,
);
Deno.exit(light.msgs + heavy.msgs > 0 ? 0 : 1);
