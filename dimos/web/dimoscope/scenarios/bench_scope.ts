// Per-scenario data-path benchmark. Measures the LIVE scenario's topics over the WS transport and
// prints one markdown table row. Reuses the SDK's `measureScenario` — it does NOT touch bench.ts.
//
// Driven by scenarios/bench.sh (which starts the matching publisher first). Or directly:
//   deno run -A scenarios/bench_scope.ts nav 6000 ws://localhost:8080/ws
import { connect } from "../packages/topics/src/client.ts";
import { measureScenario } from "../packages/topics/src/bench.ts";

const SCENARIOS: Record<string, string[]> = {
  nav: ["/nav/pose", "/nav/path", "/nav/cloud", "/nav/map"],
  arm: ["/arm/joint_states", "/arm/ee_pose", "/arm/imu", "/arm/trajectory"],
  cam: ["/cam/rgb", "/cam/depth", "/cam/points", "/cam/detections"],
};

const name = Deno.args[0] ?? "";
const durMs = Number(Deno.args[1] ?? 6000);
const url = Deno.args[2] ?? "ws://localhost:8080/ws";
const topics = SCENARIOS[name];
if (!topics) {
  console.error("usage: bench_scope.ts <nav|arm|cam> [durMs] [wsUrl]");
  Deno.exit(2);
}

const client = await connect({ url });
const r = await measureScenario(client, { name, topics }, durMs, true);
// | scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | loss% |
console.log(
  `| ${name} | ${topics.length} | ${r.msgs} | ${r.hz} | ${r.kbps} | ${r.latP50} | ${r.latP95} | ${r.latP99} | ${r.lossPct} |`,
);
client.close();
Deno.exit(0);
