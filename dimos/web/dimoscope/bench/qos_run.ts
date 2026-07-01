// qos_run.ts — quantify the client QoS knobs against a live gateway + bench_source (mixed):
//   MODE=basic (direct gateway): rate-limit (setRateLimit downsamples a topic) + on-demand (1 of N)
//   MODE=prio  (through a 4G netsim link): rate-limit the heavy lidar → the light pose stream recovers
// Driven by bench/qos.sh. Measured by delivered hz / kB/s / seq-loss.
import { createDimosClient, type DimosClient } from "../packages/topics/src/client.ts";
import { createGatewayWsTransport } from "../packages/topics/src/adapters/gatewayWs.ts";

const URL = Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8080/ws";
const DUR = Number(Deno.env.get("DUR") ?? 3000);
const MODE = Deno.env.get("MODE") ?? "basic";

interface Stat {
  count: number;
  bytes: number;
  min: number;
  max: number;
  recv: number;
}

async function measure(
  topics: string[],
  durMs: number,
  setup?: (c: DimosClient) => void,
): Promise<Map<string, Stat>> {
  const t = createGatewayWsTransport({ url: URL, reconnect: false });
  const c = createDimosClient({ transport: t });
  await t.connect();
  const stat = new Map<string, Stat>();
  const subs = topics.map((tp) =>
    c.topic(tp).subscribe((_d, m) => {
      let s = stat.get(m.topic);
      if (!s) {
        s = { count: 0, bytes: 0, min: m.seq ?? 0, max: m.seq ?? 0, recv: 0 };
        stat.set(m.topic, s);
      }
      s.count++;
      s.bytes += m.sizeBytes;
      if (m.seq != null) {
        if (m.seq < s.min) s.min = m.seq;
        if (m.seq > s.max) s.max = m.seq;
        s.recv++;
      }
    })
  );
  if (setup) setup(c);
  await new Promise((r) => setTimeout(r, durMs));
  subs.forEach((s) => s.unsubscribe());
  t.close();
  return stat;
}

const r2 = (n: number) => Math.round(n * 100) / 100;
const ZERO: Stat = { count: 0, bytes: 0, min: 0, max: 0, recv: 0 };
const hz = (s: Stat) => r2(s.count / (DUR / 1000));
const kbps = (s: Stat) => r2(s.bytes / 1024 / (DUR / 1000));
const loss = (s: Stat) => (s.recv > 1 ? r2(Math.max(0, (1 - s.recv / (s.max - s.min + 1)) * 100)) : 0);
const get = (m: Map<string, Stat>, t: string) => m.get(t) ?? ZERO;
const sumK = (m: Map<string, Stat>) => r2([...m.values()].reduce((a, s) => a + kbps(s), 0));
const POSE = ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3"];
const poseLoss = (m: Map<string, Stat>) => r2(POSE.map((t) => loss(get(m, t))).reduce((a, b) => a + b, 0) / 4);

console.error(`[qos] warming up (mode=${MODE})…`);
await measure(["/bench/p0", "/bench/img"], 12000);

if (MODE === "basic") {
  const imgFull = get(await measure(["/bench/img"], DUR), "/bench/img");
  const imgCap = get(await measure(["/bench/img"], DUR, (c) => c.topic("/bench/img").setRateLimit(2)), "/bench/img");
  console.log(`### rate-limit (heavy lidar /bench/img)`);
  console.log(`| mode | hz | kB/s |`);
  console.log(`|---|--:|--:|`);
  console.log(`| full | ${hz(imgFull)} | ${kbps(imgFull)} |`);
  console.log(`| setRateLimit(2) | ${hz(imgCap)} | ${kbps(imgCap)} |`);
  console.log(`→ ${r2(kbps(imgFull) / Math.max(1, kbps(imgCap)))}× less bandwidth, client-driven.\n`);

  const one = sumK(await measure(["/bench/p0"], DUR));
  const four = sumK(await measure(POSE, DUR));
  console.log(`### on-demand (pose topics)`);
  console.log(`1 of 4: ${one} kB/s · all 4: ${four} kB/s → ${r2((1 - one / four) * 100)}% reduction\n`);
} else {
  const full = await measure([...POSE, "/bench/img"], DUR);
  const cap = await measure([...POSE, "/bench/img"], DUR, (c) => c.topic("/bench/img").setRateLimit(2));
  console.log(`### prioritization — rate-limit lidar so pose survives (link saturated)`);
  console.log(`| lidar | pose loss% | pose hz | lidar kB/s |`);
  console.log(`|---|--:|--:|--:|`);
  console.log(`| full | ${poseLoss(full)} | ${r2(POSE.map((t) => hz(get(full, t))).reduce((a, b) => a + b, 0))} | ${kbps(get(full, "/bench/img"))} |`);
  console.log(`| capped(2Hz) | ${poseLoss(cap)} | ${r2(POSE.map((t) => hz(get(cap, t))).reduce((a, b) => a + b, 0))} | ${kbps(get(cap, "/bench/img"))} |`);
}
Deno.exit(0);
