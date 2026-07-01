// Headless SDK smoke test (M2 verify): use @dimos/topics against the live gateway.
// RUN:  deno run -A bench/sdk_smoke.ts
import { connect } from "../packages/topics/src/index.ts";

const url = Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8080/ws";
const client = await connect({ url, reconnect: false });
console.log("[smoke] connected", url);

let firstOdom = true;
const odom = client.topic("/odom");
odom.subscribeLatest((data: any, meta) => {
  if (firstOdom) {
    firstOdom = false;
    console.log(
      `[smoke] /odom first msg: pos=(${data?.pose?.position?.x?.toFixed?.(2)}, ${
        data?.pose?.position?.y?.toFixed?.(2)
      })`,
      `header=${JSON.stringify(data?.header)?.slice(0, 120)}`,
      `latencyMs=${meta.latencyMs?.toFixed?.(1) ?? "n/a"}`,
    );
  }
});

const map = client.topic("/map");
let gotMap = false;
map.subscribeLatest((data: any) => {
  if (!gotMap) {
    gotMap = true;
    console.log(
      `[smoke] /map first msg: ${data?.info?.width}x${data?.info?.height} res=${data?.info?.resolution}`,
    );
  }
});

setTimeout(() => {
  console.log(
    "[smoke] topics:",
    client.listTopics().map((t) => `${t.topic} (${t.type})`).join(", "),
  );
  console.log("[smoke] /odom stats:", JSON.stringify(odom.stats()));
  console.log("[smoke] /map  stats:", JSON.stringify(map.stats()));
  const ok = odom.stats().count > 0;
  console.log(ok ? "[smoke] OK ✅ SDK decodes + stats work" : "[smoke] FAIL ❌");
  client.close();
  Deno.exit(ok ? 0 : 1);
}, 4000);
