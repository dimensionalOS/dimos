// Headless gateway probe — verifies the gateway decodes the live LCM bus.
// RUN:  bun run servers/probe.ts            (env GATEWAY_URL, default ws://localhost:8090)
import { decodePacket } from "@dimos/msgs";

const url = process.env.GATEWAY_URL ?? "ws://localhost:8090";
const seen = new Map<string, number>();
const ws = new WebSocket(url);
ws.binaryType = "arraybuffer";

ws.addEventListener("open", () => {
  console.log("[probe] connected", url);
  ws.send(JSON.stringify({ op: "subscribe", topic: "*" }));
  ws.send(JSON.stringify({ op: "list" }));
});
ws.addEventListener("error", (e) => console.error("[probe] ws error", (e as any).message ?? e));

ws.addEventListener("message", (ev) => {
  if (typeof ev.data === "string") {
    console.log("[probe] ctrl:", ev.data.slice(0, 240));
    return;
  }
  try {
    const { channel, data } = decodePacket(new Uint8Array(ev.data as ArrayBuffer));
    const n = (seen.get(channel) ?? 0) + 1;
    seen.set(channel, n);
    if (n <= 1) {
      const keys =
        data && typeof data === "object" ? Object.keys(data as object).slice(0, 8) : [];
      console.log(`[probe] FIRST ${channel}  fields=[${keys.join(",")}]`);
    }
  } catch {
    /* unknown / still-fragmented */
  }
});

setTimeout(() => {
  console.log("[probe] ---- counts per channel (≈rate over 4s) ----");
  for (const [c, n] of seen) console.log(`   ${c}: ${n}`);
  console.log(seen.size ? "[probe] OK ✅ gateway is decoding the bus" : "[probe] NO DATA ❌");
  process.exit(seen.size ? 0 : 1);
}, 4000);
