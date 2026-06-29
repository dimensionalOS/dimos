// ─────────────────────────────────────────────────────────────────────────
// dimoscope HEADLESS PROBE  (how I verify the pipe without a browser)
//
// Acts like the browser: connects to the bridge over WebSocket, decodes every
// packet with @dimos/msgs, and after 3s prints which topics it saw + a sample
// field. Exits non-zero if nothing arrived. This proves: mock → bridge → ws →
// decode works end to end.
//
// RUN (with bridge + mockrobot already running):  bun run wsprobe.ts
// ─────────────────────────────────────────────────────────────────────────
import { decodePacket } from "@dimos/msgs";

const URL = process.env.WS_URL ?? "ws://localhost:8080";
const seen = new Map<string, number>();
let sample: Record<string, unknown> = {};

const ws = new WebSocket(URL);
ws.binaryType = "arraybuffer";
ws.onopen = () => console.log(`[probe] connected ${URL}`);
ws.onerror = (e) => console.error("[probe] ws error", e);
ws.onmessage = (e) => {
  try {
    const { channel, data } = decodePacket(new Uint8Array(e.data as ArrayBuffer));
    seen.set(channel, (seen.get(channel) ?? 0) + 1);
    if (channel.startsWith("/odom")) {
      const p = (data as any).pose.position;
      sample = { odom_xyz: [p.x.toFixed(2), p.y.toFixed(2), p.z.toFixed(2)] };
    }
  } catch {
    /* fragmented/unknown — skip */
  }
};

setTimeout(() => {
  console.log("[probe] topics seen in 3s:");
  for (const [c, n] of seen) console.log(`   ${c.padEnd(40)} ${n} msgs`);
  console.log("[probe] sample:", sample);
  process.exit(seen.size > 0 ? 0 : 1);
}, 3000);
