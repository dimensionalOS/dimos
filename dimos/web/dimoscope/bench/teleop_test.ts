// Verify the full teleop loop: browser SDK → gateway → /cmd_vel → simplerobot
// moves → /odom updates. Drives forward at 0.5 m/s for `DRIVE_MS`, prints the
// robot's x before/after. RUN: deno run -A bench/teleop_test.ts
import { createDimosClient, ws } from "../packages/topics/src/index.ts";

const DRIVE_MS = Number(Deno.env.get("DRIVE_MS") ?? 4000);
const client = createDimosClient({ transport: ws({ reconnect: false }) });
await client.connect(Deno.env.get("GATEWAY_URL") ?? "ws://localhost:8080/ws");
let x = 0,
  y = 0;
client.topic(Deno.env.get("ODOM_TOPIC") ?? "/odom").subscribeLatest((m) => {
  const d = m.data as any;
  x = d?.pose?.position?.x ?? 0;
  y = d?.pose?.position?.y ?? 0;
});

await new Promise((r) => setTimeout(r, 300));
const x0 = x;
console.log(`[teleop] start x=${x0.toFixed(3)} — driving forward 0.5 m/s for ${DRIVE_MS}ms`);
const LIN = Number(Deno.env.get("LIN") ?? 0.5),
  ANG = Number(Deno.env.get("ANG") ?? 0);
const id = setInterval(() => client.teleop(LIN, ANG, 300), 100); // 10 Hz sustained

setTimeout(() => {
  clearInterval(id);
  client.stop();
  setTimeout(() => {
    const moved = Math.hypot(x - x0, y);
    console.log(
      `[teleop] end   x=${x.toFixed(3)} y=${y.toFixed(3)}  → moved ${moved.toFixed(3)} m`,
    );
    console.log(
      moved > 0.05 ? "[teleop] OK ✅ robot moved via browser-SDK teleop" : "[teleop] no motion ❌",
    );
    client.close();
    Deno.exit(moved > 0.05 ? 0 : 1);
  }, 600);
}, DRIVE_MS);
