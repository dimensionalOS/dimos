#!/usr/bin/env -S deno run --allow-all --unstable-net

/**
 * Loopback Test — Verify bridge + DimSim sensor pipeline end-to-end.
 *
 * Connects to the bridge server's WebSocket (same as the browser does) and:
 * 1. Publishes /cmd_vel Twist packets (simulates dimos nav stack sending velocity commands)
 * 2. Listens for /odom, /camera/image, /camera/depth, /lidar/points packets back
 * 3. Reports what it receives
 *
 * Usage:
 *   1. Start bridge: deno run --allow-all --unstable-net dimos-cli/cli.ts dev
 *   2. Open http://localhost:8090 in Chrome
 *   3. Run this:  deno run --allow-all --unstable-net dimos-cli/test/loopback.ts
 */

import { encodePacket, decodePacket, geometry_msgs, std_msgs } from "@dimos/msgs";

const WS_URL = Deno.args.find((_a, i, arr) => arr[i - 1] === "--ws") || "ws://localhost:8090";

console.log(`[loopback] Connecting to bridge at ${WS_URL}...`);

const ws = new WebSocket(WS_URL);
ws.binaryType = "arraybuffer";

const received = { odom: 0, image: 0, depth: 0, lidar: 0 };
let tick = 0;
let cmdInterval: ReturnType<typeof setInterval> | null = null;

ws.onopen = () => {
  console.log("[loopback] Connected to bridge WebSocket");

  // Publish cmd_vel at 10Hz (agent walks forward while slowly turning)
  cmdInterval = setInterval(() => {
    const twist = new geometry_msgs.Twist({
      linear: new geometry_msgs.Vector3({ x: 0, y: 0, z: 0.5 }),   // forward 0.5 m/s
      angular: new geometry_msgs.Vector3({ x: 0, y: 0.3, z: 0 }),   // turn 0.3 rad/s
    });

    const packet = encodePacket("/cmd_vel#geometry_msgs.Twist", twist);
    ws.send(packet);
    tick++;

    if (tick <= 3 || tick % 20 === 0) {
      console.log(`[loopback] Sent cmd_vel #${tick}: linZ=0.5 angY=0.3`);
    }
  }, 100);
};

ws.onmessage = (event: MessageEvent) => {
  if (!(event.data instanceof ArrayBuffer)) return;

  try {
    const { channel, data } = decodePacket(new Uint8Array(event.data));

    if (channel.includes("/odom")) {
      received.odom++;
      if (received.odom <= 3 || received.odom % 10 === 0) {
        const pos = data.pose?.position;
        const posStr = pos ? `x=${pos.x.toFixed(2)} y=${pos.y.toFixed(2)} z=${pos.z.toFixed(2)}` : "?";
        console.log(`[loopback] Got odom #${received.odom}: ${posStr}`);
      }
    } else if (channel.includes("/camera/image")) {
      received.image++;
      if (received.image <= 3 || received.image % 10 === 0) {
        console.log(`[loopback] Got RGB #${received.image}`);
      }
    } else if (channel.includes("/camera/depth")) {
      received.depth++;
      if (received.depth <= 3 || received.depth % 10 === 0) {
        console.log(`[loopback] Got depth #${received.depth}`);
      }
    } else if (channel.includes("/lidar/points")) {
      received.lidar++;
      if (received.lidar <= 3 || received.lidar % 10 === 0) {
        console.log(`[loopback] Got LiDAR #${received.lidar}`);
      }
    }
  } catch {
    // not a valid LCM packet
  }
};

ws.onerror = (e) => {
  console.error("[loopback] WebSocket error:", e);
};

ws.onclose = () => {
  console.log("[loopback] WebSocket closed");
  if (cmdInterval) clearInterval(cmdInterval);
};

// Status report every 5s
const statusInterval = setInterval(() => {
  console.log(`[loopback] STATUS: cmd_sent=${tick} odom=${received.odom} rgb=${received.image} depth=${received.depth} lidar=${received.lidar}`);
  if (received.odom > 0 && received.image > 0 && received.depth > 0 && received.lidar > 0) {
    console.log("\n[loopback] SUCCESS: All channels working! (odom + RGB + depth + LiDAR)");
    cleanup(0);
  }
}, 5000);

// Timeout after 60s
setTimeout(() => {
  console.log("\n[loopback] TIMEOUT after 60s");
  console.log(`[loopback] Final: cmd_sent=${tick} odom=${received.odom} rgb=${received.image} depth=${received.depth} lidar=${received.lidar}`);
  if (received.image === 0 && received.depth === 0 && received.lidar === 0) {
    console.log("[loopback] No sensor data received. Make sure:");
    console.log("  1. Bridge server is running: deno run --allow-all dimos-cli/cli.ts dev");
    console.log("  2. Browser is open at http://localhost:8090");
    console.log("  3. DimSim loaded the scene (check browser console for [dimos] logs)");
  }
  cleanup(1);
}, 60000);

function cleanup(code: number) {
  if (cmdInterval) clearInterval(cmdInterval);
  clearInterval(statusInterval);
  ws.close();
  Deno.exit(code);
}
