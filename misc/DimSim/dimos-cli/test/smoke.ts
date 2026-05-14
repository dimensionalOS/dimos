#!/usr/bin/env -S deno run --allow-all --unstable-net

/**
 * Smoke Test — Verify DimSim sensor data flows over LCM.
 *
 * 1. Subscribes to /camera/image, /camera/depth, /lidar/points
 * 2. Publishes test /odom messages
 * 3. Waits for all sensor types to be received
 * 4. Exits 0 on success, 1 on timeout
 *
 * Requires: DimSim running in dimos mode + bridge server active.
 */

import { LCM } from "@dimos/lcm";
import { geometry_msgs, sensor_msgs, std_msgs } from "@dimos/msgs";

const TIMEOUT_MS = 30000;

const lcm = new LCM();
await lcm.start();

let receivedImage = false;
let receivedDepth = false;
let receivedLidar = false;

lcm.subscribe("/camera/image", sensor_msgs.Image, (msg: { data: { width: number; height: number; encoding: string } }) => {
  console.log(`[smoke] Got RGB: ${msg.data.width}x${msg.data.height} enc=${msg.data.encoding}`);
  receivedImage = true;
});

lcm.subscribe("/camera/depth", sensor_msgs.Image, (msg: { data: { width: number; height: number; encoding: string } }) => {
  console.log(`[smoke] Got depth: ${msg.data.width}x${msg.data.height} enc=${msg.data.encoding}`);
  receivedDepth = true;
});

lcm.subscribe("/lidar/points", sensor_msgs.PointCloud2, (msg: { data: { width: number; point_step: number } }) => {
  console.log(`[smoke] Got LiDAR: ${msg.data.width} points, ${msg.data.point_step} bytes/pt`);
  receivedLidar = true;
});

// Publish test odom at 10 Hz to drive the agent
const odomInterval = setInterval(async () => {
  const pose = new geometry_msgs.PoseStamped({
    header: new std_msgs.Header({
      stamp: new std_msgs.Time({ sec: 0, nsec: 0 }),
      frame_id: "map",
    }),
    pose: new geometry_msgs.Pose({
      position: new geometry_msgs.Point({ x: 0, y: 0.5, z: 0 }),
      orientation: new geometry_msgs.Quaternion({ x: 0, y: 0, z: 0, w: 1 }),
    }),
  });
  await lcm.publish("/odom", pose);
}, 100);

// Poll for results
const deadline = Date.now() + TIMEOUT_MS;
const checkInterval = setInterval(() => {
  if (receivedImage && receivedDepth && receivedLidar) {
    clearInterval(odomInterval);
    clearInterval(checkInterval);
    console.log("\n[smoke] PASS: All sensor types received");
    Deno.exit(0);
  }
  if (Date.now() > deadline) {
    clearInterval(odomInterval);
    clearInterval(checkInterval);
    console.error("\n[smoke] FAIL: Timeout. Missing sensors:", {
      image: receivedImage,
      depth: receivedDepth,
      lidar: receivedLidar,
    });
    Deno.exit(1);
  }
}, 500);
