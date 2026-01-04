#!/usr/bin/env -S deno run --allow-net --unstable-net

// TypeScript robot control example
// Subscribes to robot odometry and publishes twist commands

import { LCM } from "@dimos/lcm";
import { geometry_msgs } from "@dimos/msgs";

const lcm = new LCM();
await lcm.start();

console.log("Robot control started");
console.log("Subscribing to /odom, publishing to /cmd_vel");
console.log("Press Ctrl+C to stop.\n");

// Subscribe to pose - prints robot position
lcm.subscribe("/odom", geometry_msgs.PoseStamped, (msg) => {
  const pos = msg.data.pose.position;
  const ori = msg.data.pose.orientation;
  console.log(
    `[pose] x=${pos.x.toFixed(2)} y=${pos.y.toFixed(2)} z=${pos.z.toFixed(2)} | qw=${ori.w.toFixed(2)}`
  );
});

// Publish twist commands at 10 Hz - simple forward motion
let t = 0;
const interval = setInterval(async () => {
  if (!lcm.isRunning()) {
    clearInterval(interval);
    return;
  }

  const twist = new geometry_msgs.Twist({
    linear: new geometry_msgs.Vector3({ x: 0.5, y: 0, z: 0 }),
    angular: new geometry_msgs.Vector3({ x: 0, y: 0, z: Math.sin(t) * 0.3 }),
  });

  await lcm.publish("/cmd_vel", twist);
  console.log(`[twist] linear=${twist.linear.x.toFixed(2)} angular=${twist.angular.z.toFixed(2)}`);
  t += 0.1;
}, 100);

await lcm.run();
