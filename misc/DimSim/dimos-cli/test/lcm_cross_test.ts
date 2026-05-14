#!/usr/bin/env -S deno run --allow-all --unstable-net
/**
 * Quick test: can Deno send LCM messages that Python receives?
 * Publishes raw packets on /lcm_cross_test channel.
 */
import { LCM } from "@dimos/lcm";
import { geometry_msgs } from "@dimos/msgs";

const lcm = new LCM();
await lcm.start();

// Also subscribe to see if we get Python's messages
lcm.subscribe("/lcm_cross_test", geometry_msgs.Twist, (msg: any) => {
  console.log(`[deno] Got message back:`, msg.data?.linear);
});

console.log("[deno] Using patched @dimos/lcm with joinMulticastV4");
console.log("[deno] Publishing test messages on /lcm_cross_test...\n");

let count = 0;
const interval = setInterval(async () => {
  count++;
  const twist = new geometry_msgs.Twist({
    linear: new geometry_msgs.Vector3({ x: count, y: 0, z: 0 }),
    angular: new geometry_msgs.Vector3({ x: 0, y: 0, z: 0 }),
  });
  await lcm.publish("/lcm_cross_test", twist);
  console.log(`[deno] Sent message #${count}`);

  if (count >= 10) {
    clearInterval(interval);
    console.log("\n[deno] Done publishing. Waiting 2s for responses...");
    setTimeout(() => Deno.exit(0), 2000);
  }
}, 500);

await lcm.run();
