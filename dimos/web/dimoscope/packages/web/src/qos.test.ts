// Unit tests for the QoS lanes — sane defaults (auto-assign) + configurable (override) + cap degradation.
import assert from "node:assert/strict";

import { defaultLane, LANES, PRIORITY_RANK, resolveQos } from "./qos.ts";

Deno.test("defaultLane: commands win", () => {
  assert.equal(defaultLane("/cmd_vel"), "command");
  assert.equal(defaultLane("/robot/teleop"), "command");
  assert.equal(defaultLane("/clicked_point"), "command");
  assert.equal(defaultLane("/anything", "geometry_msgs.Twist"), "command");
});

Deno.test("defaultLane: heavy streams → bulk (even with a sensor-ish name)", () => {
  assert.equal(defaultLane("/lidar"), "bulk");
  assert.equal(defaultLane("/camera/image"), "bulk");
  assert.equal(defaultLane("/scan"), "bulk");
  assert.equal(defaultLane("/x", "sensor_msgs.PointCloud2"), "bulk");
  assert.equal(defaultLane("/map"), "bulk");
});

Deno.test("defaultLane: telemetry → sensor; unknown → default", () => {
  assert.equal(defaultLane("/pose"), "sensor");
  assert.equal(defaultLane("/odom"), "sensor");
  assert.equal(defaultLane("/imu/data"), "sensor");
  assert.equal(defaultLane("/x", "nav_msgs.Odometry"), "sensor");
  assert.equal(defaultLane("/some/random/topic"), "default");
});

Deno.test("resolveQos: default preset, lane override, field override", () => {
  // no override → the default lane's preset
  assert.deepEqual(resolveQos("/pose"), LANES.sensor);
  // switch lane
  assert.equal(resolveQos("/pose", "", { lane: "command" }).priority, "critical");
  // override one field, keep the rest of the lane preset
  const q = resolveQos("/lidar", "", { maxHz: 2 });
  assert.equal(q.maxHz, 2);
  assert.equal(q.priority, "low"); // still bulk
  assert.equal(q.reliability, "best-effort");
  // lane + field together
  const c = resolveQos("/x", "", { lane: "sensor", maxHz: 30 });
  assert.equal(c.priority, "high");
  assert.equal(c.maxHz, 30);
});

Deno.test("PRIORITY_RANK orders command > sensor > default > bulk", () => {
  const rank = (l: keyof typeof LANES) => PRIORITY_RANK[LANES[l].priority!];
  assert.ok(rank("command") > rank("sensor"));
  assert.ok(rank("sensor") > rank("default"));
  assert.ok(rank("default") > rank("bulk"));
});
