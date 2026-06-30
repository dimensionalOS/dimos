// Unit tests for the QoS lanes — sane defaults (auto-assign) + configurable (override) + cap degradation.
import assert from "node:assert/strict";

import { applyCaps, defaultLane, LANES, PRIORITY_RANK, resolveQos } from "./qos.ts";
import type { QosCaps } from "./types.ts";

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

Deno.test("applyCaps: keeps gateway fields + advertised transport fields, drops the rest", () => {
  const qos = resolveQos("/pose", "", { maxHz: 30, ordered: false, maxRetransmits: 0 });
  // a WS-like transport: honors nothing transport-level
  const ws: QosCaps = { maxHz: "server" };
  const wq = applyCaps(qos, ws);
  assert.equal(wq.maxHz, 30); // gateway field kept
  assert.equal(wq.priority, "high"); // scheduler field kept
  assert.equal(wq.conflation, "latest");
  assert.equal("ordered" in wq, false); // transport field dropped
  assert.equal("reliability" in wq, false);
  // a WebRTC-like transport: honors reliability + ordered + lifetime
  const rtc: QosCaps = { maxHz: "client", transport: ["reliability", "ordered", "maxRetransmits"] };
  const rq = applyCaps(qos, rtc);
  assert.equal(rq.ordered, false); // now kept
  assert.equal(rq.reliability, "best-effort");
  assert.equal(rq.priority, "high"); // gateway field still kept
});
