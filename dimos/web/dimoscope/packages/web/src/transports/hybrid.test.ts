// hybrid(): lane routing (sensor→WT datagrams, bulk/command/default→WS), WS as the control wire,
// WT-death rebinding, and pre-WT subscriptions migrating once WT comes up.
import assert from "node:assert/strict";

import { hybrid, laneWire } from "./hybrid.ts";
import type { RawSample, Status, TopicInfo, Transport } from "../types.ts";

function fakeWire(name: string, opts?: { failConnect?: boolean }) {
  const subs = new Set<string>();
  const calls: string[] = [];
  let statusCb: ((s: Status) => void) | undefined;
  let topicsCb: ((t: TopicInfo[]) => void) | undefined;
  const t: Transport = {
    caps: { onDemand: true, discovery: "live" },
    label: name,
    connect: () => opts?.failConnect ? Promise.reject(new Error("nope")) : Promise.resolve(),
    close: () => statusCb?.("closed"),
    subscribe: (topic) => {
      subs.add(topic);
      calls.push(`sub:${topic}`);
    },
    unsubscribe: (topic) => {
      subs.delete(topic);
      calls.push(`unsub:${topic}`);
    },
    publishTeleop: () => calls.push("teleop"),
    publishGoal: () => calls.push("goal"),
    rpc: () => {
      calls.push("rpc");
      return Promise.resolve("ok");
    },
    requestList: () => {},
    onSample: (_cb: (s: RawSample) => void) => {},
    onTopics: (cb) => void (topicsCb = cb),
    onStatus: (cb) => void (statusCb = cb),
  };
  return {
    t,
    subs,
    calls,
    emitTopics: (l: TopicInfo[]) => topicsCb?.(l),
    die: () => statusCb?.("closed"),
  };
}

Deno.test("laneWire: only the sensor lane rides datagrams", () => {
  assert.equal(laneWire("sensor"), "wt");
  assert.equal(laneWire("command"), "ws");
  assert.equal(laneWire("bulk"), "ws");
  assert.equal(laneWire("default"), "ws");
});

Deno.test("hybrid: routes by lane, control on WS, WT death rebinds to WS", async () => {
  const wsW = fakeWire("gw");
  const wtW = fakeWire("gw/WT");
  const t = hybrid({ _mkWires: () => ({ ws: wsW.t, wt: wtW.t }) })("host:8080");
  await t.connect();

  // discovery gives lane inference the message types
  wsW.emitTopics([
    { topic: "/load/fast", type: "geometry_msgs.PoseStamped" },
    { topic: "/load/img", type: "sensor_msgs.Image" },
    { topic: "/cmd_vel", type: "geometry_msgs.Twist" },
  ]);

  t.subscribe("/load/fast"); // Pose → sensor lane → WT datagrams
  t.subscribe("/load/img"); // Image → bulk lane → WS
  t.subscribe("/cmd_vel"); // Twist → command lane → WS
  assert.ok(wtW.subs.has("/load/fast"));
  assert.ok(wsW.subs.has("/load/img"));
  assert.ok(wsW.subs.has("/cmd_vel"));
  assert.ok(!wsW.subs.has("/load/fast"));

  // teleop/rpc ride the WS control wire only
  t.publishTeleop(0.5, 0);
  await t.rpc("GO2Load", "status");
  assert.ok(wsW.calls.includes("teleop") && wsW.calls.includes("rpc"));
  assert.ok(!wtW.calls.includes("teleop") && !wtW.calls.includes("rpc"));

  assert.equal(t.label, "gw/HYB(WS+WT)");

  // WT dies mid-session → its topics carry on over WS, label reflects the degraded mode
  wtW.die();
  assert.ok(wsW.subs.has("/load/fast"));
  assert.equal(t.label, "gw/HYB(WS-only)");

  // unsubscribe cleans the (now-WS) binding
  t.unsubscribe("/load/fast");
  assert.ok(!wsW.subs.has("/load/fast"));
});

Deno.test("hybrid: WT connect failure → everything on WS (no throw)", async () => {
  const wsW = fakeWire("gw");
  const wtW = fakeWire("gw/WT", { failConnect: true });
  const t = hybrid({ _mkWires: () => ({ ws: wsW.t, wt: wtW.t }) })("host:8080");
  await t.connect();
  t.subscribe("/odom"); // sensor by name, but WT never opened → WS
  assert.ok(wsW.subs.has("/odom"));
  assert.equal(t.label, "gw/HYB(WS-only)");
});
