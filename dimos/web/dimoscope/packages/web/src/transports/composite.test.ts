// webtransport() Auto fallback: WT connect failure → WS, WT death mid-session → WS (with the
// tracked subscriptions replayed either way), and the no-WebTransport → plain-WS path.
import assert from "node:assert/strict";

import { webtransport } from "./composite.ts";
import type { Qos, Status, Transport } from "../types.ts";

function fakeWire(name: string, opts?: { failConnect?: boolean }) {
  const subs = new Map<string, Qos | undefined>();
  let statusCb: ((s: Status) => void) | undefined;
  const t: Transport = {
    caps: { onDemand: true, discovery: "live" },
    label: name,
    connect: () => opts?.failConnect ? Promise.reject(new Error("nope")) : Promise.resolve(),
    close: () => statusCb?.("closed"),
    subscribe: (topic, qos) => void subs.set(topic, qos),
    unsubscribe: (topic) => void subs.delete(topic),
    publishTeleop: () => {},
    publishGoal: () => {},
    rpc: () => Promise.resolve("ok"),
    requestList: () => {},
    onSample: () => {},
    onTopics: () => {},
    onStatus: (cb) => void (statusCb = cb),
  };
  return { t, subs, die: () => statusCb?.("closed") };
}

Deno.test("auto: WT connect failure → WS fallback, pre-connect subs replayed", async () => {
  const wt = fakeWire("WT", { failConnect: true });
  const ws = fakeWire("ws");
  const t = webtransport({ _mkWires: () => ({ wt: wt.t, mkWs: () => ws.t }), timeoutMs: 50 })(
    "host",
  );
  t.subscribe("/odom", { maxHz: 5 });
  await t.connect();
  assert.ok(ws.subs.has("/odom"));
  assert.equal(ws.subs.get("/odom")?.maxHz, 5);
  assert.equal(t.label, "ws");
});

Deno.test("auto: WT death mid-session → WS takes over, subs replayed with QoS", async () => {
  const wt = fakeWire("WT");
  const ws = fakeWire("ws");
  const t = webtransport({ _mkWires: () => ({ wt: wt.t, mkWs: () => ws.t }) })("host");
  const statuses: Status[] = [];
  t.onStatus((s) => statuses.push(s));
  await t.connect();
  t.subscribe("/odom", { maxHz: 10 });
  t.subscribe("/scan", undefined);
  t.unsubscribe("/scan"); // dropped before the failover → must not be replayed
  assert.ok(wt.subs.has("/odom"));
  wt.die();
  assert.ok(ws.subs.has("/odom"));
  assert.equal(ws.subs.get("/odom")?.maxHz, 10);
  assert.ok(!ws.subs.has("/scan"));
  assert.equal(t.label, "ws");
  assert.ok(statuses.includes("closed")); // the drop is visible, then the fallback reopens
});

Deno.test("auto: no WebTransport wire → plain WS transport", async () => {
  const ws = fakeWire("ws");
  const t = webtransport({ _mkWires: () => ({ mkWs: () => ws.t }) })("host");
  await t.connect();
  assert.equal(t.label, "ws");
});
