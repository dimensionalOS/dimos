// Unit tests for the finalized client surface: the flat `subscribe` / `subscribeAll` / `peek` /
// `latest` methods + the `modules` RPC Proxy, all over the kept `topic()` engine. Uses a fake
// Transport (via a factory + connect(), the only wiring path) so no network is needed.
import assert from "node:assert/strict";

import { createDimosClient } from "./client.ts";
import type { RawSample, Transport } from "./types.ts";

function fakeTransport() {
  let sampleCb: ((s: RawSample) => void) | undefined;
  const rpcCalls: Array<{ target: string; method: string; args: unknown[] }> = [];
  const subs: string[] = [];
  const t: Transport = {
    caps: { onDemand: true, discovery: "passive" },
    connect: () => Promise.resolve(),
    close: () => {},
    subscribe: (topic) => void subs.push(topic),
    unsubscribe: () => {},
    publishTeleop: () => {},
    publishGoal: () => {},
    rpc: (target, method, args) => {
      rpcCalls.push({ target, method, args: args ?? [] });
      return Promise.resolve(`${target}.${method}(${(args ?? []).join(",")})`);
    },
    requestList: () => {},
    onSample: (cb) => void (sampleCb = cb),
    onTopics: () => {},
    onStatus: () => {},
  };
  const emit = (topic: string, decoded: unknown) =>
    sampleCb?.({
      topic,
      type: "geometry_msgs.PoseStamped",
      payload: new TextEncoder().encode(JSON.stringify(decoded)),
      recvTs: Date.now(),
      gatewaySendMs: Date.now(),
      decoded,
    });
  return { t, emit, rpcCalls, subs };
}

/** Build + connect a client over the fake transport (factory-only wiring). */
async function connected() {
  const fake = fakeTransport();
  const dimos = createDimosClient({ transport: () => fake.t });
  await dimos.connect("test");
  return { dimos, ...fake };
}

Deno.test("flat subscribe delivers a { data, ts, meta } envelope", async () => {
  const { dimos, emit } = await connected();
  let got: { data: unknown; ts: number; metaTopic: string } | undefined;
  dimos.subscribe("/odom", (m) => {
    got = { data: m.data, ts: m.ts, metaTopic: m.meta.topic };
  });
  const decoded = { frame_id: "3", ts: 1_750_000_000, position: [1, 2, 3] };
  emit("/odom", decoded);
  assert.equal(got?.data, decoded);
  assert.equal(typeof got?.ts, "number");
  assert.equal(got?.metaTopic, "/odom");
});

Deno.test("subscribeAll is a firehose across topics; latest() returns the last value", async () => {
  const { dimos, emit, subs } = await connected();
  const seen: string[] = [];
  const sub = dimos.subscribeAll((m) => seen.push(m.meta.topic));
  emit("/a", { ts: 1, v: 1 });
  emit("/b", { ts: 2, v: 2 });
  assert.deepEqual(seen, ["/a", "/b"]);
  assert.ok(subs.includes("*")); // asked the gateway for the wildcard
  assert.deepEqual(dimos.latest("/b"), { ts: 2, v: 2 });
  sub.unsubscribe();
});

Deno.test("peek resolves with the next message", async () => {
  const { dimos, emit } = await connected();
  const p = dimos.peek("/odom");
  emit("/odom", { ts: 5, position: [9, 9, 9] });
  const m = await p;
  assert.deepEqual(m.data, { ts: 5, position: [9, 9, 9] });
});

Deno.test("modules Proxy maps module.method(args) to an rpc call", async () => {
  const { dimos, rpcCalls } = await connected();
  const res = await dimos.modules.GO2Connection.standup(1, 2);
  assert.equal(res, "GO2Connection.standup(1,2)");
  assert.deepEqual(rpcCalls, [{ target: "GO2Connection", method: "standup", args: [1, 2] }]);
});
