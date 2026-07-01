// Client tests — two kinds in one file:
//   1. Runtime unit tests (Deno.test) for the decode-LOCATION axis: a server-json sample carries a
//      pre-decoded `decoded` object (the gateway did the work); the client must deliver it directly and
//      bill its bandwidth as the JSON wire size. The binary path (no `decoded`) still decodes via
//      @dimos/msgs.
//   2. Compile-only type tests (the `_`-prefixed async functions at the bottom): no runtime — they're
//      type-checked by `deno task check` and never executed. They prove the client generics bite:
//      `createDimosClient<TMap, TCmds>()` gives typed topic handles (read) + a typed `client.modules`
//      RPC surface (write), while an untyped client stays permissive. The `@ts-expect-error` lines fail
//      the build if the types stop constraining (a positives-only test would pass even if all were
//      `any`). `_`-prefixed so deno-lint's no-unused-vars leaves them alone.
import assert from "node:assert/strict";

import { createDimosClient } from "./client.ts";
import type { RawSample, Transport } from "./transport.ts";

// Minimal fake transport: captures the client's onSample callback so a test can push raw samples.
function fakeTransport() {
  let sampleCb: ((s: RawSample) => void) | undefined;
  const t: Transport = {
    caps: { onDemand: true, discovery: "passive" },
    connect: () => Promise.resolve(),
    close: () => {},
    subscribe: () => {},
    unsubscribe: () => {},
    publishTeleop: () => {},
    publishGoal: () => {},
    rpc: () => Promise.resolve(undefined),
    requestList: () => {},
    onSample: (cb) => {
      sampleCb = cb;
    },
    onTopics: () => {},
    onStatus: () => {},
  };
  return { t, emit: (s: RawSample) => sampleCb?.(s) };
}

Deno.test("server-json: client delivers the pre-decoded object directly (skips decodeBody)", async () => {
  const { t, emit } = fakeTransport();
  const client = createDimosClient({ transport: () => t });
  await client.connect("test");
  let got: unknown;
  let size = -1;
  let seq = -1;
  client.topic("/bench/p0").subscribe((msg) => {
    got = msg.data;
    size = msg.meta.sizeBytes;
    seq = msg.meta.seq ?? -1;
  });
  const decoded = { frame_id: "7", ts: 1_750_000_000, position: [1, 2, 3] };
  const wire = new TextEncoder().encode(
    JSON.stringify({ op: "sample", topic: "/bench/p0", data: decoded }),
  );
  emit({
    topic: "/bench/p0",
    type: "geometry_msgs.PoseStamped",
    payload: wire,
    recvTs: Date.now(),
    gatewaySendMs: Date.now(),
    decoded,
  });
  assert.equal(got, decoded); // delivered the pre-decoded object (same reference) — no client decode
  assert.equal(size, wire.length); // bandwidth billed as the JSON wire size (the rosbridge tax)
  assert.equal(seq, 7); // seqFrom read the decoded frame_id → loss detection still works
});

Deno.test("client-binary: a sample with no `decoded` and an undecodable payload is dropped", async () => {
  const { t, emit } = fakeTransport();
  const client = createDimosClient({ transport: () => t });
  await client.connect("test");
  let calls = 0;
  client.topic("/x").subscribe(() => {
    calls++;
  });
  // 3 bytes — too short for the 8-byte type hash, so decodeBody throws → silently dropped.
  emit({ topic: "/x", type: "?", payload: new Uint8Array([1, 2, 3]), recvTs: Date.now() });
  assert.equal(calls, 0);
});

// ── Compile-only type tests (no runtime; picked up by `deno task check`, not by `deno test`) ──────────

// Topics/read side: `createDimosClient<TMap>()` gives typed + autocompleting topic handles for known
// names, yields Topic<unknown> for runtime names, and leaves an untyped client behaving as before.
type Fixture = { "/odom": { x: number }; "/map": { w: number } };

async function _typed() {
  const c = createDimosClient<Fixture>();
  await c.connect("x");

  // known key → autocompletes name, infers the mapped message type
  const odom = c.topic("/odom");
  const _pose: { x: number } | undefined = odom.getLatest();
  // @ts-expect-error — the mapped type is {x:number}, not {y:number}
  const _wrong: { y: number } | undefined = c.topic("/odom").getLatest();

  // known key → the flat subscribe delivers a typed Message<{x:number}>
  c.subscribe("/odom", (m) => {
    const _x: number = m.data.x;
  });

  // runtime-discovered name → still allowed, Topic<unknown>
  const _u: unknown = c.topic("/not_in_map").getLatest();

  // no hand-typed escape hatch: the explicit message-type form is gone (K must be a string key)
  // @ts-expect-error — `topic<Msg>()` removed; the type param is the name key, not the message type
  c.topic<{ z: number }>("/z");
}

async function _untyped() {
  const c = createDimosClient(); // no TMap → every topic is Topic<unknown>
  await c.connect("x");
  const _u: unknown = c.topic("/anything").getLatest();
}

// Modules/RPC-write side: a real `DimosCommands`-shaped map makes `client.modules.<Target>.<method>`
// autocompleted + typo-checked and lifted to a callable, while an untyped client stays permissive (the
// `client.call` escape hatch). Mirrors what `generateCommandTypes` (packages/topics/scripts/genTypes.ts)
// emits: target → method → RpcCall descriptor.
type Commands = {
  ScopeNav: {
    start: { args: unknown[]; ret: unknown };
    stop: { args: unknown[]; ret: unknown };
  };
};

async function _typedModules() {
  const c = createDimosClient<Record<never, never>, Commands>();
  await c.connect("x");

  // known target + method → lifted to a callable returning a Promise
  const _ret: Promise<unknown> = c.modules.ScopeNav.start();
  await c.modules.ScopeNav.stop();

  // @ts-expect-error — unknown method on a known target
  c.modules.ScopeNav.fly();
  // @ts-expect-error — unknown target
  c.modules.NotAModule.start();
}

async function _untypedModules() {
  const c = createDimosClient(); // no TCmds → permissive ModuleMap (escape hatch preserved)
  await c.connect("x");
  const _any: Promise<unknown> = c.modules.Anything.anything();
}
