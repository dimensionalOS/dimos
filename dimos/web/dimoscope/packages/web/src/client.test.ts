// Two kinds of test: (1) runtime unit tests (Deno.test) via a pre-decoded `decoded` test-injection seam
// (binary path decodes via @dimos/msgs); (2) compile-only type tests (`_`-prefixed async fns, checked by
// `deno task check`, never run) using `@ts-expect-error` to prove the generics constrain. `_`-prefix
// dodges no-unused-vars.
import assert from "node:assert/strict";

import { createDimosClient, seqFrom, srcTsMs } from "./client.ts";
import type { RawSample, Transport } from "./types.ts";

// ── Message-metadata heuristics (the seq/timestamp parsing the bench relies on) ──────────────────
Deno.test("seqFrom: numeric frame_id → counter; names/garbage → undefined", () => {
  assert.equal(seqFrom({ frame_id: "42" }), 42);
  assert.equal(seqFrom({ frame_id: "0" }), 0);
  assert.equal(seqFrom({ frame_id: "base_link" }), undefined);
  assert.equal(seqFrom({ frame_id: "" }), undefined);
  assert.equal(seqFrom({ frame_id: "12345678901234567890" }), undefined); // >=16 chars guarded
  assert.equal(seqFrom({ header: { seq: 7 } }), 7); // header.seq fallback
  assert.equal(seqFrom({}), undefined);
  assert.equal(seqFrom(null), undefined);
  assert.equal(seqFrom(undefined), undefined);
});

Deno.test("srcTsMs: seconds / ms / ns heuristics + {sec,nsec} struct", () => {
  assert.equal(srcTsMs({ ts: 1_750_000_000 }), 1_750_000_000_000); // seconds → ms
  assert.equal(srcTsMs({ ts: 1_750_000_000_000 }), 1_750_000_000_000); // already ms
  assert.equal(srcTsMs({ ts: 1_750_000_000_000_000_000 }), 1_750_000_000_000); // ns → ms
  assert.equal(srcTsMs({ header: { stamp: { sec: 2, nsec: 500_000_000 } } }), 2500);
  assert.equal(srcTsMs({}), undefined);
});

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

Deno.test("pre-decoded: client delivers an injected `decoded` object directly (no client decode)", async () => {
  const { t, emit } = fakeTransport();
  const client = createDimosClient({ transport: () => t });
  await client.connect("test");
  let got: unknown;
  let size = -1;
  let seq = -1;
  client.topic("/load/fast").subscribe((msg) => {
    got = msg.data;
    size = msg.meta.sizeBytes;
    seq = msg.meta.seq ?? -1;
  });
  const decoded = { frame_id: "7", ts: 1_750_000_000, position: [1, 2, 3] };
  const wire = new TextEncoder().encode(
    JSON.stringify({ op: "sample", topic: "/load/fast", data: decoded }),
  );
  emit({
    topic: "/load/fast",
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
  // 3 bytes — too short for the 8-byte type hash, so decode throws → silently dropped.
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
// `client.call` escape hatch). Mirrors what `generateCommandTypes` (packages/web/scripts/gen_types.py)
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
