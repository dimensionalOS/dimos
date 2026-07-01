// Unit tests for the decode-LOCATION axis in the client: a server-json sample carries a pre-decoded
// `decoded` object (the gateway did the work); the client must deliver it directly and bill its
// bandwidth as the JSON wire size. The binary path (no `decoded`) still decodes via @dimos/msgs.
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
