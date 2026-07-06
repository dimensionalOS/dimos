// readBulkStream(): frame reassembly across chunk boundaries, and the bulk-ack credit counter
// (cumulative raw bytes, one ack per BULK_ACK_QUANTUM) the sidecar's drain gate depends on.
import assert from "node:assert/strict";

import { BULK_ACK_QUANTUM, createBulkAckCounter, readBulkStream } from "./webTransport.ts";

function streamOf(chunks: Uint8Array[]): ReadableStream<Uint8Array> {
  return new ReadableStream({
    start(controller) {
      for (const c of chunks) controller.enqueue(c);
      controller.close();
    },
  });
}

function framed(payload: Uint8Array): Uint8Array {
  const out = new Uint8Array(4 + payload.length);
  new DataView(out.buffer).setUint32(0, payload.length);
  out.set(payload, 4);
  return out;
}

Deno.test("readBulkStream reassembles frames split across chunks and reports raw bytes", async () => {
  const a = new Uint8Array(1000).fill(7);
  const b = new Uint8Array(3).fill(9);
  const wire = new Uint8Array([...framed(a), ...framed(b)]);
  // split mid-header and mid-payload
  const chunks = [wire.subarray(0, 2), wire.subarray(2, 700), wire.subarray(700)];
  const delivered: Uint8Array[] = [];
  let raw = 0;
  await readBulkStream(streamOf(chunks), (f) => delivered.push(f), (n) => raw += n);
  assert.equal(delivered.length, 2);
  assert.deepEqual(delivered[0], a);
  assert.deepEqual(delivered[1], b);
  assert.equal(raw, wire.length); // headers + payloads: credit counts every wire byte
});

Deno.test("bulk-ack counter emits cumulative totals once per quantum", () => {
  const acks: number[] = [];
  const count = createBulkAckCounter((n) => acks.push(n));
  count(BULK_ACK_QUANTUM - 1); // below quantum: silent
  assert.deepEqual(acks, []);
  count(1); // crosses: ack the cumulative total
  assert.deepEqual(acks, [BULK_ACK_QUANTUM]);
  count(BULK_ACK_QUANTUM - 1); // unacked remainder resets to 0 after an ack
  assert.deepEqual(acks, [BULK_ACK_QUANTUM]);
  count(3 * BULK_ACK_QUANTUM); // one big chunk: single ack, still cumulative
  assert.deepEqual(acks, [BULK_ACK_QUANTUM, 5 * BULK_ACK_QUANTUM - 1]);
});
