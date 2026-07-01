// Unit tests for the decode helpers — the seq/timestamp parsing the bench relies on.
// Run: `deno task test`  (or `deno test -A --node-modules-dir`).
import assert from "node:assert/strict";

import { seqFrom, srcTsMs } from "./decode.ts";

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
