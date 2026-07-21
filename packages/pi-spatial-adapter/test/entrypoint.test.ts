import test from "node:test";
import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { resolve } from "node:path";

test("compiled entrypoint emits a protocol error when stdin reaches EOF", () => {
  const result = spawnSync(process.execPath, [resolve("dist/main.js")], { input: "", encoding: "utf8" });
  assert.equal(result.error, undefined);
  assert.equal(result.status, 0);
  const frames = result.stdout.trim().split("\n").filter(Boolean).map((line) => JSON.parse(line) as Record<string, unknown>);
  assert.deepEqual(frames, [{ version: 2, type: "protocol_error", error: "missing run_start frame" }]);
});
