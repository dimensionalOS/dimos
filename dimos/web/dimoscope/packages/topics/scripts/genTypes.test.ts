// Unit tests for the blueprint→TS codegen. Pure (no network). Run: `deno test -A packages/topics/scripts/`.
import assert from "node:assert/strict";

import { generateCommandTypes, generateTypes } from "./genTypes.ts";

const KNOWN = new Set([
  "geometry_msgs.PoseStamped",
  "nav_msgs.OccupancyGrid",
  "geometry_msgs.Twist",
]);

Deno.test("generateTypes: known types → typed, sorted, imports collected", () => {
  const out = generateTypes(
    [
      { topic: "/odom", type: "geometry_msgs.PoseStamped" },
      { topic: "/cmd_vel", type: "geometry_msgs.Twist" },
      { topic: "/map", type: "nav_msgs.OccupancyGrid" },
    ],
    KNOWN,
  );
  // one import line, only the packages actually used, sorted + de-duped
  assert.ok(out.includes(`import type { geometry_msgs, nav_msgs } from "@dimos/msgs";`));
  assert.ok(out.includes(`"/odom": geometry_msgs.PoseStamped;`));
  assert.ok(out.includes(`"/cmd_vel": geometry_msgs.Twist;`));
  assert.ok(out.includes(`"/map": nav_msgs.OccupancyGrid;`));
  // topics are emitted in sorted order (/cmd_vel before /map before /odom)
  assert.ok(out.indexOf(`"/cmd_vel"`) < out.indexOf(`"/map"`));
  assert.ok(out.indexOf(`"/map"`) < out.indexOf(`"/odom"`));
  assert.ok(out.includes("export type TopicName = keyof DimosTopics;"));
});

Deno.test("generateTypes: custom/unknown type → unknown (not a broken @dimos/msgs import)", () => {
  const out = generateTypes(
    [
      { topic: "/odom", type: "geometry_msgs.PoseStamped" },
      { topic: "/my_custom", type: "my_msgs.RobotState" },
    ],
    KNOWN,
  );
  assert.ok(out.includes(`"/my_custom": unknown;`));
  assert.ok(out.includes("my_msgs.RobotState — not in @dimos/msgs"));
  // the unknown package must NOT leak into the import line (only geometry_msgs is imported)
  assert.ok(out.includes(`import type { geometry_msgs } from "@dimos/msgs";`));
  assert.ok(!/import type \{[^}]*my_msgs/.test(out));
  assert.ok(out.includes(`(1 untyped)`));
});

Deno.test("generateTypes: a bare/untyped channel → unknown", () => {
  const out = generateTypes([{ topic: "/raw", type: "?" }], KNOWN);
  assert.ok(out.includes(`"/raw": unknown;`));
});

Deno.test("generateTypes: empty discovery → placeholder, no import line", () => {
  const out = generateTypes([], KNOWN);
  assert.ok(out.includes("(no topics discovered)"));
  assert.ok(!out.includes("import type"));
  assert.ok(out.includes("export interface DimosTopics {"));
});

Deno.test("generateCommandTypes: targets+methods → typed map, sorted + de-duped", () => {
  const out = generateCommandTypes([
    { target: "GO2Connection", method: "standup", label: "Stand up" },
    { target: "GO2Connection", method: "sit", label: "Sit" },
    { target: "GO2Connection", method: "sit", label: "Sit (dup)" },
    { target: "Nav", method: "goal", label: "Goal" },
  ]);
  assert.ok(out.includes(`"GO2Connection": {`));
  assert.ok(out.includes(`"sit": RpcCall;`));
  assert.ok(out.includes(`"standup": RpcCall;`));
  // methods sorted within a target (sit before standup), de-duped (one "sit")
  assert.ok(out.indexOf(`"sit"`) < out.indexOf(`"standup"`));
  assert.equal(out.match(/"sit": RpcCall;/g)?.length, 1);
  // targets sorted (GO2Connection before Nav)
  assert.ok(out.indexOf(`"GO2Connection"`) < out.indexOf(`"Nav"`));
  assert.ok(out.includes("export type RpcTarget = keyof DimosCommands;"));
  assert.ok(out.includes("export type RpcMethod<T extends RpcTarget> = keyof DimosCommands[T];"));
});

Deno.test("generateCommandTypes: empty → placeholder, still compilable", () => {
  const out = generateCommandTypes([]);
  assert.ok(out.includes("(no rpc commands advertised)"));
  assert.ok(out.includes("export interface DimosCommands {"));
  assert.ok(out.includes("export interface RpcCall {"));
});

// String assertions above only check the shape; this one proves the emitted types actually TYPECHECK.
// Uses an all-`unknown` DimosTopics (empty `known` set → no `@dimos/msgs` import → self-contained) so
// `deno check` needs no import map, then appends the DimosCommands block.
Deno.test("generated DimosTopics + DimosCommands typecheck under `deno check`", async () => {
  const src = generateTypes(
    [
      { topic: "/odom", type: "geometry_msgs.PoseStamped" }, // → unknown (not in the empty known set)
      { topic: "/raw", type: "?" },
    ],
    new Set(),
  ) + "\n" + generateCommandTypes([
    { target: "GO2Connection", method: "standup", label: "Stand up" },
    { target: "GO2Connection", method: "sit", label: "Sit" },
  ]);
  const tmp = await Deno.makeTempFile({ prefix: "dimos_gen_", suffix: ".ts" });
  try {
    await Deno.writeTextFile(tmp, src);
    const { code, stderr } = await new Deno.Command("deno", {
      args: ["check", tmp],
      stderr: "piped",
      stdout: "null",
    }).output();
    assert.equal(code, 0, new TextDecoder().decode(stderr));
  } finally {
    await Deno.remove(tmp);
  }
});
