// Unit tests for the blueprint→TS codegen. Pure (no network). Run: `deno test -A cli/`.
import assert from "node:assert/strict";

import { generateTypes } from "./genTypes.ts";

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
