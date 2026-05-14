/**
 * Unit tests for eval rubrics — pure scoring functions, no browser needed.
 *
 *   cd DimSim && deno test dimos-cli/test/rubrics_test.ts
 */

import { assertEquals, assertAlmostEquals } from "jsr:@std/assert";
// Import from source — rubrics are pure TS, no DOM deps
import {
  scoreObjectDistance,
  scoreRadiusContains,
  type SceneState,
} from "../../src/dimos/rubrics.ts";

// -- helpers ------------------------------------------------------------------

function mkScene(assets: { title: string; x: number; y: number; z: number }[], agentPos?: { x: number; y: number; z: number }): SceneState {
  return {
    assets: assets.map((a) => ({
      title: a.title,
      transform: { x: a.x, y: a.y, z: a.z },
    })),
    agentPos,
  };
}

// -- objectDistance (existing, sanity check) -----------------------------------

Deno.test("objectDistance: pass when agent is close to target", () => {
  const scene = mkScene([{ title: "Television", x: 5, y: 0, z: 3 }], { x: 5, y: 0, z: 3.3 });
  const result = scoreObjectDistance({ object: "agent", target: "television", thresholdM: 0.5 }, scene);
  assertEquals(result.pass, true);
});

Deno.test("objectDistance: fail when agent is far from target", () => {
  const scene = mkScene([{ title: "Television", x: 5, y: 0, z: 3 }], { x: 0, y: 0, z: 0 });
  const result = scoreObjectDistance({ object: "agent", target: "television", thresholdM: 2.0 }, scene);
  assertEquals(result.pass, false);
});

Deno.test("objectDistance: fail when target not found", () => {
  const scene = mkScene([], { x: 0, y: 0, z: 0 });
  const result = scoreObjectDistance({ object: "agent", target: "nonexistent", thresholdM: 1.0 }, scene);
  assertEquals(result.pass, false);
  assertEquals(result.distanceM, Infinity);
});

// -- radiusContains -----------------------------------------------------------

Deno.test("radiusContains: pass when agent is within centroid radius", () => {
  // Kitchen objects form a triangle around (5, 0, 5)
  const scene = mkScene(
    [
      { title: "Refrigerator", x: 4, y: 0, z: 4 },
      { title: "Stove", x: 6, y: 0, z: 4 },
      { title: "Sink", x: 5, y: 0, z: 7 },
    ],
    { x: 5, y: 0, z: 5 }, // agent near centroid
  );
  const result = scoreRadiusContains(
    { targets: ["refrigerator", "stove", "sink"], radiusM: 3.0 },
    scene,
  );
  assertEquals(result.pass, true);
  assertEquals(result.foundTargets.length, 3);
  assertEquals(result.missingTargets.length, 0);
  // Centroid should be (5, 0, 5)
  assertAlmostEquals(result.centroid.x, 5, 0.01);
  assertAlmostEquals(result.centroid.z, 5, 0.01);
});

Deno.test("radiusContains: fail when agent is far from centroid", () => {
  const scene = mkScene(
    [
      { title: "Refrigerator", x: 4, y: 0, z: 4 },
      { title: "Stove", x: 6, y: 0, z: 4 },
      { title: "Sink", x: 5, y: 0, z: 7 },
    ],
    { x: 20, y: 0, z: 20 }, // agent far away
  );
  const result = scoreRadiusContains(
    { targets: ["refrigerator", "stove", "sink"], radiusM: 3.0 },
    scene,
  );
  assertEquals(result.pass, false);
  assertEquals(result.foundTargets.length, 3);
});

Deno.test("radiusContains: partial match — 2 of 3 targets found, still scores", () => {
  const scene = mkScene(
    [
      { title: "Refrigerator", x: 4, y: 0, z: 4 },
      { title: "Stove", x: 6, y: 0, z: 4 },
      // Sink is missing
    ],
    { x: 5, y: 0, z: 4 }, // agent at centroid of found targets
  );
  const result = scoreRadiusContains(
    { targets: ["refrigerator", "stove", "sink"], radiusM: 3.0 },
    scene,
  );
  assertEquals(result.pass, true);
  assertEquals(result.foundTargets.length, 2);
  assertEquals(result.missingTargets, ["sink"]);
  // Centroid should be (5, 0, 4)
  assertAlmostEquals(result.centroid.x, 5, 0.01);
  assertAlmostEquals(result.centroid.z, 4, 0.01);
});

Deno.test("radiusContains: fail when no targets found", () => {
  const scene = mkScene([], { x: 0, y: 0, z: 0 });
  const result = scoreRadiusContains(
    { targets: ["refrigerator", "stove"], radiusM: 3.0 },
    scene,
  );
  assertEquals(result.pass, false);
  assertEquals(result.distanceM, Infinity);
  assertEquals(result.missingTargets.length, 2);
});

Deno.test("radiusContains: fail when agent position not available", () => {
  const scene = mkScene([{ title: "Refrigerator", x: 4, y: 0, z: 4 }]);
  const result = scoreRadiusContains(
    { targets: ["refrigerator"], radiusM: 3.0 },
    scene,
  );
  assertEquals(result.pass, false);
});

Deno.test("radiusContains: single target degrades to point distance", () => {
  const scene = mkScene(
    [{ title: "Bed", x: 10, y: 0, z: 10 }],
    { x: 10, y: 0, z: 11 }, // 1m away
  );
  const result = scoreRadiusContains(
    { targets: ["bed"], radiusM: 2.0 },
    scene,
  );
  assertEquals(result.pass, true);
  assertAlmostEquals(result.distanceM, 1.0, 0.01);
  assertEquals(result.foundTargets, ["bed"]);
});

Deno.test("radiusContains: exact threshold boundary", () => {
  const scene = mkScene(
    [
      { title: "A", x: 0, y: 0, z: 0 },
      { title: "B", x: 2, y: 0, z: 0 },
    ],
    { x: 1, y: 0, z: 3 }, // centroid at (1,0,0), agent at (1,0,3) → dist = 3.0
  );
  const result = scoreRadiusContains(
    { targets: ["a", "b"], radiusM: 3.0 },
    scene,
  );
  assertEquals(result.pass, true); // exactly at boundary
  assertAlmostEquals(result.distanceM, 3.0, 0.01);
});

Deno.test("radiusContains: empty targets array fails", () => {
  const scene = mkScene([], { x: 0, y: 0, z: 0 });
  const result = scoreRadiusContains({ targets: [], radiusM: 3.0 }, scene);
  assertEquals(result.pass, false);
});
