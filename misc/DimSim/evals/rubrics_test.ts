import { orderedRegionVisits, searchEvidence } from "./rubrics.ts";

function assertEquals(actual: unknown, expected: unknown): void {
  const a = JSON.stringify(actual);
  const e = JSON.stringify(expected);
  if (a !== e) throw new Error(`expected ${e}, got ${a}`);
}

Deno.test("searchEvidence requires every configured motion signal", () => {
  const result = searchEvidence(
    {
      metrics: {
        pathLengthM: 7,
        headingTravelDeg: 170,
        distinctViewpoints: 5,
        trajectory: [],
      },
    },
    {
      minTravelM: 6,
      minHeadingTravelDeg: 180,
      minViewpoints: 4,
    },
  );

  assertEquals(result.passed, false);
});

Deno.test("searchEvidence passes with sufficient trajectory evidence", () => {
  const result = searchEvidence(
    {
      metrics: {
        pathLengthM: 7,
        headingTravelDeg: 210,
        distinctViewpoints: 5,
        trajectory: [],
      },
    },
    {
      minTravelM: 6,
      minHeadingTravelDeg: 180,
      minViewpoints: 4,
    },
  );

  assertEquals(result.passed, true);
});

Deno.test("orderedRegionVisits requires the full route and final containment", () => {
  const regions = [
    { name: "outside", minX: 0, maxX: 2, minZ: 7, maxZ: 9 },
    { name: "entrance", minX: -5, maxX: -3, minZ: 4, maxZ: 6 },
    { name: "bathroom", minX: 1, maxX: 6, minZ: -5, maxZ: 0 },
  ];
  const result = orderedRegionVisits(
    {
      metrics: {
        pathLengthM: 20,
        headingTravelDeg: 360,
        distinctViewpoints: 8,
        trajectory: [
          { x: 1, y: 0.5, z: 8 },
          { x: -4, y: 0.5, z: 5 },
          { x: 3, y: 0.5, z: -2 },
          { x: 0, y: 0.5, z: 2 },
        ],
      },
    },
    { regions },
  );

  assertEquals(result.passed, false);
  assertEquals(result.reason?.includes("did not end in bathroom"), true);
});

Deno.test("orderedRegionVisits passes an in-order route ending in the goal region", () => {
  const result = orderedRegionVisits(
    {
      metrics: {
        pathLengthM: 20,
        headingTravelDeg: 360,
        distinctViewpoints: 8,
        trajectory: [
          { x: 1, y: 0.5, z: 8 },
          { x: -4, y: 0.5, z: 5 },
          { x: 0, y: 0.5, z: 0 },
          { x: 1, y: 0.5, z: -2.5 },
          { x: 4, y: 0.5, z: -1 },
        ],
      },
    },
    {
      regions: [
        { name: "tree", minX: 0, maxX: 2, minZ: 7, maxZ: 9 },
        { name: "entrance", minX: -5, maxX: -3, minZ: 4, maxZ: 6 },
        { name: "main door", minX: -1, maxX: 1, minZ: -1, maxZ: 1 },
        {
          name: "bathroom door",
          minX: 0.5,
          maxX: 1.5,
          minZ: -3,
          maxZ: -2,
        },
        { name: "bathroom", minX: 1, maxX: 6, minZ: -5, maxZ: 0 },
      ],
    },
  );

  assertEquals(result.passed, true);
});
