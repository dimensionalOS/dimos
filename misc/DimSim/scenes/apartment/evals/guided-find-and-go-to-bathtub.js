import { runEval } from "@dimsim/eval";

// Guided integration workflow: the task intentionally supplies a calibrated
// visual route so failures isolate the DimSim ↔ DimOS agent loop.
const requiredRoute = [
  {
    name: "tree start",
    minX: 0.7,
    maxX: 1.7,
    minZ: 7.1,
    maxZ: 8.8,
  },
  {
    name: "kitchen exterior entrance",
    minX: -4.5,
    maxX: -3.5,
    minZ: 4.7,
    maxZ: 6.1,
  },
];

function contains(region, point) {
  return point.x >= region.minX && point.x <= region.maxX &&
    point.z >= region.minZ && point.z <= region.maxZ;
}

// Keep this workflow compatible with an already-open browser that loaded the
// previous public rubric chunk. Coordinates remain scorer-only: the model sees
// only the task and its camera/motion tools.
function orderedRoute(metrics) {
  const trajectory = metrics?.trajectory ?? [];
  let nextRegion = 0;
  for (const point of trajectory) {
    if (
      nextRegion < requiredRoute.length &&
      contains(requiredRoute[nextRegion], point)
    ) {
      nextRegion++;
    }
  }

  const passed = nextRegion === requiredRoute.length;
  const visited = requiredRoute.slice(0, nextRegion).map(({ name }) => name);
  return {
    passed,
    reason: passed
      ? `visited ${requiredRoute.map(({ name }) => name).join(" -> ")}`
      : `visited ${visited.join(" -> ") || "none"}; ${
        requiredRoute[nextRegion]
          ? `next required ${requiredRoute[nextRegion].name}`
          : "route incomplete"
      }`,
  };
}

await runEval({
  scene: "apartment",
  task:
    "Starting outside beside the large oak tree with the camera facing west along the white house, use only your camera to enter the house and locate the large freestanding oval bathtub. First sidestep right once with `relative_move(left=-0.8)` to clear the south side of the oak, then observe. Keep the white exterior wall visible on your right and walk forward west in 0.8-1.0 meter steps, observing after every step. The bright TV-facing recess near the tree is not a floor-level passage, so do not try to cross it. Continue along the wall until a real kitchen entrance appears on the right with continuous flooring, cabinets, or counters visible through it. Center that opening with pure rotations and cross straight in short steps. Always observe after a move even if its odometry report says blocked; abandon a route only if the camera also shows no progress. Do not enter windows, dark wall patches, or the covered garden structure. Inside the kitchen, move away from the exterior on visible clear floor and find the kitchen's interior doorway in the partition before trying to locate later landmarks. Cross it, pass the dining area without driving through table or chair legs, and locate the central opening framed by a wooden bookcase on one side and a gold bar cart on the other. Center it and cross straight. Beyond it, follow the divider with neon wall art until its open doorway appears. That doorway leads into a bedroom/office, not directly into the bathroom. In the bedroom, look for a colorful abstract painting and a large dark wooden wardrobe. The real bathroom entrance is the white or tiled doorway immediately beside the wardrobe. This landmark pair is first visible from several meters away, so seeing it does not mean the doorway is nearby. Once identified, keep the pair centered and advance toward it repeatedly in 0.8 meter steps on clear floor, observing after every step. Expect at least five approach steps from first detection; do not label the doorway nearby or cross any nearer opening before completing those steps and seeing the wardrobe fill much of the view. If the pair leaves the frame, re-center it from the same viewpoint instead of choosing another opening. When the actual wardrobe-side doorway is finally close, make at most one centering rotation and one observation, then cross it straight in a 0.7 meter step. A room containing an office desk or work chair is not the bathroom; if those appear after a crossing, reverse that crossing and continue toward the real wardrobe-side route. Confirm the bathroom with tiled floor plus a vanity, sink, or toilet before forbidding the entrance behind you. Do not enter `target-verify` or begin repeated room scans until the bathtub itself is visible as a large horizontal hollow basin with a rim and chrome fixture. A desk chair, white chair back, sink, or toilet is not the target. Continue through visible clear bathroom floor and locate the freestanding oval bathtub near wall shelves and an abstract painting. Verify it from a materially different second view, but do not treat a clear or close-looking camera view as proof that you are within one meter. After verification, face the bathtub and continue toward it in 0.8 meter steps, observing after every step. Take at least three such approach steps while the floor remains clear, and keep approaching until the near rim fills much of the lower camera view or forward progress is physically blocked by the basin. Only then stop with no further movement.",
  timeoutSec: 900,
  // Beside the oak, facing west along the house. The nearby TV-facing recess is
  // collision-blocked at floor level; the calibrated kitchen entrance is
  // farther west and must be reached physically along the exterior wall.
  startPose: { x: 1.2, y: 0.5, z: 7.6, yaw: 270 },
  initialSuccess: (ctx) =>
    ctx.rubrics.objectDistance({ target: "bathtub", thresholdM: 1.0 }),
  success: (ctx) => {
    const goal = ctx.rubrics.objectDistance({
      target: "bathtub",
      thresholdM: 1.0,
    });
    const search = ctx.rubrics.searchEvidence({
      minTravelM: 6.0,
      minHeadingTravelDeg: 180,
      minViewpoints: 4,
    });
    // Hidden trajectory gates prove that the robot left the tree and crossed
    // the calibrated exterior doorway. Rapier collisions prevent wall travel;
    // the final object-distance rubric proves it ended at the bathtub.
    const route = orderedRoute(ctx.metrics);
    return {
      passed: goal.passed && search.passed && route.passed,
      score: goal.score,
      reason: `${goal.reason}; ${search.reason}; ${route.reason}`,
    };
  },
});
