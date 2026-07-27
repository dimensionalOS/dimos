import { runEval } from "@dimsim/eval";

await runEval({
  scene: "apartment",
  task: "Find the bathtub and go to it. Keep searching until you find it.",
  timeoutSec: 900,
  // Start outside beside the oak, facing west along the house. The task stays
  // intentionally natural and contains no route, landmark, or tool guidance.
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
    return {
      passed: goal.passed && search.passed,
      score: goal.score,
      reason: `${goal.reason}; ${search.reason}`,
    };
  },
});
