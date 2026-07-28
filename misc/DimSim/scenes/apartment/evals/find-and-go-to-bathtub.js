import { runEval } from "@dimsim/eval";

await runEval({
  scene: "apartment",
  task: "Find the bathtub and go to it. Keep searching until you find it. " +
    "When you believe you have found the bathtub, stop right next to the bathtub with a very close distance, " +
    "and respond with exactly FOUND_BATHTUB.",
  timeoutSec: 900,
  requiredAgentOutput: "FOUND_BATHTUB",
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
    const declared = ctx.agentOutput?.text === "FOUND_BATHTUB";
    return {
      passed: declared && goal.passed,
      score: goal.score,
      reason: `${
        declared
          ? "agent declared FOUND_BATHTUB"
          : "waiting for exact agent output FOUND_BATHTUB"
      }; ${goal.reason}`,
    };
  },
});
