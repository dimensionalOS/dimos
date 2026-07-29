import { runEval } from "@dimsim/eval";

await runEval({
  scene: "apartment",
  task: "Go into the kitchen directly ahead through the open doorway. " +
    "Stop near the refrigerator.",
  timeoutSec: 60,
  startPose: { x: 0.8, y: 0.5, z: 2.5, yaw: -120 },
  success: (ctx) =>
    ctx.rubrics.objectDistance({ target: "refrigerator", thresholdM: 3.0 }),
});
