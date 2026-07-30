import { runEval } from "@dimsim/eval";

await runEval({
  scene: "apartment",
  task:
    "Go to the large freestanding gray oval soaking bathtub directly ahead in the current camera view. " +
    "Stop right next to it, as close as collision allows.",
  timeoutSec: 180,
  startPose: { x: 2.3, y: 0.5, z: -3.2, yaw: 45 },
  success: (ctx) =>
    ctx.rubrics.objectDistance({ target: "bathtub", thresholdM: 1.0 }),
});
