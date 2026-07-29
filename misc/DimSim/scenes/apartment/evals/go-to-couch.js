import { runEval } from "@dimsim/eval";

await runEval({
  scene: "apartment",
  task:
    "Go to the large brown L-shaped couch directly ahead in the current camera view. " +
    "Stop next to it.",
  timeoutSec: 60,
  startPose: { x: 1, y: 0.5, z: 3, yaw: 117 },
  success: (ctx) =>
    ctx.rubrics.objectDistance({ target: "sectional", thresholdM: 2.0 }),
});
