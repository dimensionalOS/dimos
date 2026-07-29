import { runEval } from "@dimsim/eval";

await runEval({
  scene: "apartment",
  task:
    "Go to the large wall-mounted television directly ahead in the current camera view. " +
    "Stop in front of it.",
  timeoutSec: 60,
  startPose: { x: 1.5, y: 0.5, z: 3.5, yaw: 57 },
  success: (ctx) =>
    ctx.rubrics.objectDistance({ target: "television", thresholdM: 2.0 }),
});
