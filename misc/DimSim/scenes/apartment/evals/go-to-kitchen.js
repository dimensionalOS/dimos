import { runEval } from '@dimsim/eval';

await runEval({
  scene: 'apartment',
  task: 'Go to the kitchen',
  timeoutSec: 30,
  startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
  success: (ctx) => ctx.rubrics.objectDistance({ target: 'refrigerator', thresholdM: 3.0 }),
});
