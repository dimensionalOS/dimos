import { runEval } from '@dimsim/eval';

await runEval({
  scene: 'apartment',
  task: 'Go to the TV',
  timeoutSec: 30,
  startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
  success: (ctx) => ctx.rubrics.objectDistance({ target: 'television', thresholdM: 2.0 }),
});
