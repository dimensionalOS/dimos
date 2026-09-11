import { runEval } from '@dimsim/eval';

await runEval({
  scene: 'apartment',
  task: 'Go to the couch',
  timeoutSec: 30,
  startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
  success: (ctx) => ctx.rubrics.objectDistance({ target: 'sectional', thresholdM: 2.0 }),
});
