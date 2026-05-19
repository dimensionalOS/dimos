/**
 * @dimsim/eval — public ESM facade for eval workflows.
 *
 * Eval files under `scenes/<env>/evals/*.js` import `runEval` from here and
 * call it directly:
 *
 *     import { runEval } from '@dimsim/eval';
 *     await runEval({
 *       scene: 'apartment',
 *       task: 'Go to the couch',
 *       success: (ctx) => ctx.rubrics.objectDistance({ target: 'sectional' }),
 *     });
 *
 * The import map in index.html aliases `@dimsim/eval` to this file.  At
 * runtime we wait for the engine to wire up `window.__dimsim.eval.runEval`
 * (it dispatches a `dimsim-eval-ready` event when it's available) and then
 * delegate the call.  This indirection keeps the public surface decoupled
 * from the bundled engine chunk's hashed filename.
 */

async function _ready() {
  if (window.__dimsim?.eval?.runEval) return;
  await new Promise((resolve) => {
    window.addEventListener('dimsim-eval-ready', resolve, { once: true });
  });
}

export async function runEval(workflow) {
  await _ready();
  return window.__dimsim.eval.runEval(workflow);
}
