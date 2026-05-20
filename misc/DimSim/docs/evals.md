# Evals

An eval workflow is one JS file at `scenes/<env>/evals/<name>.js`. It imports `runEval` from `@dimsim/eval` and calls it. That's the whole authoring surface.

## Create a new eval

```js
// scenes/apartment/evals/go-to-couch.js
import { runEval } from '@dimsim/eval';

await runEval({
  scene:      'apartment',
  task:       'Go to the couch',
  timeoutSec: 30,
  startPose:  { x: 0, y: 0.5, z: 3, yaw: 0 },
  success:    (ctx) => ctx.rubrics.objectDistance({ target: 'sectional', thresholdM: 2.0 }),
});
```

Drop the file under any scene's `evals/` folder and `dimsim eval list` picks it up.

## Run it

```bash
dimsim eval go-to-couch                    # against the open sim
dimsim eval --headless --scene apartment --workflow go-to-couch   # standalone / CI
deno run -A misc/DimSim/scenes/apartment/evals/go-to-couch.js     # direct execution
```

All three end up at the same harness in the browser; pick whichever feels natural for the moment.

## The workflow object

| Field | Required | Description |
|---|---|---|
| `scene` | ✓ | Scene name. Must match a directory under `scenes/`. |
| `task` | ✓ | Human-readable goal. Shown in the overlay + logged. |
| `success(ctx)` | ✓ | Returns `{passed, reason?, score?}`. Polled every 250 ms until it passes or timeout. |
| `timeoutSec` | – | Default 120. Wall-clock cap. |
| `startPose` | – | `{x, y, z, yaw?}` — applied before `setup`. Yaw in degrees. |
| `setup(ctx)` | – | Async fn run once at start — spawn obstacles, set props, anything. |

## The `ctx` object

Both `setup(ctx)` and `success(ctx)` receive:

| Field | What |
|---|---|
| `ctx.agent` | The live agent — `setPosition`, `getPosition`, `group`, etc. |
| `ctx.agentPos` | `{x, y, z}` — current translation, convenience copy. |
| `ctx.sceneState` | `{assets, agentPos}` — used by rubric helpers. |
| `ctx.setAgentPose({x, y, z, yaw?})` | Teleport the agent. |
| `ctx.findAsset(query)` | Case-insensitive search by title or id. |
| `ctx.dist(a, b)` | Euclidean distance. |
| `ctx.rubrics.objectDistance({target, thresholdM?})` | Pass if agent is within `thresholdM` of `target`'s bbox surface. |
| `ctx.rubrics.radiusContains({targets, radiusM?})` | Pass if agent is within `radiusM` of the centroid of `targets`. |

## Custom scoring

If neither built-in rubric fits, write the logic inline:

```js
success: ({ agentPos, findAsset, dist }) => {
  const tv    = findAsset('television');
  const couch = findAsset('sectional');
  if (!tv || !couch) return { passed: false, reason: 'targets missing' };
  const mid = {
    x: (tv.transform.x + couch.transform.x) / 2,
    y: 0,
    z: (tv.transform.z + couch.transform.z) / 2,
  };
  const d = dist(agentPos, mid);
  return { passed: d <= 1.5, score: d, reason: `${d.toFixed(2)}m from midpoint` };
}
```

## Scripted setup

`setup(ctx)` is async — do whatever you need before scoring starts:

```js
setup: async ({ agent }) => {
  agent.setPosition(-3, 0.5, 0);
  await new Promise(r => setTimeout(r, 250));   // let physics settle
},
```

You can spawn obstacles, change embodiments mid-eval, or set up multi-stage tests here. The harness doesn't constrain you.

## Tips

- **One eval at a time.** The harness is a singleton; running two evals concurrently isn't supported. Use `--parallel N` with multiple browser pages for throughput.
- **Score is yours to define.** Lower-is-better for distances, higher-is-better for coverage — CI consumers should not assume.
- **`startPose` yaw is in degrees**, not radians.
- **`setup`/`success` callbacks can use any browser API** (THREE, scene, Rapier) — they run in the browser context, not in Deno.
