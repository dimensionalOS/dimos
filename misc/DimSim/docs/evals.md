# Authoring evals

An eval workflow is a JS file at `scenes/<env>/evals/<name>.js` that imports `runEval` from `@dimsim/eval` and calls it. The file is a runnable program — same shape whether the browser is executing it (when triggered from the CLI / runner) or Deno is (when you `deno run` the file directly to dispatch it at a running bridge).

## Minimal workflow

```js
// scenes/apartment/evals/go-to-couch.js
import { runEval } from '@dimsim/eval';

await runEval({
  scene:       'apartment',
  task:        'Go to the couch',
  timeoutSec:  30,
  startPose:   { x: 0, y: 0.5, z: 3, yaw: 0 },
  success:     (ctx) => ctx.rubrics.objectDistance({ target: 'sectional', thresholdM: 2.0 }),
});
```

That's it. Drop the file under any scene's `evals/` folder and `dimsim eval list` will pick it up.

## The workflow object

| Field | Required | Description |
|---|---|---|
| `scene` | ✓ | Scene name. Must match the directory under `scenes/`. |
| `task` | ✓ | Human-readable goal description. Shown in the overlay + logged. |
| `success(ctx)` | ✓ | Function returning `{ passed: bool, reason?, score? }`. Called every 250 ms until it passes or timeout. |
| `timeoutSec` | – | Default 120. Max wall-clock seconds before forced fail. |
| `startPose` | – | `{x, y, z, yaw?}` (yaw in degrees). Applied to the agent before `setup`. |
| `setup(ctx)` | – | Optional async fn that runs once at start. Spawn obstacles, configure embodiments, etc. |

## The `ctx` object

`setup(ctx)` and `success(ctx)` both receive:

| Field | Description |
|---|---|
| `ctx.agent` | The live AiAvatar — has `setPosition`, `getPosition`, `group`, etc. |
| `ctx.agentPos` | `{x, y, z}` — convenience copy of the current agent translation |
| `ctx.sceneState` | `{ assets: AssetEntry[], agentPos }` — for rubric scoring |
| `ctx.setAgentPose({x, y, z, yaw?})` | Teleport the agent (no-op if no agent) |
| `ctx.findAsset(query)` | Case-insensitive search by title or id; returns an AssetEntry or null |
| `ctx.dist(a, b)` | Euclidean distance helper |
| `ctx.rubrics.objectDistance({target, thresholdM?})` | Built-in rubric — bbox-surface distance from agent to target asset |
| `ctx.rubrics.radiusContains({targets[], radiusM?})` | Pass if agent is within `radiusM` of the targets' centroid |

You can write `success` inline if neither built-in rubric fits:

```js
success: ({ agent, findAsset, dist }) => {
  const tv = findAsset('television');
  const couch = findAsset('sectional');
  if (!tv || !couch) return { passed: false, reason: 'targets missing' };
  const between = { x: (tv.transform.x + couch.transform.x) / 2, y: 0, z: (tv.transform.z + couch.transform.z) / 2 };
  const d = dist(agent.getPosition?.()?.[0] ? { x: agent.getPosition()[0], y: 0, z: agent.getPosition()[2] } : {x:0,y:0,z:0}, between);
  return { passed: d <= 1.5, score: d, reason: `${d.toFixed(2)}m from midpoint` };
}
```

## Three ways to run a workflow

1. **From the CLI against an already-running sim** (most common during dev):

   ```bash
   dimsim eval go-to-couch                    # bare name
   dimsim eval apartment/go-to-couch          # scene-qualified
   ```

   Auto-implies `--connect`. The sim has to be open already (`dimos --simulation dimsim ...` or `dimsim dev`).

2. **Directly via `deno run`** — runs the workflow file as a Deno program; dispatches to whichever bridge is on `localhost:8090`:

   ```bash
   deno run -A misc/DimSim/scenes/apartment/evals/go-to-couch.js
   ```

   Functionally identical to `dimsim eval go-to-couch`. Useful if you want to drop the `dimsim` global install.

3. **Headless / CI**:

   ```bash
   dimsim eval --headless --scene apartment --workflow go-to-couch
   dimsim eval --headless                           # runs all workflows for all scenes
   dimsim eval --headless --output junit > junit.xml
   ```

   Spins up its own bridge and a headless Chromium, no dimos involvement. Designed for CI.

## How the same file runs in two runtimes

```
import { runEval } from '@dimsim/eval';
```

| Runtime | `@dimsim/eval` resolves to | `runEval(workflow)` does |
|---|---|---|
| **Browser** (via `index.html` importmap) | `/assets/dimsim-eval.js` (bundled harness chunk) | Sets up the eval in-place — runs `setup`, polls `success`, returns when passed or timeout |
| **Deno** (via `scenes/deno.json` importmap) | `evals/deno-client.ts` | Computes the workflow URL from `Deno.mainModule`, opens a WS to `localhost:8090`, sends `{type:'runEval', workflowUrl}`, awaits the result, exits |

The `success` callback never actually runs in Deno — Deno only ships the URL. The browser re-imports the same file URL and executes the callbacks there.

## Tips

- **Set `timeoutSec` higher when running on CPU / SwiftShader** — agent setup can take 5–10s on slow renders.
- **Use a `--connect` run first** to confirm the eval logic works against your live sim. Once green, set up CI with `--headless`.
- **`startPose` is yaw-in-degrees**, not radians. `yaw: 90` looks along +X.
- **Score is yours to define.** Lower-is-better for distance-style rubrics, higher-is-better for coverage-style. CI consumers should not assume a meaning.
- **One eval at a time.** The harness is a singleton — running two evals concurrently isn't supported. Use `--parallel N` with multiple browser pages if you need throughput.
