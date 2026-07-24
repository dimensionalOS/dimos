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
dimsim eval apartment/go-to-couch --agent  # closed-loop through a running DimOS agent
dimsim eval --headless --scene apartment --workflow go-to-couch   # standalone / CI
deno run -A misc/DimSim/scenes/apartment/evals/go-to-couch.js     # direct execution
```

All three end up at the same harness in the browser. Pick whichever fits the moment.

## Closed-loop agent mode

`--agent` evaluates an already-running DimOS agent rather than only scoring
browser state:

```bash
# Terminal 1: start the MCP-enabled DimOS stack with its DimSim browser.
dimos --replay run unitree-go2-agentic

# Terminal 2: reset, dispatch the exact workflow task once, then score.
dimsim eval apartment/go-to-couch --agent
```

Agent mode requires connect mode, exactly one workflow, and no `--parallel` or
standalone `--headless` launch. The MCP endpoint is selected in this order:
`--mcp-url`, `DIMOS_MCP_URL`, then `http://127.0.0.1:9990/mcp`. API keys, model
names, and model endpoints remain DimOS configuration; DimSim never reads them.

The correlated lifecycle is:

```text
runEval → evalReady → evalReset → resetAck → agent_send → evalStart → evalResult
```

The browser imports and validates the workflow and runs `setup`, but scoring
does not start until `evalStart`. The bridge applies `startPose` to its
authoritative Rapier body, clears prior motion, publishes the resulting
pose/odometry, and acknowledges the actual pose before `agent_send` is called.
Agent workflows therefore require finite `x`, `y`, `z`, and `yaw` fields.
An initially satisfied rubric is rejected because it cannot measure agent
behavior.

Results retain `passed` and add `runId`, `status`, and (for infrastructure
errors) `failureStage`. Exit codes are `0` for pass, `1` for task failure, and
`2` for configuration or infrastructure errors. Both JSON and JUnit keep
machine-readable output on stdout; progress is written to stderr.

### Manual live-model smoke test

With the model credentials configured in DimOS:

1. Start `unitree-go2-agentic` with the apartment DimSim backend.
2. Run `dimsim eval apartment/go-to-couch --agent`.
3. Confirm the logs show one `agent_send`, after `resetAck`, and scoring begins
   only after dispatch.
4. Stop MCP and repeat. The command must exit `2` before `evalStart`.

## The workflow object

| Field | Required | Description |
|---|---|---|
| `scene` | ✓ | Scene name. Must match a directory under `scenes/`. |
| `task` | ✓ | Human-readable goal. Shown in the overlay + logged. |
| `success(ctx)` | ✓ | Returns `{passed, reason?, score?}`. Polled every 250 ms until it passes or timeout. |
| `timeoutSec` | – | Default 120. Wall-clock cap. |
| `startPose` | – | `{x, y, z, yaw?}`, applied before `setup`. Yaw in degrees. |
| `setup(ctx)` | – | Async fn run once at start. Spawn obstacles, set props, anything. |

## The `ctx` object

Both `setup(ctx)` and `success(ctx)` receive:

| Field | What |
|---|---|
| `ctx.agent` | The live agent: `setPosition`, `getPosition`, `group`, etc. |
| `ctx.agentPos` | `{x, y, z}`, current translation, convenience copy. |
| `ctx.sceneState` | `{assets, agentPos}`, used by rubric helpers. |
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

`setup(ctx)` is async. Do whatever you need before scoring starts:

```js
setup: async ({ agent }) => {
  agent.setPosition(-3, 0.5, 0);
  await new Promise(r => setTimeout(r, 250));   // let physics settle
},
```

You can spawn obstacles, change embodiments mid-eval, or set up multi-stage tests here. The harness doesn't constrain you.

## Tips

- One eval at a time. The harness is a singleton, so running two evals concurrently isn't supported. Use `--parallel N` with multiple browser pages for throughput.
- Score is yours to define. Lower-is-better for distances, higher-is-better for coverage. CI consumers should not assume.
- `startPose` yaw is in degrees, not radians.
- `setup`/`success` callbacks can use any browser API (THREE, scene, Rapier). They run in the browser context, not in Deno.
