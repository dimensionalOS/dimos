# Getting started

5-minute tour. For deeper guides see [scenes.md](scenes.md) and [evals.md](evals.md).

## The two ways DimSim runs

```bash
# 1. dimos drives DimSim (production / agentic / CI)
uv run dimos --simulation dimsim --dimsim-scene=apartment run unitree-go2-agentic

# 2. DimSim runs standalone (engine dev / scene authoring)
cd misc/DimSim/cli
deno run -A --unstable-net cli.ts dev --scene apartment
```

Both end up with a Vite-built `dist/` and a bridge on port 8090. Same scenes, same browser. Difference is just who's driving the agent.

If you install the CLI globally, replace the `deno run` boilerplate with `dimsim`:

```bash
cd misc/DimSim/cli
deno install -gAf --unstable-net --name=dimsim --config=./deno.json ./cli.ts
```

## 60-second loop: edit a scene

1. `dimsim dev --scene warehouse`
2. Open `misc/DimSim/scenes/warehouse/index.js` in your editor.
3. Change `setSky({ brightness: 0.7 })` to `setSky({ brightness: 1.5 })`. Save.
4. The browser HMR-reloads — brighter sky.

## 60-second loop: write an eval

`misc/DimSim/scenes/apartment/evals/look-at-window.js`:

```js
import { runEval } from '@dimsim/eval';

await runEval({
  scene: 'apartment',
  task: 'Stand near a window',
  timeoutSec: 20,
  startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
  success: (ctx) => ctx.rubrics.objectDistance({ target: 'window', thresholdM: 1.5 }),
});
```

From a separate terminal (while the sim is running):

```bash
dimsim eval look-at-window
```

The browser shows a green/red overlay when it finishes, and the result echoes in your terminal.

## Where things live

```
src/        engine + scene API + bridge client (browser, vite-bundled)
cli/        dimsim CLI + bridge server (Deno)
evals/      eval harness + rubrics + Deno client (both runtimes)
scenes/     YOUR scenes — author here
public/     static assets (default robot GLB)
docs/       guides
```

## Quick reference

| What | How |
|---|---|
| Launch sim + open browser | `dimsim dev --scene <name>` |
| Launch headless | `dimsim dev --scene <name> --headless` |
| List eval workflows | `dimsim eval list` |
| Run one eval against open sim | `dimsim eval <workflow>` |
| Run one eval headless | `dimsim eval --headless --scene <env> --workflow <name>` |
| Run all evals for a scene | `dimsim eval --headless --scene <env>` |
| Run all evals, JUnit XML | `dimsim eval --headless --output junit > junit.xml` |
| Direct workflow execution | `deno run -A scenes/<env>/evals/<name>.js` |
| Build the frontend manually | `cd misc/DimSim && npm run build` |
| Spin up a profiler | `bash scripts/profile-live.sh` |
| Verify cmd_vel → odom round-trip | `python cli/test/dimos_integration.py` |

## Common gotchas

- **Headless slow to boot on CI** — use `--render cpu` and bump `--timeout 120000`.
- **`unitree-go2-basic` hides lidar in Rerun** by override. Click the eye icon in the Rerun entity tree, or use `unitree-go2-spatial` / `unitree-go2-agentic` which leave it visible.
- **Click-to-nav** in Rerun only works on nav-enabled blueprints (`unitree-go2-agentic` and friends). `unitree-go2-basic` has no nav stack.
- **First launch is slow** — Vite builds `dist/` on first run (~20s). Subsequent runs reuse it.
