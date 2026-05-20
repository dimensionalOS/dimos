# Getting started

This is the 5-minute tour. For deeper guides see [scenes.md](scenes.md) and [evals.md](evals.md).

## The two ways DimSim runs

```
dimos drives DimSim         (production / agentic / CI)
    .venv/bin/dimos --simulation dimsim --dimsim-scene=apartment run unitree-go2-agentic

DimSim runs standalone      (engine dev / scene authoring)
    cd misc/DimSim/cli
    deno run -A --unstable-net cli.ts dev --scene apartment
```

Both end up with a Vite-built `dist/` and a bridge on port 8090. Both serve the same scenes. Difference is just who's driving the agent.

If you have `dimsim` installed globally:

```bash
cd misc/DimSim/cli
deno install -gAf --unstable-net --name=dimsim --config=./deno.json ./cli.ts
```

…you can replace the `deno run -A --unstable-net cli.ts` boilerplate with just `dimsim`.

## A 60-second loop: edit a scene and see it

1. `dimos --simulation dimsim --dimsim-scene=warehouse --no-dimsim-headless run unitree-go2-basic`
2. Wait for the browser to open and the agent to spawn.
3. Open `misc/DimSim/scenes/warehouse/index.js` in your editor.
4. Change `setSky({ brightness: 0.7 })` to `setSky({ brightness: 1.5 })`. Save.
5. The browser HMR-reloads — no full refresh needed. Brighter sky.

## A 60-second loop: write a new eval

1. Create `misc/DimSim/scenes/apartment/evals/look-at-window.js`:

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

2. From the sim that's already running:

   ```bash
   dimsim eval look-at-window
   ```

   You'll see a green/red overlay in the browser when it finishes, plus the result echoed in your terminal.

## Where things live

```
src/        engine + scene API + bridge client (browser, vite-bundled)
cli/        dimsim CLI + bridge server (Deno)
evals/      eval harness + rubrics + Deno client (both runtimes)
scenes/     YOUR scenes — author here
public/     static assets (default robot GLB)
docs/       guides
```

See [architecture.md](architecture.md) for the full tree + key contracts.

## Quick reference

| What | How |
|---|---|
| Launch sim + open browser | `dimsim dev --scene <name>` |
| Launch headless | `dimsim dev --scene <name> --headless` |
| List eval workflows | `dimsim eval list` |
| Run one eval against open sim | `dimsim eval <workflow>` |
| Run one eval headless | `dimsim eval --headless --scene <env> --workflow <name>` |
| Run all evals for a scene | `dimsim eval --headless --scene <env>` |
| Run all evals, JUnit XML out | `dimsim eval --headless --output junit > junit.xml` |
| Direct workflow execution | `deno run -A scenes/<env>/evals/<name>.js` |
| Build the frontend manually | `cd misc/DimSim && npm run build` |
| Spin up a profiler | `bash scripts/profile-live.sh` |
| Verify cmd_vel → odom round-trip | `python cli/test/dimos_integration.py` |

## What about apt.json?

The old apartment scene used to ship as one 97 MB `apt.json`. It was decomposed into JS-authored modules under `scenes/apartment/data/` plus a `textures/` folder. If you want to regenerate it from an apt.json blob you have lying around:

```bash
python scripts/extract_apt_to_js.py /path/to/apt.json
```

The output goes back into `scenes/apartment/data/` and `scenes/apartment/textures/`.

## Common gotchas

- **Headless browser slow to boot** on CI — use `--render cpu` and bump `--timeout 120000`.
- **macOS multicast loopback** — DimSim ships a vendored `@dimos/lcm` with a `setLoopback(true)` patch. The JSR-published version doesn't have it; dropping the vendor will break dimos↔DimSim LCM on macOS until the upstream lands the fix.
- **`unitree-go2-basic` hides lidar in Rerun** — that blueprint sets `world/lidar` visible=false in its override. Click the eye icon in the Rerun entity tree, or use `unitree-go2-spatial` / `unitree-go2-agentic` which leave it visible.
- **Click-to-nav** in Rerun only works on nav-enabled blueprints (`unitree-go2-agentic` and friends). `unitree-go2-basic` has no nav stack.
