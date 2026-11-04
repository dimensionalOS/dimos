# Getting started

5-minute tour. For deeper guides see [scenes.md](scenes.md) and [evals.md](evals.md).

## Running DimSim

```bash
cd misc/DimSim/cli
deno run -A --unstable-net cli.ts dev --scene apartment
```

This builds `dist/` with Vite and serves a bridge on port 8090.

If you install the CLI globally, replace the `deno run` boilerplate with `dimsim`:

```bash
cd misc/DimSim/cli
deno install -gAf --unstable-net --name=dimsim --config=./deno.json ./cli.ts
```

## 60-second loop: edit a scene

1. `dimsim dev --scene warehouse`
2. Open `misc/DimSim/scenes/warehouse/index.js` in your editor.
3. Change `setSky({ brightness: 0.7 })` to `setSky({ brightness: 1.5 })`. Save.
4. The browser HMR-reloads with a brighter sky.

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
scenes/     YOUR scenes (author here)
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
| Pre-build `dist/` + headless Chromium (CI, cold checkouts) | `bin/dimsim-prepare` from the dimos root, or `dimsim build` for `dist/` alone |

## Troubleshooting

- **Scene is blank / won't load.** A `build()` error is logged in the browser console
  as `[dimos] Initialization failed: …`. Almost always it's calling a helper you didn't add
  to the `build({ … })` parameter list. For example, using `setSky(...)` when `setSky` isn't in the
  params throws `ReferenceError: setSky is not defined`. Add the missing name to the params.
- **Edited engine code but nothing changed.** The CLI only auto-builds `dist/` when it's
  missing, not when it's stale. After editing `src/` (engine, scene API), run
  `cd misc/DimSim && npm run build`. Editing a scene (`scenes/*/index.js`) needs no
  build. Scenes are served from source, so just hard-refresh.
- **Embodiment or spawn-point change didn't take effect.** These apply once at scene boot,
  not on a plain save. Hard-refresh the tab (`Cmd/Ctrl+Shift+R`) to re-boot the scene. If it
  still looks stuck (stale embodiment, robot in the old spot), restart the bridge
  (`dimsim dev --scene <name>`) for a clean reset.
- **First launch is slow.** Vite builds `dist/` on first run (~20s); later runs reuse it.
- **Headless slow to boot on CI.** Use `--render cpu` and bump `--timeout 120000`.
