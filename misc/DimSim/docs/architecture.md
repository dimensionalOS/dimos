# DimSim architecture

```
                          dimos (Python)
                                │
                                │  LCM/UDP + control WS
                                ▼
            ┌───────────────────────────────────────┐
            │       Bridge   (Deno, cli/bridge/)    │
            │   • LCM multicast (/cmd_vel, /odom,   │
            │     /color_image, /lidar/points)      │
            │   • Server-side Rapier physics step   │
            │   • WS relay to browser               │
            └─────────────────┬─────────────────────┘
                              │  WebSocket (control + sensors)
                              ▼
            ┌───────────────────────────────────────┐
            │     Browser engine  (vite-bundled,    │
            │     src/engine.js + friends)          │
            │   • Three.js scene + agent visual     │
            │   • Browser-side Rapier (display)     │
            │   • Loads scenes/<name>/index.js      │
            │   • Loads scenes/<name>/evals/*.js    │
            │     (when triggered)                  │
            └───────────────────────────────────────┘
```

## File layout

```
misc/DimSim/
  src/                  ← vite-bundled browser code
    engine.js
    AiAvatar.js               # agent visual + capsule colliders
    main.js / style.css       # entry + UI
    bridge.ts                 # browser WS client to the bridge
    sceneApi.ts               # the API scenes import from
    sceneEditor.ts            # runtime exec sandbox (for SceneClient SDK)

  cli/                  ← Deno-only code (not bundled)
    cli.ts                    # `dimsim` CLI entrypoint
    deno.json                 # JSR import map + tasks
    bridge/server.ts          # the bridge server itself
    bridge/lidar.ts           # server-side ray-cast lidar
    bridge/physics.ts         # server-side Rapier step
    headless/launcher.ts      # Playwright bootstrap
    vendor/lcm/*              # vendored @dimos/lcm w/ macOS multicast fix
    test/dimos_integration.py # LCM smoke test
    agent.py                  # dimos Python agent runner

  evals/                ← eval system, both runtimes
    harness.ts                # browser orchestrator (bundled → dimsim-eval.js)
    rubrics.ts                # objectDistance, radiusContains, helpers
    runner.ts                 # Deno runner — walks scenes/, dispatches via WS
    deno-client.ts            # Deno `@dimsim/eval` (for direct deno-run)
    deno.json                 # @std/path import map + Deno LSP hint

  scenes/               ← user-authored scenes
    <env>/
      index.js                # default-exports an async build({...}) function
      data/, textures/        # optional, scene-specific
      evals/<workflow>.js     # workflow files (importable in both browser + Deno)
    deno.json                 # @dimsim/eval → ../evals/deno-client.ts

  public/                     # static assets, copied by vite
    agent-model/dimsim_unitree_stub.glb
    logo.svg

  index.html                  # boot html (carries the importmap)
  vite.config.js              # pins evals/harness.ts → dist/assets/dimsim-eval.js
  package.json                # vite + three + rapier + spark (browser only)
```

## Key contracts

**Bridge ↔ browser** — WebSockets on port 8090:

- `/?ch=control` — text JSON commands (runEval, embodimentConfig, exec, …) + LCM binary frames
- `/?ch=sensors` — sensor publishes (lidar, snapshots)
- `/?ch=rgb`, `/?ch=depth` — image streams

**Bridge ↔ dimos** — LCM multicast at `239.255.76.67:7667`:

- `/cmd_vel#geometry_msgs.Twist` — in
- `/odom#geometry_msgs.PoseStamped` — out
- `/color_image#sensor_msgs.Image` — out
- `/lidar/points#sensor_msgs.PointCloud2` — out

**Scene module** — `scenes/<name>/index.js`:

```js
export default async function build(api) {
  const { scene, THREE, physics, setSky, loadGLTF, loadLevel } = api;
  // … construct meshes, add colliders, set lighting …
  return { embodiment: null, spawnPoint: { x: 0, y: 0.5, z: 0 } };
}
```

**Eval workflow** — `scenes/<name>/evals/<workflow>.js`:

```js
import { runEval } from '@dimsim/eval';
await runEval({
  scene: '<name>',
  task: 'human-readable description',
  timeoutSec: 30,
  startPose: { x: 0, y: 0.5, z: 0, yaw: 0 },
  setup:   async (ctx) => { /* optional */ },
  success: (ctx) => ctx.rubrics.objectDistance({ target: 'X', thresholdM: 2.0 }),
});
```

The `@dimsim/eval` specifier resolves two ways via two import maps:

- **Browser** (`index.html`) → `/assets/dimsim-eval.js` (bundled harness chunk, pinned by vite.config.js)
- **Deno** (`scenes/deno.json`) → `evals/deno-client.ts` (dispatches to the bridge via WS)

Same workflow file, two runtimes.
