# DimSim

Browser-based 3D simulator (Three.js + Rapier) plus a Deno bridge that talks LCM/WS to [dimos](https://github.com/dimensionalOS/dimos). Lives inside dimos as `misc/DimSim/`.

```
src/        — browser engine (vite-bundled)
cli/        — Deno CLI + bridge server + headless launcher + LCM vendor
evals/      — eval harness (browser) + runner (Deno) + rubrics
scenes/     — user-authored scenes (JS) + per-scene eval workflows
public/     — static assets (agent GLB, logo)
docs/       — guides
```

## Run

dimsim is launched by dimos directly when you pick `--simulation dimsim`:

```bash
cd <dimos-repo>
uv run dimos --simulation dimsim --dimsim-scene=apartment run unitree-go2-agentic
```

On first run, `cli/cli.ts` will build `dist/` via Vite (dimsim ships its frontend as source — Deno+Vite materializes it in ~20s).

## Docs

- [docs/getting-started.md](docs/getting-started.md) — 5-minute tour
- [docs/scenes.md](docs/scenes.md) — create + edit scenes
- [docs/evals.md](docs/evals.md) — write eval workflows

## Install the CLI (optional)

If you want `dimsim` as a global command:

```bash
cd misc/DimSim/cli
deno install -gAf --unstable-net --name=dimsim --config=./deno.json ./cli.ts
```

After install:

```bash
dimsim dev --scene apartment              # standalone dev server + browser
dimsim eval list                          # list workflows under scenes/*/evals/
dimsim eval go-to-couch                   # run one workflow against an open sim
dimsim eval --headless --scene apartment  # full headless run (CI)
```

## Build manually

```bash
npm install      # browser deps (three, rapier, vite)
npm run build    # → dist/
```

## Canonical message encoding

The browser and Deno physics/lidar bridge publish generated ROS 2 CDR layouts
over the existing raw LCM/WebSocket transport. Channels include `/cdr/` and the
ROS type name, for example `cmd_vel/cdr/geometry_msgs/msg/Twist`. Color frames
are `rgba8` Images with raw canvas pixels; depth stays clamped uint16 millimeters.
The eval/sidebar JPEG remains a separate presentation path.

`cli/bridge/cdr_schemas.json` is a generated four-type schema artifact exported
from the installed canonical DimOS message package. The Python acceptance test
checks every complete dependency schema against that package. Browser and Deno
share one codec; no ROS installation is needed. The existing CLI rebuilds an
old cached frontend when the CDR sources, schema artifact or package manifest
change, and prepares the pinned Foxglove dependencies when missing.

For an offline encoding/physics check with development dependencies installed:

```bash
cd misc/DimSim
deno install --no-lock
deno run --allow-all --unstable-net cli/cli.ts build
cd ../..
DIMOS_DIMSIM_DENO="$(command -v deno)" DIMOS_TRANSPORT=lcm LCM_DEFAULT_URL=memq:// \
  uv run pytest -o addopts='' --import-mode=importlib -q \
  dimos/robot/unitree/test_dimsim_connection.py \
  dimos/simulation/dimsim/test_dimsim_process.py
```

Dependency preparation can download the declared packages; the tests themselves
run Deno with cached dependencies and no network permission. They exercise
actual browser publishing methods, two bounded CPU Rapier physics steps, the
command deadman, one real lidar scan, independent Python CDR decoding and local
odom-to-TF cleanup. They do not launch a rendered scene or perform a benchmark.
