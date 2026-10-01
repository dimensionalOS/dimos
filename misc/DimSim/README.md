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

## Message encoding

The browser and Deno bridge use ROS 2 CDR payloads with full `package/msg/Type`
channel suffixes. LCM/UDP and the WebSocket channel envelope remain the transport.
Color frames contain raw RGBA pixels in `sensor_msgs/msg/Image`; depth uses
`16UC1`. JPEG is used only for the eval/sidebar preview. Point clouds, poses and
velocity commands use the same generated schema contract as Python, C++ and Rust.
Old LCM-encoded clients and recordings are incompatible with this proposal.

The standalone schema bundle is exported from the canonical generated messages:

```bash
python -m dimos.simulation.dimsim.cdr_schemas
pytest dimos/simulation/dimsim/test_cdr_schemas.py
cd misc/DimSim/cli
deno test bridge/cdr_test.ts
deno check cli.ts
```

Foxglove `rosmsg` 5.0.5 and `rosmsg2-serialization` 3.1.2 are pinned in both
package manifests. Codec tests do not open network sockets or start simulation.
