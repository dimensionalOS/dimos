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

### Object labels and 3D boxes

Rebuild the frontend after changing its source. For a standalone apartment viewer,
run from the dimos repository root:

```bash
npm --prefix misc/DimSim run build
deno run -A --unstable-net --config misc/DimSim/cli/deno.json misc/DimSim/cli/cli.ts dev --scene apartment
```

Open `http://localhost:8090` and click **Object labels + boxes** in the bottom-left
controls. The toggle is also available when using an existing DimSim-backed
blueprint:

```bash
uv run dimos --simulation dimsim --dimsim-scene apartment --no-dimsim-headless --viewer none run unitree-go2
```

V1 displays a static snapshot of world-axis-aligned visual bounds and authored
asset names. Toggle off/on to refresh after editing or moving objects. Labels
cover identified scene assets plus walls: structure nodes named like `wall-north`
or `yard-wall-east` (and walls added through `SceneClient.add_wall`) are boxed
under their node name. Other baked fixtures are not separately labeled.
Decorative blob shadows are excluded from bounds. The
annotations render only in the viewer's RGB view (and RGB comparison tile),
separately from the physics scene and robot RGB/depth/LiDAR captures.

The same snapshot is available from Python as typed 3D detections
(`dimos.msgs.vision_msgs.Detection3DArray`) in the DimOS Z-up `world` frame, the
frame DimSim publishes odometry in. Each detection carries the stable asset `id`,
the authored label as `results[0].hypothesis.class_id` with score 1.0, and a
world-axis-aligned `bbox` with identity orientation. While the overlay is on, the
displayed snapshot and its capture time are exported; otherwise the current
geometry is measured once.

```python skip
from dimos.simulation.dimsim.scene_client import SceneClient

client = SceneClient()  # bridge on localhost:8090
client.start()
try:
    detections = client.get_object_detections()
    client.export_object_detections("dimsim-objects.bin")  # typed LCM payload
    client.export_object_detections("dimsim-objects.json")  # readable view
finally:
    client.stop()
```

The `.bin` file is one LCM-encoded `Detection3DArray` message, not an LCM event
log; read it back with `dimos.simulation.dimsim.object_detections.read_detection3d_array`.
The `.json` file is a convenience view with `id`, `label`, `center_xyz`, `size_xyz`
and `orientation_xyzw` per detection in the same `world` frame.
A reference export of the apartment (87 assets + 20 walls) is checked in at
`scenes/apartment/object_detections.json`.

Consumers run over LCM. Publish the snapshot on the usual 3D detection channel and
watch it with `dimos lcmspy`:

```python skip
from dimos.core.transport import LCMTransport
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.simulation.dimsim.scene_client import SceneClient

client = SceneClient()
client.start()
transport = LCMTransport("/detections_3d", Detection3DArray)
try:
    transport.publish(client.get_object_detections())
finally:
    transport.stop()
    client.stop()
```

Focused checks:

```bash
node --test misc/DimSim/src/objectAnnotations.test.js
uv run pytest dimos/simulation/dimsim/test_object_detections.py
```

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
