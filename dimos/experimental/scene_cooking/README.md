# Scene Cooking

Scene cooking turns an authored 3D environment into a DimOS scene package.

A scene package is one environment with the representations downstream systems
need: a manifest, frame metadata, browser assets, MuJoCo collision assets,
browser raycast assets, and optional runtime entities. The robot is not baked
into the normal package. Runtime code loads the package and attaches the robot
it wants to simulate.

```text
source asset + cook sidecar + alignment -> scene package -> runtime consumers
```

Use this README to cook and load a package. See
[`ARCHITECTURE.md`](ARCHITECTURE.md) for how the pipeline is organized.

## Install

Python dependencies:

```bash
uv sync --extra scene
```

External tools:

```text
blender   required for .blend files and entity visual extraction
gltfpack  recommended for browser visual optimization
```

## Inputs

```text
source asset      .glb, .gltf, .blend, .usd, .obj, .ply, .stl, ...
cook sidecar      <scene>.cook.json, optional but recommended
alignment         scale, rotation, translation, and up-axis
output directory  data/scene_packages/<name>
visual target     rerun, babylon, or generic
```

The sidecar is where authored scene intent lives: collision choices, support
surfaces, walls, fixtures, interactables, and repeated entity groups.

## Inspect

Inspect the source in the same frame the cooker will use before writing
collision policy:

```bash
python - <<'PY'
from pathlib import Path

import numpy as np

from dimos.experimental.scene_cooking.source_assets.normalize import prepare_scene_source
from dimos.experimental.scene_cooking.source_assets.mesh import SceneMeshAlignment, load_scene_prims

source = Path("data/office_scene_cooking_example/dimos_office_mesh.glb")
prepared = prepare_scene_source(source)
alignment = SceneMeshAlignment(scale=2.0, y_up=False)

for prim in load_scene_prims(prepared.cook_path, alignment=alignment):
    name = prim.visual_node_name or prim.prim_path or prim.name
    if "Floor" not in name and "Wall" not in name:
        continue
    lo = np.min(prim.vertices, axis=0)
    hi = np.max(prim.vertices, axis=0)
    print(name, "min=", lo.round(4).tolist(), "max=", hi.round(4).tolist())
PY
```

For `.blend` files, `prepare_scene_source()` runs Blender headlessly and exports
the evaluated dependency graph to a cached GLB. Geometry Nodes and collection
instances are realized before the normal cook starts.

## Author A Sidecar

Example floor policy:

```json
{
  "collision": {
    "default": "auto",
    "prim_overrides": {
      "Floor*": {
        "type": "box",
        "min_thickness": 0.04,
        "preserve": "top"
      }
    }
  }
}
```

This matches source prims named `Floor*`, cooks them as boxes, makes them at
least 4 cm thick, and keeps the authored top surface height unchanged. This is
usually better than adding an infinite MuJoCo plane because it preserves
multi-floor buildings, ramps, platforms, and holes.

Static collision types:

```text
auto | box | sphere | cylinder | capsule | plane | hull | mesh | decompose | skip
```

Interactables and entity groups become runtime entities in `scene.meta.json`.
Mesh entities are extracted once, decomposed once, and reused by instances when
the sidecar gives a shared prototype key.

## Cook

Cook the office package for Rerun:

```bash
uv run --extra scene python -m dimos.experimental.scene_cooking.cook \
  data/office_scene_cooking_example/dimos_office_mesh.glb \
  --cook-spec data/office_scene_cooking_example/dimos_office_mesh.cook.json \
  --output-dir data/scene_packages/dimos_office \
  --scale 2.0 \
  --no-y-up \
  --visual-target rerun \
  --rebake
```

Cook for a different browser backend:

```bash
uv run --extra scene python -m dimos.experimental.scene_cooking.cook \
  data/my_scene/source.blend \
  --cook-spec data/my_scene/source.cook.json \
  --output-dir data/scene_packages/my_scene \
  --visual-target babylon \
  --rebake
```

Visual targets are explicit because GLB support differs by viewer:

```text
rerun    conservative GLB for Rerun; no mesh quantization, normalized textures
babylon  web-oriented GLB for Babylon/PimSim; quantization and instancing allowed
generic  conservative generic GLB without Rerun-specific cleanup
```

Native `gltfpack` is required for WebP/KTX2 texture compression. The Node/npx
package can optimize geometry but does not support those texture modes.

## Output

```text
data/scene_packages/<name>/
├── scene.meta.json
├── browser/
│   ├── visual.rerun.glb
│   ├── visual.babylon.glb
│   ├── collision.glb
│   └── objects.json
├── mujoco/<hash>/
│   ├── wrapper.xml
│   ├── wrapper.mjb
│   └── *.obj
├── mujoco/composed/
│   └── <robot>_<spawn>.mjb
└── entities/<id>/
    ├── visual.glb
    └── mujoco_collision/
```

`scene.meta.json` stores package-relative paths, frame metadata, entities, and
cook statistics. Runtime code should read it through `load_scene_package()` or
`resolve_scene_package()`.

## Load

Runtime consumers ask for the package representation they support:

```python
from dimos.simulation.scene_assets.spec import load_scene_package
from dimos.simulation.scenes.catalog import resolve_scene_package

package = resolve_scene_package("office")
# or: package = load_scene_package("data/scene_packages/dimos_office/scene.meta.json")

rerun_glb = package.browser_visual_path("rerun")
babylon_glb = package.browser_visual_path("babylon")
mujoco_scene = package.mujoco_scene_path
entities = package.entities
```

Run the G1 GR00T WBC blueprint with a cooked package:

```bash
dimos --simulation mujoco \
  --scene-package office \
  --viewer rerun \
  --n-workers 12 \
  run unitree-g1-groot-wbc \
  -o mujocosimmodule.headless=true
```

Use `headless=true` for normal testing and inspect Rerun. Use
`headless=false` only when the native MuJoCo window is needed for contact or
render debugging; it can run much slower.

## Data Publishing

Raw scene sources and cooked packages live under `data/` and should not be
added with ordinary `git add`. Publish large data through the LFS bin workflow
documented in:

```text
docs/development/large_file_management.md
```

Code and docs changes go through normal git. Data archives should be updated
with `./bin/lfs_push` when the package is ready to ship.
