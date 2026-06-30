# Scene Cooking

Scene cooking turns an authored 3D environment into a DimOS scene package. A
package is the runtime contract for simulators and viewers: it contains a
manifest, backend-specific browser assets, MuJoCo collision assets, optional
runtime entities, and metadata about frames and alignment.

The robot is not baked into a normal scene package. Runtime code loads the
package and attaches the robot it wants to simulate.

```text
source asset + cook sidecar + alignment -> scene package -> runtime attaches robot
```

## Inputs

The cooker needs:

```text
source asset      .glb, .gltf, .blend, .usd, .obj, .ply, .stl, ...
cook sidecar      <scene>.cook.json, optional but strongly recommended
alignment         scale, rotation, translation, and up-axis
output directory  data/scene_packages/<name>
visual target     rerun, babylon, or generic
```

Install the Python dependencies with:

```bash
uv sync --extra scene
```

External tools are installed separately:

```text
blender   required for .blend files and entity visual extraction
gltfpack  recommended for browser visual optimization
```

## Inspect First

Do not write collision policy blind. Inspect the source in the same frame the
cooker will use:

```bash
python - <<'PY'
from pathlib import Path

import numpy as np

from dimos.experimental.pimsim.scene.assets.source import prepare_scene_source
from dimos.experimental.pimsim.scene.assets.mesh import SceneMeshAlignment, load_scene_prims

source = Path("data/dimos_office_mesh/dimos_office_mesh.glb")
prepared = prepare_scene_source(source)
alignment = SceneMeshAlignment(scale=2.0, y_up=False)

for prim in load_scene_prims(prepared.cook_path, alignment=alignment):
    name = prim.visual_node_name or prim.prim_path or prim.name
    lo = np.min(prim.vertices, axis=0)
    hi = np.max(prim.vertices, axis=0)
    extent = hi - lo
    if "Floor" in name or "Wall" in name:
        print(
            name,
            "min=", lo.round(4).tolist(),
            "max=", hi.round(4).tolist(),
            "extent=", extent.round(4).tolist(),
        )
PY
```

For `.blend` files, `prepare_scene_source()` runs Blender headlessly and exports
the evaluated dependency graph to a cached GLB. Geometry Nodes and collection
instances are realized before the normal cook starts.

## Author Collision Policy

The cook sidecar is where scene-specific physics intent lives. Use it for
support surfaces, walls, fixtures, interactables, and repeated entity groups.

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

That means: match source prims named `Floor*`, cook them as boxes, make them at
least 4 cm thick, and keep the authored top surface height unchanged. This is
better than adding an infinite MuJoCo plane because it preserves multi-floor
buildings, ramps, platforms, and holes in the environment.

Static collision types:

```text
auto | box | sphere | cylinder | capsule | plane | hull | mesh | decompose | skip
```

Interactables and entity groups become runtime entities in `scene.meta.json`.
Mesh entities are extracted once, decomposed once, and reused by instances when
the sidecar gives a shared prototype key.

## Choose A Browser Visual Target

Browser-facing GLBs are backend-specific. Different viewers support different
glTF extensions and texture encodings, so the cooker exposes explicit visual
targets instead of hardcoding one heuristic.

```text
rerun    conservative GLB for Rerun; no mesh quantization, normalized textures
babylon  web-oriented GLB for Babylon/PimSim; quantization and instancing allowed
generic  conservative generic GLB without Rerun-specific cleanup
```

Cooked packages store these under named artifacts. Runtime consumers should ask
for the asset they support:

```python
rerun_glb = package.browser_visual_path("rerun")
babylon_glb = package.browser_visual_path("babylon")
```

If a target returns `None`, the package was not cooked for that viewer.

## Cook

Cook the office package for Rerun:

```bash
python -m dimos.experimental.pimsim.scene.cook \
  data/dimos_office_mesh/dimos_office_mesh.glb \
  --cook-spec data/dimos_office_mesh/dimos_office_mesh.cook.json \
  --output-dir data/scene_packages/dimos_office \
  --scale 2.0 \
  --no-y-up \
  --visual-target rerun \
  --rebake
```

Cook a Babylon/PimSim visual instead:

```bash
python -m dimos.experimental.pimsim.scene.cook \
  data/my_scene/source.blend \
  --cook-spec data/my_scene/source.cook.json \
  --output-dir data/scene_packages/my_scene \
  --visual-target babylon \
  --rebake
```

The target profile supplies defaults. Override only when needed:

```bash
python -m dimos.experimental.pimsim.scene.cook \
  data/my_scene/source.glb \
  --cook-spec data/my_scene/source.cook.json \
  --output-dir data/scene_packages/my_scene \
  --visual-target babylon \
  --visual-texture-format webp \
  --visual-max-texture-size 2048 \
  --rebake
```

Native `gltfpack` is required for WebP/KTX2 texture compression. The Node/npx
package can optimize geometry but does not support those texture modes.

## Output Layout

```text
data/scene_packages/<name>/
├── scene.meta.json
├── browser/
│   ├── visual.rerun.glb       backend-specific visual, if cooked
│   ├── visual.babylon.glb     backend-specific visual, if cooked
│   ├── collision.glb          simplified raycast/picking collision
│   └── objects.json           per-prim object table for browser users
├── mujoco/<hash>/
│   ├── wrapper.xml            scene-only MuJoCo XML
│   ├── wrapper.mjb            optional scene-only MuJoCo binary
│   └── *.obj                  static collision meshes
├── mujoco/composed/
│   └── <robot>_<spawn>.mjb    optional robot+scene binary
└── entities/<id>/
    ├── visual.glb
    └── mujoco_collision/
```

`scene.meta.json` stores package-relative paths, frame metadata, entities, and
cook statistics. Runtime code should read it through `load_scene_package()` or
`resolve_scene_package()`.

## MuJoCo Artifacts

The normal MuJoCo artifact is scene-only XML:

```text
scene.meta.json -> wrapper.xml + entities -> runtime attaches robot MJCF -> MjModel
```

This keeps the package robot-agnostic. It is the right path for office-scale
scenes and for packages that should work with multiple robots.

Huge scenes can additionally use composed `.mjb` files:

```text
wrapper.xml + robot MJCF + spawn/entity selection -> composed/<name>.mjb
```

A composed `.mjb` loads quickly, but it is specific to one robot, spawn pose,
runtime entity set, and scene revision. Rebuild it when any of those change.

## Load The Cooked Package

Run the G1 GR00T WBC blueprint with a cooked package:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene office \
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

## Reference Files

```text
dimos/experimental/pimsim/scene/cook.py              cook CLI
dimos/experimental/pimsim/scene/sidecar.py           cook sidecar schema
dimos/experimental/pimsim/scene/plan.py              sidecar to entity/collision plan
dimos/experimental/pimsim/scene/assets/source.py     .blend normalization
dimos/experimental/pimsim/scene/assets/mesh.py       source mesh inspection
dimos/experimental/pimsim/scene/assets/glb.py        GLB rewrite helpers
dimos/experimental/pimsim/scene/visuals/glb.py       browser visual cooking
dimos/experimental/pimsim/scene/visuals/blender.py   Blender visual extraction
dimos/experimental/pimsim/scene/collision/browser.py browser collision mesh
dimos/experimental/pimsim/scene/collision/entity.py  entity collision hulls
dimos/experimental/pimsim/scene/collision/mujoco/export.py MuJoCo XML bake
dimos/experimental/pimsim/scene/collision/mujoco/spec.py   static collision policy
dimos/simulation/scene_assets/spec.py                package contract
dimos/simulation/scenes/catalog.py                   runtime name/path resolution
```
