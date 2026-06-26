# Scene Packages

A scene package is the cooked, robot-agnostic form of a 3D environment. It
contains the visual mesh, collision geometry, per-object metadata, a scene-only
MuJoCo wrapper, and optional runtime entities.

The runtime rule is simple:

```text
source asset + sidecar -> cooked scene package -> simulator attaches any robot
```

The robot is not part of the package. The cooker prepares the world once,
offline. `MujocoSimModule` loads the cooked world at runtime and attaches the
robot MJCF into the same `MjSpec`.

## Package Layout

```text
data/scene_packages/<name>/
├── scene.meta.json          manifest: alignment, artifact paths, entities, stats
├── mujoco/<hash>/
│   ├── wrapper.xml          scene-only MJCF, no robot
│   ├── wrapper.mjb          optional scene-only compiled MuJoCo model
│   └── *.obj                static collision assets
├── mujoco/composed/
│   └── <robot>_<spawn>.mjb  optional robot+scene compiled MuJoCo model
├── entities/<id>/
│   ├── visual.glb           per-entity visual, in entity-local frame
│   └── mujoco_collision/    cook-time convex hulls
└── browser/
    ├── visual.glb
    ├── collision.glb
    └── objects.json         semantic object table for browser/raycast users
```

Packages are content-hash keyed on the source mesh, alignment, sidecar, and cook
schema version. Changing one of those inputs creates a new cooked output.

## MuJoCo Artifact Split

There are two MuJoCo loading modes. Keep the distinction explicit when reviewing
or testing scene packages.

### Scene Package XML

This is the default path:

```text
scene.meta.json -> mujoco/<hash>/wrapper.xml + entities -> runtime attaches robot -> compile MjModel
```

Use it for normal scenes and for robot-agnostic packages. The package contains
the world only. At runtime `MujocoSimModule` loads `wrapper.xml`, attaches the
requested robot MJCF, adds dynamic entities from `scene.meta.json`, and compiles
one in-memory model. This keeps one cooked package usable by many robots and
spawn points.

Tradeoff: MuJoCo XML compile cost is paid at startup. That is fine for office-
scale scenes. It is not fine for product-dense scenes with tens of thousands of
individual geoms.

### Composed Binary MJB

This is the fast-load path for huge scenes:

```text
wrapper.xml + robot MJCF + selected spawn/entities -> composed/<robot>_<spawn>.mjb
```

Use it when XML compile time dominates startup, such as the supermarket scene
with thousands of shelf products. The `.mjb` already contains the robot, scene,
spawn pose, static collision, and any runtime entities chosen for that build.
MuJoCo loads that binary model directly.

Tradeoff: a composed `.mjb` is not robot-agnostic. Build one per robot model,
spawn/entity configuration, and meaningful scene revision. You also cannot edit
it with `MjSpec` after loading it; if the robot, spawn, or dynamic-entity set
changes, rebuild the binary from the XML package.

The scene-only `wrapper.mjb` produced by `--compile-mujoco-binary` is useful for
profiling and cache validation, but it does not replace runtime robot attach.
For G1 WBC testing, use a composed robot+scene `.mjb`.

## Spec And Backends

`dimos/simulation/scene_assets/spec.py` is the shared scene-package contract. It
defines:

- cook input specs such as `SceneCookSpec`, `BrowserVisualSpec`,
  `BrowserCollisionSpec`, and `MujocoSceneSpec`;
- the runtime `ScenePackage` object;
- the on-disk `scene.meta.json` shape;
- `load_scene_package()`, which resolves package-relative artifact paths and
  validates artifact frame metadata.

This spec is intentionally general. A module or simulator that wants to consume
a scene package should not parse `scene.meta.json` by hand. It should load the
package once:

```python
from dimos.simulation.scene_assets.spec import load_scene_package

package = load_scene_package("data/scene_packages/dimos_office/scene.meta.json")
```

and then consume the artifact it understands:

- browser/viewer systems use `package.visual_path`, `package.browser_collision_path`,
  and `package.objects_path`;
- MuJoCo uses `package.mujoco_scene_path` plus `package.entities`;
- future simulators can add their own artifact fields/specs without changing the
  source mesh loader or the scene-name catalog.

`dimos/simulation/scene_assets/mesh_scene.py` is a different layer. It loads raw
source geometry and applies `SceneMeshAlignment`; it is used by cook-time tools,
not by normal runtime modules. Runtime modules should prefer the cooked package
contract from `spec.py`.

`dimos/simulation/scenes/catalog.py` is only name/path resolution. It maps user
inputs like `--scene office` to a loaded `ScenePackage`. It does not define what
a package is, and it should not know how MuJoCo, browser viewers, or other
systems load their artifacts.

## Workflow

1. Inspect the source asset and identify the prims that need authored collision.
2. Write `<scene>.cook.json` next to the source mesh.
3. Cook the package.
4. Verify the generated `mujoco/<hash>/wrapper.xml`.
5. Load it through a blueprint with `--scene`.

The DimOS office scene is the reference example below.

## Authored Blender Sources

The cooker accepts `.blend` files as authored scene sources. A Blender file is
not a concrete mesh asset: it can contain view layers, disabled source
collections, Geometry Nodes, procedural instances, cameras, lights, text, and
other authoring data. Before the normal cook starts, DimOS runs Blender
headlessly and normalizes the evaluated dependency graph into a GLB:

```text
scene.blend -> evaluated depsgraph GLB -> normal scene cook
```

The normalizer walks `depsgraph.object_instances`, so Geometry Nodes and
instanced collection content are realized as concrete mesh nodes instead of
being dropped by a plain Blender glTF export. Direct mesh formats (`.glb`,
`.gltf`, `.obj`, `.ply`, `.stl`, `.usd`, `.usda`, `.usdc`, `.usdz`) skip this
step.

To inspect a `.blend` scene with the same geometry the cooker will see:

```bash
python - <<'PY'
from pathlib import Path

import numpy as np

from dimos.experimental.pimsim.scene.source_asset import prepare_scene_source
from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment, load_scene_prims

prepared = prepare_scene_source(Path("data/my_scene/source.blend"))
print(prepared.to_json_dict())

for prim in load_scene_prims(prepared.cook_path, alignment=SceneMeshAlignment()):
    name = prim.visual_node_name or prim.prim_path or prim.name
    if "Floor" not in name:
        continue
    lo = np.min(prim.vertices, axis=0)
    hi = np.max(prim.vertices, axis=0)
    print(name, lo.round(4).tolist(), hi.round(4).tolist())
PY
```

Then cook the `.blend` directly:

```bash
python -m dimos.experimental.pimsim.scene.cook \
  data/my_scene/source.blend \
  --cook-spec data/my_scene/source.cook.json \
  --output-dir data/scene_packages/my_scene \
  --rebake
```

The sidecar still targets normalized mesh prim names. Inspect first, then write
the sidecar against the names printed from `load_scene_prims(prepared.cook_path)`.

To also emit a scene-only MuJoCo binary for profiling or cache validation, add
`--compile-mujoco-binary`:

```bash
python -m dimos.experimental.pimsim.scene.cook \
  data/my_scene/source.blend \
  --cook-spec data/my_scene/source.cook.json \
  --output-dir data/scene_packages/my_scene \
  --compile-mujoco-binary \
  --rebake
```

That writes `mujoco/<hash>/wrapper.mjb`. It is still scene-only; it does not
include a robot.

## Tool Diagnostics

Scene cooking shells out to production asset tools. Their output is streamed
through the DimOS logger with a command label, elapsed-time heartbeats, and a
tail of recent output on failure. Long Blender imports should therefore show
which stage is running: source normalization, authored visual extraction,
browser visual import, decimation, join, or export.

`gltfpack` is the preferred browser visual optimizer for GLB inputs, but install
a native meshoptimizer `gltfpack` binary when using texture compression. The
Node/npx package can optimize geometry, but it is built without WebP/KTX texture
compression support. If a cook fails immediately after requesting
`--visual-texture-format webp` or `ktx2`, put a native `gltfpack` on `PATH` or
use `--visual-texture-format none`.

The default visual cook is conservative: DimOS passes `-noq` to gltfpack so the
output GLB has no required quantization extension. This produces larger files,
but keeps the artifact loadable by generic GLB consumers such as Rerun. Use
`--visual-quantize` only when the target viewer supports `KHR_mesh_quantization`.
With uncompressed textures, DimOS also rewrites embedded images to ordinary
8-bit PNG payloads after gltfpack. This keeps the source materials textured
while avoiding renderer-specific failures from high-bit-depth PNGs or required
texture-compression extensions. Use `--no-visual-texture-normalization` only
when a downstream viewer needs the original embedded texture encoding.
The final GLB sanitize pass also demotes `KHR_texture_transform` from
`extensionsRequired` to `extensionsUsed`, since Rerun rejects it as required but
can still load the asset when the transform is treated as optional.

If `gltfpack` exits with only `unreachable`, that is an internal optimizer
crash. Re-run with the native binary first. If it still fails, diagnose with
`--visual-optimizer blender` or `copy`, or split the visual asset into smaller
source chunks before optimizing.

For product-dense authored scenes, glTF GPU instancing can keep browser visuals
small. The supermarket source has tens of thousands of repeated product
instances; without instancing, an optimizer can turn compact source meshes into
millions of duplicated output triangles. Keep the default off when the package
must render in generic Rerun viewers, and enable it explicitly with
`--visual-gpu-instancing` only for viewers that support
`EXT_mesh_gpu_instancing`.
If preserving the authored node instancing matters more than reducing draw
calls, use `--visual-optimizer copy` after the filtered static visual source has
been generated. That keeps repeated assets as normal glTF nodes instead of
flattening them into a giant mesh.

## Office Example

The office source mesh lives in data:

```text
data/dimos_office_mesh/dimos_office_mesh.glb
```

That file is a visual asset, not a physics contract. Before writing the sidecar,
inspect the scene graph and bounds of the source prims after applying the same
alignment used by the cooker:

```bash
python - <<'PY'
from pathlib import Path

import numpy as np

from dimos.simulation.scene_assets.mesh_scene import (
    SceneMeshAlignment,
    load_scene_prims,
)

source = Path("data/dimos_office_mesh/dimos_office_mesh.glb")
alignment = SceneMeshAlignment(scale=2.0, y_up=False)

for prim in load_scene_prims(source, alignment=alignment):
    name = prim.visual_node_name or prim.prim_path or prim.name
    if "Floor" not in name:
        continue
    lo = np.min(prim.vertices, axis=0)
    hi = np.max(prim.vertices, axis=0)
    extent = hi - lo
    print(f"{name}: min={lo.round(4).tolist()} max={hi.round(4).tolist()} extent={extent.round(4).tolist()}")
PY
```

For the current office GLB, the relevant support prim is `Floor_Plane.002`. Its
Z extent is effectively zero: it is a visual sheet. If that sheet is cooked into
a very thin MuJoCo box, humanoid foot contacts can clip through it during walking
even though the robot can stand still on it.

Do not solve that by adding an infinite MuJoCo plane. That ignores the authored
scene geometry and breaks as soon as floors have different heights, holes,
stairs, ramps, platforms, or multiple stories. The sidecar should instead state
the authored physics intent for the named support surface.

Create or edit:

```text
data/dimos_office_mesh/dimos_office_mesh.cook.json
```

with the floor override:

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

Reasoning:

- `Floor*` matches `Floor_Plane.002` without depending on the sanitized MJCF
  name.
- `type: "box"` says the floor is a support slab, not a raw visual mesh.
- `min_thickness: 0.04` gives the support 4 cm of total thickness.
- `preserve: "top"` keeps the walkable surface at the authored visual height and
  expands the slab downward.

Cook the office package:

```bash
python -m dimos.experimental.pimsim.scene.cook \
  data/dimos_office_mesh/dimos_office_mesh.glb \
  --cook-spec data/dimos_office_mesh/dimos_office_mesh.cook.json \
  --output-dir data/scene_packages/dimos_office \
  --scale 2.0 \
  --no-y-up \
  --rebake
```

Verify the generated MuJoCo wrapper:

```bash
rg 'Floor.*_col' data/scene_packages/dimos_office/mujoco -g wrapper.xml
```

For a floor whose top is at `z=0`, the cooked geom should have half-thickness
`0.02` and center `z=-0.02`, for example:

```xml
<geom name="Floor_Plane_002_col" type="box" pos="... ... -0.02" size="... ... 0.02"/>
```

The important invariant is top height unchanged, bottom expanded downward:

```text
top_z = pos_z + size_z
bottom_z = pos_z - size_z
```

## Loading In MuJoCo

The G1 GR00T WBC blueprint already uses the scene package path when `--scene`
is provided:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene office \
  --viewer rerun \
  run unitree-g1-groot-wbc
```

`office` resolves through `dimos/simulation/scenes/catalog.py` to:

```text
data/scene_packages/dimos_office/scene.meta.json
```

At runtime, `MujocoSimModule`:

1. receives a `ScenePackage` resolved by `catalog.py`;
2. reads `package.mujoco_scene_path`, which points at
   `mujoco/<hash>/wrapper.xml`;
3. loads the robot-only G1 MJCF;
4. attaches the robot into the scene `MjSpec`;
5. adds `package.entities`;
6. compiles one MuJoCo model.

The robot MJCF must stay robot-only: no office floor, no scene walls, no
furniture, no manipulation rig. Scene geometry belongs in the cooked package.

For a large scene where XML compile is too slow, load a composed binary model
instead of the scene package metadata:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene /home/pim/Desktop/dimos-scene-cooking-part2/data/scene_packages/supermarket_static_product_primitives_20dyn/mujoco/composed/unitree-g1-groot-wbc_spawn_9p2_11p8_yaw_m1p57.mjb \
  --n-workers 10 \
  run unitree-g1-groot-wbc \
  -o mujocosimmodule.headless=false
```

That path skips scene-package runtime composition. The `.mjb` already contains
the G1 robot and the supermarket scene at the authored test spawn.

## Sidecar Schema

`<scene>.cook.json` can contain static-collision policy and interactables:

```json
{
  "collision": {
    "default": "auto",
    "prim_overrides": {
      "Floor*": {"type": "box", "min_thickness": 0.04, "preserve": "top"},
      "Wall_*": {"type": "box"},
      "Stairs_*": {"type": "decompose", "max_hulls": 16}
    }
  },
  "interactables": [
    {
      "id": "chair_000",
      "source_prim_paths": ["Chair_000_*"],
      "mass": 8.0,
      "physics": {"shape": "mesh"},
      "tags": ["chair"]
    }
  ]
}
```

Static collision types are:

```text
auto | box | sphere | cylinder | capsule | plane | hull | mesh | decompose | skip
```

For box overrides, `min_thickness` is full world-Z thickness in meters.
`preserve` can be `top`, `bottom`, or `center`.

Interactables become MuJoCo bodies named `entity:<id>`. They can be:

- extracted from the source mesh with `source_prim_paths`;
- synthetic primitives with an authored `pose` and `physics.extents`;
- static, kinematic, or dynamic depending on `kind` and `mass`.

## Data Publishing

Scene source meshes and cooked packages live under `data/`, which is ignored by
normal git. Do not add these artifacts with ordinary `git add`. Publish or update
them through the repository LFS bin workflow described in:

```text
docs/development/large_file_management.md
```

Code and docs changes can go through normal git. Data changes such as
`data/dimos_office_mesh/dimos_office_mesh.cook.json` and
`data/scene_packages/dimos_office` should be handled through the LFS script when
we are ready to ship them.

## Why Bake

MuJoCo treats a mesh geom as convex for collision. A raw concave building mesh is
therefore the wrong collision representation: it becomes a coarse blob or a bad
support surface. The cooker turns source prims into MuJoCo-friendly collision:
primitives where the shape is obvious, hulls where they fit, and CoACD
decompositions for concave objects.

The sidecar is the place for source-specific knowledge. It keeps the cook
deterministic and reviewable instead of relying on broad heuristics that may fail
on multi-story buildings or unusual floor geometry.

## Reference

| File | Role |
|---|---|
| `dimos/experimental/pimsim/scene/cook.py` | cook entry point and CLI |
| `dimos/experimental/pimsim/scene/sidecar.py` | `<scene>.cook.json` schema |
| `dimos/experimental/pimsim/scene/plan.py` | sidecar to cook plan |
| `dimos/experimental/pimsim/scene/inspect.py` | source asset statistics |
| `dimos/simulation/scene_assets/spec.py` | shared scene-package metadata contract |
| `dimos/simulation/scene_assets/mesh_scene.py` | source mesh loading and prim inspection |
| `dimos/simulation/mujoco/scene_mesh_to_mjcf.py` | MuJoCo scene bake |
| `dimos/simulation/mujoco/collision_spec.py` | static collision policy |
| `dimos/simulation/mujoco/entity_scene.py` | runtime entity composition |
| `dimos/simulation/scenes/catalog.py` | scene name/path resolution |
| `dimos/simulation/engines/mujoco_sim_module.py` | runtime scene + robot composition |
