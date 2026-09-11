# Scene Cooking Architecture

Scene cooking converts authored environment assets into scene packages that
runtime systems can load without understanding the original asset layout.

```text
source asset + sidecar + alignment
  -> SceneCookPlan
  -> backend artifacts
  -> scene.meta.json
  -> runtime consumers
```

The cook is experimental because asset policy will evolve. The runtime boundary
is the stable idea: consumers load a `ScenePackage`, not source assets or cook
internals.

## Contracts

```text
source asset
  Raw authored file: .blend, .glb, .usd, .obj, .ply, .stl, ...
  This is input data, not the runtime contract.

sidecar
  Authored scene intent in <scene>.cook.json. It says which prims are floors,
  walls, fixtures, interactables, repeated entity groups, or skipped geometry.

alignment
  Scale, rotation, translation, and up-axis conversion from the source asset
  frame into DimOS world frame.

SceneCookPlan
  Internal resolved recipe. It maps the sidecar onto actual source prims and
  decides what each artifact writer should see.

backend artifacts
  Viewer- or simulator-specific files: Rerun GLB, browser collision GLB,
  MuJoCo wrapper XML, entity assets, optional MuJoCo binaries.

scene.meta.json
  Runtime manifest. It records package-relative artifact paths, frames,
  entities, and cook stats.

ScenePackage
  Python runtime object loaded from scene.meta.json.
```

## The Central Boundary

`SceneCookPlan` is the boundary between authored intent and artifact writers.

The sidecar is written in terms of scene names and patterns. The plan resolves
that into concrete facts:

```text
which source prims become runtime entities
which entity prims are removed from static collision
which repeated entities share a collision prototype
which visual nodes Blender should extract/delete
which effective CollisionSpec all collision writers should use
```

After the plan exists, browser, entity, and MuJoCo writers should consume that
resolved plan instead of repeating source-scene matching themselves.

## Artifact Matrix

| Consumer | Artifact | Frame | Producer | Runtime Access |
| --- | --- | --- | --- | --- |
| Rerun visual | `browser/visual.rerun.glb` | source | `browser/visuals.py` | `package.browser_visual_path("rerun")` |
| Babylon/PimSim visual | `browser/visual.babylon.glb` | source | `browser/visuals.py` | `package.browser_visual_path("babylon")` |
| Browser picking/raycast | `browser/collision.glb` | source | `browser/collision.py` | `package.browser_collision_path` |
| Browser object lookup | `browser/objects.json` | source | `browser/collision.py` | `package.objects_path` |
| MuJoCo static scene | `mujoco/<hash>/wrapper.xml` | dimos_world | `mujoco/collision_export.py` | `package.mujoco_scene_path` |
| MuJoCo scene binary | `mujoco/<hash>/wrapper.mjb` | dimos_world | `cook.py` | `package.mujoco_binary_path` |
| MuJoCo composed binary | `mujoco/composed/*.mjb` | dimos_world | external/precompute flow | `package.mujoco_composed_binary_path(...)` |
| Runtime entities | `entities/<id>/...` | dimos_world / entity-local | `entities/*.py` | `package.entities` |

The frame column matters. Browser visual and browser collision assets stay in
source coordinates and receive the package alignment at runtime. MuJoCo assets
are baked into DimOS world coordinates.

## Runtime Boundary

Runtime code should depend on:

```python
from dimos.simulation.scene_assets.spec import load_scene_package
from dimos.simulation.scenes.catalog import resolve_scene_package
```

Runtime code should not import:

```text
dimos.experimental.scene_cooking.*
```

That keeps simulators and viewers independent from the offline cook process.
They only need `ScenePackage` fields and methods:

```python
package.browser_visual_path("rerun")
package.browser_visual_path("babylon")
package.browser_collision_path
package.objects_path
package.mujoco_scene_path
package.mujoco_binary_path
package.mujoco_composed_binary_path(robot="unitree-g1-groot-wbc")
package.entities
```

## Runtime Loading

### Rerun

`dimos/visualization/rerun/scene_package.py` resolves the package, selects the
Rerun visual with `browser_visual_path("rerun")`, and logs:

```text
rr.Transform3D(package.alignment)
rr.Asset3D(GLB bytes)
```

The GLB is sent as bytes, so a Rerun viewer does not need direct filesystem
access to the cooked package.

### MuJoCo

The normal MuJoCo path loads `package.mujoco_scene_path`, attaches the robot
MJCF at runtime, then appends `spawn=="initial"` entities from
`package.entities`.

The scene-only `wrapper.xml` keeps the package robot-agnostic. A composed `.mjb`
can be used for large scenes when startup time matters, but it is specific to:

```text
robot
spawn pose
entity policy
scene revision
MuJoCo version
```

## Extension Points

New representations should fit into the same pattern:

```text
1. Add cook-time options if the backend needs them.
2. Add an artifact writer that consumes the source and/or SceneCookPlan.
3. Add manifest fields only when runtime needs to discover the artifact.
4. Serialize artifact paths package-relative.
5. Resolve them through load_scene_package().
6. Keep runtime consumers dependent on ScenePackage, not cook internals.
```

Examples:

```text
Gaussian splats
  Cook a splat artifact, add a manifest path, and let a viewer consume it from
  ScenePackage.

Articulated assets
  Extend sidecar/entity metadata with articulation description, cook per-asset
  visual/collision/runtime files, and expose them through package.entities.

New simulator backend
  Add a backend artifact writer and a manifest path. The simulator loads the
  package artifact without knowing source prim matching or sidecar rules.
```

## Code Map

```text
cook.py                         top-level cook CLI and orchestration
package_config.py               cook-time options and visual target profiles
sidecar.py                      authored sidecar schema
planning.py                     sidecar + source prims -> SceneCookPlan

source_assets/normalize.py      source normalization and .blend export
source_assets/mesh.py           source prim loading and alignment
source_assets/inspect.py        asset budget inspection
source_assets/glb.py            GLB JSON/BIN rewrite helpers

browser/visuals.py              target-specific browser visual cooking
browser/collision.py            browser collision GLB and objects.json
entities/visuals.py             entity visual extraction/filtering
entities/collision.py           entity collision hulls
mujoco/collision_policy.py      shared static collision policy
mujoco/collision_export.py      MuJoCo XML and mesh export

dimos/simulation/scene_assets/spec.py
  runtime ScenePackage metadata contract

dimos/simulation/scenes/catalog.py
  runtime scene name/path resolution
```
