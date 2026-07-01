# Scene Cooking Architecture

Scene cooking is an offline pipeline. It takes an authored environment and
builds the package artifacts that runtime systems can load directly.

```text
source asset
  -> normalized source
  -> inspected scene prims
  -> sidecar policy
  -> cook plan
  -> backend artifacts
  -> scene.meta.json
  -> runtime consumers
```

## Concepts

```text
source asset   The authored file: .blend, .glb, .usd, .obj, .ply, .stl, ...
sidecar        Scene-specific intent: collision policy, entities, groups.
cook plan      Resolved instructions after applying the sidecar to source prims.
artifact       A backend-specific file generated from the plan.
package        The manifest plus all artifacts for one environment.
runtime        A simulator, viewer, browser app, or future consumer.
```

The source asset is not the runtime contract. The scene package is.

## Stage 1: Normalize Source

Entry points:

```text
source_assets/normalize.py
source_assets/glb.py
```

The cooker first converts source files into a form that the rest of the
pipeline can read consistently. For `.blend` files this means running Blender
headlessly, evaluating Geometry Nodes and instances, and exporting a cached GLB.

This stage should preserve authored geometry. It should not decide physics
policy.

## Stage 2: Inspect Source Prims

Entry points:

```text
source_assets/mesh.py
source_assets/inspect.py
```

The cooker reads source prims in the aligned scene frame. This is the point
where users should inspect names, bounds, triangle counts, and repeated assets
before writing a sidecar.

Inspection answers questions like:

```text
Which prims are floors?
Which prims are walls?
Which objects are repeated products?
Which objects should become runtime entities?
Which meshes are too large or too detailed for direct collision?
```

## Stage 3: Load Sidecar Policy

Entry point:

```text
sidecar.py
```

The sidecar is authored per scene. It maps source prim names or paths to cook
intent:

```text
collision overrides
support-surface rules
objects to remove from static geometry
explicit interactables
repeated entity groups
prototype sharing keys
```

The sidecar is deliberately scene-specific. Heuristics are useful defaults, but
the important decisions about floors, shelves, fixtures, and products should be
visible in authored data.

## Stage 4: Build The Cook Plan

Entry point:

```text
planning.py
```

The cook plan applies the sidecar to the inspected source prims. It decides
which prims remain static, which become package entities, and which repeated
entities can share prototypes.

This stage is the boundary between authored intent and backend output. Backends
should consume the plan instead of rediscovering source-scene meaning.

## Stage 5: Cook Browser Artifacts

Entry points:

```text
browser/visuals.py
browser/collision.py
```

Browser artifacts are split by purpose:

```text
visual GLB      what a viewer renders
collision GLB   simplified geometry for raycasts, picking, and browser queries
objects.json    object metadata keyed by source prim
```

Visual GLBs are target-specific. Rerun, Babylon, and generic web consumers do
not accept the same optimization choices. The cook therefore records named
visual targets instead of assuming one GLB is universal.

## Stage 6: Cook Entity Artifacts

Entry points:

```text
entities/visuals.py
entities/collision.py
```

Entities are objects removed from the static scene and represented separately in
the package manifest. They may be dynamic, kinematic, or static at runtime.

Repeated entities can share prototypes. For example, hundreds of repeated store
products should not require hundreds of independent collision decompositions
when they are instances of the same source asset.

## Stage 7: Cook MuJoCo Artifacts

Entry points:

```text
mujoco/collision_policy.py
mujoco/collision_export.py
```

MuJoCo artifacts are collision-first. The normal output is scene-only:

```text
wrapper.xml + collision meshes
```

Runtime attaches the robot MJCF later. This keeps the package usable with more
than one robot.

Large scenes can also ship composed `.mjb` files:

```text
wrapper.xml + robot MJCF + spawn/entity selection -> composed .mjb
```

Composed `.mjb` files load quickly, but they are specific to one robot, spawn
pose, entity selection, and scene revision. Rebuild them when any of those
inputs change.

## Stage 8: Write The Manifest

Runtime contract:

```text
dimos/simulation/scene_assets/spec.py
```

The manifest is `scene.meta.json`. It stores package-relative artifact paths,
frame metadata, entities, browser visual targets, MuJoCo paths, and cook stats.

Runtime code should use:

```python
from dimos.simulation.scene_assets.spec import load_scene_package
from dimos.simulation.scenes.catalog import resolve_scene_package
```

Consumers then ask for the representation they support:

```python
package.browser_visual_path("rerun")
package.browser_visual_path("babylon")
package.mujoco_scene_path
package.entities
```

## Runtime Boundary

Cooking code lives under:

```text
dimos/experimental/scene_cooking/
```

Runtime package loading lives under:

```text
dimos/simulation/scene_assets/
dimos/simulation/scenes/
dimos/visualization/rerun/
dimos/simulation/engines/
```

The cooker may change as new asset types are added. Runtime consumers should
depend on the scene package manifest, not on cook internals.

## File Map

```text
cook.py                         CLI and top-level orchestration
command.py                      command helpers
package_config.py               cook-time artifact options
sidecar.py                      authored sidecar schema
planning.py                     source prims + sidecar -> cook plan
source_assets/normalize.py      source normalization and .blend export
source_assets/mesh.py           source prim loading and alignment
source_assets/inspect.py        source budget inspection
source_assets/glb.py            GLB rewrite helpers
browser/visuals.py              target-specific browser visual cooking
browser/collision.py            browser collision and objects.json
entities/visuals.py             entity visual extraction
entities/collision.py           entity collision hulls
mujoco/collision_policy.py      static collision policy
mujoco/collision_export.py      MuJoCo XML and mesh export
```
