# Scene Cooking Architecture

This is a concrete map of the current implementation. The main entry point is:

```python
dimos.experimental.scene_cooking.cook.cook_scene_package()
```

It turns these inputs:

```text
source_path
SceneMeshAlignment
SceneCookSidecar
BrowserVisualSpec
BrowserCollisionSpec
MujocoSceneSpec
```

into a runtime `ScenePackage` and writes `scene.meta.json`.

## Core Data Flow

```text
source asset
  -> PreparedSceneSource
  -> SceneCookSidecar
  -> SceneCookPlan
  -> artifact writers
  -> ScenePackage
  -> scene.meta.json
```

The important boundary is `SceneCookPlan`.

The sidecar is authored intent. The plan is resolved scene membership: exactly
which source prims become runtime entities, which prims should be skipped from
static collision, which repeated entities share a prototype, and which collision
policy every backend writer should use.

## Main Types

```text
SceneMeshAlignment
  dimos/simulation/scene_assets/spec.py
  Scale, rotation, translation, and y-up conversion from source frame to
  DimOS world frame.

SceneCookSpec
  package_config.py
  Complete cook input: source path, alignment, browser visual policy,
  browser collision policy, and MuJoCo policy.

SceneCookSidecar
  sidecar.py
  Authored per-scene policy loaded from <scene>.cook.json, <scene>.scene.json,
  or legacy <scene>.collision.json.

CollisionSpec
  mujoco/collision_policy.py
  Shared static collision policy. It currently lives under mujoco because
  that code existed first, but browser collision consumes it too.

SceneCookPlan
  planning.py
  Resolved cook instructions shared by artifact writers.

ScenePackage
  dimos/simulation/scene_assets/spec.py
  Runtime metadata object. This is what simulators and viewers consume.
```

## `cook_scene_package()` Call Order

The top-level cook is intentionally boring. It normalizes the source, resolves a
plan, runs artifact writers, then serializes a runtime manifest.

```text
1. source = Path(source_path).resolve()

2. align = alignment or SceneMeshAlignment()
   visual = visual_spec or BrowserVisualSpec()
   browser_collision = browser_collision_spec or BrowserCollisionSpec()
   mujoco = mujoco_spec or MujocoSceneSpec()

3. cook_spec = SceneCookSpec(...)
   sidecar = cook_sidecar or SceneCookSidecar.auto_discover(source)

4. prepared_source = prepare_scene_source(source, rebake=rebake)
   cook_source = prepared_source.cook_path

5. stats["source"] = inspect_scene_asset(cook_source)

6. plan = build_scene_cook_plan(
       cook_source,
       sidecar=sidecar,
       alignment=align,
       output_dir=package_dir,
       collision_spec=collision_spec,
   )

7. entities = plan.entities_metadata()

8. If source-backed entity visuals are needed:
       visual_source = cook_plan_visual_assets(cook_source, package_dir, plan=plan)
   else:
       visual_source = cook_source

9. If MuJoCo is enabled:
       _cook_entity_prototype_collision(plan.prototypes, entities)
       _cook_entity_collision(entities)

10. visual_result = cook_browser_visual(visual_source, browser_dir, spec=visual)

11. browser_collision_result = cook_browser_collision(
        cook_source,
        browser_dir,
        spec=browser_collision,
        collision_spec=plan.collision_spec,
    )

12. If MuJoCo is enabled:
        mujoco_scene_path = load_or_bake(
            scene_mesh_path=cook_source,
            alignment=align,
            cache_root=mujoco_dir,
            collision_spec=plan.collision_spec,
        )
        optionally compile wrapper.mjb

13. package = ScenePackage(...)
    package.write_metadata()
```

## Source Normalization

Code:

```text
source_assets/normalize.py
source_assets/glb.py
```

`prepare_scene_source()` returns the file that the rest of the cook should read.
For `.blend`, it runs Blender headlessly and exports the evaluated dependency
graph to GLB so Geometry Nodes and collection instances are realized. For
direct mesh formats, it returns the original file or a cached normalized file.

This stage should not make semantic decisions. It exists to make source loading
predictable.

## Source Inspection

Code:

```text
source_assets/mesh.py
source_assets/inspect.py
```

`load_scene_prims()` reads source prims and applies `SceneMeshAlignment` when
requested. The returned `ScenePrimMesh` objects carry names, prim paths, visual
node names, vertices, and triangles.

`inspect_scene_asset()` records counts and size budgets for cooked stats and
for user inspection.

## Sidecar Schema

Code:

```text
sidecar.py
```

`SceneCookSidecar.auto_discover(source)` checks, in order:

```text
<scene>.cook.json
<scene>.scene.json
<scene>.collision.json
```

The sidecar can contain:

```text
collision
  CollisionSpec-compatible static collision policy.

interactables
  Explicit runtime entities. Each item either matches source prims through
  source_prim_paths or defines a synthetic entity with pose + physics.extents.

entity_groups
  Pattern-expanded runtime entities. Current mode is per_prim. Used for many
  repeated objects, with prototype_key controlling collision reuse.
```

## Plan Resolution

Code:

```text
planning.py
```

`build_scene_cook_plan()` does the expensive semantic matching once.

It produces:

```text
SceneCookPlan.collision_spec
  The effective static collision policy after entity removals are inserted as
  skip overrides.

SceneCookPlan.entities
  Tuple[EntityCookPlan]. Each item has id, matched source prim paths,
  visual-node deletion patterns, AABB, initial pose, physics descriptor,
  visual output path, and optional prototype id.

SceneCookPlan.prototypes
  Tuple[EntityPrototypePlan]. Shared source meshes cooked once and referenced
  by repeated entity instances.

SceneCookPlan.stats
  Counts for source prims, entities, and prototypes.
```

Important behavior:

```text
interactable with source_prim_paths
  -> matched against ScenePrimMesh candidates
  -> AABB and initial pose come from the matched geometry
  -> visual.glb path is assigned under entities/<id>/

synthetic interactable
  -> no source prim lookup
  -> pose comes from sidecar
  -> extents come from physics.extents

entity group
  -> one EntityCookPlan per matched prim
  -> one EntityPrototypePlan per prototype key
  -> prototype collision hulls are reused by instances

remove_from_static
  -> inserts type=skip overrides into the effective CollisionSpec
  -> inserted before broad user collision overrides so extracted entities are
     not duplicated in static collision
```

## Browser Visual Writer

Code:

```text
browser/visuals.py
package_config.py
```

`cook_browser_visual()` writes:

```text
browser/visual.<target>.glb
```

The target comes from `BrowserVisualSpec.target`. Built-in profiles are:

```text
rerun
babylon
generic
```

The visual writer optimizes or converts the source using `gltfpack`, Blender,
or copy mode, then validates budget limits such as mesh count, material count,
texture count, and vertex count.

When entity visuals need to be removed from the static visual, `cook.py` first
calls `cook_plan_visual_assets()`. That function uses the plan's visual-node
patterns to extract per-entity GLBs and produce the filtered source passed to
`cook_browser_visual()`.

## Browser Collision Writer

Code:

```text
browser/collision.py
```

`cook_browser_collision()` writes:

```text
browser/collision.glb
browser/objects.json
```

It loads source prims, applies the effective `CollisionSpec`, optionally splits
disconnected components, simplifies to `BrowserCollisionSpec.target_faces`, and
exports one fused GLB for browser-side raycasts and picking.

`objects.json` is separate from the fused mesh. It stores per-prim ids, prim
paths, and AABBs so browser consumers can answer object queries without
building one physics object per source prim.

## Entity Collision Writer

Code:

```text
entities/collision.py
cook.py
```

Mesh entities need collision hulls before runtime. Runtime does not run CoACD.

Two paths exist:

```text
entity prototype collision
  _cook_entity_prototype_collision()
  Writes one hull set for each EntityPrototypePlan and assigns those hull paths
  to every entity instance using that prototype.

individual entity collision
  _cook_entity_collision()
  Decomposes an entity visual.glb when the entity has no shared prototype.
```

The resulting paths are written into each entity metadata dict as
`collision_paths`.

## MuJoCo Static Scene Writer

Code:

```text
mujoco/collision_policy.py
mujoco/collision_export.py
```

`load_or_bake()` / `bake_scene_mjcf()` writes:

```text
mujoco/<cache-key>/wrapper.xml
mujoco/<cache-key>/*.obj
```

The wrapper is scene-only. It contains a static `dimos_scene` body and no robot.
The robot is attached later by runtime code.

For every source prim, `collision_policy.decide_for_prim()` chooses one of:

```text
primitive  box, sphere, cylinder, capsule, or plane geom
hulls      convex hull or CoACD decomposition written as mesh geoms
skip       no static collision geom
```

`include_visual_mesh=True` additionally writes non-colliding visual geoms for
MuJoCo debugging. Normal packages should not rely on MuJoCo visuals for the
browser/Rerun view; those use browser visual artifacts.

`--compile-mujoco-binary` also writes `wrapper.mjb`. That is a scene-only binary
cache. It is fast to load, but by itself it is not where robot attachment
happens.

## Runtime Manifest

Code:

```text
dimos/simulation/scene_assets/spec.py
```

`ScenePackage.write_metadata()` writes `scene.meta.json`.

The manifest stores:

```text
source_path
package_dir
alignment
artifact_frames
artifacts.browser_visual
artifacts.browser_visuals
artifacts.browser_collision
artifacts.objects
artifacts.mujoco_scene
artifacts.mujoco_binary
artifacts.mujoco_composed_binaries
entities
stats
```

All artifact paths are serialized package-relative when possible. At runtime,
`load_scene_package()` resolves them back to absolute paths and validates
`artifact_frames`.

Current frame contract:

```text
browser_visual     source
browser_collision  source
mujoco             dimos_world
mujoco_binary      dimos_world
```

## Runtime Consumers

Runtime code should not import cook internals.

Use:

```python
from dimos.simulation.scene_assets.spec import load_scene_package
from dimos.simulation.scenes.catalog import resolve_scene_package
```

### Rerun

Code:

```text
dimos/visualization/rerun/scene_package.py
```

`scene_package_static_entities()` resolves the package, picks
`package.browser_visual_path("rerun")`, and returns a static Rerun factory. The
factory logs:

```text
rr.Transform3D(alignment)
rr.Asset3D(GLB bytes)
```

The GLB is sent as bytes so native and browser Rerun viewers do not need direct
filesystem access.

### MuJoCo

Code:

```text
dimos/simulation/engines/mujoco_sim_module.py
dimos/simulation/mujoco/scene_package_entity_composer.py
```

The normal MuJoCo path uses `package.mujoco_scene_path` (`wrapper.xml`), attaches
the robot MJCF, then appends initial scene-package entities.

`add_scene_package_entities_to_spec()` turns each `spawn=="initial"` entity into
a MuJoCo body:

```text
kind == dynamic and mass > 0
  -> body with freejoint

otherwise
  -> welded/static body

shape_hint == mesh
  -> uses pre-cooked collision_paths
  -> falls back to AABB box with a warning if hulls are missing

primitive shape hints
  -> box/sphere/cylinder from descriptor extents
```

Composed robot+scene `.mjb` artifacts can be declared in the manifest for large
scenes, but they are specific to robot, spawn pose, entity policy, scene
revision, and MuJoCo version.

## Adding A Backend Artifact

A new backend should follow this shape:

```text
1. Add a cook-time spec dataclass to package_config.py if it needs options.
2. Add an artifact writer that consumes cook_source and/or SceneCookPlan.
3. Add package metadata fields only if runtime needs to discover the artifact.
4. Serialize paths package-relative in ScenePackage.to_json_dict().
5. Resolve paths in load_scene_package().
6. Keep runtime consumers dependent on ScenePackage, not cook internals.
```

Use `SceneCookPlan` for any backend that needs entity removals, collision
policy, prototype sharing, or source-prim membership.

## File Map

```text
cook.py                         CLI and top-level orchestration
command.py                      subprocess helpers with filtered logs
package_config.py               cook-time artifact options
sidecar.py                      authored sidecar schema
planning.py                     sidecar + source prims -> SceneCookPlan
source_assets/normalize.py      source normalization and .blend export
source_assets/mesh.py           source prim loading and alignment
source_assets/inspect.py        asset budget inspection
source_assets/glb.py            GLB JSON/BIN rewrite helpers
browser/visuals.py              target-specific browser visual cooking
browser/collision.py            browser collision GLB and objects.json
entities/visuals.py             entity visual extraction/filtering
entities/collision.py           entity CoACD hulls
mujoco/collision_policy.py      shared static collision policy
mujoco/collision_export.py      MuJoCo XML and mesh export
```
