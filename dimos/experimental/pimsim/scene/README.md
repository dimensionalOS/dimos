# Scene packages

A **scene package** is a robot-agnostic, self-contained directory holding
everything any DimOS simulator needs to load a 3D scene: visual mesh,
collision artifacts, per-object semantic table, a scene-only MuJoCo wrapper,
and a list of dynamic entities. One source asset → one package → consumed by
**pimsim** (browser Havok) and **MuJoCo** with the same metadata. The robot is
never part of the package; the runtime attaches it via `MjSpec.attach()`
inside `MujocoSimModule.start`.

The lifecycle is three steps:

1. **Author** — drop a `<scene>.cook.json` sidecar next to the source mesh.
2. **Cook** — `python -m dimos.experimental.pimsim.scene.cook` bakes the package.
3. **Compose** — at runtime the sim module loads the wrapper, attaches the
   robot, and adds entities as first-class MuJoCo bodies.

## Package layout

```text
data/scene_packages/<name>/
├── scene.meta.json              manifest — all paths package-relative
├── browser/
│   ├── visual.glb               gltfpack-optimised, static parts only
│   ├── collision.glb            decimated trimesh — browser/lidar raycasts
│   └── objects.json             per-prim semantic table (id, path, AABB)
├── entities/
│   └── <id>/
│       ├── visual.glb           per-entity GLB, entity-local frame
│       └── mujoco_collision/    CoACD convex hulls (hull_000.obj, …)
└── mujoco/
    └── <key>/                   content-hash key
        ├── wrapper.xml          scene-only MJCF (no robot include)
        └── *.obj                static-scene collision (per-prim hulls;
                                 multi-hull _hNNN for "decompose" prims)
```

`scene.meta.json` top-level keys: `alignment`, `artifact_frames`,
`artifacts` (paths to the items above), `entities`, `package_dir`,
`source_path`, `stats`. Packages are content-hash keyed on source mesh +
alignment + sidecar + schema version — change any of those and the cooker
writes a fresh package.

## Authoring

Two files next to each other:

```text
my_scene.glb         # source asset (USD/GLB/OBJ/PLY)
my_scene.cook.json   # cook sidecar — optional but recommended
```

Without a sidecar you still get a valid package: auto-fit static collision,
no entities. A minimal sidecar:

```json
{
  "$schema": "https://dimensional.dev/schemas/dimos/scene-cook-sidecar.v1.json",
  "collision": {
    "prim_overrides": {
      "Floor": {"type": "plane"}
    }
  },
  "interactables": []
}
```

### Static collision overrides

The `collision` block controls how static-scene prims are cooked (same
schema as the legacy `<scene>.collision.json`, which is still auto-discovered
for old scenes):

```json
{
  "collision": {
    "default": "auto",
    "prim_overrides": {
      "Floor":       {"type": "plane"},
      "Wall_*":      {"type": "box"},
      "Curtain_*":   {"type": "skip"},
      "Stairs_*":    {"type": "decompose", "max_hulls": 16},
      "FlatPanel_*": {"type": "hull"}
    }
  }
}
```

Per-pattern `type`: `"auto"` | `"box"` | `"sphere"` | `"cylinder"` |
`"capsule"` | `"plane"` | `"hull"` | `"mesh"` | `"decompose"` | `"skip"`.
Full reference: `dimos/simulation/mujoco/collision_spec.py`.

### Interactables (dynamic entities)

Entries in `interactables[]` become entities. Two sidecar flavours:

**Extracted** — already modelled in the source asset (chairs, printers).
The cooker matches `source_prim_paths` with `fnmatch` against USD prim paths
/ GLB node names, splits the prims out of the static bake, and emits a
per-entity GLB plus CoACD collision hulls:

```json
{
  "id": "office_chair_000",
  "source_prim_paths": ["Chair_000_*"],
  "kind": "dynamic",
  "mass": 8.0,
  "physics": {"shape": "mesh"},
  "tags": ["chair", "office", "movable"]
}
```

**Synthetic** — primitive props not in the source mesh (tables, test
markers). No prim matching; geometry comes from `physics.shape` +
`physics.extents`, pose is explicit:

```json
{
  "id": "manip_table",
  "pose": {"x": 0.0, "y": 1.0, "z": 0.63},
  "kind": "static",
  "mass": 0.0,
  "physics": {"shape": "box", "extents": [0.6, 0.6, 0.04],
              "friction": [1.0, 0.05, 0.001]},
  "visual": {"rgba": [0.55, 0.42, 0.3, 1.0]}
}
```

There is a third source of entities the sidecar does **not** cover:
**scanned props** (the office `manip_cup` / `manip_bottle` / `manip_can` /
`manip_box` / `manip_marker` / `manip_tape`). These are generated from
photos by the SAM3D image→mesh pipeline and injected into the cooked
package directly — per-entity `visual.glb`, `mujoco_collision/` hulls, and
a `scene.meta.json` entry with `artifacts` provenance pointers
(`sam3_source_image` etc.). They follow the same entity shape as cooked
entries but are invisible to the cook sidecar. **Caveat:** because of
this, `--rebake` of a package regenerates only sidecar-declared entities —
re-run the injection step (or port the props into the sidecar) after a
rebake, or the scanned props vanish.

### Sidecar field reference

| Field | Required | Notes |
|---|---|---|
| `id` | yes | Stable; becomes the `entity:<id>` MuJoCo body name |
| `source_prim_paths` | one of | `fnmatch` globs — extracted entity |
| `pose` | one of | `{x, y, z, qw, qx, qy, qz}` — synthetic entity |
| `kind` | – | `"dynamic"` (freejoint+mass) \| `"kinematic"` (RPC-driven) \| `"static"` (welded). Default `"dynamic"` |
| `mass` | – | kg; 0 forces kinematic. Default 1.0 |
| `physics.shape` | – | `"mesh"` \| `"box"` \| `"sphere"` \| `"cylinder"`. Default `"box"` |
| `physics.extents` | synthetic | Half-extents triple for box, `[radius]` sphere, `[radius, half_height]` cylinder |
| `physics.friction` | – | Scalar sliding or `[slide, torsional, rolling]`. Default `[0.3, 0.05, 0.001]` |
| `visual.rgba` | – | `[r, g, b, a]` 0–1, both simulators |
| `remove_from_static` | – | Strip matched prims from the static bake. Default true |
| `spawn` | – | `"initial"` (in world at boot) \| `"manual"` (RPC only). Default `"initial"` |
| `tags` | – | Free-form labels for semantic queries |

## Cooking

```bash
python -m dimos.experimental.pimsim.scene.cook \
    path/to/my_scene.glb \
    --output-dir=data/scene_packages/my_scene
```

The cooker auto-discovers `<scene>.cook.json` next to the source
(`--cook-spec` overrides). Useful flags: `--rebake` (ignore cooked files,
rebuild), `--scale` / `--translation` / `--rotation-zyx-deg` / `--no-y-up`
(source alignment), `--visual-optimizer {gltfpack,blender,copy}` and the
`--visual-simplify-*` family, `--no-browser-collision`, `--no-mujoco`. The
cooker is strictly robot-agnostic — there is no robot flag and never a robot
in the package.

## Entity collision: how a mesh entity actually collides

For `physics.shape: "mesh"` entities, collision geometry is **cooked into
the package** — there is no runtime decomposition and no per-machine
cache:

1. **Cook** — `dimos/simulation/scene_assets/entity_collision.py` runs
   CoACD on each entity's `visual.glb` and writes the convex hulls to
   `entities/<id>/mujoco_collision/hull_*.obj`, recorded per entity as
   `collision_paths` in `scene.meta.json`. Chair legs / seat / back each
   get their own hull, so contacts are chair-shaped, and MuJoCo's
   convex-only narrowphase is exact on them. If CoACD fails on a mesh,
   the cooker falls back to a single convex hull.
2. **Runtime** — `add_entities_to_spec` (see
   `dimos/simulation/mujoco/entity_scene.py`) loads exactly those files
   (all-or-nothing per entity: a partially missing hull set is rejected
   rather than colliding with half a chair).
3. **Fallback** — a mesh entity with no cooked hulls collides as its AABB
   box, with a warning telling you to re-cook the package.

Hand-tuning is supported: edit the `hull_*.obj` files in the package and
they are authoritative. Primitive entities (`box`/`sphere`/`cylinder`)
take their geom directly from the descriptor extents — no meshes
involved.

## Runtime composition

`MujocoSimModule._compose_model` does, in order:

1. Load the cooked `wrapper.xml` into an `MjSpec`.
2. Load the robot MJCF into its own `MjSpec` (with `meshdir` set so
   menagerie robots find their STLs) and
   `spec_scene.attach(spec_robot, prefix=robot_id, frame=spawn_frame)`.
   The robot is attached **first** so it keeps the leading freejoint/qpos
   block.
3. `add_entities_to_spec(spec, pkg.entities)` — each `spawn=="initial"`
   entity becomes a body named `entity:<id>`. `kind=="dynamic"` with
   positive mass gets a freejoint `entity:<id>:free`; everything else is
   welded. Entity geoms carry `priority=1` so their friction wins the
   contact pair (MuJoCo's default element-wise-max rule would let the
   μ=1.0 floor override every entity), and sit in geom group 3 so
   depth-based lidar renders see them.
4. `spec.compile()` → one `MjModel` with scene + robot + entities.
5. **Spawn audit** — `spawn_penetrators(model)` finds entities that boot
   >2 cm inside the static scene and recomposes with them welded static
   instead of letting MuJoCo eject them.

Multi-robot scenes: call `attach()` once per robot with distinct `prefix`
values. `compose_entity_model(pkg)` is the legacy pre-compiled-`.mjb` path;
new code should not use it.

Blueprint-side wiring:

```python
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.scenes.catalog import resolve_scene_package

pkg = resolve_scene_package("dimos-office")

MujocoSimModule.blueprint(
    scene_xml=str(pkg.mujoco_scene_path),  # cooked scene-only wrapper
    robot_mjcf="path/to/g1.xml",           # any robot MJCF
    robot_meshdir="path/to/g1/assets",
    robot_id="",                           # body-name prefix ("" if single robot)
    scene_entities=pkg.entities,
    spawn_xy=(0.0, 0.0),
    spawn_z=0.793,
)
```

### What the robot MJCF must not have

- **No scene geometry** — floors, walls, furniture belong to the package.
- **No manipulation rigs** — author props as interactables; they then spawn
  into pimsim and MuJoCo from one source. (The old hardcoded `manip_*` rig
  in `g1_gear_wbc.xml` was removed for exactly this reason.)
- **No external `<include>`s** outside its own directory tree — `MjSpec`
  resolves includes relative to the robot file.

## Topics published by either simulator

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/odom` | `PoseStamped` | sim | robot base pose |
| `/cmd_vel` | `Twist` | controller | drive command (sim subscribes) |
| `/coordinator/joint_state` | `JointState` | coordinator | sim writes via SHM, coordinator publishes |
| `/entity_state_batch` | `EntityStateBatch` | physics authority | same wire format from MuJoCo and pimsim — consumers don't care |
| `/lidar` | `PointCloud2` | rust `scene_lidar` | raycasts `browser/collision.glb` + dynamic entities |

The physics authority is whichever sim the blueprint wires in — MuJoCo for
whole-body flows, pimsim Havok for navigation-only. The other one mirrors
published states as kinematic bodies.

## Bundled scenes

| Scene name | Package dir | Contents |
|---|---|---|
| `dimos-office` (default) | `dimos_office` | Office: 17 chairs, printer, manip table, 6 SAM3D-scanned props |
| `dimos-office-splat` | `dimos_office_splat` | Office + Gaussian splat for `SplatCameraModule` |
| `street-lite` | `street_lite` | Outdoor nav |
| `lowpoly-tdm` | `lowpoly_tdm` | Lightweight nav stress test |
| `mall-babylon-nolights` | `mall_babylon_nolights` | Large indoor nav |

Aliases live in `dimos/simulation/scenes/catalog.py` (`office`, `street`,
`mall`, `tdm`, …). Cooked a new scene? Add it to `_PACKAGE_DIRS` and
`_ALIASES` so the rest of the stack finds it by name.

## Open

- Scanned (SAM3D) entities live only in the cooked package; the cook
  sidecar doesn't know about them, so `--rebake` drops them (see caveat
  above). The office sidecar also still lists a `manip_cube` the package
  no longer has.
- Scene-package articulation (doors, drawers) has no schema yet —
  `joints[]` on `InteractableSpec` is the next extension.

## Reference

- `dimos/simulation/scene_assets/sidecar.py` — `<scene>.cook.json` schema
- `dimos/simulation/scene_assets/plan.py` — sidecar → resolved cook plan
- `dimos/simulation/scene_assets/cook.py` — cook entry point + CLI
- `dimos/simulation/scene_assets/spec.py` — `ScenePackage` dataclass + on-disk JSON shape
- `dimos/simulation/scene_assets/browser_collision.py` — fused collision GLB + `objects.json`
- `dimos/simulation/scene_assets/entity_collision.py` — cook-time CoACD hulls per entity
- `dimos/simulation/scene_assets/visual_blender.py` — Blender pass for per-entity GLBs
- `dimos/simulation/mujoco/collision_spec.py` — static-collision policy types
- `dimos/simulation/mujoco/entity_scene.py` — runtime entity composition + spawn audit
- `dimos/simulation/scenes/catalog.py` — `resolve_scene_package(name | path)`
