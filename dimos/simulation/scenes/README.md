# Runtime Scene Packages

Scene packages are cooked assets. Runtime code should treat them as a small
manifest plus a set of simulator/viewer artifacts, not as raw meshes to inspect
or recook.

The basic consumer interface is:

```python
from dimos.simulation.scenes.catalog import resolve_scene_package

package = resolve_scene_package("office")
assert package is not None

print(package.metadata_path)
print(package.visual_path)             # GLB for Rerun/browser viewers
print(package.browser_collision_path)  # GLB for picking/raycast users
print(package.objects_path)            # semantic object table, if present
print(package.mujoco_scene_path)       # scene-only MuJoCo wrapper.xml
print(package.entities)                # optional runtime entities
```

Do not parse `scene.meta.json` directly. Use `resolve_scene_package()` when
accepting the same values as `--scene`, or `load_scene_package()` from
`dimos.simulation.scene_assets.spec` when you already have an exact
`scene.meta.json` path.

## One-Minute Smoke Test

This repository includes a tiny demo module that shows exactly what a consumer
does with a scene package:

```bash
python -m dimos.simulation.scenes.demo_scene_package_consumer office
```

To also compile the office scene with the robot-only G1 MJCF, run:

```bash
python -m dimos.simulation.scenes.demo_scene_package_consumer office --compile-g1-mujoco
```

That command does not start DimOS workers or a viewer. It resolves the package,
loads the MuJoCo scene artifact, attaches the G1 robot MJCF, adds package
entities, and compiles a MuJoCo model. It is intended as a small reference for
new runtime consumers.

## Running G1 GR00T With A Scene

The currently shipped named runtime scenes are:

```text
none    legacy G1 MuJoCo model, no cooked scene package
office  cooked office scene package from data/.lfs/scene_packages.tar.gz
```

Run the office scene with Rerun visualization:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene office \
  --viewer rerun \
  --n-workers 12 \
  run unitree-g1-groot-wbc \
  -o mujocosimmodule.headless=true
```

Run without a scene:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene none \
  --viewer rerun \
  --n-workers 12 \
  run unitree-g1-groot-wbc \
  -o mujocosimmodule.headless=true
```

Prefer `mujocosimmodule.headless=true` for normal testing. With
`headless=false`, MuJoCo opens its native window and the simulation can run much
slower. Use that only when directly debugging MuJoCo contacts or rendering; for
normal operation, look at the robot, scene GLB, lidar point cloud, costmaps, and
planned path in the Rerun native viewer.

The first run may extract these LFS archives:

```text
data/.lfs/scene_packages.tar.gz
data/.lfs/g1_urdf.tar.gz
data/.lfs/groot.tar.gz
```

## What Loading Means

For MuJoCo, a scene package is not a complete robot simulation by itself. The
runtime does this:

```text
--scene office
  -> resolve_scene_package("office")
  -> package.mujoco_scene_path      # scene-only wrapper.xml
  -> robot-only G1 MJCF
  -> package.entities
  -> MujocoSimModule composes and compiles one model
```

Large scenes may also ship a composed `.mjb`. In that case the `.mjb` already
contains the robot, scene, spawn pose, and selected entity set. Passing an `.mjb`
path to `--scene` skips runtime composition:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene /path/to/package/mujoco/composed/unitree-g1-groot-wbc_spawn.mjb \
  --viewer rerun \
  run unitree-g1-groot-wbc
```

A composed `.mjb` is fast to load but not robot-agnostic. Build one per robot
model, spawn/entity configuration, and meaningful scene revision.

## Adding A Runtime Consumer

Use the package artifact that matches your backend:

```text
Rerun/browser visual       package.visual_path
browser picking/raycast    package.browser_collision_path + package.objects_path
MuJoCo runtime compose     package.mujoco_scene_path + package.entities
precomposed MuJoCo         path to package/mujoco/composed/*.mjb
```

Keep runtime consumers independent from the cooking pipeline. Cooking code may
need Blender, CoACD, gltfpack, Open3D, USD tooling, and source-asset heuristics.
Runtime consumers should only need the stable `ScenePackage` contract.
