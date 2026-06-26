# Scene Packages At Runtime

A scene package is a cooked environment. Runtime code should not inspect raw
GLBs, run Blender, or apply cooking heuristics. It should load
`scene.meta.json`, choose the artifact for its backend, and pass that artifact
to the simulator or viewer.

## Load A Package

Use `resolve_scene_package()` for the same values accepted by `--scene`:

```python
from dimos.simulation.scenes.catalog import resolve_scene_package

package = resolve_scene_package("office")
assert package is not None
```

Use `load_scene_package()` when you already have an exact metadata path:

```python
from dimos.simulation.scene_assets.spec import load_scene_package

package = load_scene_package("data/scene_packages/dimos_office/scene.meta.json")
```

The currently shipped named scenes are:

```text
none    no cooked scene package
office  cooked DimOS office package
```

## Pick The Backend Artifact

Scene packages can contain several artifacts for different consumers:

```python
rerun_glb = package.browser_visual_path("rerun")
babylon_glb = package.browser_visual_path("babylon")
browser_collision = package.browser_collision_path
objects_json = package.objects_path
mujoco_xml = package.mujoco_scene_path
entities = package.entities
```

Browser visuals are backend-specific because GLB support differs by viewer.
Rerun should request `browser_visual_path("rerun")`; Babylon/PimSim should
request `browser_visual_path("babylon")`. If a target returns `None`, the
package was not cooked for that backend.

MuJoCo uses the scene-only XML plus runtime entities. The robot is attached by
`MujocoSimModule`; it is not part of `scene.meta.json`.

## Run G1 With A Scene

Run the cooked office scene:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene office \
  --viewer rerun \
  --n-workers 12 \
  run unitree-g1-groot-wbc \
  -o mujocosimmodule.headless=true
```

Run the same blueprint without a cooked scene:

```bash
python -m dimos.robot.cli.dimos \
  --simulation mujoco \
  --scene none \
  --viewer rerun \
  --n-workers 12 \
  run unitree-g1-groot-wbc \
  -o mujocosimmodule.headless=true
```

Use `mujocosimmodule.headless=true` for normal testing and inspect the scene,
robot, lidar, map, and path in Rerun. `headless=false` opens the MuJoCo native
window and is useful for contact debugging, but it can run much slower.

## MuJoCo Loading Modes

Normal packages are robot-agnostic:

```text
scene.meta.json -> wrapper.xml + entities -> attach robot MJCF -> compile MjModel
```

Large scenes may also ship composed `.mjb` files:

```text
wrapper.xml + robot MJCF + spawn/entity selection -> composed/<name>.mjb
```

Passing a composed `.mjb` path to `--scene` skips runtime composition and loads
the binary model directly. This is faster, but the file is specific to one
robot, spawn pose, entity set, and scene revision.

## Package Contract

Runtime consumers should depend on:

```text
ScenePackage.package_dir
ScenePackage.alignment
ScenePackage.browser_visual_path(target)
ScenePackage.browser_collision_path
ScenePackage.objects_path
ScenePackage.mujoco_scene_path
ScenePackage.mujoco_binary_path
ScenePackage.entities
```

Cooking details belong in `dimos.experimental.pimsim.scene`; runtime modules
should stay on this package contract.
