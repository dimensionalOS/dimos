# Scene Packages

A scene package is a cooked environment that runtime code can load without
knowing how the environment was produced.

Before scene packages, a scene was usually a loose set of files: MuJoCo XML,
mesh files, browser assets, object metadata, and local path assumptions. A
consumer had to know the folder layout for that one scene. Scene packages make
that explicit: load `scene.meta.json` once, then ask the package for the
artifact needed by the simulator, viewer, or planner.

Scene packages are runtime artifacts. Cooking tools may use Blender, GLB
optimizers, collision decomposition, or manual sidecars, but runtime modules
should not depend on those details.

## What A Package Can Contain

A package can carry several assets for different consumers:

- MuJoCo XML for physics simulation
- optional MuJoCo `.mjb` binaries for faster loading of large scenes
- Rerun or browser visual assets
- future visual assets such as Gaussian splats
- static environment geometry
- dynamic or separately spawned entities
- object prototypes reused by many instances
- coordinate-frame and scale alignment metadata

Not every package needs every artifact. A viewer can load the visual asset, a
simulator can load the physics asset, and another renderer can ask for its own
target-specific output later.

## Runtime API

Use `resolve_scene_package()` for the same values accepted by
`--scene-package`:

```python
from dimos.simulation.scenes.catalog import resolve_scene_package

package = resolve_scene_package(global_config.scene_package)
```

Supported inputs:

- `None` or `--scene-package none`
- named packages such as `office` or `supermarket`
- path to `scene.meta.json`
- path to a scene package directory
- path to a precompiled MuJoCo `.mjb` in consumers that support direct binary loading

Use `load_scene_package()` when you already have the exact metadata path:

```python
from dimos.simulation.scene_assets.spec import load_scene_package

package = load_scene_package("data/scene_packages/dimos_office/scene.meta.json")
```

Once resolved, consumers should use the package object instead of hardcoded
scene-specific paths:

```python
if package is not None:
    mujoco_xml = package.mujoco_scene_path
    scene_only_mjb = package.mujoco_binary_path
    rerun_visual = package.browser_visual_path("rerun")
    browser_collision = package.browser_collision_path
    objects_json = package.objects_path
    entities = package.entities
    alignment = package.alignment
```

`alignment` describes how source assets are scaled and oriented relative to the
runtime world. This lets viewers and simulators agree on frame conventions.

Browser-facing assets are target-specific. Rerun should ask for
`browser_visual_path("rerun")`; a browser renderer such as Babylon/PimSim can
ask for `browser_visual_path("babylon")`. If a target returns `None`, the
package was not cooked for that target.

## Named Packages

The current named packages are:

```text
none         no cooked scene package
office       cooked DimOS office package
supermarket  cooked supermarket package
```

## MuJoCo Loading

Normal packages are robot-agnostic:

```text
scene.meta.json -> wrapper.xml + entities -> attach robot MJCF -> compile MjModel
```

Large scenes may also ship precompiled MuJoCo binaries:

```text
wrapper.xml + robot MJCF + spawn/entity selection -> composed/<name>.mjb
```

Loading a composed `.mjb` is faster, but that file is specific to one robot,
spawn pose, entity selection, and scene revision. Keep the scene package
metadata as the source of truth and treat composed binaries as cache artifacts.

Runtime code should look up declared composed binaries through the package API:

```python
composed_mjb = package.mujoco_composed_binary_path(
    key="unitree-g1-groot-wbc_spawn_9p2_11p8_yaw_m1p57_static_only_lidar",
    robot="unitree-g1-groot-wbc",
    entity_policy="static-only",
)
```

If the lookup returns `None`, the consumer can fall back to the robot-agnostic
`wrapper.xml` flow above. If it returns a path that is missing on disk, the
package is stale or incomplete.

## Run G1 With A Scene

No scene:

```bash
dimos \
  --simulation mujoco \
  --scene-package none \
  --viewer rerun \
  --rerun-open native \
  --n-workers 12 \
  run unitree-g1-groot-wbc
```

Office:

```bash
dimos \
  --simulation mujoco \
  --scene-package office \
  --viewer rerun \
  --rerun-open native \
  --n-workers 12 \
  run unitree-g1-groot-wbc
```

Supermarket:

```bash
dimos \
  --simulation mujoco \
  --scene-package supermarket \
  --viewer rerun \
  --rerun-open native \
  --n-workers 12 \
  run unitree-g1-groot-wbc
```

Prefer headless MuJoCo with Rerun native for normal testing. Opening the MuJoCo
viewer with `mujocosimmodule.headless=false` is useful for contact debugging,
but it can run much slower.

## Boundaries

This package contract is for runtime loading. Cooking logic belongs in the
scene cooking tools. Runtime modules should stay on the `ScenePackage` API and
avoid depending on Blender, source-scene naming, or cooking heuristics.
