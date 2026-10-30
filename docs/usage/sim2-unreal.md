# Unreal cameras for sim2

One running world owns physics. One native Unreal process renders its snapshots.

```text
existing robot connection <-> robosuite / MuJoCo
                                  |
                         compiled model + poses
                                  |
                         UnrealCameraModule
                                  |
                      native Unreal application
                                  |
                      RGB + axial depth in metres
```

The renderer imports the **compiled model already used by physics**. Adding a
robot does not require an Unreal robot class, a second joint definition, or
hand-placed meshes. Unreal never advances the robot or performs collisions.

## Build on macOS

Requires an installed Unreal Engine 5.7 and its Xcode toolchain. From the repo:

```bash
misc/unreal/DimosRenderer/Scripts/build_macos.sh \
  "/Users/Shared/Epic Games/UE_5.7" \
  "$HOME/.cache/dimos/unreal-renderer"
```

The build generates one shared material, then cooks a standalone application.
Python inside Unreal is used only during asset preparation. The running renderer
is C++. Generated engine files and packaged binaries stay outside Git.

Check the packaged camera against a known plane, then move the plane and camera:

```bash
renderer="$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
.venv/bin/python misc/unreal/DimosRenderer/Scripts/demo_calibration.py \
  --executable "$renderer" --output /tmp/dimos-unreal-calibration
```

This checks colored RGB, axial depth, off-center intrinsics, matching silhouettes,
camera and geometry motion, reset identity, and shutdown when its owner disconnects.

## Attach a camera

Compose this module with a blueprint containing a `SimulationModule`. A named
camera keeps the calibration and mounting pose from the existing model:

```python
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.unreal.module import UnrealCameraModule


def with_unreal(robot_application, executable: Path):
    return autoconnect(
        robot_application,
        UnrealCameraModule.blueprint(
            executable=executable,
            robot_id="g1",
            sensor=Camera("front", "front_camera", width=640, height=480),
            rpc_timeouts={"start": 210.0},
        ),
    )
```

The camera publishes ordinary `color_image`, `depth_image`, `camera_info`,
`depth_camera_info` and optical `tf` streams. `frame_metadata` identifies the
source episode, physics tick and simulation time. `renderer_status()` reports
capture time, round-trip time, acquisition age and any error.

## Run with DimOS Desktop

Use the registered `unitree-g1-groot-wbc-unreal` blueprint. It reuses the G1
GR00T, Mid360 and Point-LIO stack and adds one Unreal observer camera. It has no
Rerun modules. The simulation flag is required; an omitted flag is rejected
before any robot modules start.

Point Desktop at this checkout. Its Controller app displays the ordinary
`color_image` and `depth_image` topics. Start the registered blueprint from the
CLI, with Desktop and the CLI using the same state directory:

```bash
.venv/bin/dimos --simulation mujoco --viewer none --transport zenoh \
  run unitree-g1-groot-wbc-unreal \
  --unrealcameramodule.executable \
  "$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
```

The executable can also be supplied through `UNREALCAMERAMODULE__EXECUTABLE`.
Desktop discovers its state and camera topics on the standard Zenoh domain.
Use Desktop's Stop control, or `dimos stop` from the same state directory, to
stop the run. A separately configured Desktop home can use a different run
registry: match its state directory to keep status and lifecycle controls
together.

Locally verified with Desktop 0.2.248, Controller 0.4.3 and Desktop's pinned
gateway 8.3.3: live RGB, metric depth, run discovery and Desktop's Stop control.
The Launcher's Customize form currently fails against this checkout because
the pinned gateway's config introspection imports `dimos.utils.shared_config`,
which this branch does not provide. Use the CLI command above until that
gateway compatibility issue is resolved.

For a Desktop started with `--home "$HOME/.local/state/dimos/unreal/desktop-home"`,
prefix the CLI command with
`XDG_STATE_HOME="$HOME/.local/state/dimos/unreal/desktop-home/state"` so Desktop
can discover and stop it. In Controller, add two Camera panels and select
`color_image` and `depth_image` respectively.

## Headless validation

After building the renderer, run the existing robot application with one camera
module added:

```bash
renderer="$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
.venv/bin/python -m dimos.sim2.unreal.demo_g1 \
  --executable "$renderer" --walk --reset \
  --output /tmp/dimos-g1-unreal
```

This is a 30-second headless validation, not an interactive viewer. Ctrl-C saves
the report and shuts down the renderer and robot stack. `--seconds 0` keeps the
headless validation running until stopped.

The validation script uses an isolated transport domain and bypasses the normal
CLI run registry. Use the registered blueprint above for Desktop; starting the
gateway alone does not attach this validation script. There is no viewer option
in the script.

The default camera observes the robot. Add `--camera front` to use the existing
G1 `front_camera`, including its authored mount and calibration. Both use the
same G1 model, connection, GR00T controller, Mid360 and Point-LIO. The demo forces
simulation and uses isolated transports.

Outputs: `preview.gif`, `rgb.png`, metric `depth.npy`, `calibration.json`, and
`report.json`. The report checks actual walking displacement, controller and
physics health, matching image timestamps, reset epochs and live lidar/odometry.

Source: [demo_g1.py](/dimos/sim2/unreal/demo_g1.py). The only robot-specific model
code is still [simulation.py](/dimos/robot/unitree/g1/simulation.py); Unreal has no
G1 class or separate robot asset definition.

Measured on an M4 Max with UE 5.7.4, 640×480 RGB-D requested at 10 Hz:

| Packaged run | Physics | Control | RGB-D | Median capture |
|---|---:|---:|---:|---:|
| 30 s, walking + reset | 199.8 Hz | 50.0 Hz | 9.15 fps | 28.9 ms |
| 10 s, existing front camera | 199.9 Hz | 50.0 Hz | 9.72 fps | 27.8 ms |

The reset run includes scene reload time. These are local measurements, not a
hardware-independent throughput guarantee. The calibrated 5 m plane read
5.0000005 m in the packaged renderer.

## Timing and reset

- Physics runs independently of capture and GPU readback. The camera takes the
  newest available snapshot; slow rendering does not create an image backlog.
- RGB, depth, calibration and TF describe the same snapshot and carry its
  acquisition timestamp. Delivery time is measured separately.
- Reset reloads the compiled model and scene when needed. A capture from an old
  episode is discarded if reset arrives before publication.
- Shutdown cancels pending I/O and reaps only the renderer process this module
  started. No fixed network port or external server is required.

## Current scope

This is one concrete MuJoCo-to-Unreal integration, validated on Apple Silicon
with Metal. Windows and Linux packaging have not been validated.

Rigid meshes, boxes, spheres, capsules, cylinders and planes preserve their
compiled placement, opaque color and roughness. Source textures, transparency,
deformables and source lighting are not reproduced. Lighting is currently one
fixed Unreal setup. Unsupported geometry fails explicitly.

The camera produces aligned RGB-D with shadows. Point clouds and modeled sensor
latency are not implemented in this module. Per-reset model changes propagate;
editing mesh/material definitions during an episode does not.

## Which assets to provide

The shipped G1 model and logistics scene already contain everything needed to
run this demo. No additional assets are required.

Currently the loader accepts the compiled MuJoCo world, built from MJCF and its
referenced meshes. It transfers positions, surfaces, plain colors and roughness.
It does not transfer UV coordinates or image textures: a box's printed label or
a floor's wood grain becomes a plain surface. Partially transparent geometry,
including glass, is rejected explicitly.

For a textured environment, provide its editable Unreal project and referenced
`Content` assets, including the map, meshes, materials, textures and simple
collision shapes. Keep movable parts separate and named, with correct scale and
pivots. Retain the robot's existing joint/physics definition and collision
geometry.

**An authored Unreal map is not loaded by the current helper.** Supplying better
assets alone will not fix the current importer's material limitations.

## Authoring one scene for Unreal and MuJoCo

This is the intended workflow; the scene exporter and authored-asset bindings
below still need implementation. The current bridge works in the opposite
direction: it copies an existing MuJoCo world into Unreal.

Unreal already separates a visible mesh from its collision shapes. Those shapes
normally feed Unreal's physics engine, Chaos. Using MuJoCo requires translating
them into MuJoCo geometry; packaging an Unreal project does not perform that
translation. See Epic's [simple and complex collision documentation](https://dev.epicgames.com/documentation/en-us/unreal-engine/simple-versus-complex-collision-in-unreal-engine?application_version=5.7).

1. **Prepare each asset once.** Keep its detailed mesh, materials and textures in
   Unreal. Add or review simple collision shapes: boxes, spheres, capsules or
   convex pieces. Reuse those collision shapes for every instance of that asset.
2. **Place the scene once in Unreal Editor.** Place shelves, floors and props;
   identify fixed objects, movable objects and the robot spawn. Movable objects
   also need mass and contact settings, with inertia supplied or derived from
   validated colliders. A door
   needs a joint and limits, not just a mesh marked movable.
3. **Export the simulation scene.** A DimOS editor exporter must generate the
   MJCF environment, collision meshes and stable actor/component-to-body
   bindings from those placements. It must convert units, axes, component
   transforms and scaling, and reject unsupported collision or joint
   definitions. These are generated outputs, not a second
   scene to edit by hand.
4. **Reuse the robot definition.** Add the existing G1 model at its spawn pose.
   Its joints, limits, inertias, collision geometry and controller remain in the
   robot package. Unreal's robot visuals follow those link poses.
5. **Run through DimOS Desktop.** MuJoCo computes motion and contact. Unreal
   loads the authored map and updates bound visual objects from MuJoCo's poses,
   with Chaos simulation disabled for those objects. Camera images and robot
   state flow through the normal DimOS streams and gateway to Desktop.

| Scene object | What you author in Unreal | What the exporter must generate for MuJoCo |
|---|---|---|
| Fixed shelf | Textured mesh, boxes for each shelf and upright, one placement | Fixed collision geometry at that placement; open shelf spaces remain open |
| Movable crate | Textured mesh, one box collider, mass/friction, initial placement | Free body with that collider, inertia and initial pose |
| G1 robot | Spawn placement referencing the existing robot | Existing G1 model at that pose; no second joint or controller definition |

Do not export an entire room or shelf as one ordinary collision mesh: MuJoCo's
standard mesh collision uses its convex hull, which can fill doorways and shelf
openings. Export separate primitives or convex pieces instead. A scene that uses
Unreal's triangle-mesh collision alone needs suitable collision shapes before it
can use this workflow. See [MuJoCo mesh collision behavior](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh).

The Unreal project supplies the environment's authored visuals and collision
shapes. The robot package supplies the robot. The exporter connects them; the
user should not maintain matching object placements in an Unreal map and a
handwritten MJCF file.

`MotorScene` removes robosuite's extra editor target gizmos. Unmodified upstream
tasks may still contain translucent gizmos and require their visibility to be
configured before using this opaque renderer.
