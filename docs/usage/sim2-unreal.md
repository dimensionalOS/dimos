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

## Run G1 with GR00T and Point-LIO

After building the renderer, run the existing robot application with one camera
module added:

```bash
renderer="$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
.venv/bin/python -m dimos.sim2.unreal.demo_g1 \
  --executable "$renderer" --viewer rerun --seconds 0 --walk --reset \
  --output /tmp/dimos-g1-unreal
```

This opens a live Rerun browser view: Unreal RGB and depth beside the existing
G1/Point-LIO view. Unreal produces the camera images; Rerun displays them.
`--seconds 0` keeps running until Ctrl-C, which saves the report and shuts down
the renderer and robot stack. Use `--viewer-open native` for a native Rerun
window. For the original 30-second headless check, omit `--viewer rerun` and
`--seconds 0`.

This demo is **not integrated with DimOS Desktop**. It runs on an isolated
transport domain and bypasses the normal CLI run registry. Desktop needs a
compatible gateway, a registered runnable composition and matching discovery;
starting the gateway alone does not connect this demo.

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
`Content` assets, including the map, meshes, materials and textures. Keep movable
parts separate and named, with correct scale and pivots; retain the robot's
existing joint/physics definition and collision geometry. A UV-mapped GLB with
PBR textures is a portable alternative handoff.

**Neither an Unreal map nor GLB is loaded by the current helper.** Adding the
normal Unreal asset import/cook path and matching visual actors to physics
bodies is still required. Supplying better assets alone will not fix the current
importer's material limitations.

`MotorScene` removes robosuite's extra editor target gizmos. Unmodified upstream
tasks may still contain translucent gizmos and require their visibility to be
configured before using this opaque renderer.
