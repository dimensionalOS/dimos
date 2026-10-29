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

`MotorScene` removes robosuite's extra editor target gizmos. Unmodified upstream
tasks may still contain translucent gizmos and require their visibility to be
configured before using this opaque renderer.
