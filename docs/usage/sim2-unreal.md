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
Environment visuals can also reference native Unreal assets: generated MJCF
contains their physics shapes, placements and asset bindings, while Unreal
retains the assets' opaque materials and textures.

The Python camera worker, protocol client and geometry conversion live under
`dimos/sim2/sensors/camera/unreal`. The native Unreal project lives separately at
`native/unreal/DimosRenderer`, with its C++ renderer and engine build configuration.
Its `Scripts/demo_native_scene.py` is an editor authoring example that generates
physics geometry and visual bindings; it is not part of camera acquisition.

## Where the world attaches

The G1 blueprint declares `G1Connection` once. With
`--simulation mujoco`, `ModuleCoordinator.build` calls `attach_simulation` after
resolving CLI settings. It reads `G1Connection.describe_simulated_robot`, adds
the shared `SimulationModule`, and supplies a simulation device to the retained
connection. The camera is an additional module: its `SimulationSpec` reference
and `snapshots` input connect to that world through normal blueprint wiring.
Adding `UnrealCameraModule` does not create another simulation.

The `g1_groot_wbc_simulation_profile()` helper only returns the G1 control and
navigation stack without a viewer and requires the simulation operating profile.
The world still attaches at build time. See
[attach_simulation](/dimos/sim2/composition.py) and the
[connection binding](/dimos/robot/unitree/g1/connection.py).

## Build on macOS

Requires an installed Unreal Engine 5.7 and its Xcode toolchain. From the repo:

```bash
native/unreal/DimosRenderer/Scripts/build_macos.sh \
  "/Users/Shared/Epic Games/UE_5.7" \
  "$HOME/.cache/dimos/unreal-renderer"
```

The build generates one shared material, then cooks a standalone application.
Python inside Unreal is used only during asset preparation. The running renderer
is C++. Generated engine files and packaged binaries stay outside Git.

To package a renderer project containing authored assets, pass its `.uproject`
as the third argument. Add the authored map to that project's `MapsToCook` and
keep `bCookAll=False`; Unreal cooks the map's referenced mesh, material, and
texture dependencies. The runtime still starts in the empty Entry map.

```bash
native/unreal/DimosRenderer/Scripts/build_macos.sh \
  "/Users/Shared/Epic Games/UE_5.7" \
  "$HOME/.cache/dimos/unreal-konbini-packaged" \
  "$HOME/.cache/dimos/unreal-konbini/DimosRenderer/DimosRenderer.uproject"
```

Check the packaged camera against a known plane, then move the plane and camera:

```bash
renderer="$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
.venv/bin/python native/unreal/DimosRenderer/Scripts/demo_calibration.py \
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
from dimos.sim2.sensors.camera.unreal.module import UnrealCameraModule


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

### Drive with the keyboard

Click inside the camera image before driving. Hold **W/S** to move forward/back,
**A/D** to turn, or **Q/E** to strafe; release to stop. **Space** stops immediately.
Keys do not drive while a text field or an overlay has focus.

The observer camera is fixed in the world. Forward means the direction the
robot faces, which may be sideways or diagonal on screen. The registered
blueprint is not a first-person camera view.

A local three-second W hold published 0.4 m/s at approximately 20 Hz through
`tele_cmd_vel` and `g1/cmd_vel`, moved the robot about 0.93 m according to
odometry, and ended with zero velocity commands. This verifies keyboard control;
the cause of an earlier unresponsive-key report was not established.

## Headless validation

After building the renderer, run the existing robot application with one camera
module added:

```bash
renderer="$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
.venv/bin/python -m dimos.robot.unitree.g1.demo_unreal \
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

Source: [demo_unreal.py](/dimos/robot/unitree/g1/demo_unreal.py). The only robot-specific model
code is still [simulation.py](/dimos/robot/unitree/g1/model.py); Unreal has no
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

Two visual paths share the same physics poses:

| Visual source | What Unreal receives | Current material support |
|---|---|---|
| Compiled MuJoCo rigid meshes and primitives | Geometry, pose, color and roughness | Opaque colors; no imported UVs or source textures |
| Referenced native Unreal static meshes | Asset path, pose and scale | The asset's existing opaque materials and textures |

Both paths reject partial transparency; the native path also rejects non-opaque
material blend modes. Deformables and source lighting are not transferred.
Lighting is currently one fixed Unreal setup. Unsupported geometry or missing
native assets fail explicitly.

The camera produces aligned RGB-D with shadows. Point clouds and modeled sensor
latency are not implemented in this module. Per-reset model changes propagate;
editing mesh/material definitions during an episode does not.

## Which assets to provide

The shipped G1 model and logistics scene already contain everything needed to
run this demo. No additional assets are required.

For the procedural path, provide MJCF and its referenced meshes. A box's printed
label or a floor's wood grain becomes a plain surface because this path does not
transfer UV coordinates or image textures.

For a textured environment, provide its editable Unreal project and referenced
`Content` assets, including the map, meshes, materials, textures and simple
collision shapes. Keep movable parts separate and named, with correct scale and
pivots. Retain the robot's existing joint/physics definition and collision
geometry. Native assets must use opaque materials and be present in the renderer
project or its cooked package.

The current exporter handles a small, explicitly selected set of static-mesh
actors. It is not an arbitrary full-map importer. The renderer starts in its
empty Entry map and creates the exported asset instances; it does not run the
authored map's gameplay, lighting or Chaos simulation.

## Authoring one scene for Unreal and MuJoCo

The bounded implementation is
[demo_native_scene.py](/native/unreal/DimosRenderer/Scripts/demo_native_scene.py).
It authors a scratch map using copied Konbini assets: one shelf, one 0.3 kg
cereal box, a floor, a back wall and a G1 spawn. It exports those four mesh
actors and the spawn; it does not import the original convenience-store map.

Unreal already separates a visible mesh from its collision shapes. Those shapes
normally feed Unreal's physics engine, Chaos. Using MuJoCo requires translating
them into MuJoCo geometry; packaging an Unreal project does not perform that
translation. See Epic's [simple and complex collision documentation](https://dev.epicgames.com/documentation/en-us/unreal-engine/simple-versus-complex-collision-in-unreal-engine?application_version=5.7).

1. **Copy the assets into an isolated renderer project.** Keep Konbini's content
   at `/Game/Konbini`, including its material and texture dependencies. Keep the
   original project unchanged. Build the renderer's Editor target so its
   `DimosAssetLibrary` can inspect the source simple collision shapes.
2. **Author placements once.** The proof script creates and saves
   `/Game/DimosTests/KonbiniNativeProof`, tags each object's identity and fixed or
   movable role, and supplies the cereal box's mass and the robot spawn.
3. **Generate physics and visual bindings together.** The exporter converts
   component and collider transforms, positive scale, centimetres and handedness
   into MJCF. Source box and convex collision shapes are retained as convex
   geometry; the detailed mesh remains an Unreal asset reference.
4. **Attach the existing robot.** G1's model, joints, inertia and controller come
   from its existing package at the exported spawn. No Unreal G1 class or second
   robot definition is required.
5. **Run the registered blueprint and open Controller in Desktop.** MuJoCo owns
   contact and motion. Unreal updates the bound mesh instances from those poses
   with their Chaos simulation and collision disabled.

| Proof object | Native Unreal visual | Generated MuJoCo representation |
|---|---|---|
| Shelf | `SM_Shelf_01` with `MI_Shelves` | Its existing single solid convex collider, fixed in place |
| Cereal box | `SM_CerealBox_01` with `MI_Cereal_01` | Its existing convex collider on a free body, mass 0.3 kg |
| Floor and wall | Scaled Engine cube assets | The source box colliders at the same placements |
| G1 | Existing robot mesh export | Existing G1 model at the authored spawn |

**The source shelf has one solid outer collision hull. Its openings remain
blocked in this proof.** Exporting the exact source collider does not make it
suitable for reaching into shelves. To enable that task, author separate convex
pieces for the uprights and individual shelves, then export again. MuJoCo's
standard mesh collision uses convex hulls; it does not infer open shelf spaces
from the detailed visual mesh. See [MuJoCo mesh collision behavior](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh).

This exporter accepts box and convex simple collision only, requires one hull
for a movable prop, and rejects zero or reflected scales. It does not export
articulated fixtures, arbitrary actor types or per-instance material overrides.
Sphere/capsule collision export and complex triangle-mesh collision are outside
this proof. Assets configured to use complex collision as simple collision are
rejected. Mass comes from the authored DimOS tag and friction uses demo values;
Unreal physical materials are not imported.

### Export the local Konbini proof

These commands use the isolated renderer project with copied Konbini content,
not the original store project. The script verifies original content hashes when
`DIMOS_NATIVE_SOURCE_PROJECT` is set. Run from this repository:

```bash skip
engine_dir="/Users/Shared/Epic Games/UE_5.7"
native_project="$HOME/.cache/dimos/unreal-konbini/DimosRenderer/DimosRenderer.uproject"
source_root="$HOME/Downloads/Japanese_Convenience_Store_Konbini_UE5"
scene_dir="$HOME/.local/state/dimos/unreal/konbini-proof"

"$engine_dir/Engine/Build/BatchFiles/Mac/Build.sh" \
  DimosRendererEditor Mac Development "$native_project" -WaitMutex
DIMOS_NATIVE_SCENE_OUTPUT="$scene_dir" DIMOS_NATIVE_SOURCE_PROJECT="$source_root" \
  "$engine_dir/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" \
  "$native_project" -unattended -RenderOffscreen -nosound \
  "-ExecutePythonScript=$PWD/native/unreal/DimosRenderer/Scripts/demo_native_scene.py"
```

Use a real graphics backend for this editor script. `-NullRHI` crashed in UE
5.7.4 while spawning actors; `-RenderOffscreen` completed the author/export step.

The generated outputs are:

| File | Purpose |
|---|---|
| `scene.xml` | MuJoCo collision geometry, bodies, initial placements and embedded Unreal asset bindings |
| `scene.json` | DimOS entities and the robot spawn |
| `native-scene-audit.json` | Source assets, collider descriptions, transforms and original-content hash verification |

The binding is embedded in MJCF `<custom>` text named `dimos_unreal_assets`.
`MotorScene` preserves it when composing the robot. Native mesh references bind
to noncolliding, massless marker geoms at their asset origins; the normal pose
stream handles fixed and moving objects. There is no separate mapping file to
maintain. Generated `hidden_geoms` omit collider visuals only from Unreal;
colliders stay opaque in MuJoCo so lidar rays still see them.

Add `/Game/DimosTests/KonbiniNativeProof` to the scratch project's `MapsToCook`,
then package it using the third-argument build command above. The referenced
assets and their dependencies must be cooked before running the standalone
renderer. Once that package is available, launch its exported physics scene:

```bash skip
XDG_STATE_HOME="$HOME/.local/state/dimos/unreal/desktop-home/state" \
  .venv/bin/dimos --simulation mujoco --viewer none --transport zenoh \
  --scene-package "$HOME/.local/state/dimos/unreal/konbini-proof/scene.xml" \
  run unitree-g1-groot-wbc-unreal \
  --unrealcameramodule.executable \
  "$HOME/.cache/dimos/unreal-konbini-packaged/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
```

The local proof passed authoring, repeat export, direct MuJoCo drop/contact and
lidar-ray checks, plus walking and reset with both Editor and the standalone
renderer. All 881 original project/content files retained their hashes. The
package includes 12 Konbini asset dependencies; the source store map is not
cooked or loaded.

The packaged native scene ran for 30 seconds at 199.8 physics steps/s, 50.0
controller ticks/s and 8.28 RGB-D frames/s, with a 31.1 ms median capture time.
G1 walked 0.54 m and remained standing after reset. The cereal box contacts the
floor after 0.268 s in the isolated drop test; small residual rocking remains.
Desktop displays the same package's live RGB and metric depth. The earlier
performance table describes the procedural logistics scene.

`MotorScene` removes robosuite's extra editor target gizmos. Unmodified upstream
tasks may still contain translucent gizmos and require their visibility to be
configured before using this opaque renderer.
