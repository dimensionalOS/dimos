# Unreal cameras for sim2

One running world owns physics. One native Unreal process renders its body poses.
For the full local convenience store, see [Load the full Konbini store](#load-the-full-konbini-store).

```text
G1Connection <-> SimRobotClient <-> SimulationModule <-> MuJoCo / Genesis
                                         |
                               WorldState + VisualScene
                                         |
                                UnrealCameraModule
                                         |
                              native Unreal application
                                         |
                             RGB + axial depth in metres
```

The renderer receives static visual assets and named body poses. Asset import
happens once per scene revision; each frame uses neutral `WorldState`, without
loading a physics checkpoint or reconstructing an engine state. Adding a robot
does not require an Unreal robot class or a second joint definition. Unreal does
not advance physics. Native Unreal assets retain their opaque materials/textures;
source MJCF supplies collision geometry and visual bindings.

The Python camera worker, protocol client and geometry conversion live under
`dimos/sim2/sensors/camera/unreal`. The native Unreal project lives separately at
`native/unreal/DimosRenderer`, with its C++ renderer and engine build configuration.
Its `Scripts/demo_native_scene.py` is an editor authoring example that generates
physics geometry and visual bindings; it is not part of camera acquisition.

## Where the world attaches

The G1 blueprint declares `G1Connection` once. With
`--simulation mujoco` or `--simulation genesis`, `ModuleCoordinator.build` calls `attach_simulation` after
resolving CLI settings. It reads `G1Connection.describe_simulated_robot`, adds
the shared `SimulationModule`, and supplies a simulation device to the retained
connection. The camera is an additional module: its `SimulationSpec` reference
and `world_state` input connect to that world through normal blueprint wiring.
Adding `UnrealCameraModule` does not create another simulation.

The control-only G1 blueprints work with either engine. The full navigation
profile currently needs MuJoCo's camera/lidar acquisition. The world attaches
at build time in both cases. See [attachment](/dimos/sim2/composition.py) and the
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

For Genesis, install the separate environment described in
[sim2](/docs/usage/sim2.md), then use the control-only profile:

```bash
.venv-genesis/bin/dimos --simulation genesis --transport zenoh \
  run unitree-g1-groot-control-unreal \
  --unrealcameramodule.executable \
  "$HOME/.cache/dimos/unreal-renderer/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer"
```

`unitree-g1-control-unreal` uses the stock locomotion policy instead of GR00T.
Both control-only variants also work with MuJoCo. They omit lidar and navigation.

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

For the same control/RGB-D/reset proof with Genesis:

```bash
.venv-genesis/bin/python -m dimos.robot.unitree.g1.demo_unreal \
  --engine genesis --control-only --executable "$renderer" --walk --reset \
  --output /tmp/dimos-g1-genesis-unreal
```

The control-only check validates RGB-D, movement, the retained connection and
reset; it does not claim to validate uncomposed lidar/navigation modules.

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
code is still [model.py](/dimos/robot/unitree/g1/model.py); Unreal has no
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
- Reset reloads visual assets when the scene revision changes. A capture from an old
  episode is discarded if reset arrives before publication.
- Shutdown cancels pending I/O and reaps only the renderer process this module
  started. No fixed network port or external server is required.

## Current scope

The renderer consumes neutral poses from MuJoCo or Genesis, validated on Apple
Silicon with Metal. The full-store navigation run below uses MuJoCo. Windows and
Linux packaging have not been validated.

The renderer supports these visual sources:

| Visual source | What Unreal receives | Current material support |
|---|---|---|
| Compiled MuJoCo rigid meshes and primitives | Geometry, pose, color and roughness | Opaque colors; no imported UVs or source textures |
| Referenced native Unreal static meshes | Asset path, pose and scale | The asset's existing opaque materials and textures |
| Authored native level | A cooked map selected with `--unrealcameramodule.level` | Native materials, overrides, instances, decals and transparency |

Manifest-created geometry rejects partial transparency and non-opaque materials.
An authored level retains its native transparent materials in RGB; Unreal
`SceneDepth` reports depth-writing surfaces, not invented depths on glass. The
level retains direct light placement and intensity; lights become movable because
this renderer disables baked lighting. Baked GI and reflections are not restored.
Without an authored level, the renderer uses its fixed lighting setup. Missing
native assets and unsupported manifest geometry fail explicitly.

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

The small native-asset exporter below handles an explicitly selected set of
static-mesh actors. Full static levels can instead remain native in Unreal, with
their collision exported separately. Background actor/component ticks and Chaos
are disabled; cinematic autoplay must be disabled in the copied map. Articulated
fixtures and dynamic map gameplay are outside this static-level path.

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


## Load the full Konbini store

The local source project is `~/Downloads/Japanese_Convenience_Store_Konbini_UE5`.
Its full `/Game/Konbini/Maps/Konbini` map contains 6,779 mesh components, including
stocked shelves, refrigerators, counters and storefront glass. It is already
copied into the isolated renderer project. The original source assets remain
unchanged. Other available local projects include CityShoppingMall, its clothing
store maps, and ShenzhenOffice; those are not prepared physics scenes yet.

The store runs as the native background, with G1 rendered from its existing model.
All store fixtures and products are static. The copied map's cinematic autoplay
is disabled. For physics, all 6,771 collidable component instances are exported
as 6,896 collision geoms with 225 shared meshes under one static world body.
Collider visuals are hidden only in the Unreal manifest; they stay opaque and
visible to MuJoCo lidar rays.

Two source colliders needed repair: the building's original single hull filled
the entire interior, and the window hull sealed the storefront. Unreal convex
decomposition generated 64 and 63 hulls respectively in the copied assets. These
are approximate collision pieces, validated for aisle clearance and G1 walking.
The finite asphalt plane gets an explicit 1 mm thickness below its original
surface. Shelf openings remain blocked by their source outer hulls; this is a
navigation scene, not a shelf-manipulation validation.

Preparation and export are reproducible with
[demo_prepare_konbini.py](/native/unreal/DimosRenderer/Scripts/demo_prepare_konbini.py)
and [export_level_collisions.py](/native/unreal/DimosRenderer/Scripts/export_level_collisions.py):

```bash skip
engine_dir="/Users/Shared/Epic Games/UE_5.7"
store_project="$HOME/.cache/dimos/unreal-konbini/DimosRenderer/DimosRenderer.uproject"
store_scene="$HOME/.local/state/dimos/unreal/konbini-store"

"$engine_dir/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" \
  "$store_project" -unattended -RenderOffscreen -nosound \
  "-ExecutePythonScript=$PWD/native/unreal/DimosRenderer/Scripts/demo_prepare_konbini.py"
.venv/bin/python native/unreal/DimosRenderer/Scripts/export_level_collisions.py \
  /tmp/dimos-konbini-map-audit.json "$store_scene" \
  --spawn .8 2.1 .13916100885982935 --yaw-deg -90 \
  --freeze-movable --surface-thickness .001
```

The audit records original asset hashes, collision repairs and actor transforms.
The collision manifest maps every exported geom back to its source component.
The spawn height is the measured support surface in the corrected collision.
The stock position-servo G1 asset requires a named floor contact; this store run
uses the existing GR00T torque-control plant.

Add `/Game/Konbini/Maps/Konbini` to the copied project's `MapsToCook`, then package
with the existing build script into `~/.cache/dimos/unreal-konbini-store-packaged`.
The already prepared local launch file is:

```bash skip
"$HOME/.local/state/dimos/unreal/konbini-store/run.sh"
```

It selects `unitree-g1-groot-wbc-unreal`, the exported store scene, the packaged
renderer, `--unrealcameramodule.level /Game/Konbini/Maps/Konbini`, and a fixed
960x640 observer camera above the clear front aisle. It uses the same Desktop
home/state directory as the earlier proof. Add `--daemon` to run in background.
Desktop Controller's `color_image` and `depth_image` panels show the new scene;
click the image before using WASD/QE, and Space to stop.

The isolated G1 walking/reset check can also use the full map:

```bash skip
.venv/bin/python -m dimos.robot.unitree.g1.demo_unreal \
  --executable "$HOME/.cache/dimos/unreal-konbini-store-packaged/Mac/DimosRenderer.app/Contents/MacOS/DimosRenderer" \
  --level /Game/Konbini/Maps/Konbini --scene "$store_scene/scene.xml" \
  --camera front --walk --reset --seconds 20 --output "$store_scene/g1-proof"
```
