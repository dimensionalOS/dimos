# xArm Room Simulation

`xarm-room-sim` launches the complete headless room demo: xArm7 MuJoCo
simulation, wrist-camera OWL-ViT scene registration, perception-backed planner
obstacles, composed pick-and-place, and the control coordinator.

```bash
MUJOCO_GL=egl LIBGL_ALWAYS_SOFTWARE=true MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  dimos --viewer none run xarm-room-sim
```

The blueprint disables the MuJoCo and manipulation viewers itself. On a host
where `/dev/dri` must be hidden from Mesa, run the same command in the team's
existing `/dev/dri`-masked mount namespace. CPU OWL-ViT inference takes about
11 seconds per prompt/frame on the validation host, so allow the scan to
finish rather than issuing another scan concurrently.

The scene is an enclosed 2.6 m by 3.0 m room. The xArm stands on a 12 cm base
pedestal at the room origin; its planning model uses that base pose directly.
The 38 cm by 60 cm desk is in front of the arm, with its work surface at
`z=0.13 m`. Six scaled household targets sit on it:

| Object | Body position `(x, y, z)` m | Maximum grasp width | Approximate size |
|---|---:|---:|---:|
| Dark blue bottle | `(0.58, 0.19, 0.13)` | 6.0 cm | 6.0 × 6.0 × 17.6 cm |
| Gray can | `(0.545, -0.02, 0.13)` | 6.6 cm | 6.6 × 6.6 × 12.2 cm |
| Red cup | `(0.56, -0.22, 0.13)` | 6.8 cm | 6.8 × 6.8 × 6.1 cm |
| Green tape roll | `(0.38, 0.24, 0.13)` | 6.2 cm | 6.2 × 6.2 × 2.3 cm |
| Blue marker | `(0.35, 0.03, 0.13)` | 3.2 cm | 14.0 × 3.2 × 3.2 cm |
| Brown box | `(0.40, -0.19, 0.13)` | 6.0 cm | 8.4 × 6.0 × 4.5 cm |

The canonical positions and geometry live in `data/xarm_grasp_sim/scene.xml`.
The visual meshes were cooked from the `dimos_office` scene package and scaled
for the xArm gripper; primitive geometry is used only for contact. Every target
has a grasp axis below 7 cm, and the gray can is the designated pick smoke
target. The room blueprint starts the arm at an elevated, collision-free
top-down scan pose so all six visual meshes fit in the wrist-camera frame.

In a second terminal, connect to the running blueprint:

```bash
dimos shell
```

Then run this complete scan and obstacle-inspection sequence:

```python skip
from dimos.robot.manipulators.xarm.blueprints.simulation import XARM_ROOM_PROMPTS

app.ManipulationSkills.go_init()
scan = app.PickAndPlaceModule.scan_objects(XARM_ROOM_PROMPTS)
print(scan)

print(app.ObjectSceneRegistrationModule.get_detected_objects())
print(app.ManipulationModule.refresh_obstacles())
print(app.ManipulationModule.get_obstacles())
```

Wait for `scan_objects` to finish before issuing another scan. The prompt set
includes a `green ring` fallback because the tape loses its category silhouette
in the wrist camera's top-down view.
