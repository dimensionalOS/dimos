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

## Selecting the grasp generator

The room, basic perception simulation, and real perception blueprints share
`GraspProposalModule`. Its default generator is the top-down heuristic:

```bash
dimos run xarm-room-sim
```

To select GraspGenX, install its optional dependencies and supply your gripper
configuration. The `all` extra does not include GraspGenX.

```bash
uv sync --extra all --extra graspgenx
dimos run xarm-room-sim --config xarm-grasp.json \
  --graspproposalmodule.generator.backend graspgenx
```

Use the headless rendering environment from the launch example above when
needed. The same options work with `xarm-perception-sim` and
`xarm-perception`; the real blueprint still requires its camera mount TF and
hardware coordinator to be completed.

The JSON file has this structure. Replace each placeholder with measured
values before use; this template is not a calibrated xArm configuration:

```text
{
  "graspproposalmodule": {
    "generator": {
      "backend": "graspgenx",
      "gripper": {
        "extents_open": [OPEN_X, OPEN_Y, OPEN_Z],
        "offset_open": [OPEN_OFFSET_X, OPEN_OFFSET_Y, OPEN_OFFSET_Z],
        "extents_half_open": [HALF_X, HALF_Y, HALF_Z],
        "offset_half_open": [HALF_OFFSET_X, HALF_OFFSET_Y, HALF_OFFSET_Z],
        "fingertip_depth": DEPTH_METRES,
        "family": "parallel_2f"
      },
      "grasp_frame_to_tcp": [
        [R00, R01, R02, TX],
        [R10, R11, R12, TY],
        [R20, R21, R22, TZ],
        [0, 0, 0, 1]
      ],
      "max_candidates": 100
    }
  }
}
```

Lengths and translations are in metres. Choose the family matching your
gripper: `parallel_2f`, `revolute_2f`, or `revolute_3f`. The TCP transform
maps the model grasp frame to the robot TCP. Its default is identity, which is
valid only if those frames coincide.

CLI overrides take precedence over JSON values. For example, add
`--graspproposalmodule.generator.max-candidates 20` to limit returned proposals.
Run `dimos run xarm-room-sim --help` to inspect available config fields.
Backend changes take effect on restart. GraspGenX initializes in a dedicated
worker, downloads its pinned checkpoint on first use, and reports failures
without falling back to the heuristic. Heuristic startup and CLI help do not
load the optional model runtime.

After scanning, inspect proposals without moving the arm in `dimos shell`:

```python skip
print(scan.metadata["objects"])
object_id = "COPY_AN_ID_FROM_THE_SCAN"
cloud = app.ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id(object_id)
assert cloud is not None
proposals = app.GraspProposalModule.propose_grasps(cloud)
print(proposals.header.frame_id, len(proposals))
for candidate in proposals.candidates[:5]:
    print(candidate.score, candidate.pose)
```

`app.PickAndPlaceModule.pick_object(object_id)` generates a fresh set of proposals
and attempts its first candidate. It does not execute a previously inspected
proposal or retry other candidates. Retrieve that set through
`app.PickAndPlaceModule.get_grasp_candidates()`.
