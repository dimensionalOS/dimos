# xArm Grasping

Two blueprints, differing only in which grasp provider they compose. Both carry
the control coordinator, the wrist camera, scene registration and
pick-and-place; both run on the real arm by default and switch to the MuJoCo
room scene with `--simulation`:

| Blueprint | Grasps |
|---|---|
| `xarm-grasp` | one top-down heuristic grasp, score 1.0 |
| `xarm-grasp-graspgenx` | up to 100 ranked learned grasps |

```bash
dimos run xarm-grasp-graspgenx --xarm7-ip 192.168.1.x     # hardware
dimos run xarm-grasp-graspgenx --simulation mujoco        # the room scene
```

Miss `--xarm7-ip` on hardware and the arm has no address to reach; leave
`--simulation` set and everything reverts to MuJoCo regardless of the IP.
`xarm-grasp-agent` and `xarm-grasp-graspgenx-agent` add an MCP agent over the
top; drive those with `dimos agent-send "..."`.

`xarm-grasp-graspgenx` needs the `graspgenx` extra and a CUDA GPU. Checkpoints
download once from Hugging Face and cache under `~/.cache/huggingface`.

```bash
uv sync --extra graspgenx
```

What differs between the arm and the sim is decided at import time: the hardware
adapter, the base pose, the camera (RealSense plus its mount edge, versus the
MuJoCo wrist camera), the detector backends, and the home pose.

The manipulation viewer is on viser at `http://127.0.0.1:8095`. To watch the
MuJoCo scene itself, add `--headless false` with `MUJOCO_GL=glfw`. On a host
where `/dev/dri` must be hidden from Mesa, run inside the team's existing
`/dev/dri`-masked mount namespace:

```bash
MUJOCO_GL=egl LIBGL_ALWAYS_SOFTWARE=true MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  dimos --viewer none run xarm-grasp --simulation mujoco
```

## Voxel map obstacles

The wrist camera feeds a live voxel map that the planner treats as one octree
obstacle, so trajectories avoid whatever has actually been seen rather than only
the registered objects:

```
camera pointcloud
  -> PointCloudSelfFilter        drops the arm's own returns, emits a clear mask
  -> RayTracingVoxelMap          accumulates occupied cells in the world frame
  -> ManipulationModule.voxel_map   rebuilt as the "mapping/voxel-map" obstacle
```

`XARM_GRASP_VOXEL_SIZE` is the single resolution all three stages share; they
must agree or the clear mask names cells the map does not hold and the octree
does not line up with what was mapped. The blueprint also enables the camera's
`pointcloud` output, which is off by default on both the RealSense and the
MuJoCo camera, and publishes TF for every one of the arm's collision links — the
self filter drops a whole cloud if any link transform is missing at capture time.

`RayTracingVoxelMap` is a native module, so it needs the Rust binary once per
box:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
cargo build --release -p dimos-voxel-ray-tracing
```

Because the target object is itself mapped geometry, a collision-checked plan
into it can only ever be rejected. The pregrasp-to-grasp leg and the retreat are
therefore straight-line `move_linear` servos with collision checking off; only
the approach to the pregrasp pose is a checked plan.

## Seeing the proposals

The viser scene draws the ranked proposals as pose glyphs: an approach axis with
the closing axis across it, coloured best-green through worst-orange so the
ordering reads at a glance, with the top three drawn thicker and labelled with
their score. Only the leading twenty are drawn — a hundred glyphs bury the
ranking they exist to show. `manipulation.grasp-proposals` in the Scene panel
toggles them.

The markers are pose indicators, not a gripper: what they promise is where a
grasp points and in what order the generator ranked it. To see what the arm will
actually do with one, watch the plan preview.

## The scene

The scene is an enclosed 2.6 m by 3.0 m room. The xArm is bolted to the world
origin — unlike `data/xarm7`, this scene has no 12 cm pedestal, so the planning
model overrides `base_pose` to match. The 38 cm by 60 cm desk is in front of the
arm, with its work surface at `z=0.13 m`. Six scaled household targets sit on it:

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
target. The blueprint starts the arm at an elevated, collision-free top-down
scan pose so all six visual meshes fit in the wrist-camera frame.

OWL-ViT labels these synthetic renders unreliably — a scan routinely returns the
right six positions under swapped names. Match objects by position, not label.

## Driving it

In a second terminal, connect to the running blueprint:

```bash
dimos shell
```

Then run this complete scan and obstacle-inspection sequence:

```python skip
from dimos.robot.manipulators.xarm.blueprints.grasp import XARM_GRASP_PROMPTS

app.ManipulationSkills.go_init()
scan = app.PickAndPlaceModule.scan_objects(XARM_GRASP_PROMPTS)
print(scan)

print(app.ObjectSceneRegistrationModule.get_detected_objects())
print(app.ManipulationModule.refresh_obstacles())
print(app.ManipulationModule.get_obstacles())
```

CPU OWL-ViT inference takes about 11 seconds per prompt/frame on the validation
host, so let a scan finish rather than issuing another concurrently.

To pick, hand `pick_object` an `object_id` from that scan. Choose the target by
where its point cloud actually is rather than by name:

```python skip
scene = app.ObjectSceneRegistrationModule

for obj in scan.metadata["objects"]:
    cloud = scene.get_object_pointcloud_by_object_id(obj["object_id"])
    print(obj, cloud.points_f32().mean(axis=0) if cloud else None)

# the bottle sits at roughly (0.58, 0.19); pick whichever id landed there
pick = app.PickAndPlaceModule.pick_object("<object_id>")
print(pick)

app.PickAndPlaceModule.place_at(0.45, -0.25, 0.25)
```

`pick_object` generates the grasps itself, so there is no separate grasp call.
It opens the gripper, plans to the pregrasp, servos in, closes, verifies, and
retreats; with a learned provider it walks the ranked candidates until one is
reachable, and the result metadata carries the winning rank, its score and the
candidate count. To inspect grasps without moving the arm, call `propose_grasps`
on the provider directly:

```python skip
cloud = scene.get_object_pointcloud_by_object_id("<object_id>")
candidates = app.GraspGenXModule.propose_grasps(cloud)   # HeuristicGraspModule in the base blueprint
print(len(candidates.candidates), [c.score for c in candidates.candidates[:5]])
```

The prompt set includes a `green ring` fallback because the tape loses
its category silhouette in the wrist camera's top-down view.

A failed grasp knocks free-body targets out of place, and `MujocoSimModule.reset()`
does not respawn them. Restart the blueprint between pick attempts that need a
pristine scene.
