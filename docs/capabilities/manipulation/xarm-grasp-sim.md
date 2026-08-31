# xArm Grasp Simulation

`xarm-grasp-sim` launches the complete room demo: xArm7 MuJoCo simulation,
wrist-camera OWL-ViT scene registration, perception-backed planner obstacles,
composed pick-and-place, and the control coordinator.

```bash
MUJOCO_GL=glfw dimos run xarm-grasp-sim --headless false
```

The blueprint ships `headless: True`, so `--headless false` is what opens the
MuJoCo passive viewer. To run without a window instead, drop the flag and use
the offscreen renderer:

```bash
MUJOCO_GL=egl dimos run xarm-grasp-sim
```

## Grasp providers

The scene composes one `GraspGenSpec` provider, chosen by blueprint:

| Blueprint | Provider | Output |
|---|---|---|
| `xarm-grasp-sim` | `HeuristicGraspModule` | one top-down grasp, score 1.0 |
| `xarm-grasp-sim-graspgenx` | `GraspGenXModule` | up to 100 ranked learned grasps |

`xarm-grasp-sim-graspgenx` needs the `graspgenx` extra and a CUDA GPU. The
checkpoints download once from Hugging Face on first use and are cached under
`~/.cache/huggingface`; no other setup is required.

```bash
uv sync --extra graspgenx
MUJOCO_GL=egl dimos run xarm-grasp-sim-graspgenx
```

## Voxel map obstacles

The wrist camera feeds a live voxel map that the planner treats as one octree
obstacle, so trajectories avoid whatever has actually been seen rather than only
the registered objects:

```
MujocoSimModule.pointcloud
  -> PointCloudSelfFilter        drops the arm's own returns, emits a clear mask
  -> RayTracingVoxelMap          accumulates occupied cells in the world frame
  -> ManipulationModule.voxel_map   rebuilt as the "mapping/voxel-map" obstacle
```

`XARM_GRASP_VOXEL_SIZE` is the single resolution all three stages share; they
must agree or the clear mask names cells the map does not hold and the octree
does not line up with what was mapped. The blueprint also enables the sim's
`pointcloud` output, which is off by default, and publishes TF for every one of
the arm's collision links — the self filter drops a whole cloud if any link
transform is missing at capture time.

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
has a grasp axis below 7 cm. The blueprint starts the arm at an elevated,
collision-free top-down scan pose so all six visual meshes fit in the
wrist-camera frame.

OWL-ViT labels these synthetic renders unreliably — a scan routinely returns the
right six positions under swapped names. Match objects by position, not label.

## Driving it

In a second terminal, connect to the running blueprint:

```bash
dimos shell
```

Then run this complete scan and obstacle-inspection sequence:

```python skip
from dimos.robot.manipulators.xarm.blueprints.simulation import XARM_GRASP_PROMPTS

app.ManipulationSkills.go_init()
scan = app.PickAndPlaceModule.scan_objects(XARM_GRASP_PROMPTS)
print(scan)

print(app.ObjectSceneRegistrationModule.get_detected_objects())
print(app.ManipulationModule.refresh_obstacles())
print(app.ManipulationModule.get_obstacles())
```

Wait for `scan_objects` to finish before issuing another scan. The prompt set
includes a `green ring` fallback because the tape loses its category silhouette
in the wrist camera's top-down view.

A failed grasp knocks free-body targets out of place, and `MujocoSimModule.reset()`
does not respawn them. Restart the blueprint between pick attempts that need a
pristine scene.
