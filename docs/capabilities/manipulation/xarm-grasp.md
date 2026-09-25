# xArm Grasping

Two blueprints, differing only in which grasp provider they compose. Both carry
the control coordinator, the wrist camera, Dan's live localization memory and
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

GraspGenX requires Linux x86_64, a CUDA 12.8-compatible GPU, and `uv >=0.9.25`.
The first launch prepares its isolated Python 3.12 environment and downloads the
checkpoints. Runtime sources come from the development checkout or the shared
repository clone used by installed dimOS.

To show the MuJoCo window with Rerun disabled:

```bash
MUJOCO_GL=glfw dimos --viewer none run xarm-grasp-graspgenx \
  --simulation mujoco --headless false
```

In another terminal, use `dimos shell` and follow [Driving it](#driving-it) to scan
objects and request grasps. See the [isolated-runtime development guide](/dimos/experimental/isolated_python/README.md#runtime-development)
for test and type-check commands.

What differs between the arm and the sim is decided at import time: the hardware
adapter, the base pose, the camera (RealSense plus its mount edge, versus the
MuJoCo wrist camera), the localization thresholds, and the home pose.

## Wrist-camera memory

`LiveLocalizeModule` uses SigLIP frame retrieval, OWLv2 detection, EdgeTAM
segmentation, and `Rig` to lift registered depth into the world frame. It keeps
bounded RGB-D frame memory using `dimos.memory`; verified object groups accumulate
across queries. EdgeTAM requires CUDA or MPS. Only the eye-in-hand camera feeds
localization: the environment camera is not fused into this memory.

The default verification policy needs two camera positions. Move the arm to
another reachable viewpoint while keeping the target visible, and let it settle
at each pose. Capture-time TF accounts for the changing wrist pose. Waiting at
one position, or rotating without translating the camera, is not a second view.
An explicit `policy='{"min_views": 1}'` query permits a single-view result.

| API | Result |
|---|---|
| `LiveLocalizeModule.state()` | Readiness or initialization stage (skill/RPC). |
| `LiveLocalizeModule.localize(objects, start, duration, policy, max_age)` | Text summary of remembered instances (skill/RPC). |
| `LiveLocalizeModule.localize_objects(prompts, start, duration, policy, max_age)` | Lists of typed `Localization` results, one list per label (RPC). |
| `PickAndPlaceModule.scan_objects(prompts, start, duration, policy, max_age)` | Selectable snapshot with positions, scores, view counts, ambiguity, and last-seen timestamps (skill/RPC). |

The default window is the last ten seconds. Negative `start` is relative to the
newest embedded frame; non-negative `start` is relative to the oldest retained
frame. The window selects evidence to examine, not the lifetime of known objects.
Previously verified objects remain answerable. Optional `max_age` filters their
last-seen age against the latest RGB timestamp, using the sensor/replay clock.
`policy` is a JSON object overriding `LocalizePolicy` for that call.

Memory is in RAM and lasts for the module's lifetime. A cloud fuses past sightings;
it is not proof an object is still at that location. Objects moved by manipulation
may leave old groups behind. Last-seen filtering does not rebuild fused geometry.

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
MuJoCo camera, and publishes TF for every one of the arm's collision links. The
self filter drops a whole cloud if any link transform is missing at capture time.

Because the target object is itself mapped geometry, a collision-checked plan
into it can only ever be rejected. The pregrasp-to-grasp leg and the retreat are
therefore straight-line `move_linear` servos with collision checking off; only
the approach to the pregrasp pose is a checked plan.

## Seeing the proposals

The viser scene draws the ranked proposals as pose glyphs: an approach axis with
the closing axis across it, coloured best-green through worst-orange so the
ordering reads at a glance, with the top three drawn thicker and labelled with
their score. Only the leading twenty are drawn, because a hundred glyphs bury
the ranking they exist to show. `manipulation.grasp-proposals` in the Scene panel
toggles them.

The markers are pose indicators, not a gripper: what they promise is where a
grasp points and in what order the generator ranked it. To see what the arm will
actually do with one, watch the plan preview.

## The scene

The scene is an enclosed 2.6 m by 3.0 m room. The xArm is bolted to the world
origin. Unlike `data/xarm7`, this scene has no 12 cm pedestal, so the planning
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

OWL-ViT labels these synthetic renders unreliably. A scan routinely returns the
right six positions under swapped names, so match objects by position, not label.

## Driving it

In a second terminal, connect to the running blueprint:

```bash
dimos shell
```

Check readiness, gather viewpoints, then query the memory:

```python skip
from dimos.robot.manipulators.xarm.blueprints.grasp import XARM_GRASP_PROMPTS

app.ManipulationSkills.go_init()
print(app.LiveLocalizeModule.state())
# Use ManipulationSkills.move_to_pose / move_to_joints to gather another
# reachable wrist-camera position with the target visible before scanning.
scan = app.PickAndPlaceModule.scan_objects(XARM_GRASP_PROMPTS)
print(scan)

print(app.ManipulationModule.get_obstacles())
```

To pick, pass a `selection` from that scan. Each matching instance gets a separate
selection, including multiple objects with the same label. Choose by position
and evidence as well as name:

```python skip
for obj in scan.metadata["objects"]:
    print(obj["selection"], obj["name"], obj["position"], obj["last_seen_timestamp"])

# Select the intended target from the printed results.
selection = scan.metadata["objects"][0]["selection"]
pick = app.PickAndPlaceModule.pick_object(selection)
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
hits = app.LiveLocalizeModule.localize_objects(["gray can"])
cloud = hits[0][0].point_cloud   # inspect results and choose the intended instance first
candidates = app.GraspGenXModule.propose_grasps(cloud)   # HeuristicGraspModule in the base blueprint
print(len(candidates.candidates), [c.score for c in candidates.candidates[:5]])
```

The prompt set includes a `green ring` fallback because the tape loses
its category silhouette in the wrist camera's top-down view.

Picking uses the exact cloud cached by `scan_objects`; it does not perform a
second perception lookup. Every new scan invalidates the previous selections.
Selections are not persistent object IDs and are not reused during a
`PickAndPlaceModule` session.

A failed grasp knocks free-body targets out of place, and `MujocoSimModule.reset()`
does not respawn them. Restart the blueprint between pick attempts that need a
pristine scene.
