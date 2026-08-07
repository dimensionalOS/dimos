# Pick And Place

This directory contains the configurable xArm6 `picknplace` operator pipeline.
It uses the wrist-mounted RealSense and object-scene registration in `link_base`.

## Setup

GraspGenX runs in the main worktree `.venv` so it shares the live DimOS
pipeline. Its CUDA requirements differ from the repository lockfile; install
them once from the worktree root:

```bash
bash bin/setup-graspgenx-env
```

The setup installs Torch 2.7.1 CUDA 12.8, which supports the RTX 5070's
`sm_120` architecture, along with GraspGenX and its inference dependencies.
Use `uv run --no-sync` afterwards. Plain `uv run` reconciles the environment to
the lockfile's Torch 2.6 and removes the GPU architecture support required by
GraspGenX.

The first GraspGenX startup downloads the pinned model checkpoint to the
Hugging Face cache and loads it onto the GPU. Later starts reuse that cache.
The setup also installs `edgetam-dimos`, which provides the `sam2` runtime used
by the EdgeTAM blueprint.

## Run

Start the default YOLO-E and OBB-center-grasp pipeline:

```bash
uv run --no-sync dimos run picknplace --daemon
```

Use text-prompted Moondream detection, EdgeTAM segmentation, and an OBB-center grasp:

```bash
uv run --no-sync dimos run picknplace --daemon \
  -o osr.det=moondream -o osr.seg=edgetam -o pnp.grasp=obb_center
```

Use the same perception stack with GraspGenX:

```bash
uv run --no-sync dimos run picknplace --daemon \
  -o osr.det=moondream -o osr.seg=edgetam -o pnp.grasp=graspgenx
```

`osr.det` accepts `yoloe` or `moondream`; `osr.seg` accepts `yolo` or `edgetam`.
Moondream requires EdgeTAM because it produces detection boxes rather than masks.
`pnp.grasp` accepts `obb_center` or `graspgenx`. GraspGenX loads only when selected.

Then connect the console:

```bash
uv run --no-sync python -m dimos.manipulation.pnpconsole
```

Stop a running pipeline with:

```bash
uv run --no-sync dimos stop
```

## Operator Flow

The console intentionally keeps planning and execution separate:

1. Select `1` to scan the current scene.
2. Select `2` to inspect object number, name, and confidence.
3. Select `3` and choose an object. The GraspGenX blueprint prints its top
   proposals and displays the selected grasp. Viser shows the selected object
   cloud in amber, the grasp TCP axes in red, and the pre-grasp TCP axes in green.
   The top ten proposals are filtered through collision-aware xArm IK; after
   table calibration, candidates intersecting the table are omitted.
4. Select `4` to plan and preview the approach. Each Viser preview plays once
   at a slow two-second duration.
5. Execute the approach only after inspecting the proposal and preview.
6. Select `6` to plan and preview descent, then select `7` to execute it.
7. Close the gripper with `8`, then select `9` to plan and preview ascent.
8. Select `10` to execute the ascent, `11` to open, and `13` to return home.
9. After a scene scan, select `14` to estimate and preview the tabletop. Once
   the blue Viser outline matches the table, enter a collision clearance in
   millimeters. The recommended clearance is 10 mm; enter `0` for no extra clearance.
   The manual action installs the collision slab at the measured tabletop position for
   all subsequent IK and trajectory plans. The pick-and-place blueprints do not install
   a fixed floor slab.
10. After executing the approach, select `15` to collision-plan and execute the
    descent, close the gripper, and execute the ascent without previews. It stops at
    the first failed stage.

Do not execute a learned grasp without checking its pose, the 100 mm pre-grasp
pose, the point-cloud/overlay visualization, and the collision-free preview.

## Grasp Geometry

`PickNPlaceModule.get_goal_pose()` stores the top ranked GraspGenX candidate as
the TCP goal in the candidate point cloud's frame. Its pre-grasp is computed as:

```text
pre_grasp_position = grasp_position - grasp_orientation * (0, 0, 0.100 m)
```

GraspGenX local `+Z` is the final approach direction, so the pre-grasp retreats
along local `-Z`. It is not a world-Z lift: an angled or side grasp receives an
equally angled or sideward pre-grasp. Descent and ascent use Cartesian paths
between the current TCP pose and the selected grasp or pre-grasp target.

The `picknplace-graspgenx` blueprint uses the xArm 85 mm gripper sweep-volume
and calibrated base-to-TCP transform. The TCP is rolled 90 degrees around the
GraspGenX approach axis so its closing jaws are perpendicular to a bottle's
length. Candidate score order comes from GraspGenX; no additional ranking is
applied by the operator pipeline.

## Implementation Guide

- `blueprints.py`: robot, camera, OBB, and GraspGenX blueprint composition.
- `picknplace.py`: scan request, target selection, OBB fallback, learned grasp
  selection, and tool-axis pre-grasp calculation.
- `pnpconsole.py`: explicit operator stages and manual gripper/home controls.
- `grasping/grasp_gen_x.py`: import-safe proposal adapter and candidate contract.
- `grasping/grasp_gen_x_runtime.py`: in-process checkpoint load and GPU inference.
- `visualization/pose_overlay.py` and `visualization/rerun.py`: selected-object
  cloud, image, and grasp overlays.

The current scan is a single wrist-camera view. Automatic multi-view scanning,
EdgeTAM segmentation, and fused object clouds are planned follow-up work. Until
then, select targets with a complete enough visible point cloud for grasping.
