# Handoff prompts

Paste one block per session. Each block is self-contained. Every task ends
with a report back into the planning chat using the format at the bottom.

## Common preamble (prepend to every prompt)

```
Repo: /home/mustafa/dimos (DimOS robotics framework). Load the
dimos-architecture skill first.

Branch: base everything on `mustafa/task/hackmit-manip` (CC's learning stack
+ main merged). Work in a git worktree on your own branch
`hackmit/<task-id>-<slug>` so parallel tasks do not collide:
  git worktree add /home/mustafa/dimos-wt/<task-id> -b hackmit/<task-id>-<slug> mustafa/task/hackmit-manip
  cd /home/mustafa/dimos-wt/<task-id> && uv sync --extra all
  ln -s /home/mustafa/dimos/data/xarm_grasp_sim data/xarm_grasp_sim   # extracted assets, LFS pull is blocked
Use `.venv/bin/python` and `.venv/bin/python -m pytest <file>`; never `uv run`.
Small commits, plain messages, no Claude co-author trailer (the repo hook
strips it and CI fails). Do not push, do not open a PR, do not post GitHub
comments. Nothing here merges to main.

Read first: openspec/changes/hackmit-dual-openyam-sim-demo/plan.md in the
main checkout (/home/mustafa/dimos), especially "What already exists" and
your task section.

House rules (maintainers reject violations):
- Minimal docstrings and comments; no `# --- section ---` comments (CI).
- Tests live in-package as `test_*.py` beside the code; no top-level tests/.
  No `# type: ignore` in tests. No test-per-method filler, no log-line
  assertions. One meaningful test per behaviour.
- No `__init__.py` / `__main__.py` (PEP 420).
- Typed `In[T]` / `Out[T]` streams; SI units (rad, m, s).
- If you override `start` or `stop` on a Module, keep the `@rpc` decorator
  or the worker dies with "cannot pickle _thread.lock".
- `In/Out.transport` is never None after start; do not gate on it.
- Blueprint registry is a syntactic AST scan: blueprints must be
  module-scope assignments. `dimos/robot/all_blueprints.py` is generated;
  regenerate with `.venv/bin/python -m pytest dimos/robot/test_all_blueprints_generation.py`.
- Mock adapters run silently; when hardware "does nothing", check the
  adapter type before debugging wiring.

Git LFS is blocked in this environment (no credentials for
https://lfs.dimensionalos.com). Do not depend on pulling new assets. Use
what is extracted under /home/mustafa/dimos/data or synthetic MJCF in tests.
```

## T1 Generic MuJoCo whole-body adapter

```
Task T1: give the dual OpenYAM a physics-backed MuJoCo sim adapter.

Today `dual_openyam_hardware()` in
dimos/robot/manipulators/dual_openyam/config.py selects `mock_whole_body`
whenever no CAN ports are given, so in sim nothing has physics or contact.

Pattern to copy: dimos/simulation/adapters/whole_body/g1.py
(SimMujocoG1WholeBodyAdapter) talks to MujocoSimModule over shared memory
(dimos/simulation/engines/mujoco_shm.py, ManipShmReader/Writer, PD mode
CMD_MODE_PD_TAU, up to 32 joints). It hardcodes 29 motors and IMU.
The sim side is `_WholeBodySimHooks` in
dimos/simulation/engines/mujoco_sim_module.py; joint mapping comes from
`RobotSimSpec` in dimos/simulation/engines/robot_sim_binding.py (see how
dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py builds one
under `if global_config.simulation == "mujoco":`).

Do:
1. New dimos/simulation/adapters/whole_body/generic.py:
   `SimMujocoWholeBodyAdapter(address, num_motors, require_imu=False)`,
   registered as `sim_mujoco_whole_body` in
   dimos/simulation/adapters/whole_body/_registry.py. Same lifecycle and
   SHM protocol as the G1 adapter, N motors from config, IMU optional
   (return a default IMUState when absent). Grippers are ordinary position
   joints here (14 arm + 2 gripper = 16 motors); do not use the single
   gripper SHM slot.
2. New dimos/robot/manipulators/dual_openyam/sim.py: the dual yam
   `RobotSimSpec` (hardware joint names -> MJCF joint names, no floating
   base, no IMU) and a `dual_openyam_sim_module(scene_path)` blueprint
   helper for MujocoSimModule (headless, dof=16, robot_sim_spec).
3. `dual_openyam_hardware()` gains a sim path: when
   `global_config.simulation == "mujoco"` and an MJCF path is available,
   return a HardwareComponent with adapter_type="sim_mujoco_whole_body",
   address=<MJCF>, wb_config kp/kd as today. Keep mock otherwise.
4. Blueprint `dual_openyam_sim` (sim module + DualOpenYamCoordinator with
   trajectory task + left/right gripper tasks + planner) in
   dimos/robot/manipulators/dual_openyam/blueprints/simulation.py.

Asset discovery is step 0, report what you find:
  a. `ls /home/mustafa/dimos/data | grep -i yam`; if the LFS pull has been
     done since, yam_description/ may contain an MJCF next to the URDF.
  b. The dual model package is derived from Amazon ABC
     (github.com/amazon-far/abc, revision
     6bc6586721cf0c409ccee80f675a28de9b9b2f5e). Check whether it ships a
     MuJoCo dual-YAM MJCF and its license (data/.lfs package has
     ABC_LICENSE and I2RT_YAM_LICENSE). If usable, put it under
     data/dual_openyam_sim/ (untracked; note it in the report).
  c. Fallback: I2RT's public yam MJCF composed twice (left/right offsets
     matching dimos/robot/manipulators/dual_openyam/model.py URDF), plus a
     table.
  Joint names in the MJCF must match `DUAL_OPENYAM_JOINTS` after
  `mjcf_joint_names_from_hardware`, or the spec maps them explicitly.

Tests: in-package tests with a tiny synthetic MJCF (two 2-dof chains +
one slider each) proving the adapter reads N states and PD commands reach
the sim. Do not depend on the real asset in tests.

Verify: `dimos --simulation mujoco run dual-openyam-sim` shows both arms;
`plan_to_joints` on ManipulationModule moves them and the grippers
open/close. Report the exact command and what you saw.
```

## T2 Multi-camera MujocoSimModule + body poses

```
Task T2: make MujocoSimModule publish several cameras and expose body poses.

dimos/simulation/engines/mujoco_sim_module.py renders only
`config.camera_name` in `_publish_loop` and has a single `color_image` /
`depth_image` output. The engine (dimos/simulation/engines/mujoco_engine.py)
already renders any number of `CameraConfig`s and serves them through
`read_camera(name)`.

Do:
1. `MujocoSimModuleConfig.extra_cameras: list[SimCameraSpec]` where
   SimCameraSpec = (name, stream, width, height, fps, rgb only). Register
   each in `start()` via the existing `add_camera` helper and publish each
   as its own `Out[Image]` named by `stream`. Declare the outputs so
   Blueprint autoconnect still sees typed ports (see how
   `declare_policy_module` in dimos/imitation/policy/module.py builds a
   class with `In[Image]` annotations, or add a fixed set of named outputs
   if dynamic declaration fights the Module metaclass). The primary camera
   keeps its current behaviour.
2. One publish thread per extra camera, or one loop reading all frames;
   rendering blocks the sim thread, so measure sim step rate before and
   after with three 320x240 cameras at 15 Hz and report the numbers.
3. Port `get_body_poses(names) -> dict[str, list[float]]` (RPC on
   MujocoSimModule) and `MujocoEngine.get_body_pose(name)` from
   `origin/manip/mobile-manipulation-demo-aug18` (see
   `git show origin/manip/mobile-manipulation-demo-aug18:dimos/simulation/engines/mujoco_sim_module.py`).
   Add `get_body_geoms(name)` returning, per geom: type, size, world pose,
   and for mesh geoms the mesh name. T3 builds on these.

Target stream names for the demo: `top_image`, `left_wrist_image`,
`right_wrist_image` (CC's profile names in
dimos/robot/manipulators/dual_openyam/learning.py).

Tests: in-package test with a synthetic MJCF containing two cameras
proving both streams publish and body poses come back in world frame.
Use data/xarm_grasp_sim/scene.xml for a manual check if present.

Verify: three image streams visible in Rerun at the configured rate; sim
step rate within 10 % of single-camera. Report numbers.
```

## T3 Privileged scene registration

```
Task T3: ground-truth perception for pick/place in MuJoCo.

`PickAndPlaceModule` (dimos/manipulation/pick_and_place_module.py) only
needs `ObjectSceneRegistrationSpec`
(dimos/perception/experimental/object_scene_registration_spec.py):
scan_scene(text) -> Detection3DArray, get_object_pointcloud_by_object_id,
get_object_pointcloud_by_name, get_full_scene_pointcloud(exclude, ...),
set_prompts. Grasps come from `GraspGenSpec.propose_grasps(PointCloud2)`
(HeuristicGraspModule in dimos/manipulation/grasping/heuristic_grasp.py).
Look at ObjectSceneRegistrationModule
(dimos/perception/experimental/object_scene_registration.py) for how it
fills Detection3D (id, results[0].hypothesis.class_id, bbox) and which
streams (obstacles etc.) ManipulationModule consumes from it.

Do:
1. New dimos/simulation/perception/mujoco_surface.py: pure functions on
   an MjModel/MjData that sample surface points of a body's geoms (box,
   cylinder, sphere, capsule, mesh via model.mesh_vert/mesh_face) in world
   frame, and of the whole scene minus a set of bodies, voxelised.
2. A small RPC on MujocoSimModule wrapping those
   (`sample_body_surface(name, n)`, `sample_scene_surface(exclude, voxel)`).
   Keep it thin; T2 is editing the same file, so put logic in the new
   module and add the RPC at the end.
3. New dimos/simulation/perception/sim_scene_registration.py:
   `SimSceneRegistrationModule` implementing the spec. `scan_scene(prompts)`
   matches MuJoCo body names to prompts (exact, then substring, then a
   `aliases: dict[str, str]` config map like {"red cube": "cube_red"}),
   returns one Detection3D per match with id = body name, class_id = the
   prompt, bbox from the geoms, pose in `frame_id="world"`. Object
   pointclouds and the full-scene cloud come from the RPCs. Publish the
   same obstacle stream the real module publishes if ManipulationModule
   depends on it.
4. Blueprint helper so the xArm sim stack can use it in place of
   ObjectSceneRegistrationModule for a quick check.

Tests: in-package test with a synthetic 3-body MJCF: scan returns the
matching bodies, object cloud is on the box surface, full-scene cloud
excludes the requested body.

Verify: xarm-perception-sim with this module in place of the detector,
`scan_objects(["cube"])` then `pick_object(id)` returns grasp candidates
in `world` and the arm executes the pick. Report what worked.
```

## T4 Per-group gripper in ManipulationModule

```
Task T4: let a two-arm model have one gripper per planning group.

Today `RobotModelConfig.gripper_hardware_id` (single, model-wide,
dimos/manipulation/planning/spec/config.py) drives everything in
dimos/manipulation/manipulation_module.py: `list_planning_groups` sets
has_gripper for all groups from it, `_get_group_gripper_position` and
`set_gripper_position` call the coordinator task `f"{id}_gripper"` via
task_invoke("get_normalized" / "set_normalized"), and
`_resolve_group_with_capability` ignores the group for the "gripper"
capability. `dual_openyam_model_config()` sets no gripper id, so
`PickAndPlaceModule._resolve_group` finds zero gripper-capable groups and
pick_object refuses.

Do:
1. `PlanningGroupDefinition.gripper_hardware_id: str | None = None`
   (dimos/manipulation/planning/groups/models.py). Resolution order:
   group value, then the model-level value. xArm/OpenYAM configs stay
   untouched.
2. ManipulationModule resolves the id per group at the four call sites
   above; `PlanningGroupInfo.has_gripper` becomes per group.
3. `dual_openyam_model_config()` sets `left_arm` / `right_arm`
   (gripper task names become `left_arm_gripper` / `right_arm_gripper`,
   matching dimos/robot/manipulators/dual_openyam/blueprints/teleop.py).
   Add both gripper TaskConfigs to `dual_openyam_planner_coordinator` in
   dimos/robot/manipulators/dual_openyam/blueprints/basic.py.
4. ManipulationSkills.set_gripper/open/close already take
   `planning_group`; confirm they pass it through.

Tests: one in-package test that a two-group model resolves different
gripper ids per group and rejects a gripper call on a group without one;
one that `PickAndPlaceModule._resolve_group` returns both groups.

Verify: `dimos --simulation run dual-openyam-planner-coordinator` (mock
adapter, no physics needed) then via the Python client
`set_gripper_position(0.0, planning_group="left_manipulator")` moves only
`left_arm/gripper` in coordinator_joint_state. Report the joint state
before/after.
```

## Report-back format

```
T<id> report
Branch: hackmit/<id>-<slug> @ <sha>, N commits
Changed: <files, one line each>
Verified: <exact commands> -> <what happened, numbers if any>
Not done / open: <bullets>
Surprises for the hub: <anything that changes plan.md>
```

## T1-T4 status (2026-09-06)

All four are implemented and verified. Nothing is pushed and nothing is merged.

### Branches

Two independent stacks, both based on `mustafa/task/hackmit-manip`:

| Branch | Base | Commits | Task |
|---|---|---|---|
| `hackmit/t1-generic-mujoco-wb` @ 33c68aefe | hackmit-manip | 5 | T1 |
| `hackmit/t4-per-group-gripper` @ b7e2f040e | **T1** | +1 | T4 |
| `hackmit/t2-multi-camera-sim` @ 2848b028c | hackmit-manip | 1 | T2 |
| `hackmit/t3-privileged-perception` @ 0bb7d5c81 | **T2** | +1 | T3 |

T4 stacks on T1 because both edit `dual_openyam/config.py`. T3 stacks on T2
because it consumes `get_body_geoms`. The two stacks touch disjoint files, so
they can be reviewed and merged in either order.

### What each task delivered

**T1 - generic MuJoCo whole-body adapter.** `SimMujocoWholeBodyAdapter`
(`sim_mujoco_whole_body`): N motors from config, optional IMU, and a
selectable command path - `position` for MJCFs whose actuators close their own
loop, `pd_tau` for the G1's direct-torque path. `dual_openyam_hardware()` picks
it under `--simulation mujoco`. New blueprint `dual-openyam-sim`.

Verified: `plan_to_joints` -> COMPLETED, both arms tracked +-0.25 rad to
1.1 mrad, grippers cycled 0.004 -> 0.999 -> 0.004 normalised, sim at 96% real
time.

**T2 - multi-camera sim module + body poses.** `MujocoSimModuleConfig.extra_cameras`
with `SimCameraSpec`; `declare_sim_camera_module()` builds a subclass carrying
one declared `Out[Image]` per stream so blueprint autoconnect still sees typed
ports. Extra cameras render RGB only. Ports `get_body_pose`/`get_body_poses`
from the water-demo branch and adds `get_body_geoms`.

Verified on the dual-YAM scene: `top_image`, `left_wrist_image`,
`right_wrist_image` all publishing at 14.75-14.83 Hz against 15 Hz configured.
Sim step rate 439.0 steps/s with three cameras vs 452.5 with one, **-3.0%**,
inside the 10% budget. All three at rgb+depth costs -5.9%, so skipping the
depth pass is worth about half the added cost.

**T3 - privileged scene registration.** `mujoco_surface.py` samples geom
surfaces (box, sphere, capsule, cylinder, ellipsoid, mesh) in world frame and
the scene minus a set of bodies, voxelised. `SimSceneRegistrationModule`
implements `ObjectSceneRegistrationSpec` off those. Thin RPCs on
`MujocoSimModule` behind a new `SimSceneGeometrySpec`. New blueprint
`xarm-privileged-sim`.

Verified: `scan_objects(["cup","bottle","can"])` -> 3 objects with body-name
ids; cup detection centre (0.560, -0.219, 0.160), size 6.8x7.0x6.1 cm, frame
`world`; object cloud 512 pts, scene cloud 19816 pts, both in `world` with the
robot excluded; `pick_object("cup")` planned and executed.

**T4 - per-group gripper.** `PlanningGroupDefinition.gripper_hardware_id`, with
the model-level value as fallback so xArm and OpenYAM are untouched.
`ManipulationModule` resolves it per group at all four call sites.
`dual_openyam_model_config()` sets `left_arm`/`right_arm`; the planner
coordinator gains `left_arm_gripper` and `right_arm_gripper`.

Verified on `dual-openyam-planner-coordinator`: both groups now report
`has_gripper=True` (was False); `set_gripper_position(1.0, "left_manipulator")`
moved only `left_arm/gripper`, `set_gripper_position(0.0, "right_manipulator")`
moved only `right_arm/gripper`; an unqualified call is REJECTED as ambiguous.

### What we found that changes the plan

1. **The dual-YAM MJCF already exists upstream.** `amazon-far/abc` @ 6bc6586
   ships `assets/put_bottles/put_bottle.xml`: both arms with joint names that
   already match `DUAL_OPENYAM_ARM_JOINTS`, grippers as actuated slide joints,
   cameras named `top`/`left`/`right`, plus a table, a bin and six bottles.
   Apache-2.0 + MIT. Vendored to `data/dual_openyam_sim/` (untracked).
   **T0's "compose the I2RT yam MJCF twice" fallback is unnecessary - delete it.**

2. **The plan says "14 arm joints"; it is 12 arm + 2 grippers = 14 total.**
   `dof=14`, not 16. T1's prompt has the same error.

3. **Git LFS is still dead, so `dual_openyam_abc_box_v2` (the planning URDF)
   cannot be fetched.** `DUAL_OPENYAM_MODEL` raises on load, so
   `dual-openyam-sim` and `dual-openyam-planner-coordinator` cannot run here.
   A URDF rebuilt from the MJCF exists at `data/dual_openyam_sim/dual_openyam.urdf`
   (FK matches MuJoCo to 0.0 mm / 0.0 deg, same 21-link/20-joint shape as the
   real package; only `{side}_grasp_frame` is invented). **Decision 2026-09-06:
   wait for LFS, leave it unwired.** It was used only as a throwaway local
   harness to run T1's and T4's verifies, then reverted.

4. **`dual_openyam_hardware()` declared no arm position limits**, so
   `JointTrajectoryTask` rejected every trajectory - on mock and CAN too, not
   just sim. Fixed in T1. `openyam_hardware()` (single arm) still has the gap.

5. **MuJoCo starts at qpos zero, which is exactly joint2/joint3's lower limit.**
   Every plan failed with "Invalid start configuration". T1 sets
   `reset_joint_positions` to the home posture. Do not read this as
   self-collision.

6. **The scene's `home` keyframe scrambles the scene on reset.** The key has 16
   qpos values but the full scene has nq=65, so MuJoCo zero-pads it: the arm
   posture lands in `bin_joint`'s freejoint and every bottle drops to the
   origin. `MujocoEngine._reset_unlocked` calls `mj_resetDataKeyframe` whenever
   `nkey > 0`. **T5's scene-reset RPC walks straight into this.** Fix is to
   delete the `<keyframe>` block (T1 already sets the arm posture separately).

7. **Coordinators are built before the sim module starts.** `adapter.connect()`
   runs inside `ControlCoordinator._setup_from_config`, so a sim adapter can
   latch a dead predecessor's SHM that `MujocoSimModule` then replaces. T1's
   adapter re-attaches while waiting; `SimMujocoG1WholeBodyAdapter` still has
   this hole.

8. **`--simulation` bare means `mujoco`.** After T1 that selects the physics
   adapter, so `dimos --simulation run dual-openyam-planner-coordinator` (which
   has no sim module) now stalls waiting for SHM. Run it with no flag for the
   mock. T4's prompt needs this correction.

9. **Every MuJoCo blueprint needs `MUJOCO_GL=egl` here**, plus `--viewer none`
   and `headless: True`. Without it the sim thread dies inside `_init_cameras`
   in the background and the only visible symptom is ManipulationModule
   spamming "Current model state is stale".

10. **`get_body_geoms` had to treat welded child bodies as part of the named
    body.** Scene assets wrap geometry in a child body under the freejoint, so
    asking for `bottle_1` otherwise returns nothing. Children carrying a joint
    are excluded. T3's sampler follows the same rule.

11. **CC's `DUAL_OPENYAM_CAMERA_SHAPE` is (480, 640, 3).** At 15 Hz the
    resolution is nearly free (448.5 vs 450.8 steps/s), so match the policy
    profile rather than downscaling wrist cams.

### Open, and the biggest risk

**The grasp does not hold.** T3's `pick_object("cup")` returned "Pick complete",
but measuring the sim afterwards showed the cup still on the table, nudged ~4 cm
sideways, 5.5 cm from the finger midpoint. Perception, grasp generation,
planning and execution all ran correctly in `world`; the physical grasp slipped
and `_close_and_verify` accepted it anyway - consistent with the known
overlapping grasp-verification bands. **T5 cannot demo without this fixed, and
it is not currently anyone's task.**

Also open:

- At the home posture the wrist cameras sit close enough to the table that the
  view is mostly flat white. Wants a different rest posture before T7 collects
  demonstrations.
- T5-T8 exist in plan.md but have no handoff prompts written.

## T5 Classical demo blueprint + agent (written 2026-09-06, after T1-T4 merged)

Prepend the common preamble. Base on `mustafa/task/hackmit-manip` @ febdc3c1b
(T1-T4 merged). Worktree branch `hackmit/t5-classical-demo`.

```
Task T5: the stage demo, classical mode: an agent picks bottles with either
arm and drops them in the bin, in the ABC dual-YAM MuJoCo scene.

Read first: plan.md sections "Status 2026-09-06" and "Architecture", and
handoffs.md "T1-T4 status" (numbered findings 1-11). Everything below builds
on merged T1-T4 code.

What exists now:
- `dual-openyam-sim` (dimos/robot/manipulators/dual_openyam/blueprints/simulation.py):
  MujocoSimModule on data/dual_openyam_sim/put_bottle.xml + planner +
  DualOpenYamCoordinator with trajectory + left/right gripper tasks.
  Sim binding in dimos/robot/manipulators/dual_openyam/sim.py (dof=14,
  cameras top/left/right, home posture via reset_joint_positions).
- Per-group grippers: `left_manipulator` / `right_manipulator` both report
  has_gripper; tasks `left_arm_gripper` / `right_arm_gripper`.
- Ground-truth perception: `sim_scene_registration(...)` in
  dimos/simulation/perception/blueprints.py (aliases map prompt -> body
  name; robot_body_substrings exclude the arms from the obstacle cloud).
  Reference wiring: `xarm_privileged_sim` in
  dimos/robot/manipulators/xarm/blueprints/simulation.py.
- Multi-camera: `MujocoSimModuleConfig.extra_cameras` with `SimCameraSpec`
  and `declare_sim_camera_module()` (dimos/simulation/engines/mujoco_sim_module.py).
- Agent wiring reference: dimos/robot/manipulators/xarm/blueprints/agentic.py
  (McpServer + McpClient with a system prompt).

Step 0, planning model. `DUAL_OPENYAM_MODEL` in
dimos/robot/manipulators/dual_openyam/model.py loads the LFS package
`dual_openyam_abc_box_v2`. Check `ls data/dual_openyam_abc_box_v2`; if the
LFS pull has landed, use it. If not, wire the rebuilt
data/dual_openyam_sim/dual_openyam.urdf as a fallback gated on the LFS
package being absent (dev-only; say so in the report). Do not block on LFS.

Step 1, scene hygiene. Delete the `<keyframe>` block from
data/dual_openyam_sim/put_bottle.xml (finding 6) and confirm
`MujocoSimModule.reset()` puts arms at home and bottles back on the table.
Note the asset is untracked; describe the edit in the report.

Step 2, prove one grasp holds before building anything else. On
`dual-openyam-sim` plus perception, run scan -> pick_object on one bottle
with `planning_group="left_manipulator"`, then read
`get_body_poses(["bottle_1"])` (or whichever id) after the retreat: z must
rise by more than 5 cm and stay there for 2 s. If it does not, find out why
before continuing. Suspects in order: gripper command direction (MJCF finger
range is 0 to 0.0475 m open; the real arm opens by decreasing motor
position; check what normalized 0/1 does in sim through T1's adapter), grasp
width vs bottle diameter, approach axis of HeuristicGraspModule, and
`GraspVerificationConfig` accepting a slipped grasp (readback bands overlap
at 0.85-0.95; verify by object displacement, not readback). Report the
numbers.

Step 3, blueprints in dimos/robot/manipulators/dual_openyam/blueprints/:
- `dual_openyam_sim_pick_place` = dual_openyam_sim + ManipulationSkills +
  PickAndPlaceModule(planning_frame="world") + HeuristicGraspModule +
  sim_scene_registration(aliases for the six bottles and the bin,
  robot_body_substrings for both arms) + extra cameras `top`, `left`,
  `right` published as `top_image`, `left_wrist_image`,
  `right_wrist_image` at 480x640, 15 Hz + RerunBridgeModule.
- `dual_openyam_sim_agent` = the above + McpServer + McpClient with a new
  `DUAL_ARM_MANIPULATION_AGENT_SYSTEM_PROMPT` in
  dimos/robot/manipulators/common/agent_prompts.py: world frame, arm choice
  rule (object y > 0 -> left_manipulator, else right_manipulator; always
  pass planning_group), the scan -> pick -> place_at -> go_home sequence,
  bin coordinates, and reset on FAULT.
- Regenerate all_blueprints.py.

Step 4, a scene-reset path for repeat runs: an RPC on MujocoSimModule is
enough (reset + re-apply reset_joint_positions); document the call.

Tests: one in-package test that the pick_place blueprint resolves (module
set, no missing connections) and that the prompt lists both planning
groups. No physics in unit tests.

Verify end to end: `MUJOCO_GL=egl dimos --simulation mujoco --viewer none run dual-openyam-sim-agent`
then ask the agent "put bottle_1 in the bin with the left arm, then bottle_4
with the right arm". Expect both bottles inside the bin body's bounds
(check with get_body_poses), no FAULT, and 5/5 after reset. Report pass
counts, wall time per pick, and anything that had to be hand-tuned.
```

## T5 status (2026-09-06) — partial

`hackmit/t5-classical-demo` @ e373b7fcd, based on T4 with T3 merged in, so it
carries all of T1-T4. Unpushed.

**LFS works again.** `dual_openyam_abc_box_v2` and `yam_description` both pull.
Everything below runs against the real planning URDF; the reconstruction is
retired. It was 62 mm off at the grasp frame, so waiting for LFS was right.

### Delivered

- `dual-openyam-sim-pick-place` and `dual-openyam-sim-agent`, mirroring the
  xArm stack: privileged scene registration, heuristic grasps, pick/place in
  `world`, three policy-named camera streams, planner, per-group grippers.
- `BIMANUAL_MANIPULATION_AGENT_SYSTEM_PROMPT` in `common/agent_prompts.py`,
  with the arm-choice rule (`y > 0` -> left, else right) and a hard
  always-pass-planning_group rule.
- Scene reset: the malformed `<keyframe>` is deleted from the vendored scene,
  so `MujocoSimModule.reset()` is now a safe stage reset.

### Verified

Single-arm pick and place, repeated after scene reset:

```
trial 1..5: reset=True respawn_err=0.0040 m pick=True place=True moved~0.17 m
RESULT: 5/5 successful
```

The bottle is genuinely grasped, lifted and carried — this is not the T3
false-positive. Scene reset restores it to within 4 mm of spawn every time.

### Three integration bugs this uncovered

1. **`RobotModelConfig.base_pose` was identity.** The scene stands the rig at
   (0.2525, 0, 0.76) but the planner read world targets as base-relative, so
   every reach failed IK. `dual_openyam_sim_model_config()` sets it.
2. **Pink's default gains never converge on this model** — every IK call, even
   to the arm's current pose, returned "did not converge". The Quest teleop
   blueprint already tunes gains for this arm; reuse them.
3. **`HeuristicGraspModule` assumed the tool frame points Z along the approach.**
   The OpenYAM grasp frame does not, so every proposal came back unreachable.
   It now takes `tool_rotation_rpy`, default identity (xArm untouched); the
   dual rig uses a half turn about Y.

Also: `declare_sim_camera_module` changes the module's class name and therefore
its RPC topic, so `dual_openyam_sim_module` pins `instance_name` back to
`MujocoSimModule`.

### NOT done — the left arm

The demo is single-arm. `bottle_4` with `left_manipulator` fails during pick,
while the identical operation on the right arm succeeds 5/5.

What is ruled out: reach and collision. Solving IK directly for the left arm's
real grasp and pregrasp poses succeeds, with `check_collision=True`, and the
world has no registered obstacles. The failure is in the planning stage, not
in reachability.

Two leads for whoever picks this up:

- **The arms are not mirrored.** In the MJCF both chains have identical link
  poses and are only translated in y, so the left arm's workspace is the
  right's shifted by +0.62 m. Grasp orientations that suit one arm are not
  automatically right for the other.
- **`HeuristicGraspModule` returns exactly one candidate and `pick_object`
  takes `candidates[0]` unconditionally.** There is no fallback when the single
  proposal is unplannable. A parallel-jaw grasp is invariant under a 180-degree
  wrist rotation and its yaw is only weakly constrained on a round bottle, so
  emitting several candidates and letting `pick_object` try them in order is
  likely the real fix — and would also help the T3 grasp-slip case.

### Still open from before

The T3 grasp-slip finding is **not** reproduced here: with the tool rotation
fixed, the right arm's grasp holds. Whether the xArm case was the same root
cause is untested.

## Preamble changes from 2026-09-06 (apply on top of the common preamble)

```
- Base every new branch on `mustafa/task/hackmit-manip` @ 45332452b (T1-T5
  merged, main merged). Hub merges happen in /home/mustafa/dimos-wt/hub;
  the main checkout /home/mustafa/dimos is on other branches, do not use it.
- Git LFS works now. `data/dual_openyam_abc_box_v2` and `data/yam_description`
  pull on demand.
- The vendored scene is untracked and 45 MB; link it into your worktree or
  one config test fails at import:
    ln -s /home/mustafa/dimos/data/dual_openyam_sim data/dual_openyam_sim
- Worktrees have no venv. Run `/home/mustafa/dimos/.venv/bin/python -m pytest ...`
  from the worktree directory; `python -m` puts the worktree first on
  sys.path. Add `-m "mujoco or not mujoco"` to include MuJoCo tests.
- Every MuJoCo run: `MUJOCO_GL=egl dimos --simulation mujoco --viewer none run <blueprint>`.
- Read plan.md "Status 2026-09-06 (later)" and handoffs.md "T5 status".
```

## T5b Left arm + multi-candidate grasps

Worktree branch `hackmit/t5b-left-arm`. Can start now; T6 and T7 do not
depend on it.

```
Task T5b: make the left arm pick, and make picking robust to one bad grasp
proposal.

State: on `dual-openyam-sim-agent` the right arm picks and places 5/5.
`bottle_4` with `planning_group="left_manipulator"` fails inside
`pick_object` at the planning stage, while solving IK directly for the same
grasp and pregrasp poses succeeds with check_collision=True and the world
has no registered obstacles. See handoffs.md "T5 status" for the full
account.

Step 1, get the real failure. Run the left-arm pick with ManipulationModule
logging visible and capture the exact plan error (start-state validity,
IK inside the planner vs the standalone solve, RRT timeout, trajectory
start tolerance, self-collision between the two arms at the home posture).
Compare the planner's IK config with the standalone solve you did; they may
not share the Pink gains from `DUAL_OPENYAM_PINK` in
dimos/robot/manipulators/dual_openyam/blueprints/simulation.py. Report the
message verbatim before changing anything.

Step 2, structural fix regardless of step 1's answer. `HeuristicGraspModule`
(dimos/manipulation/grasping/heuristic_grasp.py) returns exactly one
candidate and `PickAndPlaceModule.pick_object`
(dimos/manipulation/pick_and_place_module.py) takes `candidates[0]`
unconditionally.
- Emit several ranked candidates: the narrow-axis yaw, its 180-degree wrist
  flip (parallel jaws are symmetric), and for round objects where the
  narrow axis is ambiguous also +-90 degrees; optionally one lower grasp
  height. Score them so the current single proposal stays first for the
  xArm (its tests must not change).
- `pick_object` walks the candidates in order: plan pregrasp, plan grasp,
  and only then execute; on a planning failure move to the next candidate.
  Keep the returned SkillResult fields (`rank`, `score`, `candidates`)
  meaningful.

Step 3, tests. One in-package test that the generator emits the flip
candidate and keeps the original first; one that pick_object skips an
unplannable first candidate (fake manipulation spec). Also add the T5 test
that was skipped: `dual_openyam_sim_pick_place` resolves with no dangling
connections. And make
`test_config.py::test_dual_openyam_hardware_switches_to_physics_under_mujoco`
not depend on the untracked scene existing (patch the path or stop
resolving the LfsPath at config time).

Verify: on `dual-openyam-sim-agent`, "put bottle_4 in the bin with the left
arm" 5/5 after resets, then "bottle_1 with the left arm, then bottle_4 with
the right arm" 5/5. Report per-arm pass counts, which candidate rank won,
and wall time per pick.
```

## T6 Dual-arm LeRobot policy binding, policy skills, policy blueprint

Worktree branch `hackmit/t6-policy`. Can start now with a stub checkpoint;
the real checkpoint arrives from T7.

```
Task T6: let the agent start and stop an ACT policy on the dual OpenYAM in
sim, beside the classical skills.

CC's stack (read these first, all on the hub branch):
- dimos/imitation/profile.py (PolicyIOProfile), dimos/imitation/policy/module.py
  (declare_policy_module, PolicyRolloutConfig, POLICY_ROLLOUT_TASK_NAME,
  the preflight/start/stop/rollout_status RPCs),
  dimos/imitation/policy/lerobot/module.py (OpenYamLeRobotPolicy, the
  single-arm binding to copy), dimos/imitation/policy/runtime.py (the
  shared rollout loop), dimos/imitation/policy/lerobot/python/dimos_lerobot/runtime.py
  (checkpoint feature validation: input keys must equal the profile's
  observation keys, action width must equal the joint count).
- dimos/robot/manipulators/dual_openyam/learning.py (DUAL_OPENYAM_TWO_WRIST_IO,
  DUAL_OPENYAM_ABC_IO), dimos/robot/manipulators/dual_openyam/blueprints/learning_rollout.py
  (how the ABC rollout wires the coordinator's policy task).
- dimos/imitation/workflows.py (the CLI catalogs).
- Our sim: dimos/robot/manipulators/dual_openyam/sim.py publishes
  `top_image`, `left_wrist_image`, `right_wrist_image` at 640x480;
  dimos/robot/manipulators/dual_openyam/blueprints/simulation.py has
  `dual_openyam_sim_pick_place`; agentic.py has `dual_openyam_sim_agent`.

Do:
1. Profile `DUAL_OPENYAM_LEROBOT_IO` in dual_openyam/learning.py:
   observations `observation.images.top` <- top_image,
   `observation.images.left_wrist` <- left_wrist_image,
   `observation.images.right_wrist` <- right_wrist_image,
   `observation.state` <- coordinator_joint_state over DUAL_OPENYAM_JOINTS
   (14, arms then grippers, the hardware order); action key "action" from
   `applied_joint_position_command` over the same joints; sync anchor top.
   `rate_hz` must equal what the sim cameras actually publish; T7 measures
   it, start with 15 and make it one constant shared with T7.
2. `DualOpenYamLeRobotPolicy` in dimos/imitation/policy/lerobot/module.py
   via declare_policy_module with LeRobotPolicyConfig and the same
   implementation string as the single-arm one. Add a rollout workflow
   `dual-openyam-lerobot` in workflows.py (no CAN).
3. `PolicySkills` module (dimos/imitation/policy/skills.py): a Module holding
   a reference to the policy module (see how SimSceneRegistrationModule or
   PickAndPlaceModule declare typed references to other modules for
   autoconnect) with `@skill run_policy()` (preflight, then start; returns
   the RolloutStatus fields), `@skill stop_policy()`, `@skill policy_status()`.
   ACT is not language-conditioned, so the task string stays the config
   value.
4. Coordinator arbitration. The policy needs a trajectory task named
   POLICY_ROLLOUT_TASK_NAME over all 14 joints, including both grippers,
   next to the planner trajectory task (priority 20) and the two gripper
   tasks (priority 20) in `_dual_openyam_sim_tasks`. Find out from
   dimos/control/coordinator.py whether an idle higher-priority task
   blocks a lower one, then pick the policy task's priority so that: while
   the policy runs it owns all 14 joints; after stop_policy the planner and
   gripper tasks regain them without a restart. Write down what you found.
5. Blueprint `dual_openyam_sim_policy_agent` = dual_openyam_sim_pick_place
   + DualOpenYamLeRobotPolicy.blueprint(instance_name=POLICY_ROLLOUT_INSTANCE_NAME,
   artifact=..., task=..., device="cuda", trajectory_task_name=POLICY_ROLLOUT_TASK_NAME)
   + PolicySkills + McpServer + McpClient with the bimanual prompt extended
   by the three policy skills. `artifact` must be settable at launch;
   check how `dimos run <bp> --left-can-port` reaches a module config
   field and use the same mechanism, or a `DUAL_OPENYAM_POLICY_ARTIFACT`
   env var read in the blueprint file if that is what works.
   Regenerate all_blueprints.py.
6. Pre-warm the isolated env early, it is a multi-GB download:
   `cd dimos/imitation/policy/lerobot/python && uv sync --frozen`.

Tests: one that the profile validates and matches the sim stream names;
one that PolicySkills routes run/stop to the RPCs (fake policy module);
one for the blueprint resolving. CC's tests show how to build a tiny
synthetic checkpoint; reuse that to run `preflight_rollout` against the
live sim streams without a trained policy.

Verify: preflight passes from the sim streams (observations_ready true,
policy_ready true) with the synthetic checkpoint; with a fake constant
action chunk the arms follow it and `stop_policy` stops them within one
chunk; afterwards `go_home` and a gripper call work. Report the priority
you chose and why, and the exact launch command.
```

## T7 Scripted demonstrations + ACT training

Worktree branch `hackmit/t7-data`. Start now on the right arm; it is the
long pole.

```
Task T7: produce a LeRobot dataset of scripted pick-and-place demonstrations
in sim and train an ACT policy on it.

Decision already made: record with the Python `Recorder` (SQLite). CC's
Rust recorder needs a Nix build and a plain cargo build fails here.
`dimos imitation prepare` reads `.db` recordings. Read:
- dimos/imitation/collection/recorder.py (CollectionRecorder: a
  `dimos.memory.module.Recorder` with fixed In ports; only connected
  streams are recorded), dimos/imitation/collection/native_recorder.py
  (`declare_recorder` builds a recorder class with one In port per profile
  stream; mirror it over the Python Recorder),
  dimos/imitation/collection/episode_monitor.py (`command("start"|"save"|"discard")`
  RPC, `status` Out stream that segments episodes),
  dimos/imitation/dataprep/core.py and dimos/cli/commands/imitation.py
  (`prepare`, `train`), dimos/imitation/workflows.py.
- The profile comes from T6 (`DUAL_OPENYAM_LEROBOT_IO` in
  dual_openyam/learning.py). If T6 has not landed yet, define it yourself
  with the same keys listed in T6 step 1 and coordinate the merge.

Do:
1. `declare_python_recorder(name, module_name, profile)` next to
   declare_recorder, returning a `Recorder` subclass with In ports for the
   profile streams plus `status`. `DualOpenYamSimRecorder` from it.
2. Blueprint `dual_openyam_sim_collect` = dual_openyam_sim_pick_place
   + DualOpenYamSimRecorder(db_path under RECORDINGS_DIR)
   + EpisodeMonitorModule(task=...). Collection workflow
   `dual-openyam-sim` in workflows.py so `prepare` finds the profile.
3. Generator `tool_generate_demos.py` beside the dual_openyam blueprints
   (tool_ files are outside mypy). Loop N episodes: MujocoSimModule.reset,
   randomise the target bottle's pose on the table (add a
   `set_body_pose(name, xyz, quat)` RPC on MujocoSimModule for free
   bodies if none exists; check what T2 added), a short settle,
   `command("start")`, scan_objects, pick_object with the arm chosen by y,
   place_at the bin, go_home, `command("save")`; `command("discard")` on
   any failure. Log per-episode outcome and duration. Right arm only until
   T5b lands.
4. Camera rate. Measure what the three sim cameras sustain during a pick
   (T2 saw 14.8 Hz at 15 configured). The profile's `rate_hz` and the
   SimCameraSpec fps must agree, and strict quality mode rejects sources
   below 95 % of it. Fix one constant and share it with T6.
5. Wrist views. At the home posture the wrist cameras see mostly table.
   Choose an observe posture (also used as the episode start) where both
   wrist cameras see the workspace; put it in one constant the sim module
   and the generator share. Do not change DUAL_OPENYAM_HOME_JOINTS for
   hardware.
6. Prepare and train: `dimos imitation prepare dual-openyam-sim RECORDING.db --output DATASET`,
   then `dimos imitation train --dataset.repo_id=local/dual-openyam-sim --dataset.root=DATASET --policy.type=act --output_dir=outputs/dual-openyam-act`
   with a step count that fits a couple of hours on the local RTX 4090
   (start around 20k steps, batch 8; chunk_size 50 at 15 Hz). The train
   command is a pass-through to the LeRobot CLI in the isolated env; the
   env needs `uv sync --frozen` in dimos/imitation/policy/lerobot/python
   first.

Targets: 100 saved right-arm episodes, dataset passes strict quality,
training runs to completion. Then hand the checkpoint path to T6 for the
rollout check (>= 7/10 on the training task is the bar).

Tests: one for declare_python_recorder port declaration; one for the
generator's arm-choice and discard-on-failure logic with fakes. No physics
in unit tests.

Report: episodes saved/discarded, mean episode length, measured camera
rates, dataset quality report, training loss curve summary, checkpoint
path, and anything in the data that looks wrong (pauses, gripper timing).
```

## T5b status (2026-09-07) — done

`hackmit/t5b-left-arm` @ 21d428cc9, one commit on `mustafa/task/hackmit-manip`
@ 45332452b. Unpushed.

### Verified

Full pick-and-place cycles, scene reset plus both arms sent home between
trials, place target (0.50, +-0.30, 0.86):

```
bottle_4 / left_manipulator   5/5   7.5-11.3 s   ranks 0,0,1,0,0
bottle_1 / right_manipulator  4/5   7.5-9.0 s    ranks 0,-,0,0,3
```

The one right-arm miss was an IK non-convergence on the pick; the run before
the place-target change was 5/5 on pick alone. Ranks 1 and 3 are the candidate
fallback doing its job — without it those two trials fail.

### Step 1: the failure, verbatim

```
left_manipulator / bottle_4: success=False (1.8 s)
  error_code : PLANNING_FAILED
  message    : IK failed: NO_SOLUTION: Pink IK did not converge within the iteration budget
  candidates : 1 in frame world
    score=1.00 pos=(0.460,0.381,0.823) quat=(-0.000,0.000,-0.755,-0.656)
```

The planner does share `DUAL_OPENYAM_PINK`: a standalone `solve_ik` in the same
process, with `check_collision=True`, succeeds on that exact grasp pose. The
1.8 s and the single candidate were the tells - it failed on the *pregrasp*.

### Four stacked causes, each hiding the next

1. **The arm limits were wrong.** T1 hardcoded them from the MJCF, which
   disagrees with the planning URDF on three joints (joint1 upper 3.054 vs
   3.142, joint3 upper 3.665 vs 3.142, joint4 lower -1.571 vs -1.693). IK
   candidates are checked against these, so the tighter ones rejected valid
   solutions. Now read from the URDF via `dual_openyam_arm_position_limits()`.
2. **The pregrasp was on the wrong side.** `_offset_pose` backs off along the
   tool's -Z. This gripper's grasp frame points Z out of the *back* of the
   palm, so the pregrasp sat 10 cm under the object, inside the table. Nothing
   caught it because the world has no registered obstacles, so the planner
   drove through the table happily - which is also why the right arm appeared
   to work in T5. New `PickAndPlaceModuleConfig.pregrasp_along_tool_z`.
3. **`{side}_grasp_frame` is 5.9 cm from the fingertip midpoint.** Driving it
   to an object's centre closes the jaws past the object; the bottle was being
   knocked 4-5 cm and the grasp came up empty. `dual_openyam_sim_model_config()`
   now adds a `{side}_tcp` fixed frame at (0.044, 0, -0.039) from the grasp
   frame and plans to that. Measured against the fingertip collision spheres;
   residual is 13 mm, which is the jaw opening moving the midpoint.
   **This is very likely the T3 xArm grasp-slip too - check that frame next.**
4. **One candidate, no fallback.** The two arms accept *different* wrist bands
   over the same object (measured at the TCP: left 0/60/90/120 deg,
   right 30/60/90/270/330), so a single yaw is a coin flip.

### Step 2, as built

`HeuristicGraspConfig` gains `tool_rotation_rpy` and `yaw_candidates`, both
inert by default so the xArm is bit-identical and its tests are untouched.
Candidates are the narrow-axis yaw (score 1.0), its half turn (0.95, always
valid for parallel jaws), and - only when the cross section has no narrow axis
- six finer yaws. `pick_object` walks them: a proposal that will not plan is
skipped, a grasp-verification failure still returns, since the jaws already
reached the object and another angle is not the answer. `place_at` got the
same fallback and the same offset fix.

### Step 3, tests

- `dimos/manipulation/test_grasp_candidate_fallback.py`: the generator offers
  the flip and keeps the narrow-axis grasp first; `pick_object` abandons an
  unplannable first candidate and reports `rank=1`.
- `test_blueprints.py`: `dual_openyam_sim_pick_place` resolves with the specs
  bound and the three coordinator tasks present.
- `test_config.py` no longer needs the untracked scene: `config.py` stops
  calling `str()` on the `LfsPath`, so building a hardware component never
  touches disk.
- `test_mujoco_multi_camera.py` skips instead of failing when there is no
  offscreen GL, rather than reporting "assert (0 >= 2)".

603 passed across manipulation, simulation and manipulators. The
`openyam/blueprints/test_learning_collection_e2e.py` error is a pre-existing
`cargo build` dependency, unrelated.

### Open

- **Right arm 4/5, not 5/5.** One IK non-convergence on the pick. More yaw
  candidates or a second IK seed would probably close it.
- **`MujocoSimModule.reset()` does not send the robot home.** The coordinator
  keeps its latched position target, so the arm springs back and knocks the
  objects that were just respawned. Every reset must be paired with a home
  move; the verify harness does this. Worth folding into a single stage-reset
  skill for T8.
- **`bottle_5` is ejected from the scene** on reset (ends up at y=2.3, z=2.1).
  It spawns in contact with something. Avoid it, or fix the spawn, before
  choosing demo objects.
- The demo still places to an explicit coordinate rather than into the bin;
  the bin at (0.65, 0, 0.83) is cross-body for both arms.

Current execution reports continue in [handoffs.md](handoffs.md).
