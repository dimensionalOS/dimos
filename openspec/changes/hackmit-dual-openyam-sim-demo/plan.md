# HackMIT demo: bimanual OpenYAM pick-and-place in MuJoCo

Planning hub. Research done 2026-09-05 against `origin/main` (8b880d77e) and
TomCC7's learning branches. Each task below is meant to be handed off as one
prompt and reported back here.

## Goal (stage demo)

Bimanual OpenYAM in a MuJoCo tabletop scene with a few objects. Two modes,
same blueprint family, both driven through the agent:

1. Classical agentic: "pick the red cube with the left arm and put it in the
   bin" -> scan_objects -> pick_object -> place_at (planner + heuristic grasp).
2. Learned: "run the policy: put the bottle in the bin" -> ACT (Action
   Chunking Transformer, via LeRobot) rollout from sim cameras.

Perception is privileged (ground truth from MuJoCo). Pim's scene is swapped in
at the end; build against a composed scene first.

## What already exists (verified on main)

- Dual OpenYAM hardware + model: `dimos/robot/manipulators/dual_openyam/`
  (12 arm joints + 2 grippers = 14, planning groups `left_manipulator` /
  `right_manipulator`, tips `left_grasp_frame` / `right_grasp_frame`).
  Blueprints: `coordinator-dual-openyam`, `dual-openyam-planner-coordinator`,
  `teleop-quest-dual-openyam`.
- Sim selection today is `mock_whole_body` (no physics, no contact, so it can
  never actually grasp anything).
- xArm sim stack is the template: `xarm-perception-sim` /
  `xarm-perception-sim-agent` in `dimos/robot/manipulators/xarm/blueprints/`
  = MujocoSimModule + coordinator + ManipulationModule + ManipulationSkills +
  PickAndPlaceModule + HeuristicGraspModule + scene registration + McpServer/
  McpClient.
- Agent skills: `ManipulationSkills` (move_to_pose, move_to_joints, go_home,
  set/open/close_gripper, get_robot_state) and `PickAndPlaceModule`
  (scan_objects, pick_object, place_at). Both take `planning_group`.
- Pick/place only needs two Protocols: `ObjectSceneRegistrationSpec`
  (scan_scene, get_object_pointcloud_by_object_id, get_full_scene_pointcloud)
  and `GraspGenSpec` (propose_grasps). A privileged sim module implementing the
  first slots in with zero changes to pick/place.
- MuJoCo engine already renders N cameras (`CameraConfig` dict), and the SHM
  bridge supports PD/position commands for up to 32 joints
  (`dimos/simulation/engines/mujoco_shm.py`).
- G1 sim whole-body adapter (`dimos/simulation/adapters/whole_body/g1.py`) is
  the pattern for a MuJoCo `WholeBodyAdapter` over SHM, but it hardcodes 29
  motors and IMU.
- `SimBodyPose` + `MujocoSimModule.get_body_poses` exist on
  `origin/manip/mobile-manipulation-demo-aug18` (water demo), not on main.

## What CC (TomCC7) has, unmerged

PR stack, oldest first, each based on the previous:
#3853 -> #3854 -> #3855 -> #3921 -> #3931 -> #3942. Tip branch
`cc/feat/flexible-policy-module` (54 commits ahead of main, 17 behind, last
touched 2026-09-04). Separate: #3944 `cc/feat/manip-client-sdk` (rewrites
`pick_and_place_module.py` and `manipulation_module.py`, conflicts with T4).

The tip branch gives us:

- `PolicyIOProfile` (feature key -> typed stream), `declare_policy_module`,
  `IsolatedPythonModule` policy hosts with their own uv env.
- `OpenYamLeRobotPolicy` (single arm, wrist cam, ACT via LeRobot) and
  `DualOpenYamAbcPolicy` (Amazon ABC-DiT, 3 cams). There is NO dual-arm
  LeRobot binding yet. `DUAL_OPENYAM_TWO_WRIST_IO` (collection, 14-D +
  2 wrist cams) exists.
- Policy RPCs: `preflight_rollout`, `start_rollout`, `stop_rollout`,
  `rollout_status`. They work without Quest. Actions go to a coordinator
  trajectory task named `policy_rollout`, capped at 0.5 s horizons.
- `dimos imitation collect|prepare|train|run` CLI, Rust MCAP recorder,
  LeRobot dataset writer.
- Cameras are hard-wired to `WebcamConfig` in `dimos/imitation/cameras.py`.
  In sim we bypass that and remap MuJoCo image streams to the profile names.

## Gaps to close

| # | Gap | Size |
|---|-----|------|
| G1 | No physics sim adapter for a whole-body arm rig (mock only) | M |
| G2 | MujocoSimModule publishes ONE camera (`camera_name`) | M |
| G3 | Gripper is model-wide (`RobotModelConfig.gripper_hardware_id`), so a two-gripper model has `has_gripper=False` on both groups and pick_object refuses | S-M |
| G4 | No privileged scene registration module | M |
| G5 | No dual-arm LeRobot policy binding, no agent skill to start/stop a policy | S |
| G6 | No sim demonstrations to train ACT on | M |
| G7 | LFS assets are pointers locally (`data/.lfs/yam_description.tar.gz`, `dual_openyam_abc_box_v2.tar.gz`); no dual-yam MJCF in the repo | S |
| G8 | `lerobot` not in the venv (CC's isolated env pulls it on first run) | S |

## Architecture

```
MujocoSimModule (scene MJCF, headless, cams: top, left_wrist, right_wrist)
   | SHM joints                 | Out[Image] x3            | body/geom poses
   v                            v                          v
sim_mujoco_whole_body ----> DualOpenYamCoordinator     SimSceneRegistration
   adapter (G1)              tasks: trajectory(20)        (G4, implements
                                    left/right gripper     ObjectSceneRegistrationSpec)
                                    policy_rollout(10)          |
                                        ^                       v
ManipulationModule (dual model, per-group gripper G3) <--- PickAndPlaceModule
ManipulationSkills                                          HeuristicGraspModule
PolicySkills (G5: run_policy/stop_policy) -> DualOpenYamLeRobotPolicy (G5)
McpServer + McpClient (bimanual prompt)
RerunBridge / Viser
```

Blueprints to end with (names TBD):

- `dual-openyam-sim` : sim + coordinator + planner (smoke test, no agent)
- `dual-openyam-sim-pick-place` : + skills + pick/place + privileged perception
- `dual-openyam-sim-agent` : + MCP agent
- `dual-openyam-sim-policy-agent` : + policy module + policy skills
- `dual-openyam-sim-collect` : sim + recorder (for G6)

## Tasks (handoff units)

Each task: scope, files, verify. Keep docstrings and comments minimal, tests
in-package, no `__init__.py`, no section-marker comments.

### T0 Setup (you, 30 min)

- `git lfs pull --include="data/.lfs/yam_description.tar.gz,data/.lfs/dual_openyam_abc_box_v2.tar.gz"`
- Scene: resolved by T1. Amazon ABC ships a dual-YAM MuJoCo scene
  (`put_bottle.xml`: both arms, slide-joint grippers, cameras `top`/`left`/
  `right`, table, bin, six bottles; Apache-2.0 + MIT). Vendored untracked at
  `data/dual_openyam_sim/`. Pim's scene is a later swap.
- Branch `hackmit/dual-openyam-sim` from main. Policy work (T6, T7) branches
  from `cc/feat/flexible-policy-module` until the stack lands.
- Ask CC: (a) can #3853..#3942 land within a week, (b) any sim rollouts done,
  (c) keep #3944 off the gripper code paths in T4 or we coordinate.

### T1 Generic MuJoCo whole-body adapter (G1)

- New `dimos/simulation/adapters/whole_body/generic.py`:
  `SimMujocoWholeBodyAdapter(address, num_motors, require_imu=False)`.
  Same SHM protocol as the G1 adapter, N motors from config, position or PD
  mode. Register as `sim_mujoco_whole_body` in
  `dimos/simulation/adapters/whole_body/_registry.py`.
- `dual_openyam_hardware()` picks it when `global_config.simulation ==
  "mujoco"` and an MJCF address is given; keep `mock_whole_body` otherwise.
- `RobotSimSpec` for dual yam (hardware joints -> MJCF joint names, no
  floating base, no IMU) in `dimos/robot/manipulators/dual_openyam/sim.py`.
- Verify: `dimos --simulation mujoco run dual-openyam-sim` moves both arms
  via `plan_to_joints`, grippers open/close in the viewer.

### T2 Multi-camera MujocoSimModule (G2)

- Add `extra_cameras: list[SimCameraSpec]` (name, width, height, fps,
  stream) to `MujocoSimModuleConfig`; declare dynamic `Out[Image]` per
  camera (same trick `declare_policy_module` uses, or a fixed set of named
  outputs). Render loop already exists per camera; only the publish loop is
  single-camera.
- Keep color at 15 Hz, 320x240 for wrist cams if rendering stalls the sim
  thread (the code warns about this).
- Port `get_body_poses` from the water-demo branch while in this file.
- Verify: three image streams visible in Rerun at the configured rate; sim
  step rate unchanged within 10 %.

### T3 Privileged scene registration (G4)

- New `dimos/simulation/perception/sim_scene_registration.py`:
  `SimSceneRegistrationModule` implementing `ObjectSceneRegistrationSpec`.
  `scan_scene(prompts)` -> `Detection3DArray` of MuJoCo bodies whose name
  matches a prompt (id = body name, class_id = prompt). Object pointcloud =
  surface samples of the body's geoms (box/cylinder/sphere/mesh) transformed
  to world. `get_full_scene_pointcloud` = all static geoms minus the robot and
  the excluded object, voxelised. Needs an RPC on MujocoSimModule exposing
  geom type/size/pose per body (add next to `get_body_poses`).
- Verify: pick_object on the xArm-style pipeline returns grasp candidates in
  `world`; in-package test with a 3-body MJCF.

### T4 Per-group gripper (G3)

- `PlanningGroupDefinition.gripper_hardware_id: str | None`; keep the
  model-level field as the fallback so xArm/OpenYAM configs are untouched.
- `ManipulationModule`: `list_planning_groups`,
  `_get_group_gripper_position`, `set_gripper_position`,
  `_resolve_group_with_capability` resolve the id per group.
- `dual_openyam_model_config()` sets `left_arm` / `right_arm`; planner
  coordinator gets `left_arm_gripper` / `right_arm_gripper` gripper tasks
  (they exist only in the teleop blueprint today).
- Verify: `set_gripper(0.0, planning_group="left_manipulator")` closes only
  the left gripper in sim; `PickAndPlaceModule._resolve_group` returns both
  groups as gripper-capable.

### T5 Classical demo blueprint + agent

- `dimos/robot/manipulators/dual_openyam/blueprints/simulation.py` and
  `agentic.py`, mirroring xArm. `PickAndPlaceModule(planning_frame="world")`.
- Bimanual system prompt in `common/agent_prompts.py` (arm choice rule:
  objects with y > 0 -> left, else right; always pass `planning_group`).
- Scene reset RPC (respawn objects) for repeat runs on stage.
- Verify E2E: three objects, agent picks and places two of them with different
  arms, no FAULT, repeatable 5/5 after reset.

### T6 Policy binding + skills (G5)

- `DualOpenYamLeRobotPolicy` = `declare_policy_module` over a new
  `DUAL_OPENYAM_LEROBOT_IO` profile (top + left wrist + right wrist,
  14-D joints in `DUAL_OPENYAM_JOINTS` order). Decide cams in D3.
- `PolicySkills` module with `@skill run_policy(task)`, `stop_policy`,
  `policy_status` calling the policy RPCs; register in the agent blueprint.
- Coordinator: `policy_rollout` trajectory task at priority 10 beside the
  planner trajectory task at 20. Confirm the planner can preempt a running
  policy and that `stop_policy` cancels cleanly.
- Verify: with a dummy checkpoint, preflight passes from sim streams;
  `run_policy` moves the arms; `stop_policy` halts within one chunk.

### T7 Data + ACT training (G6)

- Scripted demonstration generator: loop the T5 pipeline over randomised
  object poses with CC's recorder attached (`dual-openyam-sim-collect`),
  writing `applied_joint_position_command` + 3 cams at 30 Hz. Target 100
  episodes, 1 task.
- `dimos imitation prepare` -> `dimos imitation train --policy.type=act`.
  RTX 4090 laptop is available locally; a few hours for 100 episodes.
- Fallback: Quest teleop in sim via `teleop-quest-dual-openyam` + recorder.
- Verify: rollout success >= 7/10 on the training task in sim. If lower,
  the policy is the "bonus" segment and classical carries the demo.

### T8 Stage polish

- Swap in Pim's scene, Rerun blueprint (3D + cameras), run script, reset
  button, recorded fallback video, laptop dry run of the full script twice.

## Decisions (2026-09-05)

- D1 Demo 2026-09-15. Full dress rehearsal 2026-09-11.
- D2 Build on CC's tip now. Nothing merges to main from this work; expect
  more missing pieces to surface as we go.
- D3 Policy cameras: top + left wrist + right wrist.
- D4 ACT data from scripted classical demos in sim.
- D5 Classical is the primary segment, policy is the bonus.

## Branch

`mustafa/task/hackmit-manip` = `origin/cc/feat/flexible-policy-module`
(737a8ec12) with `origin/main` (8b880d77e) merged, 55 commits ahead of main.
One conflict resolved in `dimos/cli/dimos.py` (both sides added imports).

Keep current by merging, not rebasing: `git merge origin/main` regularly,
and `git merge origin/cc/feat/flexible-policy-module` whenever CC moves the
tip. Rebasing would rewrite CC's 54 commits underneath us. Merge commits
need `SKIP=lfs_check` when `data/.lfs/*` changes, because the hook flags the
extracted data dirs.

Everything lands as small commits on this branch. No PR to main.

## Timeline

| Dates | Work |
|-------|------|
| Sep 5-7 | T0 done; T1, T2, T3, T4 in parallel (four handoffs) |
| Sep 8-9 | T5 classical end to end; T6 policy binding + skills |
| Sep 9-11 | T7 demo generation + ACT training; Sep 11 rehearsal |
| Sep 12-14 | T8 polish, Pim's scene, run script, fallback video |
| Sep 15 | Demo |

## Status 2026-09-06: T1-T4 merged

Both task stacks merged into `mustafa/task/hackmit-manip` @ febdc3c1b
(`hackmit/t4-per-group-gripper` then `hackmit/t3-privileged-perception`).
Hub-checked: 31 new tests pass with the MuJoCo marker on, ruff clean, no
`all_blueprints.py` drift, neighbouring suites green except one pre-existing
LFS failure (`test_openyam.py::test_make_openyam_model_config_uses_canonical_arm_joints`).
Full write-up in handoffs.md under "T1-T4 status".

Corrections to the text above, learned from T1-T4:

- `dof=14` (12 arm + 2 gripper), not 16. Grippers are slide joints
  `left_left_finger` / `right_left_finger`, range 0 to 0.0475 m.
- New blueprints: `dual-openyam-sim` (physics sim + planner + coordinator with
  both gripper tasks, no skills yet) and `xarm-privileged-sim` (xArm room scene
  with ground-truth perception, used to prove T3).
- Every MuJoCo run here needs `MUJOCO_GL=egl`, `--viewer none`, headless.
  Bare `--simulation` now means the physics adapter, so
  `dual-openyam-planner-coordinator` (no sim module) must run with no flag.
- The ABC scene's `<keyframe>` scrambles the scene on reset (16 values into
  nq=65). Delete it before T5's reset RPC; T1 already sets the arm posture.
- `dual-openyam-sim` still cannot start here: `DUAL_OPENYAM_MODEL` (planning
  URDF, `dual_openyam_abc_box_v2`) is LFS-blocked. T1 rebuilt an equivalent
  URDF at `data/dual_openyam_sim/dual_openyam.urdf` (FK matches MuJoCo) but
  left it unwired. One LFS pull unblocks T5.
- The "grasp does not hold" observation was on the xArm room scene (cup,
  `xarm_grasp_sim`). Nothing has been picked on the dual-YAM scene yet; its
  finger pads carry friction 4.0 with contact priority, so T5 measures there
  first.

## Status 2026-09-06 (later): T5 merged, partial

Hub is `mustafa/task/hackmit-manip` @ 45332452b (T5 + one new main
commit). Hub git operations now happen in the worktree
`/home/mustafa/dimos-wt/hub`; the main checkout `/home/mustafa/dimos` is
used for other branches between turns, so never merge there. Verified on
the hub: 246 tests green with MuJoCo on, ruff clean, LFS pulls again so the
real planning URDF is in use (the rebuilt one was 62 mm off and is retired).

Where this gets us:

- Right arm, agent-driven scan -> pick -> place into the bin: 5 of 5,
  bottle genuinely carried, scene reset restores it to 4 mm. The classical
  demo is real for one arm.
- Left arm: every pick fails in the planning stage even though IK for its
  grasp and pregrasp succeeds with collision checking. Two leads: the arms
  are translated copies, not mirrors, so one grasp orientation does not fit
  both; and the grasp generator emits exactly one candidate that
  `pick_object` takes unconditionally, with no fallback.
- Three integration fixes landed with T5: planner `base_pose` for a rig
  that is not at the world origin, Pink IK gains reused from the Quest
  blueprint, `HeuristicGraspModule.tool_rotation_rpy` because the OpenYAM
  grasp frame does not point Z along the approach.
- Blueprints: `dual-openyam-sim-pick-place`, `dual-openyam-sim-agent`.
- The vendored scene (45 MB, untracked) must be linked into every worktree:
  `ln -s /home/mustafa/dimos/data/dual_openyam_sim data/dual_openyam_sim`.
  One config test resolves the scene path at import time and fails without
  it.

Learning-stack facts for T6/T7 (checked on the hub):

- CC's LeRobot backend is profile-generic: it checks the checkpoint's input
  feature keys against the profile and the action width against the joint
  count. A dual profile (three images + 14-D state/action) works unchanged.
- The policy module's `preflight_rollout` / `start_rollout` /
  `stop_rollout` RPCs work without Quest.
- The isolated LeRobot env (`dimos/imitation/policy/lerobot/python`,
  lerobot 0.6.0, Python 3.12) is created by `uv sync --frozen` on first
  module start. Pre-warm it; it is a multi-GB download.
- CC's recorder is a Rust native module expecting a Nix-built binary. This
  host has cargo but no Nix, and a plain cargo build fails (the generated
  `lcm_msgs` Rust crate has no `imitation_msgs`). Decision: T7 records with
  the Python `Recorder` (SQLite), which `dimos imitation prepare` reads as
  well as MCAP.
- `EpisodeMonitorModule.command("start"|"save"|"discard")` is an RPC, so
  scripted collection needs no Quest or keyboard.

## Status 2026-09-07: T5b done, T6 done, T7 blocked on the GPU

Hub `/home/mustafa/dimos-wt/hub` is at fd94f07ec: T5b (21d428cc9) plus
seven commits from a takeover session on `feat/openyam-sim-completion`.
Hub-checked today: 734 CPU-side tests green (MuJoCo rendering tests
excluded, the GPU is down), no drift against main or CC's tip, shared-code
edits small and sane (gripper tasks drop a preempted target, Pink IK
reports which joint violated its limit, isolated policy process inherits
global config, recorder gains a RESUME mode).

The takeover session's own docs are the operating manual now:
`docs/demos/openyam-simulation.md` (setup, stage commands) and
`docs/demos/openyam-validation.md` (evidence table, resume and train
commands). Evidence lives in `/home/mustafa/dimos/recordings/openyam-completion`.

Where this gets us:

- Classical bimanual sequence with cameras: 5/5 cycles, both bottles fully
  inside the bin, sustained lift checked. `reset_scene` MCP tool waits for
  scene settling (a scan right after reset used to cache pre-settle
  geometry). Local fixed-response agent playback 5/5.
- T5b root causes, all fixed: arm limits hardcoded from the MJCF disagreed
  with the URDF; pregrasp offset went along the wrong tool axis (into the
  table, nothing caught it because the world had no obstacles); the grasp
  frame is 5.9 cm from the fingertips so a `{side}_tcp` frame was added;
  one candidate with no fallback. Grasp generator now emits ranked yaw
  candidates and pick/place walk them.
- T6: real ACT checkpoint ran in the isolated CUDA process on all three
  cameras, moved 14 joints, stop RPC 3.7 ms, classical reset worked after.
  Policy task priority 30 over planner/grippers at 20. Blueprints:
  `dual-openyam-sim`, `dual-openyam-sim-pick-place`, `dual-openyam-sim-agent`,
  `dual-openyam-sim-policy-agent`.
- T7: 47 physically successful takes recorded, 34 pass the strict timing
  gate, 34-episode LeRobot export done, 20-step CPU training smoke passes
  the checkpoint round trip. Then the GPU fell off the bus (Xid 79, node
  reboot required). No trained policy exists yet.
- The 13 rejected takes all failed on one or two 130-167 ms camera stalls.
  Decision for T7 v2: switch the sim profile to `mode="fill"` with a cap on
  filled frames, re-export to about 47 valid, resume for the shortfall.
  Prompt in handoffs.md "T7 v2".
- Cameras for the dataset are 320x240 at 30 Hz capture, 15 Hz alignment
  and control; this supersedes the 640x480 at 15 Hz plan.
- A live-model agent acceptance run (`tool_check_agent --episodes 5`) has
  not happened. The takeover session blocked itself on sending prompts to
  the OpenAI endpoint. The McpClient default model is `gpt-5.6-luna` with
  the host key; this is a normal run of the demo, do it once before the
  rehearsal.

What is left for the stage:

1. Reboot, then T7 v2 (about one hour of collection, two of training,
   evaluation to the 7/10 bar).
2. Live agent acceptance, five trials, with the bimanual prompt.
3. T8: Rerun layout, run script, fallback video, second rehearsal.

## Timeline, re-baselined 2026-09-06

| Dates | Work |
|-------|------|
| Sep 6-8 | T5b left arm + multi-candidate grasps; T6 policy wiring with a stub checkpoint; T7 generator + 100 right-arm episodes + first ACT training. All three in parallel. |
| Sep 8-9 | T6 final verify on T7's checkpoint; bimanual classical end to end after T5b lands. |
| Sep 10 | T8 begins: Rerun layout, run script, reset flow. |
| Sep 11 | Rehearsal. |
| Sep 12-14 | Fallback video, second rehearsal, Pim's scene only if everything else is stable. |
| Sep 15 | Demo. |

## Blockers right now

- LFS credentials: `git lfs pull` fails with "Git credentials for
  https://lfs.dimensionalos.com/dimensionalOS/dimos not found". Needs one
  interactive `git lfs pull --include="data/.lfs/yam_description.tar.gz,data/.lfs/dual_openyam_abc_box_v2.tar.gz"`
  with the LFS user/password, ideally with a credential helper so the
  runtime `get_data()` path works afterwards.
- Venv was behind CC's branch (`dimos_lcm.imitation_msgs` missing);
  `uv sync --extra all` was run on 2026-09-05. Re-run after every merge.

## Risks

- CC's stack is rebased often; freeze a SHA once T6 starts.
- MuJoCo grasp physics: gripper MJCF needs real contact geometry and
  friction, or the cube slides out. Grasp verification bands overlap at
  0.85-0.95 open, so verify by object displacement in sim, not readback.
- Three rendered cameras block the sim thread; drop resolution/rate first.
- #3944 rewrites the pick/place module; land T4 before it or on top of it,
  not in parallel.
- ACT from 100 sim episodes may be brittle; keep it as the bonus.
