# R1Pro ACT / house simulation handoff

Updated: 2026-09-10
Branch: `feat/r1pro-act-sim`
Worktree: `/home/mustafa/dimos-wt/r1pro-act-sim`
Base: `hackmit/t7-resume` at `0b816edf49`; initial diagnostic commit `eb2efd8d03`.

## Active extension: physically carried tray (2026-09-10)

The user successfully replayed `policy-house`, then requested a supported tray,
two-handed pickup after ACT loads the bottle, and delivery onto the actual house
table beside the laptop. The previous mobile tray is a fixed child of `base_link`;
that is a shortcut, not a two-handed physical grasp. Preserve the old working ACT
checkpoint and baseline while implementing the new free-body scene.

Implemented: `tray_sim.py` adds a free tray, rectangular handles and friction
refinement (`noslip_iterations=20`); no welds/attachments or actual object-pose
writes after reset. `tray_motion.py` plans bimanual waypoints. `tray_task.py`
scores actual four-pad forces, support contacts, tilt and release, and derives
the laptop tabletop from a downward ray. `tray_delivery.py` orchestrates the
new coordinator `tray_manipulation` task and `base_transport` after ACT stops.
Native launcher option: `--deliver-to-laptop` (requires house package).

Physical evidence so far:
- `tray-dev/probe-5`: successful contact-only lift, bottle contained.
- `jobs/tray-checks/physics.log`: nine passing new/existing physics tests,
  including physical carrying and a release test proving the tray falls when
  both hands open away from the table.
- `tray-dev/house-3`: successful physical carry through the narrow kitchen exit
  to the laptop desk, all four pad contacts and bottle containment retained.
  Turns near the workbench, then travels with yaw -pi/2. Placement reach failed.
- `tray-dev/house-8`: restored that arrival for a placement experiment; successful
  release onto the actual tabletop, final tray approx (-1.9877,-2.9463,.768),
  bottle contained and settled. This is NOT yet one complete fresh native run.
- `tray-dev/house-1..7`: retained failed geometry/reach/contact experiments;
  do not count them as successful complete deliveries.

Current destination is the clear front-left part of the actual laptop table:
geom `43ae5bbfa16a74dbc82d29041846e0b92ccafabc-articulated-001_collision_000`.
Target derives from its bounds (left edge +.20m, front edge -.18m, top .768m),
base approaches from the north with yaw -pi/2 and .48m reach. Earlier side
approaches hit furniture; overextending the arms/torso was unreliable.
Placement approach clearance is +.07m. The arm planner now chooses nearby IK
solutions and checks interpolated arm paths against environment geometry before
execution. A blocked-path regression proves rejection preserves actual state.

Selected assumption: ACT handles the bottle; coordinated trajectories handle
the tray. No learned bimanual policy has been trained. An optional user question
about also training the tray task remains unanswered.
The old `policy-house` scored only 2/3 on the changed tray scene (5000 failed,
5001/5002 passed). It is preserved. The detached pipeline completed with exit 0:
`jobs/tray-finetune/run.sh` + `job.log` + `pid` + `exit-code`.
Thirty successful new demonstrations (7000–7029) are in `raw-free-tray-30`;
`dataset-free-tray-30` and `train-free-tray-1000` hold data/optimizer state.
`policy-free-tray` is the separate new deployment, 30-action chunks.
`eval-free-tray-1000/result.json`: **10/10** physical bottle successes,
seeds 5000–5009. Training and evaluation are no longer running.

Native integration evidence is under `jobs/tray-native-N/` and `native-tray-N/`:
1. Type-only ModuleProxy import failed before sim creation; fixed with TYPE_CHECKING.
2. Old policy seed 5001 hit 351ms camera skew while fine-tuning was starting;
   no overlap/orphan was present. Camera tolerance was not relaxed.
3. New ACT succeeded (8 chunks), then contact monitoring caught a wide left-arm
   approach hitting furniture. Choosing nearby IK solutions fixed that path.
4. ACT, lift and full carry succeeded, but settling timed out at the desk due
   to torso oscillation. The grasp and bottle containment remained intact.
5. Added loaded-torso damping (velocity coefficient 80 instead of 20 after ACT)
   and paused unused sensor-camera rendering; the full viewer remains active.
   This exposed a phase handoff that briefly removed grip preload.
6. Handoffs now preserve previous commanded positions (especially .009m finger
   commands while measured openings are about .012m). Unit regression passes.
   Lift was steady (tray speed norm .000408, all four loaded pads), but 5cm A*
   cells missed a narrow valid passage. A planning-only reproduction from its
   commands found a collision-free route with a 2.5cm grid.
7. The finer grid found the route, but native-tray-7 lost its bottle after a
   wall-time trajectory advanced ~12 seconds ahead of lagging physics. Measured
   base speed jumped from ~.05 to 1.8 m/s. Do not loosen containment checks.
8. Added opt-in per-joint position-target slew limits per physics timestep in
   MujocoEngine; free-tray base limits are .1 m/s XY and .15 rad/s yaw. The
   delivery runner now executes smooth segments individually and waits for
   measured settling before the next segment. A physical-time jump regression
   and 20 other shared simulation tests pass; scoped 10-file mypy passes.
   native-tray-8 stalled at the second policy RPC and failed cleanly after the
   stop timeout. native-tray-9 subsequently completed all 41 stages with
   success=true and exit 0. ACT loaded the bottle; four loaded finger pads
   remained in contact throughout transport; max base XY speed .09914 m/s,
   max carrying tilt .05099 rad. Tray released onto the actual desktop at
   (-1.987915,-2.957273,.767992), all hand contacts gone, velocity norm 1.9e-6,
   bottle contained/settled, robot obstacles empty. Full result includes final
   snapshot. native-tray-10 also completed with success=true and exit 0 on
   seed 5001: eight ACT chunks, all four pads held throughout transport, no
   obstacle contacts, peak XY speed .08581 m/s, peak tilt .05005 rad. Final
   tray (-1.987898,-2.957170,.767992), supported/released/settled, bottle inside.
   Native tests use 224.0.0.224:19468, seed 5000, no --stay-open.

User steering: a nearby kitchen counter or stool is an acceptable destination
instead of the laptop desk if the passage is difficult. Counter top is .904m
(27e2... kitchen island); two adjacent stools have seats near .49m. Temporary
/tmp/r1_counter_* probes are checking reach. The initial vertical wrist pose
cannot reach counter height; pitch -pi/2 can, but transition must preserve
physical grip. The loaded wrist rotation retained both hands after preserving
preload, but tilted the tray beyond .25 rad; rejected without relaxing criteria.
No counter/stool option is shipped. The request was conditional on the passage
being too narrow; the original laptop route now succeeds after fixing control
timing, so fallback is unnecessary. Communicated this result and retained the
original destination. No production wrist/torso experiments were applied.

`simulation_snapshot()` now records read-only qpos/qvel/ctrl and actuator damping
at pickup, arrival, and final placement, for reproducible diagnostics/rendering.
Actual simulation object poses are never overwritten by the delivery runner.
Native viewer tracking is optional `viewer_track_body` on the shared engine and
module. `set_camera_streaming_enabled()` pauses only offscreen policy sensors;
physics and the native viewer continue, and sensor rendering can resume.

Checks: `jobs/tray-checks-2` has 10 passing contact/path tests and eight-file
scoped mypy success. `jobs/tray-final-checks` had 17 passes plus a test typo in
the new camera pause test; corrected public `read_camera()` call now passes all
four timing/camera-pause tests (`timing-final.log`). Ten-file scoped mypy passes.
Final `jobs/tray-release-checks`: 32 physics/shared simulation tests pass,
including phase preload and slew/reset checks; 10-file scoped mypy passes.
`jobs/tray-evidence`: six CI-mode registry tests and isolated evaluator mypy
also pass. Native runs 9 and 10 exited with all workers shut down. Saved
read-only snapshots render the actual arrival/final state. Rendering exposed
that the package's laptop meshes default to hidden group 3; loaded-tray setup
now shows them in group 2 after ACT stops. No geometry/contacts/poses changed.
Images are in native-tray-9/delivery-{arrival,final}.png. All training and
validation jobs finished; no R1Pro demo is left running. Implementation and
validation are complete; the source commit includes this handoff and run guide.

All long tuning and validation jobs are detached with saved logs/PIDs. Preserve
OpenYAM PID 3599494 and never run global dimos stop. Do not add `.venv`, recordings
or MUJOCO_LOG.TXT. All source remains in the isolated R1Pro worktree.
The main checkout is `cc/feat/unbounded-planar-base`; its R1Pro planar preview uses
a mock adapter. This ACT worktree uses its own MuJoCo XY/yaw stage with matching
joint names, not that preview/planar-model implementation. No merge performed.

## User replay failure and launcher fix (2026-09-10)

The user saw the assistant's visual run succeed, then copied the README command
while the assistant-launched R1Pro process (PID 1176767) was still alive under
`--stay-open`. Both used `224.0.0.224:19467`. The first user attempt failed at
startup with 142.6 ms camera skew; the next stopped after one ACT chunk with
388.7 ms wrist/head skew, above the unchanged 20 ms limit. Evidence is the user's
pasted terminal output and `recordings/r1pro-act-task/my-house-run/result.json`.
This is a launch/resource conflict, not evidence that training resumed or that
the successful learned task was a scripted placeholder.

Stopped only the old assistant-launched R1Pro after checking its exact command
and output path; OpenYAM PID 3599494 was preserved. `sim_session.py` now reserves
the messaging address and resolved output path with OS-backed FileLocks before
scene creation. A duplicate receives the owning PID and a clear exit status 2;
its own scene is never created. Locks remain held through worker teardown and
are released on failure too. The path lock protects motor shared memory even if
a second launch uses a different messaging address. Closing the native viewer
after `--stay-open` now exits the demo and shuts down its workers.

An original isolated ACT process also survived the overlapping starts: uv PID
1177509 and Python PID 1177573, instance
`__isolated_python__/PolicyRolloutModule/70UNBqecM78StV88DL`. The old viewer log
shows later start/stop RPCs from the user's new launches reaching the old stack,
including another native child being started. Terminated only that verified
original orphan process group. The launcher also now treats an active/error
status returned by `stop_rollout` as a failure instead of printing that ACT
stopped and entering the viewer wait.

Final verification passed in the detached job
`recordings/r1pro-act-task/jobs/launch-fix/` (`verification-exit-code` is 0):

- Four focused reservation tests and mypy on three affected production files.
- First native house pick/load/carry: success, eight accepted chunks, 11.45 cm
  lift, ACT stop 4.29 ms.
- Real duplicate CLI launch: exit 2, identifies the original PID, creates no
  second scene/output, and leaves the running demo intact.
- Closing the R1Pro window through its window-manager close protocol caused
  clean worker shutdown and process exit 0; OpenYAM was preserved.
- Immediate restart on the same address AND output: complete house task
  succeeded again, eight chunks, 11.81 cm lift, stop 158.52 ms, exit 0.

The diagnostic allowed 90 seconds to capture a potential stall; both successful
manipulations finished in about 12 seconds, within the README's unchanged
25-second default. No checkpoint, camera skew tolerance, or physics setting was
changed. Two earlier verification attempts stalled on the second motion RPC;
their evidence is retained in `before-orphan-cleanup/` and
`after-orphan-cleanup-stall/`. No separate core RPC fix is claimed. If that stall
recurs without overlapping/orphaned processes, capture the runtime/coordinator
thread stacks before its stop timeout and investigate the transport separately.

The runbook now explains how to stop a completed demo before replaying it.
Final verification closed its own R1Pro viewers; the user's next launch owns the
screen and messaging address.

## Current outcome

**Actual trained ACT manipulation and native house transport have passed.** The
user explicitly wanted a trained manipulation task, not the earlier untrained
wrist-motion diagnostic. That scope correction is now implemented and evaluated.

R1Pro uses its right gripper to pick up the blue bottle and release it into an
orange onboard tray. After ACT stops, a collision-checked coordinator trajectory
moves the simulated planar base through the HSSD house with the bottle retained.
The user can watch the full native GLFW display. No LLM/API key is needed.

The trained task is restricted to this known workstation and a bottle start
jitter of +/-12 mm in XY. Base transport is scripted planar-stage simulation,
not learned navigation or calibrated wheel/steering dynamics. Arbitrary house
objects, navigation to multiple workstations, destination unloading and the
user's real locomanipulation integration remain future work.

## Run or resume work

See `dimos/robot/galaxea/r1pro/ACT_SIM.md` for the complete native launch command,
module composition, training/evaluation commands and background-job recovery.
Use `recordings/r1pro-act-task/policy-house` for the full house demo; use `policy`
for the simple tabletop. The old `r1pro-act-check/checkpoint` is diagnostic only.

The verified house command uses:

- Scene package `/home/mustafa/dimos/data/scene_packages/hssd_102344115`.
- `--mobile --seed 5000 --transport-x -0.35 --transport-y -1.0`.
- Separate multicast address `224.0.0.224:19467` and a unique output directory.
- `--stay-open` to retain the full viewer after completion; Ctrl-C closes this
  stack. The result is saved before waiting in the viewer.

Preserve the user's main checkout (`cc/feat/unbounded-planar-base`) and existing
OpenYAM daemon (PID 3599494, confirmed alive after the work). Do not run a global
`dimos stop`. No R1Pro training or evaluation job remains active at this handoff.
The successful native R1Pro stack shut down after its run.

## User instruction: background work must survive disconnect

The user explicitly requested that training, tuning and all long jobs continue
on the GPU host if their client PC or terminal disconnects. For future work,
launch a saved script using `nohup setsid --fork`, stdin from `/dev/null`, and
stdout/stderr redirected to a persistent log. Save PID and completion exit code;
retain training checkpoints and atomic per-episode collection manifests. Do not
rely on an interactive tool session or an attached shell for a long job.

This was exercised with the remaining final checks:
`recordings/r1pro-act-task/jobs/final-checks/run.sh`. After its launch command
exited, its process had PID/SID 1124663 and no controlling terminal. All checks
finished and `exit-code` contains 0. Logs and individual statuses are in that
same directory. Earlier training and physical evaluation had already completed
when the user requested this; their artifacts are safely on disk. No redundant
training run was started. The `next-house` job in the runbook is an example,
not an active run.

Detached jobs survive a client disconnect, not GPU-host shutdown. On host
restart, use the checkpoint's `pretrained_model/train_config.json` with
`--resume=true` in another detached job. Collection resumes from its manifest.
Graphics-dependent collection/evaluation using GLFW also needs the host's X
session to stay available. Training itself does not depend on a viewer. New
agent decisions require reconnecting even while previously launched jobs run.

## Implementation

- `grasping_sim.py`: pinned Galaxea R1Pro CAD; tabletop/free bottle/tray;
  optional full HSSD composition and actuated planar XY/yaw base.
- Grippers: opposing 0-0.05 m finger travel, symmetric joint coupling, simple
  flat fingertip pads and bounded position servos. CAD has zero finger limits;
  travel comes from the Galaxea G1 reference, while contact geometry/gains remain
  explicit simulation approximations. No object welds or scripted pose writes
  after reset. The onboard tray clears the table by 2 mm.
- `grasping_task.py`: real physics, teacher IK/contact demonstrations, two RGB
  observations, measured state, repeatable reset and physical success scoring.
- `grasping_transport.py`: bounded A* translation planning in a copied MuJoCo
  model/state, obstacle checks, and smooth transport targets. Chassis ground
  contact is excluded because the planar stage fixes height/tilt; obstacle
  collisions remain. This does not merge/replace ongoing locomanipulation code.
- `learning.py`: separate 20-joint `r1pro-sim-pick-place-v1` profile, head/wrist
  RGB 160x160, 20 Hz absolute commands. Free props and follower fingers are
  excluded from actions. The old 18-joint diagnostic profile remains compatible.
- `grasping_blueprint.py`: real DimOS simulation, shared-memory whole-body
  adapter, coordinator, isolated LeRobot ACT runtime and PolicySkills.
  Manipulation controls 20 joints; optional transport controls three more.
- `demo_pick_place_stack.py`: native viewer, policy readiness, actual ACT task,
  physical checks, cancellation, optional house carry, JSON evidence and cleanup.
  Failed task completion returns a failing process status.
- `demo_collect_grasping.py`: resumable successful NPZ demonstrations, atomic
  manifest updates and matching scene/profile/randomization checks on resume.
- Isolated `prepare_r1pro_dataset.py`: standard LeRobot conversion with stable
  normalization. `prepare_r1pro_deployment.py`: immutable deployment copies with
  recorded original weight checksum and explicit 30-action inference setting.
- Isolated `demo_r1pro_pick_place.py`: real-policy physical evaluation, optional
  native viewer and overview videos. Ground truth is used for evaluation only.
- Standard imitation workflow `r1pro-sim-lerobot`; Space start/stop and Q quit.
  It requires the normal CLI's idle-coordinator condition, so use the independent
  demo command while the current OpenYAM stack remains running.
- Small shared changes: trajectory limits tolerate float32 roundoff only (e.g.
  0.05 m); native viewer optionally starts with robot-focused lookat/distance.
  Earlier diagnostic commit fixed simulation-clock/render scheduling.

## Training and physical evidence

All paths below are under `recordings/r1pro-act-task/` and remain Git-ignored.

1. `raw-60` / `dataset-60`: 60/60 successful tabletop demonstrations, seeds 0-59,
   222 frames each (13,320 total). Training split: 54 train, six validation.
2. `train-stable-4000`: actual ACT, 4,000 steps, ResNet18 pretrained backbone,
   width 256, FF 1024, eight heads, encoder/decoder/VAE depths 2/1/2, chunk 30,
   batch 16, four workers, LR 1e-4 and backbone LR 1e-5.
3. `policy`: unchanged 4,000-step weights, inference `n_action_steps=30`.
   Weight SHA256 `4716f34aa70247b491cc8461f2f1241aaf7d7c1456d9e6c3864e4f1fdaec5051`.
   `eval-stable-4000-chunk30/result.json`: **20/20** physical tabletop successes,
   new seeds 1010-1029.
4. `raw-house-30`: 30/30 successful house demonstrations, seeds 100-129, 6,660
   frames. `raw-mixed-40` hard-links the first 20 tabletop and first 20 house
   episodes with domain metadata. `dataset-mixed-40`: 8,880 frames, 36 train/four
   validation episodes.
5. `train-mixed-1500`: fine-tuned from `policy` for 1,500 steps with LR 5e-5,
   backbone LR 2e-5. `policy-house` is its final unchanged weight deployment.
   Weight SHA256 `8c4abe961fa5c9892d2a9ba5f2e5330af5693d65a8316d40a9432b64d18e6e2a`.
   `eval-mixed-final-house/result.json`: **10/10** physical house successes,
   new seeds 4000-4009, after the final tray/table clearance correction.
6. `native-house-complete/result.json`: **full native DimOS + CUDA + GLFW
   manipulation and transport succeeded**, seed 5000. Seven accepted ACT chunks,
   0.116686 m peak lift with bilateral fingertip contact, released and settled
   inside tray. ACT stopped in 0.008016 s. Base carried the bottle to
   (-0.35, -1.0), about 1.06 m displacement, with no reported obstacle contacts.
   Final bottle world position (0.006910, -1.006823, 0.786977); still released,
   inside tray and settled. Final simulation time 27.996 s. Workers shut down.

Success requires lift >6 cm, bilateral fingertip contact while lifted, complete
tray containment, open/no-contact gripper, low bottle motion and one-second
stability. Transport checks cargo retention, obstacle contacts and settled
arrival. Object poses are never part of ACT input.

## Defects found and resolved during training

Do not reuse `train-4000` as the final model. It predates the normalization fix:
LeRobot float32 reduction put constant command means outside their min/max while
returning zero variance. Tiny inactive-joint vibration was also amplified.
Converter statistics now use float64 accumulation and unit scale for joints
whose demonstration commands never change. Original statistics are preserved
locally in `original-lerobot-stats.json`.

Partial chunk execution drifted during the approach: a corrected 2,500-step
checkpoint scored 2/5 at eight actions, 2/5 at 16, and 5/5 at all 30. Final
4,000-step evaluation uses 30 actions. House images required house adaptation;
the final mixed-data model was chosen by physical outcomes, not loss alone.

The first mobile scenes had a table leg overlapping the chassis and a tray
bottom coplanar with the worktop, causing scrape/oscillation/tipping. Moving the
near table legs and giving the tray 2 mm clearance resolved those contacts.
Final physics tests, 10 house evaluations and the combined native run use the
corrected geometry.

## Verification

- Seven asset-backed contact/reset/mapping/mobile-cargo physics tests passed.
- 48 trajectory-task tests passed, including float32 boundary acceptance and
  actual limit violation rejection.
- Ten isolated runtime/profile/normalization tests passed.
- Earlier host workflow/CLI/policy checks had 34 passes plus one expected registry
  failure while its generated update was uncommitted. The final CI-mode registry
  comparison subsequently passed all six tests; generated content is current.
- Final detached check: all 18 simulator timing/reset/camera tests passed.
- Final detached check: host mypy passed nine production files; isolated mypy
  passed four runtime/dataset/deployment/evaluation files from the project cwd.
- Logs: `task-tests.log`, `trajectory-tests.log`, `isolated-tests.log`,
  `host-tests.log`, and `jobs/final-checks/`.
- Python 3.12 shared-memory resource-tracker bookkeeping warnings occurred on
  earlier native teardown. They are separate from the successful physical task;
  do not describe them as a policy failure or claim they have been fixed.

## Repository and push status

All source work belongs in this isolated feature branch. Do not add the shared
`.venv` symlink, generated `MUJOCO_LOG.TXT`, recordings, weights or cached vendor
assets. Activate `.venv` before commits; keep normal hooks enabled. The host
Python environment and root dependency lock have not been changed.

No push completed. Automatic approval review previously rejected pushing
`hackmit/t7-resume` to `git@github.com:dimensionalOS/dimos.git`, citing exact
payload/destination authorization and repository trust. A question naming that
branch/destination/commit `0b816edf49` remains unanswered. Do not bypass the
rejection. The R1Pro source is also local until an authorized push completes.
A source push alone does not transfer the local training artifacts.


## 2026-09-10: Five-bottle ACT packing in progress

User chose planned neat placements and stop when full, with no rearrangement.
Tray grasp learning and navigation changes are deferred. Existing single-bottle
and tray-delivery demos remain separate.

New source (not yet committed): `packing.py`, `packing_sim.py`, `packing_task.py`,
`demo_collect_packing.py`, typed `VectorObservation`/`VectorSource` support,
`R1PRO_PACKING_IO`, packing dataset conversion/initialization, and isolated native
viewer evaluator `dimos_lerobot.demo_r1pro_packing`.

The new policy receives RGB + 20 measured joints + an eight-value simulator
geometry vector (selected source XYZ, destination XYZ, radius, half-height).
This explicitly differs from the older image/joint-only policy. Original 5 cm
wide, 14 cm tall bottles fit in the original 21 cm square tray. Local overlay
moves the tray to (0.30, -0.10) and places five sources within right-arm reach;
house package files are untouched. No object poses change after reset.

Joint-interpolated demonstrations failed randomized orders because the gripper
swept through the tray. The revised teacher follows Cartesian paths, releases
at TCP Z=0.855 to clear the wall, and uses torso damping 80 in the new packing
scene. Four randomized full sequences (8100–8103, ±3 mm position jitter) passed
all five physical picks and final containment, with no rejected sequences.
Evidence: `recordings/r1pro-act-task/packing-physical-cartesian/raw/manifest.json`.
These are TEACHER results, not learned ACT results.

Detached pilot started at PID 1403319:
`recordings/r1pro-act-task/jobs/packing-pilot/run.sh` / `run.log`, `stage`, `pid`,
`exit-code`. It collects 20 successful house scenes (100 pick episodes) starting
at seed 8200, converts data, initializes the new ACT environment token from
`policy-free-tray`, trains 5000 steps, and evaluates 10 fresh seeds 9000–9009.
At this update collection is running; no packing checkpoint is trained or
validated yet. All jobs are detached from the terminal. Do not relaunch a
second pipeline over these outputs. Watch stage/log and inspect physical
outcomes before calling this complete. Planned outputs: `raw-packing-20`,
`dataset-packing-20`, `init-packing-20`, `train-packing-5000`,
`policy-packing-pilot`, `eval-packing-5000`.

31 focused planner/profile/runtime tests passed. Host mypy identified a context
manager return type issue; corrected to Self, recheck pending. Native DimOS
ControlCoordinator integration for multi-pick selection is not yet implemented;
the current new evaluator uses the same LeRobot backend directly with a native
MuJoCo viewer. The old single-bottle stack remains available. No push performed;
prior automatic approval rejection remains unresolved.


### Packing update: clear source order and native wiring

The first house trial with unconstrained random order failed because carrying
rear source 5 could knock over still-unpicked source 2. Owned job PID 1403319
was stopped. `clear_pick_order()` now chooses among accessible bottles while
requiring front bottles to be cleared first. No object dimensions were reduced.
The corrected teacher passed **20/20 full scenes, 100/100 picks**, seeds
8200–8219, ±3 mm jitter, no rejected scenes:
`recordings/r1pro-act-task/packing-physical-clear/raw/manifest.json`.

Replacement detached pipeline PID **1410041**:
`recordings/r1pro-act-task/jobs/packing-clear-pilot/{run.sh,run.log,stage,pid,exit-code}`.
Outputs now use `raw-packing-clear-20`, `dataset-packing-clear-20`,
`init-packing-clear-20`, `train-packing-clear-5000`,
`policy-packing-clear-pilot`, `eval-packing-clear-5000`.
Collection is running (20 successful house scenes required). A separate
technical smoke test PID 1441712, `jobs/packing-smoke/`, converts one completed
scene, initializes the goal token and runs two training steps. Its checkpoint
is NOT a learned packing skill; this only checks the training pipeline.

Native DimOS wiring is now implemented but not yet run: `packing_blueprint.py`
with `R1ProPackingSim`/`R1ProPackingPolicy`, `demo_packing_stack.py`. The existing
builder delegates common hardware/camera/coordinator wiring to
`build_r1pro_manipulation`; existing single-bottle callers keep the same API.
`PackingMonitor` tracks per-bottle contact/lift evidence on physics steps.
Goal vectors share the head-camera timestamps. Between picks,
`clear_rollout_observations()` discards stale prior-goal inputs before preflight.
33 focused tests pass; host mypy found a disposable wrapper type issue, fixed
with `Disposable(...)`; rerun pending. Registry generation, isolated typing,
physical regression, learned evaluation and repeated native runs remain.


### Training and integration checkpoint

Smaller early feedback pipeline PID **1446801**, `jobs/packing-early/`, snapshots
four complete scenes into `raw-packing-early` (20 pick episodes), holds one scene
out (`eval_split=.25`), trains 3000 steps and evaluates seeds 9100–9102.
Its current stage is training. Larger job PID 1410041 continues collecting
`raw-packing-clear-20` (six scenes completed at this update, zero rejects).
Do not confuse either job with the completed two-step technical smoke test.

Both native preflight tests passed with CUDA, synchronized goal inputs and zero
motion chunks. Latest: `native-packing-preflight-2/result.json`, exit 0. Added an
initial 0.6 s physics-settle gate so the first goal uses settled source/floor
heights, and a timestamp floor in `clear_rollout_observations()` so delayed
prior-goal packets cannot re-enter the buffers after clearing.

Ten physical regression tests passed, including two full five-bottle sequences
and old single-bottle/mobile-task tests. 33 focused tests passed; 20 combined
registry/runtime tests passed after regeneration (normal generator first reports
uncommitted output, CI=1 comparison passes). Host mypy passed eight integration
files; isolated LeRobot mypy passed all five changed/new files. New operator doc:
`dimos/robot/galaxea/r1pro/BOTTLE_PACKING.md`, linked from ACT_SIM.md. It explicitly
marks learned validation pending. Source changes are still uncommitted.

Next: inspect the early learned evaluation; adjust training/chunk settings from
physical evidence; finish the larger pipeline and evaluate fresh seeds. Run the
trained checkpoint repeatedly through `demo_packing_stack` with the full native
viewer, inspect final geometry visually, and update docs/results before committing.
Pre-commit and final broader checks still pending. No push is authorized after
the earlier automatic-review rejection; do not bypass it.


### Learned pilot failure and conditioning correction

The four-scene pilot finished 3000 steps. **0/3 full learned sequences passed**
on seeds 9100–9102, both at 20 and 30 executed actions/chunk. Reports:
`eval-packing-early/result.json`, `eval-packing-early-30/result.json`. Failures
include selecting the wrong bottle, inaccurate grasps and not releasing; no
native learned rollout should be presented as ready. A direct same-observation
probe showed changing source XY barely changed predicted approach. Checkpoint
ENV feature/normalization configuration is correct, so this is learned goal
use, not a missing stream or profile mismatch. Diagnostic:
`jobs/packing-goal-probe/run.log` (kinematic predictions only).

New supplementary collector `demo_collect_packing_choices.py` generates three
successful first-pick episodes from exactly the same reset scene, selecting
bottles 1, 2 and 4 (the initially accessible sources). These are explicitly
single-pick demonstrations, not successes at packing all five. The first pair
was verified to have bit-identical initial RGB and joint state, different goal
vectors and different teacher approaches. This removes the scene-appearance
shortcut for identifying the requested bottle.

Detached collection PID **1462561**, `jobs/packing-choices/`, writes
`raw-packing-choices` (10 layouts × 3 choices, seeds 8300+). Converter accepts
verified `choice_groups` as supplementary demonstrations. Main job 1410041
continues unchanged (11 complete house scenes at the last count, no rejects).

Queued conditioning run PID **1470063**, `jobs/packing-conditioned/`, waits for
4 complete choice groups and 8 complete full sequences. It snapshots a combined
manifest (`raw-packing-conditioned`) with choices first, full scenes last, so
`eval_split=.09` holds out exactly the last full scene (5 of 52 episodes).
Then it converts, fine-tunes from `policy-packing-early` for 8000 steps at LR
5e-5/backbone 1e-5, prepares `policy-packing-conditioned` with 30-action chunks,
and evaluates seeds 9100–9102. All job scripts/logs/PIDs/exit files persist.
No new architecture variation (e.g. disabling VAE) has been applied.

Next agent should inspect these jobs and physical learned results before
choosing a final artifact. The original user task is not complete: five-bottle
ACT success, repeated native learned runs, final docs, and commit are pending.
Mixed shapes remain deferred; original bottle geometry is retained. Do not
claim the 20/20 teacher results are learned policy performance.


### Paired conditioning in progress; stricter neat-placement checks

Conditioned run 1470063 is training (approximately 3600/8000 steps at 14:40 PDT).
A direct probe of its 2000-step checkpoint now produces clearly different
reaches when only the requested source changes; errors still reach several cm,
so this is diagnostic progress, not learned pick success. Main collection has
16 full successful scenes; supplementary collection has 8 choice groups.

Packing-specific scoring now also requires upright orientation within 15 degrees
of the tray normal. Shared single-bottle scoring remains unchanged. The slot
planner conservatively expands the footprint of tilted bottles. Five packing
physics tests pass, including both complete teacher sequences under stricter
criteria, rejecting a contained lying bottle, and refusing a placement without
mutating state when a fallen bottle blocks all slots. Focused Ruff passes.
Default host mypy follows unrelated imports and reports 10 pre-existing errors
in five modules; a focused `--follow-imports=silent` check is being used for the
changed packing sources.

Additional owned diagnostic jobs: `jobs/packing-native-lifecycle` PID 1478985
uses the known failing early checkpoint for six seconds solely to exercise
native ACT trajectory/stop plumbing, on isolated bus 19468. Do not score it as a
trained demo. `jobs/packing-conditioned-mid` waits for checkpoint 004000, prepares
`policy-packing-conditioned-mid`, and physically evaluates three tuning scenes
9100–9102 while the 8000-step run continues.


### First learned picks, native timing fix, full-data continuation

The 4000-step paired checkpoint completed first picks in seeds 9100 and 9101,
then failed the second pick; 9102 failed its first pick. Full learned score is
still **0/3**. See `eval-packing-conditioned-mid/result.json`. This establishes
some learned goal use but does not satisfy the task.

Native lifecycle 1 failed after one chunk with a transient 431 ms camera skew.
The runtime now waits up to `max_observation_age_s` for a complete matching input
set at each inference, preserving the existing strict age/skew limits. Any valid
input wakes the condition variable, and stop interrupts it immediately. All 17
runtime tests pass. Native lifecycle 2 then completed the first ACT pick and
started the next; the second physically failed as in the offline test. Both
rollouts accepted 11 chunks and stopped with `active=false`, `last_error=null`;
the process exited 0. Result: `native-packing-lifecycle-2/result.json`.
Shared-memory resource_tracker teardown KeyErrors remain visible; no claim of
fixing those or the earlier unreproduced RPC stall is made.

Full-data continuation `jobs/packing-full-conditioned` is queued. Current PID
**1489438** (supersedes its never-started-training PID 1488083). It waits for all
20 full scenes plus 10 choice groups, then stops only the owned old baseline
job 1410041 so we do not spend another GPU round on unpaired data. It snapshots
130 successful picks, converts `dataset-packing-full-conditioned`, and waits for
`packing-conditioned` to finish. Then fine-tunes from its 8000-step checkpoint
for up to 20000 steps, batch 32, 8 loaders, LR 5e-5/backbone 1e-5. Last two complete
scenes (10 episodes) are held out by `.077` split. Saves every 5000 steps so we
can physically evaluate and choose an earlier checkpoint if it performs better.
Final planned paths: `policy-packing-full-conditioned`,
`eval-packing-full-conditioned` (fresh 9200–9204). All exact scripts/logs are saved.
The source task remains incomplete until full learned sequences and native runs
pass; docs deliberately mark the proposed command as pending validation.


### Implementation checkpoint before full-data training

The 8000-step paired pilot finished with eval loss 0.0094, but still **0/3 full
physical sequences** (9100–9102). Earlier first-pick successes are insufficient:
failed later motion can also knock already-placed bottles out. The eight-scene
training subset contained no bottle-2-as-second-pick examples. The complete
20-scene dataset contains seven such sequences and covers more source/slot
combinations. Do not conflate the low supervised loss with physical success.

All collection finished: 20 full scenes / 100 picks, plus 10 paired-choice
groups / 30 single picks. The old unpaired baseline job is superseded after
collection (see its `superseded-by` file even if its shell exit code is 0).
Full-data job PID 1489438 finished conversion and is waiting for the pilot to
exit before training. Its 5000/10000/15000-step physical tuning evaluations are
also detached: `jobs/packing-full-checkpoints`, PID **1490960**, seeds 9100–9102.
Fresh final validation must use unused seeds (e.g. 9400+) after choosing a model.

Focused host mypy passed 16 changed/new files with `--follow-imports=silent`;
isolated mypy passed five LeRobot files with the same import setting. Ruff and
`git diff --check` pass across all changed Python source. Saving source in a local
implementation commit with explicit pending-validation documentation; trained
weights/data remain ignored local artifacts. Final learned validation, repeated
native full-viewer runs and final operator documentation are still outstanding.


### Full-data split correction and active job IDs

Implementation commit: **95bdae0bc** (29 files; all normal pre-commit hooks pass).
The first larger run logged 119 training episodes: LeRobot rounds the held-out
fraction up, so `.077` held 11 episodes and split a scene. Stopped that owned
training attempt after its first few hundred steps and preserved it at
`train-packing-full-split-119`; its scripts/logs are archived as
`before-split-fix-*`. Corrected the fraction to **.0769** and reused the completed
130-episode dataset. The new run confirms **120 training episodes / 31,680
frames**, leaving exactly two whole scenes (10 picks) for validation.

Current active detached PIDs:
- `jobs/packing-full-conditioned`: **1495914**, training 20,000 steps, batch 32,
  eight loaders, ~200 samples/s (~6 updates/s). Started 14:56 PDT Sept 10.
- `jobs/packing-full-checkpoints`: **1495915**, waiting for checkpoints at
  5000/10000/15000 to test tuning seeds 9100–9102.

Do not use older PIDs from earlier handoff entries. Inspect each saved `pid`,
`stage`, `run.log`, `exit-code` as the source of current job status. First physical
checkpoint feedback should arrive around 15:10 PDT; full training about 15:50.
Timing is approximate. No complete learned packing sequence has passed yet.


### Corrected per-bottle release reporting (after implementation commit)

Audit found that shared single-bottle scoring required an open right gripper
for **every** bottle, so closing it for the next pick made already-placed bottles
report `released=false`. Earlier `packed=0` summaries did NOT establish that those
bottles were knocked out. Correction to the earlier handoff wording: saved poses
show the successfully placed bottles remained in the tray. The 8000-step pilot
actually left **1, 1 and 4** bottles correctly packed on 9100, 9101 and 9102. Full
success remains 0/3. Original reports are preserved; exact saved-state rescore is
`eval-packing-conditioned/release-audit.json`.

`score_task` now has a backward-compatible `require_open_gripper=True` option.
Packing passes False for per-object release (no pad contact). A shared
`open_gripper_at_home()` separately gates selected-pick completion and the next
native goal, so this does not weaken the open/home requirement. Six packing plus
seven original grasp physics tests pass (13 total), including both full teacher
sequences and the new closed-gripper-elsewhere regression. Source changes since
95bdae0bc are not committed yet.

Detached horizon tuning `jobs/packing-conditioned-horizons` PID 1500248 finished:
20 and 10 executed actions/chunk both failed all three tuning scenes. Keep the
30-action baseline. Full training PID 1495914 and checkpoint watcher 1495915
remain active; step count was approximately 4400 at the latest check. Need full
learned success and final native viewer validation before claiming completion.


### First full learned sequences, completion tuning, native diagnostics

`policy-packing-full-5000` (5000 steps into the full-data continuation) passed
**1/3** complete tuning scenes with the original one-second-active completion
rule: 9100 packed 3 then missed bottle 1; 9101 packed 3 but left home again before
one-second confirmation; 9102 packed all 5. No teacher actions were used.
`eval-packing-full-5000/result.json` preserves this baseline.

A completion experiment confirms home for ~0.1 s, stops ACT, then verifies after
0.5 s held. Native uses the coordinator's normal cancellation. First offline
implementation held measured joints; it passed **2/3**, counts 1,5,5 in
`eval-packing-full-5000-stop`. First scene regressed but two full scenes passed.
An attempted broad HEAD restore was automatically rejected (potential loss of
uncommitted work); it did NOT execute. After inspecting complete experiment
results we retained the candidate and preserved its exact source/patch in
`jobs/packing-5000-stop/source/`. No tracked source was lost.

Corrected offline stop hold to retain `task.data.ctrl[task.aids]`, matching native
hardware's hold-last-command semantics, instead of introducing a measured-joint
setpoint. Current offline result rows include `stop_hold_target=last_policy_command`.
Current native launcher confirms 0.1 s, stops, waits 0.5 s, then rechecks the same
physical success/open-home constraints. Every manipulating motion remains ACT;
there is no scripted return-home or object trajectory fallback. This completion
change still needs final native validation. Its source is uncommitted.

`policy-packing-full-10000` passed **2/3** in the automatic watcher, counts 5,2,5.
Those results (`eval-packing-full-10000`) still used the earlier measured hold
(the script had loaded before the correction). Fresh same-seed evaluation with
command holding is now detached at `jobs/packing-10000-command-hold`; inspect its
saved PID/status and `eval-packing-full-10000-command-hold`.

Native full viewer baseline `jobs/packing-native-5000`, PID 1513179, reproduced
the OLD stall: first chunk accepted, second inference completed, no second
trajectory accepted, stop times out after 2 s with active=true. Result
`native-packing-5000-9102/result.json`. Do not mistake this for a model failure.

Trace attempts:
- `packing-native-trace` PID 1518699: broad sitecustomize after-fork tracing
  interfered with worker startup. Stopped this owned process group; diagnostics
  are not evidence of the original stall.
- `packing-native-trace-2` PID 1527776: timer traceback limited to isolated policy.
  Completed two picks, then isolated Python 1529017 (uv wrapper 1528981) exited
  **139** at 22:27:25 UTC. Host RPC waited 120 s and then reported runtime not
  ready. The dump ended mid-stack; tracing itself may have contributed. Do not
  assume it proves a Zenoh root cause. Shared-memory close/resource_tracker
  warnings remain at teardown. Last healthy rollout stack was waiting normally
  between chunks, not the original blocked control call.
- Current `packing-native-control-trace`, PID **1536270**, uses checkpoint 10000,
  seed 9100, full native viewer, isolated bus 19468. `/tmp/packing_trace/sitecustomize.py`
  now wraps ONLY the isolated policy's rollout thread with sys.setprofile;
  logs call/return boundaries for policy runtime and RPC methods to
  `jobs/packing-native-control-trace/threads/calls-<pythonpid>.log`. No after-fork
  hooks or timed stack dumps. Examine the last call if the stall recurs. Exact
  scripts/PIDs remain in each job. All other old native jobs have ended.

No source changes have been made to ZenohRPC yet. Its `_issue_query.on_finalize`
currently sleeps then retries synchronously inside the callback; this is a
possible reentrancy problem, not an established diagnosis. Need concrete trace
before changing transport. Native deployment is NOT reliable yet. Continue full
training PID 1495914 and watcher 1495915 (checkpoint 15000 still coming), verify
stronger models on unused seeds, fix native failures, then finalize docs/commit.


### 2026-09-10: full checkpoint, native timing diagnosis, snapshot cameras (in progress)

20,000-step continuation completed at 15:54 PDT in 58m25s. Artifact:
`recordings/r1pro-act-task/policy-packing-full-conditioned`. Fresh randomized
physical evaluation 9200–9204 passed **2/5**, counts **1,3,1,5,5**. Final held-out
loss .0063 is not a success metric. All training/checkpoint watcher jobs ended.
The 10k and 15k tuning checkpoints each passed 2/3 complete scenes (5,2,5).

Production planner now defaults to accessible sources from left to right:
4,2,5,1,3 (one-based), still front-before-rear. Native `--random-order` restores
randomized accessible selection. Collector remains randomized. Offline evaluator
remains randomized by default, with `--canonical-order` to match native. Fresh
canonical evaluation 9500–9504 passed **3/5**, counts **5,1,5,1,5**. Failures and
original random reports are preserved. This is not robust arbitrary-item ACT.

Native status is STILL NOT COMPLETE. Several runs place the first bottle but
fail the second. Native input NPZ recordings in `native-packing-inputs-9401/inputs`
prove the second goal reaches ACT correctly. Logging wall/sim time exposed a
large timing problem: physics ran at .53–.57 of real time while coordinator
trajectories used wall time. Fixed an independent bug: primary RGB-only cameras
were still rendering unused depth. Four consumer-mode tests pass.

Direct engine profiling (`jobs/packing-engine-profile`) confirmed GPU rendering
and viewer synchronization dominate. Actual GLFW renderer is NVIDIA RTX 4090
Laptop, not software rendering. A state-only viewer sync experiment performed
worse and was removed; there is no remaining state-only configuration change.
Current opt-in `background_camera_rendering` captures mjSTATE_INTEGRATION and
capture wall timestamp under the engine lock, then runs mj_forward and both
sensor renders on separate MjData outside the lock. GL resources are created,
used and closed on the camera thread. Only packing opts in; other stacks keep
the existing default. Intended for models fixed after startup. Full native
viewer remains enabled. Camera failures stop the sim; teardown signals and joins
the renderer. The full simulator test file passed 17 tests, including a blocked
render proving live physics remains unlocked and snapshot state independent.

Current native job `jobs/packing-native-background`, PID **1578410**, output
`native-packing-background-9403`, final checkpoint, full viewer, bus19468. First
pick passed with measured physics/wall ratio **.9996**. Remaining picks pending.
Need inspect its physical results and repeat full native success before claiming
completion. Fresh native jobs use distinct output directories. Do not touch
other user stacks (notably a separate `dimos` PID1527073 exists).

Temporary Zenoh trace edits were removed narrowly with a backup at
`jobs/packing-native-rpc-trace/zenohrpc-with-trace.py`. Trace showed one call's
session.get returned but callback never reached the waiting client; root cause
still unproven. Another trace run had healthy request/reply delivery. LCM trial
could not start because host multicast configuration requires sudo. No network
or sudo changes were made. Native preflight now retains a loaded backend while
waiting for new goal/camera observations instead of reloading the model per pick;
18 runtime tests pass. 11 planner tests pass including the canonical order.
All these final scoring/runtime/native fixes remain uncommitted after95bdae0bc.


### Image sensitivity isolated; two bounded follow-up training jobs

Native snapshot-rendering run9403 completed first pick but missed second;
physics/wall ratios .9996 and1.0002. Native15k checkpoint randomized9102
(`jobs/packing-native-15000-random`, PID1587187, finished) placed bottles2and5,
then missed4. Stops were clean. No complete native five-bottle success yet.

CPU replay of saved native input0049 using the same20k artifact reproduced
native actions (maximum CPU/CUDA action delta .000103). The predicted1.5s end
TCP was (.359,-.403,.883). Matching teacher8210/pick1 input predicted
(.341,-.403,.908). Replacing ONLY the native images with those teacher images
predicted (.341,-.405,.908), correcting the approach. Replacing only joints or
only goal barely changed the bad prediction. Native/teacher images have correct
RGB orientation; first-pick camera mean absolute differences <1/255, second-pick
head3/255 andwrist5.7/255. Script `/tmp/packing_prediction_audit.py` holds exact
comparison. Evidence supports sensitivity to small visual differences, not an
unreceived goal or a large joint-state mismatch.

Detached follow-ups:
- `jobs/packing-direct-finetune`, PID1590590: 5000 updates from final20k on same
  120training episodes, use_vae=False, lr3e-5/backbone6e-6, batch32. Exports
  `policy-packing-direct`, evaluates canonical fresh9600–9602. This is an
  ablation; no claim it will fix visual sensitivity. About2500updates atlastcheck.
- `jobs/packing-augmented`, PID1600519: waits for direct job exit, then5000
  updates from original20k (VAE retained), same data, small RGB transforms:
  ±2degrees, ±2%translation, brightness/contrast.95–1.05. Exports
  `policy-packing-augmented`, canonical fresh9700–9702 evaluation. Check actual
  physical outcomes before selecting either checkpoint. Exact commands saved.

Current BOTTLE_PACKING.md explicitly marks native validation incomplete. Need
finish learned evaluation/native repeated runs, then rewrite docs around the
chosen checkpoint and commit remaining source. Added local MuJoCo state API
stubs instead of suppressing typing; last host typing passed7files. Isolated
mypy and13physical regressions were launched; inspect pending sessions6148and72523
if their completion has not yet been recorded. Source has no temporary RPC logs.


### User-requested pause: 2026-09-10 16:46 PDT

The user needs the machine for other tests and requested no live DimOS stack
from this work. Live testing and implementation are paused until they resume.
Background training/tuning was explicitly allowed.

Stopped the stalled `packing-native-lcm-verified` job and all nine tracked
processes (1621779, 1621780, 1621781, 1623141, 1623142, 1623144, 1623145,
1623461, 1623482) with SIGTERM. No SIGKILL was needed. Host `/proc` verification
found no remaining non-training runtime processes in this R1Pro worktree.
Cleanup evidence: `recordings/r1pro-act-task/jobs/packing-native-lcm-verified/pause-cleanup.json`.
The separate planar preview PID 1527073 had already exited. A newly started
user Go2 holonomic benchmark was deliberately left alone.

Preserved detached training group **1600519**, job `jobs/packing-augmented`.
It was at approximately 4336/5000 updates at cleanup (about 2–3 minutes of
fitting remaining at the observed rate, plus export and evaluation). Its saved
run.sh exports `policy-packing-augmented`, then runs three standalone offline
MuJoCo evaluations, seeds 9700–9702, canonical order, without a viewer. It does
not start a DimOS blueprint/RPC stack. This job continues to use GPU/CPU and exits
automatically afterward. No further live stack is scheduled. Check `stage`,
`run.log`, `exit-code`, and `eval-packing-augmented-canonical` on resumption.

Current result remains experimental: five bottles, neat slot planning, and
stop-when-full are implemented. Best final-checkpoint offline canonical result
is **3/5 complete scenes** (counts 5, 1, 5, 1, 5); randomized result is 2/5.
No native five-bottle sequence has completed successfully. The no-VAE ablation
finished: final 5000-update checkpoint 1/3 complete (counts 1, 5, 4), halfway
checkpoint 2/3 (5, 1, 5). Neither established an improvement. Image replay
isolated the wrist image as the main source of sensitivity; augmentation results
are still pending. Do not present this as ready or repeatedly validated.

Latest uncommitted infrastructure work:
- Local LCM multicast actually delivered 10/10 fragmented 76,800-byte payloads
  without host network changes. Added an explicit ttl=0 delivery probe before
  static interface checks; failed probes preserve existing checks. 69 system
  configurator tests passed before the final small typing/URL-parse cleanup.
- The LCM native trial got past startup network checks (optional host changes
  declined), then crashed in GLFW X11 initialization because camera and viewer
  initialization ran concurrently. Added a camera-ready event so viewer creation
  waits for renderer initialization. The focused synchronization test passed;
  native validation of this fix is pending. The immediate retry exited because
  the old crashed stack still held its session lock, now released by cleanup.
- Remaining checks from the previous entry completed successfully: isolated
  evaluator mypy and 13 packing/grasp physics regressions. Source changes after
  commit 95bdae0bc7 remain uncommitted; no push was performed.

On resumption: inspect augmentation results first; finish review/lint/typing for
uncommitted infrastructure changes, including renderer shutdown lifetime; then
validate the GLFW initialization fix and repeated full native sequences before
updating the documented recommended artifact. Do not start live testing during
this pause without the user's resumption instruction.


### Resumed at user request: 2026-09-10 17:49 PDT

User explicitly asked to continue. No existing DimOS runtime or training process
was active at the initial host check; GPU use was low. The paused augmentation
job finished with exit 0: **3/3 complete offline scenes**, all 15 picks, seeds
9700–9702. Artifact `policy-packing-augmented` is now the leading checkpoint.

First full native run `native-packing-resume-augmented-9700/result.json` passed
all five bottles, clean stops and shutdown, physics/wall ratios .99935–1.00073.
Used LCM with explicit ttl=0 address 224.0.0.224:19468; optional buffer changes
were declined. The camera-ready barrier resolved the earlier GLFW startup race
in this run. No host network modifications were made.

Completed renderer lifecycle review: track camera thread on the engine, retain
references after a timed-out disconnect, reject reconnect until prior threads
exit, make camera initialization interruptible by stop, and cover initialization
in sim-loop cleanup. Camera streaming and published packing goal reads now use
the same locks as writes. 88 simulator/configurator tests and 29 runtime/planner
tests passed; mypy passed 10 source files; changed Python files pass ruff.

Detached bounded validation continues in `jobs/packing-resume-validation`,
PID **1722170**. Exact run.sh and logs saved. Native seeds 5000 and 9800 already
passed all five, and 9801 is starting. After those runs, the job evaluates ten
fresh canonical scenes (9800–9809) and five randomized-order scenes (9900–9904)
without a viewer, then exits. No further training is currently needed or queued.
Check the actual result files before finalizing counts. BOTTLE_PACKING.md now
uses the augmented artifact, GLFW full display, explicit LCM configuration and
thread limits. Final source commit and validation-summary update remain pending.


### Five-bottle baseline validated: 2026-09-10 18:09 PDT

The resumed work reached a usable matching-bottle baseline. **No active R1Pro
runtime or training processes remain**, verified against host `/proc` after the
last run. No new jobs are queued. Leave the ignored datasets and model artifacts
in place so the documented command remains runnable.

Selected artifact: `recordings/r1pro-act-task/policy-packing-augmented`, ACT with
30 actions per chunk at 20 Hz, head/wrist 160x160 RGB, 20 joint positions and an
8-value geometric goal. It uses the existing 120-episode training split; the
successful final improvement was 5000 updates with small image augmentation.
No new fitting was necessary during this resumed turn.

Measured results, with all bottles still upright, released, settled and contained
at the end, plus learned return to an open gripper at home:
- Native **6/6 complete runs**, 30/30 bottles: seeds 9700, 5000, 9800, 9801, 5001,
  5002. All per-pick stops have active=False and last_error=None. First four
  physics/wall ranges were .9984–1.0011. All use LCM ttl=0 on bus 19468.
- Offline canonical **13/13 scenes**, 65/65 bottles: 9700–9702 and 9800–9809.
- Offline randomized accessible order **5/5 scenes**, 25/25 bottles: 9900–9904,
  five different source orders. Total offline **18/18 scenes**, 90/90 bottles.
- Focused tests in this resumed turn: **126 passed** (88 sim/configuration,
  29 policy/planner, 9 existing camera/timing). Touched production code passes
  mypy and ruff. The 13 physical task regressions had passed before the pause;
  the scoring logic was unchanged in this resumed turn.

Visual inspection caught cabinet occlusion in the old default view. Setting the
MJCF default free-camera angle was ignored by the passive viewer; that temporary
scene edit was removed. Engine/module now accept optional viewer azimuth and
elevation. Packing sets those, its look-at and distance directly; other stacks
retain their previous defaults. Sensor views and trained inputs are unchanged.
The corrected native screenshot is
`jobs/packing-display-verified/native-final.png`, showing all five bottles in the
supported tray. This is an actual native-window capture, not an illustrative
render. `validation-summary.json` beside it lists exact source result files.

Both display-capture helpers ran with --stay-open and deliberately sent Ctrl-C
AFTER successful completion and screenshot, exercising normal teardown. Raw
child signal 2 is recorded in each `child-exit.json`. The first wrapper returned
130 and its stage remained native because it did not recognize that expected
close; it was not a policy crash. The final wrapper recognizes the intentional
close, reports complete/exit0, and all workers exited. Preserve raw exit records.

Runtime/transport fixes are committed locally as **bc7c31db0d**. Final packing
scoring, default order, viewer framing, runbook and this handoff are being saved
in the next local commit. No push was attempted in this resumed turn.
BOTTLE_PACKING.md contains the full desktop command using the selected artifact,
GLFW and explicit LCM configuration; optional socket-buffer changes were declined
throughout testing. It needs no API key. The base stays parked; learned tray
handling and heterogeneous items remain outside this completed baseline. Broader
shape/layout generalization requires new demonstrations and physical validation.


### 2026-09-10 — five-bottle tray delivery integration in progress

User requested an end-to-end run: existing ACT packs all five bottles, then
coordinator trajectories grasp the loaded tray, drive to the actual laptop table,
and place it on that surface. No new ACT training is involved. Added optional
`demo_packing_stack --deliver-to-laptop` and generalized tray support, route collision
checks and runtime cargo monitoring to all five free bottles. Physical source
and destination support remain mandatory. This integration is not yet validated.

Current tests found the parked left arm sweeps the table when directly approaching
the packing tray (which is closer/off-centre compared with the original single
bottle tray). Working on a checked preparatory arm movement. First native attempt
`jobs/packing-delivery-5000` exited 1 during bottle 4's first pick with a 1367.5 ms
head/state observation skew after 4 chunks; it never reached delivery. Preserve
`native-packing-delivery-5000/result.json`. No checkpoint changes or relaxed timing
thresholds. Added a camera pause barrier before changing the model for loaded
tray servo tuning; focused concurrency tests are being added. Existing native
6/6 and offline 18/18 evidence applies only to the committed bottle-packing baseline.


The native seed-5000 retry completed the full delivery successfully:
`native-packing-delivery-5000-retry/result.json` has packing_success=True,
delivery.success=True and overall success=True. All five bottles remain upright,
released, settled and inside the tray on the actual laptop tabletop. During 31
base segments (2230 monitored carrying samples), all four handle contacts stayed
loaded, every bottle stayed inside/upright, and maximum tray tilt was 2.257 degrees.
Final tray position error was 4.06 mm. Full run took roughly five minutes with
startup. The preparatory left shoulder movement is fully collision checked;
physical five-bottle carrying regression and 17 shared tray/camera regressions pass.

The capture helper saved native-final.png and intentionally sent Ctrl-C, then
mistook the same completion text printed in a KeyboardInterrupt traceback for a
second completion event. Consequently the wrapper exited 1 after the successful
run and screenshot. This is not a failed physical delivery. The next job fixes
the helper to await EOF after its one capture. The CLI now handles Ctrl-C without
a traceback and exits 1 for an unsuccessful completed rollout.

Visual inspection found temporary wall occlusion during travel with the original
low camera. Added a one-shot elevated native camera request after pickup (225 deg,
-75 deg, 2.3 m); mouse orbit remains available. Detached repeat
`jobs/packing-delivery-5001`, PID 1828614, is validating the current source and this
view. No training, weights or policy timing tolerances were changed.


### 2026-09-10 — five-bottle end-to-end delivery complete

The repeat with seed 5001 also passed: five ACT picks, two-handed physical tray
pickup, collision-checked navigation, and supported release beside the laptop.
Both complete runs retained every bottle upright and contained and all four
handle contacts during transport. Maximum carrying tilt was 2.257 / 2.281 degrees;
final placement error was 4.06 / 3.44 mm. Elapsed time including startup and teardown
was 306 / 309 seconds. The repeat wrapper reports stage=complete and exit-code=0;
its raw child-exit.json records the intended Ctrl-C close as exitstatus=130.
All owned simulation, policy and test processes have exited; no training is queued.

Exact metrics and paths, including the earlier observation-sync failure, are in
`recordings/r1pro-act-task/jobs/packing-delivery-5001/validation-summary.json`.
`native-final.png` beside it is an actual native-window capture: all five bottles
are visible in the tray on the tabletop beside the laptop. The elevated delivery
view was inspected during carrying and after placement. User camera controls remain
available after the one-time change; policy image inputs are unchanged.

The combined command is in BOTTLE_PACKING.md under “Pack, carry, and deliver to the
laptop”: same policy-packing-augmented checkpoint, add --deliver-to-laptop and use
a fresh output directory. Packing-only remains the default. ACT controls bottle
picks via policy_rollout; tray_manipulation and base_transport coordinate the rest.
No retraining, live object attachments, teleports, or scene-package edits were used.
The five-object state now reports grasp/lift evidence only in each bottle's row,
removing unrelated inherited single-bottle summary fields.

Validation: 53 distinct focused tests passed (46 engine/packing planner, six shared
tray/guard, one physical five-bottle carrying), including single-bottle backward
compatibility and camera pause/concurrency. All touched production files pass mypy;
repository pre-commit checks passed. Source changes are ready for the local commit.
User explicitly requested committing and pushing all changes and no Codex co-author
trailers. Outgoing commit metadata will be audited before a normal feature-branch
push; Git tracking refs are the authoritative remote status. Generated simulation
logs, .venv, datasets and checkpoints remain local artifacts.


## 2026-09-10: KronkNav integration in progress

User requested whole-house point-cloud planning through KronkNav and velocity execution by HolonomicPoseFollowerTask after physical tray pickup. Baseline pushed at 155d5493 (origin/feat/r1pro-act-sim); root worktree is unrelated mb/go2-zenoh-test and must remain untouched.

Detached Rust build completed in recordings/r1pro-act-task/jobs/navigation-build (run.sh/log/stage/exit-code). Native target/release/mls_planner exists; probe Python binding installed only in this job's isolated venv. No training or shared environment changes. New navigation_cloud.py samples complete static physical house surfaces (293064 points at 4cm), excludes all articulated robot descendants and free tray/bottles. navigation_base.py integrates bounded body-frame Twist into physical actuator targets with watchdog, slew limits, and antiwindup.

Offline real MLSPlanner paths initially collided because loaded geometry is asymmetric around base_link. Measured loaded AABB centre offset [0.00694632,-0.07883024] in base frame solves this: .06m voxel, .35/.40/.42 clearance, 1.5m height, .12 closing, .3 node spacing, .65 wall buffer, weight20, step .07 all produced paths with no full-geometry collision on every segment using the loaded 5001 pickup snapshot. Plan in this footprint frame, then convert to base poses at fixed carrying yaw. This is a frame transform, no alternate/custom A* route. Files offset-path-*.npy in build job.

Implementation/physical validation still in progress: native map/TF/goal/path streams, separate Twist base hardware, holonomic follower and pure-rotation support, native full five-bottle validation, docs and focused tests. No new navigation completion claimed yet. Keep all long validation detached and use isolated LCM19468; user docs19467.


Navigation implementation is complete behind `demo_packing_stack --deliver-to-laptop --kronknav`. The new navigation modules supply the complete static PointCloud2 map, measured base odometry, carrying-footprint TF, native MLS goal/path exchange, and a physical Twist actuator bridge. Separate BASE resources own vx/vy/wz; whole-body SHM owns the twenty arm/torso/gripper joints. The holonomic task executes departure and the full native route; existing trajectories perform pickup and placement.

The physics probe uncovered a terminal hairpin problem: preview feedforward reversed before projection reached the corner, stalling about 2 cm from the goal. The generic follower now uses the incoming tangent when preview travel points backwards or perpendicular; a closed-loop regression test covers this. It also accepts turn-in-place paths and clears pending streamed paths on cancellation. The detached physics retry passed with the cargo held. The first full native run packed all five bottles and picked up the tray, then failed because the map message lacked a timestamp. Explicit timestamps fix that; native map acknowledgement is now required before ACT begins.

### Completed native navigation validation

- Job: `recordings/r1pro-act-task/jobs/navigation-native-5000-retry` (finished; wrapper exit 0).
- Result: `recordings/r1pro-act-task/native-navigation-delivery-5000-retry/result.json`.
- Screenshot: job `native-final.png`; quantitative evidence: job `validation-summary.json`.
- All five ACT picks, physical pickup, native KronkNav travel and supported table placement passed. All three success flags are true.
- Planner received 293,064 map points and exposed 20,205 surface points. Native route: 38 poses, 3.874 m.
- 1,887 navigation observations: all bottles upright and inside, continuous bimanual grasp, no obstacle contacts, maximum tilt 1.618 degrees.
- Measured base arrival error: 5.251 mm. Final released tray placement error: 4.067 mm on the actual laptop tabletop.
- 103 focused controller, registry, coordinator, navigation, packing and delivery unit checks passed. Generated-registry CI check: 6 passed. Strict mypy: 10 production files passed. Repository pre-commit hooks passed.
- `all_blueprints.py` was regenerated by its test for the new `R1ProNavigationSim` module. No hand-edited registry entries.

The updated native command and one-time Rust build are in `BOTTLE_PACKING.md`. No ACT retraining or shared environment changes were needed. The map has perfect static scene knowledge and the base remains the physical planar servo simulation; no claims of real lidar/localization or hardware calibration. The controller JSON deliberately records conservative configured simulation gains, not a measured hardware plant fit. Root worktree remains untouched. Navigation changes are currently local and uncommitted; baseline 155d5493 was already pushed before this task.
