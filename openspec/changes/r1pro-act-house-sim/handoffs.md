# R1Pro ACT / house simulation handoff

Updated: 2026-09-10
Branch: `feat/r1pro-act-sim`
Worktree: `/home/mustafa/dimos-wt/r1pro-act-sim`
Base: `hackmit/t7-resume` at `0b816edf49`; initial diagnostic commit `eb2efd8d03`.

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
