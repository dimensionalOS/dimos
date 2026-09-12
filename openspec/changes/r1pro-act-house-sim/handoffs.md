# R1Pro ACT / house simulation handoff

Updated: 2026-09-11
Branch: `feat/r1pro-act-sim`
Worktree: `/home/mustafa/dimos-wt/r1pro-act-sim`

Earlier single-bottle ACT and free-tray development is preserved in
[the tray baseline archive](handoffs-tray-baseline.md). The latest interactive
validation updates are at the end of this file.

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


## 2026-09-11: home blueprint, main rebase and faster navigation (validation underway)

User requested 0.6 m/s travel and a normal `dimos run r1pro-home-sim` command with Zenoh, without environment/scout-address setup. Saved previously validated navigation as bb39ed4d3 and branch `backup/r1pro-before-home-blueprint-rebase`, then rebased with merges onto origin/main 194f665ca2. Rebased navigation is aaccb5e21d. Applied the user's existing Zenoh adapter fix 53939f3d0d as 116c4382a8. Current branch has not yet been pushed after the rebase. Root checkout mb/go2-zenoh-test remains untouched.

New home blueprint lazily prepares scene/home/limits/cloud during build, then runs the shared cancellable ACT packing/delivery sequence. Zenoh base topics use logical remappings and the adapter factory. Native viewer stays open. Scene default and checkpoint are local assets; worktree dimos/data/scene_packages symlinks the existing root data/scene_packages (local only). `.home-venv` installs this checkout's CLI and Zenoh 1.10.1, with a .pth sharing other existing root-venv dependencies; root .venv is not modified. User launcher installation is pending full validation.

Detached speed sweep `recordings/r1pro-act-task/jobs/home-speed-sweep` completed: 0.15, 0.3 and 0.6 m/s all passed replay of the 5000 loaded route with both-hand contact, all five bottles upright/contained and zero obstacle contacts. At 0.6, actual peak speed 0.60066 m/s, carry 19.64 s, max tray tilt 0.12067 rad. Chosen controller accel/decel 0.4 m/s², lateral accel 0.15; physical servo accel 0.6. Departure remains 0.055 m/s and yaw 0.12 rad/s. Rust planner rebuilt against main successfully.

New-main compatibility work: explicit 2 rad/s arm / 0.25 m/s gripper trajectory limits; corrected old tests for required connected-hardware limits and replacement semantics. Separate OpenYAM learning_profile.py modules prevent importing recorder-only EpisodeStatus into isolated rollout environments. Home coordinator is in a lightweight module so RPC proxy unpickling doesn't import MuJoCo in the isolated policy runtime. Zenoh RPC now reports argument unpickle errors through its exception reply; previously those failures silently caused a 120 s timeout.

Current full display job: `recordings/r1pro-act-task/jobs/home-zenoh-5000`; result directory `recordings/r1pro-act-task/home-zenoh-5000`. Uses installed .home-venv/bin/dimos with PYTHONPATH, transport, GL and CPU-thread exports explicitly unset; only DISPLAY/XAUTHORITY supplied for the remote agent's desktop access. All five ACT picks passed; tray pickup completed and faster native route is running. Remaining: verify final supported placement, capture native result, stop owned stack, finish checks/docs, install user launcher, commit and update remote. Earlier failed startup logs retained in the same job.

### Home blueprint validation completed

Full native Zenoh run completed successfully (seed 5000): five ACT picks, physical two-handed pickup, native KronkNav at 0.6 m/s, supported table placement and release. Actual route time 19.668 s, peak speed 0.600357 m/s, max tilt 6.992 degrees, all 696 navigation samples retained upright/contained cargo and bimanual grasp, zero obstacle observations. Base arrival error 5.597 mm; tray placement error 4.221 mm. Native screenshot and validation-summary.json saved in the job. Captured the final native viewer, then sent Ctrl-C to the owned CLI; all workers shut down cleanly. No demo/training jobs are intentionally left running.

User-level ~/.local/bin/dimos now links to this worktree's .home-venv/bin/dimos. Old launcher backed up as job/previous-user-dimos-launcher. `dimos run r1pro-home-sim --help` works through the normal user PATH with no activation/PYTHONPATH exports. Fresh desktop terminals resolve this launcher; a shell with another checkout's venv active must leave that venv first. No root branch/source/.venv changes. BOTTLE_PACKING.md now leads with the one-command Zenoh/native-display workflow. No scout-address option or LCM fallback is part of that workflow.

195 focused tests passed across blueprint setup/cancellation/full-tray handling, servo/navigation, coordinator/trajectory, Pink IK, MCP, OpenYAM profiles and RPC error reporting (the final single RPC test rerun follows the 194-pass combined batch). Strict mypy passed 14 changed production files. Registry regenerated with its test; final pre-commit and commit/push audit are the remaining housekeeping steps.

Final native transport/follower/generated-registry batch: 54 passed (249 focused checks across the completed batches). `dimos status` confirms no running DimOS instance. All source changes and evidence documentation are prepared for commit; no Codex/OpenAI co-author trailers in branch commits above main.

Implementation and validation are complete. All pre-commit hooks passed, including lockfile, LFS and generated-signature checks. Source is committed on feat/r1pro-act-sim; publishing the rebased branch requires an explicit lease against prior remote 155d5493ebbc0e3b58f196a4294df977121935d9. Use git status / origin tracking ref as the authoritative push status; the pre-rebase backup branch remains available. Normal user command: `dimos run r1pro-home-sim` from a fresh desktop terminal.

## 2026-09-11: interactive house actions and SDK integration (in progress)

User requested individual ACT bottle choices in any order/subset, tray delivery to dining and kitchen, classical bottle unloading, a continuous initial lift, and use of the new manipulator SDK. Work is uncommitted. Do not present the interactive deliverable as validated yet.

The original tray-departure stall was a double-slash topic mismatch: coordinator `/` prefix plus an absolute remapping produced `//r1pro_nav_base/cmd_vel`; the adapter subscribed to the single-slash topic. Zenoh normalized this but the user's LCM environment did not. Fixed transport topic construction and added both-backend regression coverage. Full native LCM rerun `recordings/r1pro-act-task/lcm-topic-fix/result.json` passed all five ACT picks, physical pickup, navigation, and placement. Added actual base-odometry preflight and a five-second measured-motion watchdog.

New `r1pro-home-sim-agent` blueprint is idle between exclusive cancellable skills: get_scene, pick_bottle, pick_up_tray, go_to, put_down_tray, place_bottle, wait_for_action, stop_action. Raw policy skills are disabled to avoid bypassing exclusivity. Inventory uses explicit simulator ground truth. Cargo membership follows selected bottles and is not inferred away after a spill. Native navigation remains KronkNav/holonomic ControlCoordinator execution. Continuous tray lift is one 15 cm / 3 second trajectory; careful placement is retained.

`HomeKinematics` uses the standard manipulation RobotModel, planning groups, RoboPlan world and Pink IK for torso-assisted poses. FK is checked against both MuJoCo TCPs. MuJoCo swept geometry checks remain required before execution. The isolated `.home-venv` now also exposes the existing root cmeel native dependency directory through `shared-native.pth`; no root environment changes. This is planning-backend reuse, not yet live Arm convenience-API execution.

Physical unloading and tray regrasp passed offline MuJoCo tests (`/tmp/probe-unload-sdk3.log`, `/tmp/probe-regrasp2.log`). Detached native interactive test 02 passed bottle_4, bottle_1, dining navigation, and one bottle unloading. Kitchen resolution failed because the ray hit a non-colliding visual mesh. Fixed selection to use the actual horizontal collidable countertop box, and departure rotation now chooses the shorter turn. Test 03 is rerunning; logs/PIDs/results are under `recordings/r1pro-act-task/jobs/interactive-03`, output under `recordings/r1pro-act-task/interactive-03`. Its wrapper is `/tmp/r1_trip_test3.py`, detached; it leaves its owned stack running for inspection. Never kill unrelated DimOS processes.

Arbitrary selection is NOT yet solved by the existing checkpoint: requesting rightmost correctly selected bottle_5 in the skill but ACT first picked bottle_2 and then tipped bottle_5. Evidence is `interactive-01/action-001.json`. New `FlexiblePackingTask` is an offline demonstration teacher, not a rollout fallback. Latest prototype individually moves all five targets into the tray, but rear picks 3 and 5 still tip a neighboring source bottle, so those demonstrations must not enter training. Clearance diagnostic runs from `/tmp/probe_bottle_clearance.py` to `/tmp/probe-bottle-clearance.log`. No new ACT training has started yet. Need physically clean arbitrary-order demos, detached collection/training, strict target/non-target evaluation, and a complete native/LLM test before claiming readiness.

69 focused unit tests passed before the latest SDK/preflight edits; final ruff/mypy/tests/registry/docs and commit/push remain. All new training/tuning/full-stack runs must stay detached as the user requested. Root checkout and user processes remain out of scope.


### Post-demo interactive failures and resumed work (2026-09-11)

User completed the native agentic demo successfully for nearest-bottle packing and dining/kitchen navigation. Presentation evidence is in `recordings/r1pro-home-sim/`. Runs `373e33058b9549e88cfc3f48f0e6d018` and `c8287ec29da8428d9155eff99c88ee74` selected bottle 5 correctly for furthest/rightmost, but the deployed `policy-packing-augmented` moved bottle 2; the non-target guard stopped it. Run `19fb6568bcd1429aa1f430833be306b4/action-006.json` failed the second dining-table unload: bottle 1 remained about 5 mm above support and the gripper stayed closed. Next tray pickup consequently failed. These are confirmed failures, not validated arbitrary-selection capability.

Before reboot, collected 5 complete unrestricted-order sequences (25 picks) and 6 paired-choice groups (30 picks), all verified physically. Host reboot cleared the stuck NVIDIA/X jobs; GPU works again. Started detached `jobs/flexible-training/run.sh` from all 55 accepted episodes and resumed `jobs/flexible-sequences/run.py` toward 12 sequences. Both use EGL only for background jobs and persistent logs/PID/stage/exit-code files; desktop launch remains GLFW. Training pipeline converts a frozen hard-linked manifest snapshot, fine-tunes 5,000 updates, exports `policy-packing-flexible-5k`, then physically evaluates all five first choices and mixed orders. The deployed agent remains on the old checkpoint until validation. No new training results yet.

User now requests bed/floor placements and recovery after failed actions. In progress: inspect real support geometry and SDK workspace; bounded support-seeking before release; explicit recovery and simulator reset tools. Do not claim these complete. Kitchen support lookup already excludes robot/tray/bottles; kitchen placement and lateral unload succeeded offline before the demo. Latest previous focused checks: 27 passed; strict mypy passed before the latest small changes. New work remains uncommitted; do not include local venv/assets/checkpoints/logs.

### Recovery and low surfaces: verified progress, remaining integration (2026-09-11 afternoon)

- Completed first ACT continuation: 5,000 updates on 55 accepted demonstrations, exported policy-packing-flexible-5k. All five individual first choices passed on seed 5000 (eval-flexible-5k-{1..5}/result.json). Mixed order 5→4 passed seeds 5000 and 5001 (eval-flexible-resumed-5,4). Longer order 3→1→5→2→4 stalled holding bottle 1 above the second slot on seed 5000. Do not call arbitrary ordering reliable or change the deployed default yet.
- Collection completed 12 accepted unrestricted-order scenes (60 picks) plus 6 paired-choice layouts (30 picks), total 90. Started detached jobs/flexible-full-training/run.sh to convert a frozen hard-linked snapshot, continue 10,000 updates from the 5k checkpoint, export policy-packing-flexible-15k, and test all first choices plus mixed orders on two seeds. jobs/flexible-sequence-evaluation/run.py continues evaluation of the 5k checkpoint independently. Both write persistent stage/log/exit-code files.
- A host reboot around 13:07 Pacific interrupted prior mixed evaluation (CUDA unavailable) and native regression (X display unavailable). GPU and desktop were healthy at 13:46; resumed jobs from saved assets. Temporary /tmp diagnostic scripts/results were lost. Persistent scripts now live with their job evidence.
- Added recover_action, reset_scene, finite support-seeking before bottle release, recovery gating, and measured selected_bottle status. Worktop ACT failures automatically release supported contacts and retreat, preserving the original failed outcome; unsupported objects remain held. Explicit reset clears progress, resumes policy cameras, restores packing actuator damping, and retains the native window.
- Native interactive-recovery-03 verified the old checkpoint's wrong-bottle failure → automatic supported release/retreat → explicit reset → two successful nearest picks. The first immediate action race was fixed by waiting for the scene to settle.
- Added geometry-grounded bed/floor destinations and get_surfaces. Bed approach identifies the mattress's actual collision meshes; floor uses a clear patch. Low placement reaches outward/downward together through the existing torso-assisted manipulation SDK. Both physical isolated placements passed with three bottles, supported release and upright containment; native end-to-end remains in progress.
- Native bed trip initially rejected the raw fixed-heading route at the cabinet (full geometry guard caught it before execution). Added a separate KronkNav approach with the travel heading, followed by a local full-geometry docking path through the same holonomic task. Offline checked the local docking path against the failed run's actual carried posture. Detached jobs/interactive-recovery-04/run.py tests two picks → bed placement → floor placement → reset → pick.
- Combined focused tests: 36 passed (one existing JAXopt warning). Five-file strict mypy passed before the latest route/contact edits. New regression tests cover support-seeking bounds, preserving the lowered command while releasing, blocked retries, reset state, visual-vs-collision geometry, no live-model mutation, and upward support normals.
- Still outstanding: finish native delivery/unloading verification, validate the 15k policy and its matching left-shoulder home pose (0.8), update user docs/defaults, check latest main/rebase, commit/push without coauthor trailers. No new commits/pushes yet.


### Recovery continuation and bounded bed docking (2026-09-11, 14:21 Pacific)

- Native interactive-recovery-05 reached the bed approach, but local plan_transport exceeded the 120 s RPC timeout exploring a 2.5 cm grid. Fixed the 20-joint/separate Twist-base eligibility guard, tried fully checked straight/elbow approaches before A*, and bounded fallback planning. The exact saved carried-state docking now plans in 0.45 s. No live state is changed by planning.
- Recovery now returns all 20 manipulation joints to the configured ACT home, not only the right TCP. Native 07 exposed loose final tolerance (2.3 cm TCP error); added a commanded settling hold and a 0.005 rad final joint threshold. A stopped bottle just above the original worktop can descend in 3 mm steps, at most 24 mm, until upward support is measured. Opening permits only the existing supported bottle contact. All other bottle motion and obstacle checks remain active.
- Stop validation now checks inactive policy, confirmed trajectory cancellation and exited inference thread. A retained inference/observation error no longer incorrectly blocks recovery after a successful stop. Native 06 hit stale images during concurrent training; no freshness guard was relaxed.
- Saved-state full-posture recovery passed (jobs/recovery-posture/result.json). Saved near-support grasp recovery passed including descent, release, lift, home, final posture (jobs/recovery-seek-posture/result.json, final max joint error 0.00455 rad).
- Detached native jobs/interactive-recovery-09/run.py: old checkpoint rightmost failure, successful automatic recovery, then two successful nearest picks (bottles 4 and 2, without reset). Bed trip is currently running. It will test bed release, bed pickup to floor release, explicit reset, then another ACT pick. Inspect results.json and exit-code rather than assuming completion.
- Full focused regression suite: 128 passed, 20 excluded markers, one existing JAXopt warning. Latest additional recovery checks: 17 passed, covering finite lowering, safe cancellation, and settling. Broad mypy found and fixed a missing numpy annotation import in grasping_blueprint; rerun is pending. Ruff formats/checks all owned Python files successfully.
- BOTTLE_PACKING now documents the interactive blueprint, humancli, action status, recovery/reset, and current checkpoint/low-surface validation limits. The deployed checkpoint remains policy-packing-augmented. Flexible continuation is around 59% of 10,000 additional steps; the detached pipeline exports/evaluates policy-packing-flexible-15k automatically. Do not promote it until physical mixed-order results pass.
- origin/main is fetched at ffdffb8ddf (six new commits); rebase/commit/push remains pending while jobs import this worktree. Exclude local .venv, data symlinks, recordings, checkpoints, and MUJOCO_LOG.TXT. No coauthor trailers.


### Native bed/floor pass and ACT deployment diagnosis (2026-09-11 afternoon)

- Native `interactive-recovery-11` completed: nearest bottles 4 and 1, bed navigation/placement, bed regrasp, local departure, KronkNav to floor, supported floor placement, explicit reset, fresh nearest pick. Every action completed; job exit-code 0. Source pickup allows the actual source furniture collision meshes, while final lift still requires no support contacts. Departure first leaves the furniture overhang with a full-geometry checked local path, then asks the 2D planner for the travel route. Earlier jobs 09/10 preserve the failures that motivated these fixes.
- Flexible continuation finished 10,000 additional updates in 38m26s on 90 accepted demonstrations. `policy-packing-flexible-15k` passed first choices 1/3/4/5 and order 5→4 on seeds 5000/5001. Bottle 2 failed both seeds as first choice and later in both five-bottle mixed orders. Keep the deployed `policy-packing-augmented`; do not equate training completion/lower validation loss with successful robot behavior.
- Native `interactive-flexible-15k` selected and lifted the correct bottle 5 but stalled: measured torso_joint1 was 0.45556, while the dataset action range was fixed at 0.4. Runtime clipped the trajectory start to that statistical bound, producing perpetual START_STATE_MISMATCH (>0.05). Fixed anchors to use measured positions bounded only by authoritative hardware joint limits, obtained through the trajectory task's exposed get_position_limits. Predicted actions retain their demonstration bounds. Regression tests cover both out-of-demonstration measured poses and slight physical-stop overshoot.
- Broader policy/coordinator/registry tests: 121 passed after updating RPC contract assertions. Initial native retry 02 caught a missing Any annotation import during Spec matching, fixed before retry 03. Detached native `jobs/interactive-flexible-15k-03` now tests 5→4→1, dining unload twice, kitchen unload once, reset, then 3→1→5→2→4. Inspect results/exit-code before claiming success.
- Detached `jobs/flexible-checkpoint-sweep` compares the 2.5k/5k/7.5k intermediate continuation checkpoints with action_steps=20, matching left-shoulder home=0.8. Tests bottle 2 first and both long mixed orders at seed 5000. First intermediate checkpoint passed bottle 2 individually. Further results pending. Jobs/scripts/evidence live under recordings, not /tmp; raw artifacts stay out of Git.
- Remaining: native ACT/unloading verification, choose a reliable checkpoint (if any), final mypy/pre-commit, rebase on fetched main, commit/push without coauthors. Bed/floor/recovery checks need no repeat unless related code changes.


### Unloading clearance regressions and continuing ACT evaluation (2026-09-11, afternoon)

- Committed the hardware-limit trajectory anchor fix as `2844b5b59a` (no coauthor trailers). The remaining interactive work is uncommitted. Native `interactive-flexible-15k-03` completed the requested bottle 5, confirming the deployment stall fix; bottle 4 was placed but did not finish its return within 50 seconds. Recovery's high lift was blocked by furniture. Recovery now tries three fully checked clearance heights (1.04/0.98/0.94 m) before reporting no route. A saved-state replay reached the calibrated home with 0.00081 rad maximum error.
- All three saved continuation checkpoints (cumulative 7.5k/10k/12.5k) still failed bottle 2 late in both five-bottle orders at seed 5000. Two passed bottle 2 as a first choice. `jobs/flexible-horizon-sweep` compares 30/10 action steps (up to the configured 1.5-second horizon); inspect physical result JSON, not evaluator exit status. No candidate is deployed.
- Diagnostic `jobs/flexible-home-posture` required the learned full-joint home phase before completion; it failed its first bottle 5 (ACT eventually reapproached the placed bottle). `jobs/flexible-calibrated-start` explored a checked classical home return BETWEEN completed ACT picks, but its first pick failed before that return ran. This does not establish the arm-posture hypothesis. Identical seeded runs can diverge from tiny numerical differences; one passing seed is insufficient.
- Native `interactive-unloading-verified-01` passed 3 nearest picks, dining navigation and one unload. The second unload opened beside a tray handle and nudged it. Added explicit supported-tray contact checks to unloading, including opening the fingers; lateral placement candidates moved from 0.22 to 0.30 m. Saved failure replay `jobs/unload-tray-clearance` passed.
- Native `interactive-unloading-verified-02` exposed a second issue: already-delivered bottle 4 was excluded from planning collisions. Only the selected bottle is now exempt during unload/descent planning; other bottles remain obstacles. More bounded tabletop candidates are tried. Full SDK right_pose solving preserves the unused left hand's measured pose during torso motion, and tray clearance is 15 mm. Saved second failure replay `jobs/unload-tray-clearance-02` passed complete grasp/transfer/support-seeking/release/retreat. Full native `jobs/interactive-unloading-verified-03` is running (3 picks, two dining unloads, kitchen unload, reset, fresh pick). Do not claim this full chain passed until its results say so.
- Combined regression: 253 passed, 20 marker exclusions; the only failure was the generator requiring its uncommitted output be committed. Separate CI-mode generation check: 6 passed. New real MuJoCo gripper/handle regression plus recovery/tray tests: 24 passed. Five-file runtime/navigation mypy passed; latest three-file unload mypy is in /tmp/r1-unload-mypy-final.log. All hooks passed before the latest unloading refinements.
- Current background jobs use persistent scripts/PIDs/logs under recordings/r1pro-act-task/jobs and survive terminal disconnects. Keep the original checkpoint/default home pose. Rebase on main and final commit/push are still pending; latest main changes include more than R1Pro code, so avoid rebasing while native tests are running.


### User steering: random objects and ACT generalization (2026-09-11)

User challenged permutation-specific training and wants 4-5 randomly generated objects. Do not equate this narrow bottle checkpoint with general grasping. Existing source fixes five bottle positions, one radius/height, and fixed slot geometry. ACT does not inherently require enumeration of orders; our data/model representation is too narrow. Asked whether next random-object demo should use classical grasping first (ACT optional) or require ACT for grasps. Pause further bottle-specific training while clarifying that choice.

`jobs/flexible-balanced-training` supervisor was stopped before training. Any already-running collection child can finish and preserve its manifest under `flexible-balanced-collection`; no subsequent fitting/export will launch. Prior coverage audit found no bottle-2-as-fourth or bottle-4-as-fifth demonstrations. All saved checkpoint/horizon sweeps failed full mixed sequences. Do not promote those artifacts.

Unloading remains under repair: fixed-hand IK blocked the first native unload in `interactive-unloading-verified-03`. Now the SDK first withdraws/raises the empty left hand (5 mm / 0.03 rad parking tolerance, full collision checks), then parks left arm at its calibrated 0.8 shoulder pose. Bottle IK retains precise tolerances. This passed saved first-unload physics in `jobs/unload-left-parking`. The current full native regression is `jobs/interactive-unloading-verified-04`; it keeps running independently. The latest parking changes need full-run validation and final type/style checks. No rebase/push yet; runtime anchor fix is the only new commit.


### ACT requirement confirmed and regression checkpoint (2026-09-11)

User answered: ACT must perform the grasps. The next direction is four or five randomly generated objects with a reusable selected-object ACT skill. See random-object-act.md for the implementation scope, data changes and acceptance gates. Existing packing already has a goal input; the problem is not that it lacks one. Its fixed geometry, tiny positional variation, teacher routes and limited observations/data are the current limitations. No new random-object implementation or validated checkpoint yet.

The balanced-training supervisor and its collection child have both exited. The child rejected all permitted full-sequence attempts; no further fitting/export was started. Keep this narrow pipeline paused. Native interactive-unloading-verified-04 passed three ACT picks, dining delivery and TWO consecutive bottle unloads. The following go_to kitchen failed during SDK pose planning while reacquiring the tray (QP failure); full multi-stop validation remains incomplete. Workers shut down and exit-code is 1. Do not claim the complete sequence passed. Five-file ruff check/format and strict mypy passed. Rebase, remaining commits and push are pending.
