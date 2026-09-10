# Trained R1Pro ACT in MuJoCo

For the new five-bottle packing work, see [BOTTLE_PACKING.md](BOTTLE_PACKING.md).
Its new goal-conditioned policy is being trained and evaluated separately.

The physical delivery task is: **ACT picks up the blue bottle and puts it in
an orange tray resting on the worktop; both hands lift the tray, the base drives
through the house, and both hands set it down on the table beside the laptop**.
ACT controls the bottle. Coordinated joint trajectories control the tray and
base; the two-handed task is not a learned ACT policy.

The free-tray ACT checkpoint passed 10/10 new physical bottle trials (seeds
5000–5009). Two fresh native runs completed the full supported-tray pickup,
carry and release beside the laptop (seeds 5000 and 5001). The earlier
20/20 tabletop and 10/10 house results apply to the original bottle task and
fixed-tray transport baseline, not the new complete delivery. All bottle trials
vary starting XY by up to 12 mm at a known workstation.

## Run physical tray delivery

On the current GPU machine, first close any completed R1Pro demo window or
stop that demo with Ctrl-C in its launch terminal. A window kept open with
`--stay-open` still has a running simulator and messaging connections.

```bash
cd /home/mustafa/dimos-wt/r1pro-act-sim
source .venv/bin/activate
export PYTHONPATH="$PWD"
export MUJOCO_GL=glfw

python -m dimos.robot.galaxea.r1pro.demo_pick_place_stack \
  --artifact "$PWD/recordings/r1pro-act-task/policy-free-tray" \
  --output "$PWD/recordings/r1pro-act-task/my-tray-delivery" \
  --zenoh-scout-addr 224.0.0.224:19467 \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --deliver-to-laptop --seed 5000 \
  --stay-open
```

This opens the full native MuJoCo window, waits for synchronized observations
and CUDA ACT, loads the tray with the bottle, stops ACT, grasps both handles,
and carries the tray to the actual laptop table. It backs away from the
workbench, turns to fit through the kitchen exit, and follows a collision-checked
route. The display camera follows the robot; orbit/zoom controls remain available.
The complete sequence takes about four minutes including startup on this machine.
The viewer stays open after the hands release the supported tray.
Close the MuJoCo window after completion, or press Ctrl-C in this terminal,
to shut down this stack. Rerun for a fresh episode.
`result.json` is written in the output directory before waiting in the viewer.

Use a fresh output directory for each run. Concurrent instances need distinct
output paths and multicast addresses. The launcher reserves both resources until
its workers shut down. A duplicate exits before building a scene and reports the
existing process PID. This address is separate from the existing OpenYAM run.
No OpenAI API key, language model, MCP server, or real robot connection is needed. The demonstration is started by this command, not by
`dimos agent-send`.

Keep the existing host environment. LeRobot runs in its separate locked Python
3.12 project with Torch 2.10/CUDA 12.8; do not run a root `uv sync` to install it.
The cached vendor assets, house package and trained checkpoints are needed.
Weights, datasets and results are local under ignored `recordings/`; a Git push
of the source does not upload them.

For the simple tabletop, use `policy`, omit the scene-package/delivery
options, and choose another output directory. The standard imitation interface
also supports the tabletop when no other DimOS coordinator is running:

```bash
dimos imitation run r1pro-sim-lerobot \
  "$PWD/recordings/r1pro-act-task/policy" \
  --task "Pick up the blue bottle with the right gripper and place it inside the orange bin."
```

Space starts/stops ACT; Q closes the stack. This checkpoint learns one task;
changing the task text does not teach it another task.

## If a replay reports camera synchronization errors

Two R1Pro launches on the same messaging address can mix head images, wrist
images, joint state and RPC responses. This caused the reported startup and
mid-rollout failures at 142.6 ms / 388.7 ms image skew while another completed
demo's viewer was still open. The policy correctly stopped at its 20 ms limit.

The launcher now rejects overlapping use of either the messaging address or
output directory, and closing a completed viewer shuts down its stack. Wait for
that process to exit, then run the command again. The ACT checkpoint and camera
skew threshold are unchanged. Resource-tracker messages printed during cleanup
are separate from the original synchronization failure.

## What is loaded

`build_r1pro_pick_place()` in `grasping_blueprint.py` composes:

- `R1ProGraspingSim`: MuJoCo, native viewer, head/wrist RGB, measured motor state,
  and read-only physical task scoring.
- `ControlCoordinator`: the existing shared-memory whole-body simulation adapter,
  `policy_rollout` for ACT's 20 joint commands, `tray_manipulation` for both arms
  and grippers, and `base_transport` for the three planar base coordinates.
- `R1ProPickPlacePolicy`: the isolated LeRobot runtime with trained ACT weights,
  synchronized observations, normalization, joint bounds and cancellable chunks.
- `PolicySkills`: explicit rollout preflight, start, status and stop controls.

ACT is a separate inference module. It sends trajectory chunks to the
coordinator's `policy_rollout` task. After ACT stops, the delivery runner sends
bimanual IK trajectories to `tray_manipulation`, followed by base trajectory
segments while the arms hold their commanded positions. Each segment waits for
measured arrival and settling before the next begins. Only the coordinator commands
motors. Ground-truth object geometry is used by the tray planner and evaluator;
ACT still receives only camera images and measured joints.

Profile `r1pro-sim-pick-place-v1` uses two RGB images of 160x160 and 20 measured
positions: torso 1-4, left arm 1-7, right arm 1-7, left/right grippers. Actions
are absolute positions in that same order at 20 Hz. Grippers use metres; arm and
torso joints use radians. Passive follower fingers and household free joints
never become policy action coordinates. Three separate base coordinates are
added to the coordinator only for mobile scenes.

The tested deployment executes all 30 predicted actions (1.5 seconds), then
observes again. `prepare_r1pro_deployment` records the original weight checksum
and this inference setting without changing trained weights. Shorter partial
chunks performed worse in physical evaluation.

## Physical acceptance and simulation approximations

Success requires a lift above 6 cm with both right fingertip pads in contact,
followed by release, full containment in the tray and low bottle velocity for
one second. Transport additionally checks obstacle contacts, tray containment,
destination error and settling. The evaluator uses ground truth to score the
result; ACT only receives images and measured joints.

The bottle and delivery tray are free bodies. Neither is welded to the robot,
and neither receives scripted pose updates after reset. The tray starts with
physical table support. Its two rectangular handles are gripped by the four
finger pads. Friction-solver refinement prevents numerical creep during a long
hold. Once ACT stops, extra torso damping steadies the load, and unused policy
RGB rendering pauses while the full native viewer and physics continue. The physical regression test also opens both hands away from the table
and verifies that the tray falls.

Delivery checks four loaded finger contacts, tilt, bottle containment and
obstacle contacts. Before opening the hands at the destination, the actual
laptop tabletop must support the tray. Final success requires release, low tray
velocity, correct position and bottle containment. Failed delivery keeps
`success: false` in `result.json`, even if the initial ACT bottle task succeeded.

The model uses pinned Galaxea R1Pro CAD converted to MuJoCo. The CAD has zero
finger travel limits; the task scene supplies opposing 0-0.05 m finger travel,
joint coupling and simple flat fingertip contact pads. Travel is based on the
Galaxea G1 gripper reference. Contact friction, servo gains, gravity compensation
and collision geometry are simulation approximations, not hardware calibration.

Mobility is an actuated planar XY/yaw stage with parked wheel joints. It keeps
chassis height and tilt fixed; this is not calibrated wheel/steering dynamics.
The planner checks the full robot and held-cargo geometry in a copied physics
state, including the departure turn. Only that planning copy transforms cargo
poses; the live tray moves through finger contacts. Motor position targets are
limited per physics timestep (.10 m/s on each XY axis, .15 rad/s yaw), so a
viewer pause cannot turn a wall-clock trajectory delay into a position jump.
The measured peak translation speed in the first complete native run was .099 m/s.

The kitchen passage is usable in the checked carrying posture. The suggested
counter/stool fallback is not part of this command: the higher countertop and
lower stools need different grasp/placement postures. The verified destination
remains the real table beside the laptop.

This worktree does **not** use the main checkout's R1Pro planar preview stack.
It shares the `r1pro/base_x`, `r1pro/base_y`, `r1pro/base_yaw` joint names, but
the preview uses a mock adapter and a newer planning-model API. The user's
unfinished locomanipulation branch has not been merged or replaced.

The old floating-tray baseline remains available with `policy-house`, `--mobile`
and `--transport-x/-y`. That mode fixes the tray to the base; it does not perform
the two-handed delivery. Use `--deliver-to-laptop` for the free physical tray.

References:

- [Galaxea URDF](https://github.com/userguide-galaxea/URDF), pinned at
  `2e5d31e1784481a34d178006c0d0e18e0a84a82a`.
- [GalaxeaManipSim](https://github.com/OpenGalaxea/GalaxeaManipSim), gripper
  reference at `abe7f5161eeaa150e6eaffdf443af5df7f23f356`. Its SAPIEN runtime is
  not used by this MuJoCo stack.
- [Galaxea R1 hardware guide](https://docs.galaxea-ai.com/Guide/R1/R1_Hardware_Guide).

## Training and evidence

All paths below are relative to `recordings/r1pro-act-task/`.

| Artifact | Contents / result |
|---|---|
| `raw-60`, `dataset-60` | 60 successful tabletop demonstrations, 13,320 frames; 54 train / 6 validation episodes |
| `raw-house-30` | 30 successful house demonstrations, 6,660 frames |
| `raw-mixed-40`, `dataset-mixed-40` | First 20 tabletop + first 20 house episodes; 36 train / 4 validation |
| `train-stable-4000` | ACT trained 4,000 steps with corrected joint normalization |
| `policy` | Selected tabletop deployment; unchanged 4,000-step weights, 30-action execution |
| `train-mixed-1500` | Tabletop weights fine-tuned 1,500 steps on the balanced dataset |
| `policy-house` | House bottle / fixed-tray baseline deployment |
| `raw-free-tray-30`, `dataset-free-tray-30` | 30 contact-based bottle demonstrations with the free handled tray, seeds 7000–7029 |
| `train-free-tray-1000`, `policy-free-tray` | Separate 1,000-step house adaptation for the changed tray |
| `eval-free-tray-1000/result.json` | 10/10 physical bottle successes with the free tray, seeds 5000–5009 |
| `eval-stable-4000-chunk30/result.json` | 20/20 physical tabletop successes, seeds 1010-1029 |
| `eval-mixed-final-house/result.json` | 10/10 physical house successes, seeds 4000-4009 |
| `native-house-complete/result.json` | Original fixed-tray ACT + carry success, seed 5000 |
| `native-tray-9/result.json`, `native-tray-10/result.json` | Complete native ACT + physical two-handed tray delivery, seeds 5000 and 5001 |

The original fixed-tray native run accepted seven ACT chunks, lifted the bottle 11.67 cm,
stopped ACT in 8.02 ms, and carried the bottle about 1.06 m from the starting
base position to (-0.35, -1.0). The bottle remained released and settled inside
the tray, with no reported obstacle contacts.

ACT uses a pretrained ResNet18 image backbone, model width 256, feedforward width
1024, eight heads, two encoder layers, one decoder layer and two VAE encoder
layers. Training used batches of 16, four data workers, learning rate 1e-4
(backbone 1e-5); house fine-tuning used 5e-5 (backbone 2e-5). Complete training
configuration and optimizer state are retained under each run's checkpoints.

The converter recomputes joint statistics in float64 and gives constant-command
joints unit scale. LeRobot's original float32 reduction produced incorrect means
and zero variance for fixed commands; normalizing tiny passive vibration also
created misleading inputs. Earlier `train-4000` results predate this correction
and are not the selected policy. Checkpoint selection used physical outcomes,
not just validation loss. The legacy 18-joint `demo_act_sim` and labelled
`demo_r1pro_checkpoint` remain deployment diagnostics with untrained outputs;
they are not the trained task described here.

## Collect, convert and evaluate

The collector saves each completed episode and atomically updates its manifest.
Rerunning with the same output and settings resumes collection. The converter
creates a standard local LeRobot dataset in a new output directory.

```bash
python -m dimos.robot.galaxea.r1pro.demo_collect_grasping \
  --output "$PWD/recordings/r1pro-act-task/raw-new" --episodes 60

uv run --frozen --project dimos/imitation/policy/lerobot/python \
  --with-editable . python -m dimos_lerobot.prepare_r1pro_dataset \
  --source "$PWD/recordings/r1pro-act-task/raw-new" \
  --output "$PWD/recordings/r1pro-act-task/dataset-new"

uv run --frozen --project dimos/imitation/policy/lerobot/python \
  --with-editable . --with mujoco==3.10.0 \
  python -m dimos_lerobot.demo_r1pro_pick_place \
  --artifact "$PWD/recordings/r1pro-act-task/policy-house" \
  --output "$PWD/recordings/r1pro-act-task/eval-new-house" \
  --episodes 10 --start-seed 6000 --action-steps 30 \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --mobile --no-viewer
```

Add `--scene-package ... --mobile` to the collector for the old house baseline.
For the physical delivery scene, collect with `--scene-package ... --free-tray`.
Evaluate its bottle policy with `policy-free-tray`, `--scene-package ...` and
`--free-tray` in place of the old `policy-house`/`--mobile` options. That evaluator
scores ACT bottle loading; the native `--deliver-to-laptop` launcher additionally
executes and scores both-hand carrying and placement.
The evaluator can also open the full viewer; `--no-viewer` is for unattended
batch scoring, and `--video` saves overview MP4s. Offscreen RGB capture still
uses GLFW here, not EGL, and requires the GPU host's graphical session.

## Background jobs and recovery after disconnect

Run further collection, training, tuning and long evaluations as detached jobs
on the GPU host. A saved Bash script, `nohup`, `setsid`, redirected input/output,
and a PID/exit-code file keep the work independent of this terminal or Codex
connection. Closing the client PC does not stop the job. Host shutdown/reboot
still stops processes; training can then resume from its saved checkpoint.

For example, prepare a new house fine-tuning job (this does not modify either
selected deployment):

```bash
job_dir="$PWD/recordings/r1pro-act-task/jobs/next-house"
mkdir -p "$job_dir"
cat > "$job_dir/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd /home/mustafa/dimos-wt/r1pro-act-sim
source .venv/bin/activate
export PYTHONPATH="$PWD"
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
job_dir="$PWD/recordings/r1pro-act-task/jobs/next-house"
printf '%s\n' "$$" > "$job_dir/pid"
trap 'printf "%s\n" "$?" > "$job_dir/exit-code"' EXIT
uv run --frozen --project dimos/imitation/policy/lerobot/python \
  --with-editable . python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/r1pro-pick-place \
  --dataset.root="$PWD/recordings/r1pro-act-task/dataset-mixed-40" \
  --dataset.eval_split=0.1 \
  --policy.path="$PWD/recordings/r1pro-act-task/policy" \
  --policy.device=cuda --policy.push_to_hub=false \
  --policy.optimizer_lr=0.00005 --policy.optimizer_lr_backbone=0.00002 \
  --steps=1500 --batch_size=16 --num_workers=4 --env_eval_freq=0 \
  --eval_steps=500 --max_eval_samples=256 --log_freq=100 --save_freq=500 \
  --wandb.enable=false \
  --output_dir="$PWD/recordings/r1pro-act-task/train-next-house"
SH
nohup setsid --fork bash "$job_dir/run.sh" > "$job_dir/job.log" 2>&1 < /dev/null &
```

Choose new job/output names for each experiment and avoid launching duplicate
GPU training jobs. Monitor without reattaching a terminal to the process:

```bash
tail -f recordings/r1pro-act-task/jobs/next-house/job.log
cat recordings/r1pro-act-task/jobs/next-house/pid
cat recordings/r1pro-act-task/jobs/next-house/exit-code
```

Ctrl-C while tailing only stops the log viewer. `exit-code` appears on completion;
0 means success. To recover interrupted training, replace the training invocation
in a new detached job script with:

```bash
uv run --frozen --project dimos/imitation/policy/lerobot/python \
  --with-editable . python -m lerobot.scripts.lerobot_train \
  --config_path="$PWD/recordings/r1pro-act-task/train-next-house/checkpoints/last/pretrained_model/train_config.json" \
  --resume=true
```

This uses saved optimizer/RNG/training state. Resuming a completed run does not
add training steps unless its target step count is increased. Saved jobs keep
executing across a client disconnect; new agent decisions require reconnecting.
