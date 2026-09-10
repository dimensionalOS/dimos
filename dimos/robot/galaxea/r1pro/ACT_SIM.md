# Trained R1Pro ACT in MuJoCo

The implemented task is: **use the right gripper to pick up the blue bottle,
release it into the orange tray, then carry the loaded tray through the house**.
ACT controls manipulation from RGB and measured joints. A separate planner and
coordinator trajectory control the simulated mobile base after ACT stops.

Verified on 2026-09-10: 20/20 new tabletop trials, 10/10 new house manipulation
trials, and one complete native DimOS house manipulation plus transport run.
These trials vary the bottle's starting XY position by up to 12 mm at a known
workstation. They do not establish general household manipulation.

## Run the completed house demo

On the current GPU machine, first close any completed R1Pro demo window or
stop that demo with Ctrl-C in its launch terminal. A window kept open with
`--stay-open` still has a running simulator and messaging connections.

```bash
cd /home/mustafa/dimos-wt/r1pro-act-sim
source .venv/bin/activate
export PYTHONPATH="$PWD"
export MUJOCO_GL=glfw

python -m dimos.robot.galaxea.r1pro.demo_pick_place_stack \
  --artifact "$PWD/recordings/r1pro-act-task/policy-house" \
  --output "$PWD/recordings/r1pro-act-task/my-house-run" \
  --zenoh-scout-addr 224.0.0.224:19467 \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --mobile --seed 5000 --transport-x -0.35 --transport-y -1.0 \
  --stay-open
```

This opens the full native MuJoCo window, waits for observations and CUDA ACT
inference to become ready, runs the learned manipulation, checks the physical
result, stops ACT, and transports the loaded tray. The route first backs away
from the table, then moves across the room. The viewer stays open afterward.
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

For the simple tabletop, use `policy`, omit the scene-package/mobile/transport
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
  a manipulation trajectory task, and an optional base transport trajectory task.
- `R1ProPickPlacePolicy`: the isolated LeRobot runtime with trained ACT weights,
  synchronized observations, normalization, joint bounds and cancellable chunks.
- `PolicySkills`: explicit rollout preflight, start, status and stop controls.

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

The bottle is a free body. It is never welded to the gripper or tray, and its
pose is not scripted after episode reset. The teacher uses IK and gripper
commands to generate contact-based demonstrations.

The model uses pinned Galaxea R1Pro CAD converted to MuJoCo. The CAD has zero
finger travel limits; the task scene supplies opposing 0-0.05 m finger travel,
joint coupling and simple flat fingertip contact pads. Travel is based on the
Galaxea G1 gripper reference. Contact friction, servo gains, gravity compensation
and collision geometry are simulation approximations, not hardware calibration.

Mobility is an actuated planar XY/yaw stage with parked wheel joints. It keeps
the chassis height and tilt fixed. Its chassis ignores low ground contact while
retaining obstacle collision; articulated links and the tray/bottle still
collide with the floor. The onboard tray clears the table by 2 mm. The local
planner checks a copied physics state, then the coordinator executes the path
while the bottle moves through contact with the tray. This is not learned
navigation or validated wheel/steering dynamics. The user's unfinished
locomanipulation branch has not been merged or replaced.

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
| `policy-house` | Selected house deployment from that final checkpoint |
| `eval-stable-4000-chunk30/result.json` | 20/20 physical tabletop successes, seeds 1010-1029 |
| `eval-mixed-final-house/result.json` | 10/10 physical house successes, seeds 4000-4009 |
| `native-house-complete/result.json` | Full native DimOS house ACT + carry success, seed 5000 |

The full native run accepted seven ACT chunks, lifted the bottle 11.67 cm,
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

Add `--scene-package ... --mobile` to the collector for house demonstrations.
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
