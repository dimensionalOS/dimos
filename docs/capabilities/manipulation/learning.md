---
title: "The Learning Loop"
description: "Collect demonstrations, build a dataset, train a policy with LeRobot, and run it back on the robot."
---

This is the full loop, on one page:

```
demonstrate ──▶ session.db ──▶ dataset ──▶ train ──▶ run on the robot
 (VR / teach)    (recorded)    (LeRobot      (ACT)     (policy module)
                                or HDF5)
```

Each arrow is one command. By the end you will have taught a robot a task by showing it, not programming it.

## 1. Demonstrate

Two ways to produce demonstrations. Both record the same thing — camera frames, joint states, and episode markers — into a timestamped session database.

### With a VR headset

Best when you have a Quest 3. You see what you are doing, and the arm tracks your hand.

```bash
dimos --simulation run learning-collect-quest-xarm7   # MuJoCo, no hardware
dimos run learning-collect-quest-piper                # real Piper + RealSense
```

Open `https://<host-ip>:8443/teleop` in the Quest browser, accept the certificate, tap Connect. Then:

| Button | Action |
|--------|--------|
| **A** (hold) | Engage — the arm tracks your controller only while held |
| **B** | Start recording / save the episode |
| **Y** | Discard the episode in progress |

A take is: hold **A**, move into place, press **B**, do the task, press **B** again. The terminal confirms every save:

```
[collect] ▶ RECORDING episode  (state=recording  saved=0  discarded=0)
[collect] ✓ SAVED episode      (state=idle       saved=1  discarded=0)
```

Save each good take with **B** before quitting — an episode still recording at shutdown is dropped.

### By hand-teaching

Best for arms you can move by hand. The Galaxea A1Z runs gravity compensation while you guide it through the task:

```bash
uv run dimos a1z teach --camera-index 0 --task "pick up the object"
```

`SPACE` starts an episode and saves it, `g` toggles the gripper, `d` discards — and pressing `d` between episodes undoes the last save. Real hardware only for now; there is no A1Z simulator yet. Replay any episode to check what you captured:

```bash
uv run dimos a1z replay ~/.local/state/dimos/recordings/a1z_teach_<timestamp>.db
```

### What you end up with

Either way, the recorder prints the path of a session database:

```
~/.local/state/dimos/recordings/session_xarm7_20260622_120000.db
```

One file per run, never overwritten. It holds three streams: `color_image`, `coordinator_joint_state`, and the episode start/save/discard markers. Record enough successful episodes to cover the variation the policy will encounter, including camera pose, lighting, object pose, and motion timing.

## 2. Build a dataset

DataPrep reads the session database, aligns camera and joint streams onto one clock, splits at the episode markers, and writes a training dataset:

```bash
dimos dataprep build \
  --source ~/.local/state/dimos/recordings/session_xarm7_20260622_120000.db \
  --config dimos/learning/dataprep/example_config.json
```

The config maps recorded streams to dataset features — which stream is the observation image, which is the state, what rate to resample at. Copy `example_config.json` and adjust; the A1Z ships its own at `dataprep/galaxea_a1z_state_config.json`. By default the **action** for each frame is the *next* frame's measured joint state, which is what next-state behavioral cloning expects.

Output is a LeRobot v3 dataset by default; pass `-f hdf5` for HDF5. Check what you built:

```bash
dimos dataprep inspect data/datasets/session
```

You get features, shapes, dtypes, and episode counts — worth a look before spending GPU hours. Each dataset also carries a `dimos_meta.json` recording exactly how it was built.

## 3. Train

Training happens in [LeRobot](https://github.com/huggingface/lerobot), pointed straight at your DataPrep output. No upload step, no format shuffling:

```bash
uv sync --extra lerobot     # once

uv run lerobot-train \
  --dataset.repo_id=my_task \
  --dataset.root=./data/datasets/session \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=outputs/my_task_act \
  --job_name=my_task_act \
  --wandb.enable=false
```

Use `--policy.device=mps` on Apple silicon or `cpu` for a slow smoke test. Policy quality depends on demonstration count, consistency, and coverage; a completed training run does not by itself mean the policy will generalize. The checkpoint you deploy lands at:

```
outputs/my_task_act/checkpoints/last/pretrained_model
```

## 4. Run it on the robot

`LeRobotPolicyModule` wraps the checkpoint as a DimOS module: camera frames and joint states in, joint commands out, at the policy's control rate.

On the A1Z it is one command:

```bash
uv run dimos a1z run-policy \
  outputs/my_task_act/checkpoints/last/pretrained_model \
  --task "pick up the object" \
  --duration 20
```

It will not surprise you: loading and hardware initialization ask for confirmation, and inference starts only once live camera and joint observations are flowing.

For other arms, compose `LeRobotPolicyModule` into a blueprint next to a camera and the coordinator. The module never moves hardware at startup — its `execute_learned_policy` skill starts inference explicitly, and `stop_learned_policy` halts it with the robot holding position.

For the complete A1Z dependency setup, exact dataset conversion command, camera selection, safety notes, and multi-policy agent blueprint, use the [Galaxea A1Z hardware and learning guide](/dimos/robot/manipulators/galaxea_a1z/README.md).

## When something looks wrong

Work backwards through the loop. A policy behaving strangely is usually a dataset problem; a dataset problem is usually a recording problem.

- `dimos dataprep inspect` — do shapes, rates, and episode counts match what you expect?
- `dimos a1z replay` — does the recorded motion look like what you demonstrated?
- Recordings from before the `coordinator_joint_state` stream rename need a config pointing at the old name — or just re-record.
