---
title: "A1Z Learning Workflow"
description: "Record demonstrations, build a dataset, train a LeRobot policy, and execute it on a Galaxea A1Z."
---

The A1Z learning loop is:

```text
hand-teach → session.db → dataset → train → run-policy
```

Complete the [A1Z hardware setup](/docs/capabilities/manipulation/a1z.md) before using
these commands. The arm has no brakes; support it whenever motors may be
disabled and keep the workspace clear.

## Record demonstrations

Start hand-teaching with a webcam selected by its `/dev/videoN` index:

```bash
uv run --no-sync dimos a1z teach --camera-index 0 --task "pick up the object"
```

The arm runs gravity compensation while you guide it. The controls are:

| Key | Action |
| --- | --- |
| Space or Enter | Start an episode, or save the active episode |
| `g` | Toggle the powered gripper open or closed |
| `d` | Discard the active episode, or undo the latest save while idle |
| `q` | Quit, confirming what to do with an active episode |

Use `--gripper-free-drive` to manipulate the gripper by hand instead. Each run
creates a timestamped Memory2 database under the dimOS state directory unless
an explicit output path is supplied. Existing recordings are never
overwritten.

## Validate by replaying

Replay the latest saved episode:

```bash
uv run --no-sync dimos a1z replay /path/to/a1z_teach_<timestamp>.db
```

Use `--episode N` to select another saved episode and `--speed 0.5` to request
half speed. Preflight rejects incomplete, non-finite, out-of-range, or malformed
joint data. Valid motion is smoothed and time-scaled before the command asks for
confirmation and approaches the recorded start pose.

## Build a dataset

Convert the recording with the provided 15 Hz A1Z profile:

```bash
dimos dataprep build \
  --profile dimos.robot.manipulators.a1z.learning:A1Z_LEARNING_PROFILE \
  --source /path/to/a1z_teach_<timestamp>.db \
  --output data/datasets/galaxea_a1z
```

The profile aligns `color_image` and `coordinator_joint_state`, uses the next
measured joint state as the behavioral-cloning target, and excludes discarded
or undone episodes.

## Train and execute a policy

LeRobot inference uses an isolated optional environment because its dependency
versions conflict with the perception and development environments:

```bash
uv sync --extra lerobot --no-default-groups
```

After syncing, ensure the pinned A1Z SDK from the hardware guide remains
installed. Train with LeRobot against the generated dataset, then execute its
`pretrained_model` checkpoint:

```bash
uv run --no-sync dimos a1z run-policy \
  outputs/my_task/checkpoints/last/pretrained_model \
  --task "pick up the object" \
  --duration 20
```

The command asks before initializing hardware, waits for fresh camera and
joint observations, and stops on completion, timeout, interruption, invalid
policy output, or stale observations. The policy runtime does not clip actions;
the A1Z coordinator and adapter remain the actuation and safety boundary.
