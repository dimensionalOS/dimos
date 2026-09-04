# Imitation Learning for Manipulation

DimOS provides one CLI workflow for collecting robot demonstrations, preparing a
LeRobot dataset, training a policy, and running the checkpoint. The preview
supports OpenYAM with a 640×480, 30 FPS wrist RGB camera.

```text
collect ──▶ recording ──▶ prepare ──▶ dataset ──▶ train ──▶ checkpoint ──▶ run
                .mcap                    LeRobot
```

An **Imitation Workflow** is a built-in binding between three robot-specific
pieces: a collection Blueprint, a DataPrep Profile, and a rollout Blueprint. It
does not replace a Blueprint, store session state, or configure LeRobot
training. Choose the workflow explicitly at each robot-facing step.

## Choose a workflow

```bash
dimos imitation list
```

| Workflow | Demonstration control | Required hardware |
| --- | --- | --- |
| `openyam-teach` | Hand guidance with gravity compensation | OpenYAM, wrist camera |
| `openyam-quest` | Quest teleoperation | OpenYAM, wrist camera, Quest |

Quest is optional. The main path uses `openyam-teach`; policy rollout also runs
without Quest unless you pass `--quest-control`.

## 1. Collect demonstrations

Support the arm before starting. Collection activates hardware, and stopping
the command de-torques the arm.

```bash
dimos --can-port follower_l imitation collect openyam-teach \
  --task "pick up the red block" \
  --camera-device 0
```

The command starts the collection stack, opens its terminal controls, and stops
the complete stack when you exit. It prints a unique recording path under the
DimOS state directory. Pass `--recording PATH` to choose another new path; the
command refuses to overwrite an existing artifact.

| Key | Action |
| --- | --- |
| Space | Start an episode; press again to save it |
| D | Discard the current episode |
| Q | Stop while idle; press twice to confirm de-torque |
| Ctrl-C | Emergency best-effort shutdown |

Normal exit is blocked during a take. Save or discard first. An interruption
during a take leaves it incomplete, so DataPrep can report and exclude it.

To collect through Quest instead, select the other workflow:

```bash
dimos --can-port follower_l imitation collect openyam-quest \
  --task "pick up the red block"
```

The CLI refuses to start collection or rollout while another DimOS coordinator
is active.

## 2. Prepare and inspect the dataset

Use the recording path printed by `collect`:

```bash
dimos imitation inspect RECORDING.mcap --workflow openyam-teach
dimos imitation prepare openyam-teach RECORDING.mcap
```

`prepare` selects the workflow's fixed DataPrep Profile and applies strict
episode validation. It writes a unique default directory under the DimOS state
directory and prints the resolved source and destination. Use `--output DIR` to
choose another new directory.

Inspect either the recording or prepared dataset:

```bash
dimos imitation inspect RECORDING.mcap --workflow openyam-teach
dimos imitation inspect DATASET_DIR
```

The prepared LeRobot dataset contains these fixed features:

| Feature | Shape | Source |
| --- | --- | --- |
| `observation.images.wrist` | RGB, 480×640×3 | Wrist camera |
| `observation.state` | 7 values | Six OpenYAM joints and gripper |
| `action` | 7 values | Measured teach state or accepted Quest command |

## 3. Train with LeRobot

`dimos imitation train` is a transparent pass-through to `lerobot-train` in
the pinned LeRobot environment. DimOS adds no training defaults and does not
rewrite arguments, output, or exit codes.

```bash
dimos imitation train \
  --dataset.repo_id=local/openyam-wrist \
  --dataset.root=DATASET_DIR \
  --policy.type=act \
  --output_dir=outputs/openyam-act
```

Run `dimos imitation train --help` for the installed LeRobot options.

## 4. Run the checkpoint

The normal rollout requires no Quest headset:

```bash
dimos --can-port follower_l imitation run openyam-teach CHECKPOINT_DIR \
  --task "pick up the red block" \
  --camera-device 0 \
  --device cuda
```

Before enabling the terminal's start control, DimOS performs a non-moving
preflight. It loads the checkpoint and processors and checks:

- required feature keys and image, state, and action dimensions;
- finite checkpoint action bounds and an available inference device;
- fresh 640×480 RGB observations and all configured live joints;
- the configured policy trajectory task in the control coordinator.

Preflight never sends a trajectory. After it passes, Space starts or stops the
policy. Stop the policy before exiting the stack.

Add Quest only when an operator wants teleoperation takeover:

```bash
dimos --can-port follower_l imitation run openyam-teach CHECKPOINT_DIR \
  --task "pick up the red block" \
  --quest-control
```

Quest tasks have higher control priority than policy trajectories. Quest input
cannot bypass policy preflight.

## Compatibility boundary

DimOS can detect feature keys, tensor dimensions, action bounds, device
availability, image shape, and live joint availability. Matching dimensions do
not prove that a checkpoint was trained for the same robot or joint order.
Because training is a transparent pass-through and checkpoints carry no DimOS
workflow lineage, the operator must pair the checkpoint with the correct
workflow and task.

## Maintainer notes

Built-in workflow bindings live in `dimos.imitation.workflows`. A binding keeps
the public CLI small while the collection and rollout implementations remain
ordinary Blueprints and DataPrep remains an offline profile-driven transform.
External workflow discovery is outside this preview.

The merge gate is automated: registry and CLI tests, lifecycle tests, Blueprint
composition tests, DataPrep tests, isolated runtime preflight tests, formatting,
and type checks. Release still requires an OpenYAM hardware smoke test covering
one saved teach episode, dataset preparation, non-moving preflight, policy
start/stop, Ctrl-C cleanup, and optional Quest takeover.
