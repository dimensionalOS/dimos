# Imitation Learning for Manipulation

DimOS separates demonstration collection from policy rollout. A collection
profile defines what is recorded and converted to LeRobot. A rollout profile
defines the live inputs and joint outputs expected by one policy backend.

```text
collection profile                         rollout profile + backend
        │                                            │
        ▼                                            ▼
robot + cameras ──▶ MCAP ──▶ LeRobot dataset     robot + cameras ──▶ actions
```

Run `dimos imitation list` to see both catalogs.

| Collection | Inputs | Joint order |
| --- | --- | --- |
| `openyam-teach` | Wrist RGB, measured joint state | OpenYAM 7-D |
| `openyam-quest` | Wrist RGB, accepted commands | OpenYAM 7-D |
| `dual-openyam-quest` | Left and right wrist RGB | DimOS dual 14-D |

| Rollout | Backend | Inputs | Joint order |
| --- | --- | --- | --- |
| `openyam-lerobot` | LeRobot | Wrist RGB | OpenYAM 7-D |
| `dual-openyam-abc` | Amazon ABC-DiT | Top, left wrist, right wrist RGB | ABC 14-D |

Collection and rollout names need not match. The dual collection schema is a
LeRobot dataset contract; the released ABC checkpoint has its own three-camera
contract and does not train from that dataset in this change.

## Declare cameras

Every physical camera is explicit as `--camera STREAM=DEVICE`. Repeat the
option for multi-camera profiles. The stream names come from the selected
profile; unknown, duplicate, or missing names fail before hardware starts.

```bash
dimos imitation collect dual-openyam-quest \
  --task "fold the towel" \
  --left-can-port follower_l \
  --right-can-port follower_r \
  --camera left_wrist_image=/dev/video0 \
  --camera right_wrist_image=/dev/video2
```

For one arm:

```bash
dimos --can-port follower_l imitation collect openyam-teach \
  --task "pick up the red block" \
  --camera wrist_image=0
```

The terminal uses Space to start/save, D to discard, and Q twice to stop while
idle. Stopping the command de-torques the arm.

## Prepare and train LeRobot data

```bash
dimos imitation inspect RECORDING.mcap --workflow dual-openyam-quest
dimos imitation prepare dual-openyam-quest RECORDING.mcap
dimos imitation train \
  --dataset.repo_id=local/dual-openyam-quest \
  --dataset.root=DATASET_DIR \
  --policy.type=act \
  --output_dir=outputs/dual-openyam-act
```

`prepare` uses the collection profile's feature keys, joint order, 30 Hz rate,
and 20 ms alignment tolerance. It currently writes LeRobot datasets. The train
command remains a transparent pass-through to the pinned LeRobot CLI; ABC
training and an ABC dataset writer are outside this scope.

## Run a policy

LeRobot single-arm rollout:

```bash
dimos --can-port follower_l imitation run openyam-lerobot CHECKPOINT_DIR \
  --task "pick up the red block" \
  --camera wrist_image=0 \
  --device cuda
```

Released ABC-DiT dual-arm rollout:

```bash
dimos imitation run dual-openyam-abc CHECKPOINT.pt \
  --task "put the plastic bottles in the bin" \
  --left-can-port follower_l \
  --right-can-port follower_r \
  --camera top_image=/dev/video0 \
  --camera left_wrist_image=/dev/video2 \
  --camera right_wrist_image=/dev/video4 \
  --device cuda
```

ABC never fabricates the checkpoint's top view from a wrist camera. Missing
top, left, or right input fails before the policy can move.

Preflight loads the selected backend, validates its feature dimensions and
control task, then requires a fresh aligned observation set. The newest fresh
anchor sample is paired with the nearest buffered sample for every other input;
the maximum skew is 20 ms. Preflight sends no trajectory.

During rollout, the common runtime validates finite 2-D action chunks, applies
backend bounds when available, and caps every submitted trajectory to 0.5 s.
The ABC binding predicts 30 steps and executes at most 15 at 30 Hz. Trajectory
joint names use the profile's exact order, including both grippers.

## ABC dependency and verification boundary

The ABC isolated environment vendors only inference code from the
[Amazon ABC repository](https://github.com/amazon-far/abc) at revision
`6bc6586721cf0c409ccee80f675a28de9b9b2f5e`: the DiT model, image/normalization
helpers, CLIP text encoder, and optional CUDA graph helper. It does not install
MuJoCo, Warp, visualization, training, dataset export, or RTC dependencies.

Automated tests use tiny synthetic backends and arrays. They do not download or
load the multi-gigabyte released checkpoint. Before hardware rollout, perform a
manual offline check on the target GPU:

1. Start the three cameras and coordinator with motion disabled.
2. Call `preflight_rollout()` and confirm backend `abc`, all observations ready,
   and no error.
3. Run one inference without submitting its trajectory and inspect the returned
   shape `(30, 14)`, finite values, and grippers in `[0, 1]`.
4. Enable motion at low-risk initial conditions and verify Space/Ctrl-C cancels
   the `policy_rollout` task.

Matching dimensions do not prove embodiment compatibility. Operators must pair
an artifact with its exact profile, camera roles, calibration, joint convention,
and task.

## Maintainer contract

`PolicyIOProfile` maps backend feature keys directly to typed DimOS image or
joint-position streams. `declare_recorder()` and `declare_policy_module()` turn
that import-time profile into ordinary module subclasses, so Blueprint
autoconnect still sees concrete `In[Image]` and `In[JointState]` ports.

The shared runtime owns input buffering, timing, lifecycle RPCs, cancellation,
action validation, execution horizon, and trajectory creation. Isolated
backends own artifact loading, tensor layout, preprocessing, normalization, and
in-process prediction. Add a new backend by declaring a backend-specific config,
a profile-bound host module, and a small `PolicyBackend` implementation without
changing the coordinator or autoconnect.
