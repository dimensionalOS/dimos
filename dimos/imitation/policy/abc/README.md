# ABC-DiT on dual YAM

This backend targets the released 75k-step bottles-in-bin checkpoint from
[ABC](https://github.com/amazon-far/abc). The inference source is pinned to
`6bc6586721cf0c409ccee80f675a28de9b9b2f5e`; see
[upstream provenance](python/abc_minimal/UPSTREAM.md). Model loading, normalization,
image processing, sampling, and CUDA graphs use that source. Training and
simulation packages are excluded from the deployment environment.

## Checkpoint and assets

Download the official checkpoint (about 8 GB) and normalization statistics:

```bash
mkdir -p checkpoints/abc
curl -L --fail https://abc-data.timehorizons.org/checkpoints/bottles_release_prep_75k.pt -o checkpoints/abc/bottles_75k.pt
curl -L --fail https://abc-data.timehorizons.org/misc/norm_stats.json -o checkpoints/abc/norm_stats.json
```

CLIP text weights and its tokenizer are fetched by the upstream loader on first
use. Set `--policy.clip-cache-dir` to choose their cache location. DINO weights
are included in this complete inference checkpoint; separate DINO pretraining
weights are only needed by the upstream training pipeline. The snapshot retains
ABC, CLIP, and DINO license texts under `python/`.

## Launch

The rig needs a D405 at each wrist and a separate overhead RGB camera. Device
identities belong to their camera module configuration. The wrist cameras use
the existing RealSense native module with depth disabled; overhead uses the
existing webcam module. All provide 640×480 RGB at 30 Hz.

```bash
dimos run dual-openyam-policy-quest-rollout --daemon \
  --policy.backend abc \
  --policy.policy-path checkpoints/abc/bottles_75k.pt \
  --policy.norm-stats-path checkpoints/abc/norm_stats.json \
  --policy.device cuda \
  --controlcoordinator.left-can-port follower_l \
  --controlcoordinator.right-can-port follower_r \
  --left-wrist.serial-number LEFT_D405_SERIAL \
  --right-wrist.serial-number RIGHT_D405_SERIAL \
  --overhead.hardware.camera-index /dev/video0
```

Use `dual-openyam-policy-rollout` for the same policy and cameras without Quest.
No policy motion starts at launch. Preflight through the policy RPC, then use
Quest A, or attach `dimos imitation rollout` and press Space. Existing stop
controls remain in place. Detaching the UI does not stop an active policy.

The blueprint maps wrist views to `left` and `right` and overhead to `top`.
ABC expects `[left joints 1–6, left gripper, right joints 1–6, right gripper]`.
The adapter translates to/from the coordinator's canonical joint order.
Grippers use normalized opening in `[0, 1]`; check direction on the physical rig.

The official default predicts 30 actions with 10 diffusion steps, executes the
first 15, then infers from fresh observations. The reference simulator uses
17 control substeps of 0.002 seconds: the default action period is 0.034 seconds.
`--policy.fps` and `--policy.execution-steps` override those execution settings.
Compilation and graph capture occur during motion-free preflight.

## Selecting LeRobot on the same rig

Change the backend and checkpoint, and bind the same camera ports to the
LeRobot checkpoint's feature names. For a checkpoint trained on canonical
DimOS joint order and three images:

```bash
dimos run dual-openyam-policy-rollout \
  --policy.backend lerobot \
  --policy.policy-path CHECKPOINT_DIR \
  --policy.policy-joint-names null \
  --policy.fps 30 \
  --policy.image-mapping '{"left_wrist_image":"observation.images.left_wrist","right_wrist_image":"observation.images.right_wrist","overhead_image":"observation.images.overhead"}'
```

Supply the same hardware/device flags as above. The existing two-camera
collection preset has no overhead feature; use a checkpoint with the declared
camera features, or explicitly bind only the views the checkpoint requires.

## Physical acceptance

1. Confirm camera identities and orientation, joint names, and gripper direction.
2. Run preflight; confirm model readiness and fresh inputs with no motion.
3. Start a rollout and verify both arms and grippers execute valid chunks.
4. Stop during execution, then explicitly restart from fresh observations.
5. Record inference latency, accepted/rejected chunks, errors, and task outcomes.

Physical rollout is pending on the prepared rig. Software tests do not establish
task success or physical calibration. No task success-rate threshold is required
for this integration increment.

## Verified GPU smoke test

On 2026-09-11, the released checkpoint loaded and ran on an RTX 3090 with
PyTorch 2.11.0+cu128. Three synthetic 640×480 RGB views and the normalization
statistics' mean state produced finite `(30, 14)` chunks. The adapter reported
15 execution steps at 29.4118 Hz, matching the reference defaults.

| Measurement | Observed value |
|---|---|
| Checkpoint and text encoder loading | 10.07 s |
| First prediction, including compilation and graph warmup | 46.17 s |
| Two subsequent predictions | 74.58 ms, 74.49 ms |
| Peak allocated GPU memory | 8.43 GB |

These are inference smoke measurements, not task-performance results. Physical
cameras, CAN buses, gripper calibration, and real robot execution remain untested
on this host. Both backend environments also passed fresh-subprocess RPC checks.
