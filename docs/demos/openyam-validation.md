# OpenYAM simulation validation, 2026-09-07

Work is on `feat/openyam-sim-completion`, based on T5b commit `21d428cc9`.
It has not been pushed. T7 and the live model acceptance run are incomplete.

## What was verified

| Check | Result | Evidence under the local artifact directory |
| --- | --- | --- |
| Planner and simulator calibration | Joint limits agree; maximum measured TCP error 1.472 mm across six poses | `openyam-model-check.json` |
| Classical bimanual sequence with cameras | 5/5 cycles, both bottles wholly contained after both placements, each lifted at least 5 cm for two seconds | `openyam-bimanual-identical.jsonl` |
| ACT deployment using a diagnostic checkpoint | Real isolated process, GPU, three camera streams, 14 controlled joints; 12 accepted chunks, stop RPC returned in 3.7 ms; rollout inactive, subsequent reset works | `openyam-policy-smoke3.log`, `smoke-checkpoint/` |
| Scripted collection | 47 physically successful saved takes, 34 valid for training, one interrupted take discarded | `openyam-training.db`, `openyam-before-resume-inspect.json` |
| LeRobot export | 34 episodes, 11,084 frames, three RGB cameras and 14 state/action values, 15 Hz | `dataset-34/` |
| Training and checkpoint compatibility | 20 CPU ACT training steps completed; saved checkpoint loads through the DimOS adapter and returns a finite 50×14 action chunk | `act-cpu-smoke/`, `trained-checkpoint-roundtrip.json` |
| Focused regressions | 185 passed, 3 deselected; separate 13 CPU simulation tests and 23 isolated-process tests passed | `openyam-regression-final.log`, `openyam-sim-cpu-final.log`, `openyam-isolated-tests.log` |
| Reset stability regression | Six tests passed, including moving and never-settling scenes | `openyam-reset-settle-tests.log` |
| Local MCP execution after reset correction | 5/5 fixed-response trials, both bottles contained, both arms home, no failed skills; cameras disabled | `openyam-local-mcp-settle.jsonl` |
| Recording portability | Six generator tests passed; equivalent planner assets match across checkouts, changed mesh contents do not | `openyam-manifest-tests.log` |

The artifact directory is `/home/mustafa/dimos/recordings/openyam-completion`.
`openyam-classical-right.mp4` is a three-camera recording of one actual scripted
right-arm take. It is not a learned-policy rollout. Neither the diagnostic
constant-action checkpoint nor the 20-step training checkpoint is a demonstrated
pick-and-place policy.

The earlier T5 verification moved objects to easier positions and treated
displacement as success. Those results do not establish this demo's acceptance.
The replacement generator uses the composed task's spawn positions, verifies a
sustained lift, requires each skill to succeed, and checks complete sampled
object containment inside the bin after the arm retreats.

## Agent acceptance and reset correction

The local MCP diagnostic uses fixed tool-call playback with API credentials
removed. Its camera-free CPU variant initially failed five of five strict
trials, even though several bottles accidentally landed in the bin after a
reported grip loss. The checker now requires successful skill results, both
arms home, and physical containment.

The failure came from scanning immediately after reset: simulation registration
caches the scan's point cloud while bottles are still settling onto the table.
The successful generator waited before scanning. Two direct CPU comparison
trials passed after waiting, with and without setting the bottle pose.
`reset_scene` now waits for measured position and orientation stability before
returning. Adding a post-lift delay did not fix the problem and was removed.
The corrected reset passed all five local MCP trials, recorded in
`openyam-local-mcp-settle.jsonl`.

A live model has not been tested in this run. Automatic approval review rejected
the proposed external model call because its payload and destination had not
been explicitly authorized. The proposed destination, prompt, and data are
documented in [the model-call review](/docs/demos/openyam-model-call-review.md); user approval
is pending. Fixture playback validates transport and execution, not model judgment.

## Remaining work and host failure

At 02:14:22 PDT the NVIDIA kernel driver reported Xid 79, "GPU has fallen off
the bus", followed by Xid 154, "Node Reboot Required". `nvidia-smi` reports no
devices. EGL initialization hangs even outside DimOS. The driver log is saved
as `gpu-failure.log`; the underlying hardware/driver cause has not been determined.
The recovery status is described in
[NVIDIA's Xid catalog guidance](https://docs.nvidia.com/deploy/xid-errors/analyzing-xid-catalog.html).

Mesa software rendering worked in an isolated renderer check but sustained only
about two three-camera batches per second at the required resolution. It cannot
satisfy this dataset's timing gates. A later host renderer test also entered a
driver wait and was stopped. CPU physics without cameras remains usable.

The collection restart did not produce additional episodes. The source recording
and its manifest are preserved. Resume now keeps existing rows, checks the full
IO contract and scene/model hashes, and counts only new takes that pass both
physical and timing validation. Each collection module gets its own worker by
default. The effect of that worker separation on timing has not yet been measured.
Planner identity includes mesh contents and excludes machine-specific checkout
paths. The preserved manifest was upgraded after verifying its previous digest
and proving identical geometry in the feature and durable hub checkouts.

After restoring the GPU, use the feature checkout and preserved recording:

```bash
MUJOCO_GL=egl uv run python -m dimos.robot.manipulators.dual_openyam.tool_generate_demos \
  --resume --episodes 66 --max-attempts 110 --seed 47 \
  --zenoh-scout-addr 224.0.0.224:17467 \
  --recording /home/mustafa/dimos/recordings/openyam-completion/openyam-training.db \
  --report /home/mustafa/dimos/recordings/openyam-completion/collection-resumed.jsonl

dimos imitation prepare dual-openyam-sim \
  /home/mustafa/dimos/recordings/openyam-completion/openyam-training.db \
  --output /home/mustafa/dimos/recordings/openyam-completion/dataset-100
```

Confirm at least 100 valid exported episodes, then train locally:

```bash
dimos imitation train \
  --dataset.repo_id=local/dual-openyam-sim \
  --dataset.root=/home/mustafa/dimos/recordings/openyam-completion/dataset-100 \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --policy.chunk_size=50 --policy.n_action_steps=7 \
  --policy.dim_model=256 --policy.n_heads=8 --policy.dim_feedforward=1024 \
  --policy.n_encoder_layers=2 --policy.n_vae_encoder_layers=2 \
  --policy.optimizer_lr=0.0001 --policy.optimizer_lr_backbone=0.00001 \
  --steps=30000 --batch_size=8 --num_workers=4 --env_eval_freq=0 \
  --log_freq=500 --save_freq=10000 --wandb.enable=false \
  --output_dir=/home/mustafa/dimos/recordings/openyam-completion/act-full
```

The CPU check used these architecture/optimizer settings with `device=cpu`,
20 steps, batch size 2, two workers, and a separate output directory. Loss fell
from the first logged average 52.385 at step 5 to 7.442 at step 20; that short
run establishes training compatibility only.

Evaluate the full checkpoint with `tool_evaluate_policy --episodes 10`; require
at least 7/10 actual lifts, gripper releases, and stable deposits before calling ACT stage-ready.
Run `tool_check_agent --episodes 5` through the approved live model and verify
the Rerun viewer in the composed stack. Pim's replacement scene has not been
supplied and would need fresh calibration and physical acceptance checks.

Shutdown still prints shared-memory resource-tracker warnings inherited from
the SHM attachment/unregister lifecycle. The completed motion and stop checks
are separate evidence; these warnings have not been resolved.

Targeted mypy passed 34 source files with `--follow-imports=silent`; this excludes
the separate LeRobot environment and is not a repo-wide type check. Applicable
pre-commit hooks passed; `lfs_check` was skipped for the downloaded scene, which
the pinned setup command reproduces.
