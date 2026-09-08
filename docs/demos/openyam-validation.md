# OpenYAM simulation validation, 2026-09-08

Current source is in `/home/mustafa/dimos-wt/hub`, branch `hackmit/t7-resume`,
based on fd94f07ec. The T7 implementation commits are b1d17eac6 and 00351206e.
Nothing has been pushed. The 100-episode dataset is complete; the full ACT
training run is in progress. Learned physical success has not yet been measured.

The artifact directory is `/home/mustafa/dimos/recordings/openyam-completion`.
Paths in the evidence tables are relative to that directory. `RESUME.md` and
`t7-commands.md` preserve the current service, recovery instructions, and exact
commands. The source recording and its manifest remain at their original paths.

## T7 v2 results

| Check | Result | Evidence |
| --- | --- | --- |
| GPU after reboot | RTX 4090 Laptop GPU, driver 570.211.01, CUDA 12.8; initially 48 C, 1569/16376 MiB | T7 continuation in the project handoff |
| Planner/simulator calibration | Joint limits match; six TCP poses, maximum error 1.471656 mm | `openyam-model-check.json`; repeated after reboot |
| EGL rendering | Real RGB render completed, shape 240×320×3, pixel range 60–255 | Post-reboot preflight; geometry check alone does not test EGL |
| Isolated LeRobot environment | Frozen sync checked 99 packages; PyTorch 2.10.0+cu128 reports CUDA available | T7 continuation in the project handoff |
| Existing data with strict quality | 34 valid out of 47 saved, 11,084 aligned frames | `t7-before-strict.json` |
| Existing data with bounded fill | 47 valid out of 47 saved, 15,342 aligned frames | `t7-before-fill.json`, `dataset-47/` |
| Resumed collection | 53 attempted, 53 physically successful and quality-valid saved, zero discarded | `t7-collection-resumed.jsonl`, `t7-collection-summary.json` |
| Collection time | 1325.98 s summed cycle wall time (22.10 min), plus startup, warmup, and shutdown | `t7-collection-summary.json` |
| Final dataset | 100 episodes, 33,319 frames, 15 Hz; 47 filled frames total (0.1411%) | `dataset-100/dimos_meta.json`, `dataset-100/meta/info.json` |
| Camera-enabled local MCP acceptance | 5/5 bimanual trials, both bottles wholly contained, both arms home, no failed skills | `t7-camera-mcp.jsonl`, `t7-camera-mcp.messages.jsonl` |
| Quality/generator tests | 49 passed; targeted mypy passed three changed production files | T7 implementation checks |
| Profile, workflow, and CLI tests | 24 passed | `t7-profile-cli-tests.log` |

The source database contains 100 saved episodes and one previously interrupted
and discarded episode. No old recording rows were changed during resume.
All new accepted takes lifted the bottle at least 0.09915 m during the required
sustained lift. Exported frames use three 240×320 RGB videos and 14 state/action
values in the documented left-arm, right-arm, left-gripper, right-gripper order.

### Quality decision and camera stalls

Only the simulation profile changed to fill mode. Its normal alignment limit
remains 20 ms, and both collection and export reject episodes with more than 3%
filled emitted frames. Generic quality configuration defaults to no additional
fill cap, and existing hardware profiles remain strict.

Fill operates per source: the nearest value within the normal alignment window
is used when available; otherwise a previous causal value is held. Leading
incomplete targets are trimmed. A filled frame can therefore contain a mixture
of new and held sources, and held values can exceed 20 ms of age. Camera gap
and rate limits are diagnostics in this mode. This is more specific than the
original prompt's description of repeating a whole frame.

| Filled frames in an episode | Original 47 episodes | New 53 episodes |
| --- | --- | --- |
| 0 | 34 | 40 |
| 1 | 3 | 5 |
| 2 | 8 | 7 |
| 3 | 2 | 1 |

The maximum per-episode filled fraction is 0.9231% across the final dataset.
The first five resumed takes all passed: 341/335/340/339/335 aligned frames,
with 0/0/0/1/2 filled frames. Simultaneous camera gaps of 100–166 ms remained
visible despite separate workers. The stall's root cause is unresolved; no
`gc.freeze()` or garbage-collection threshold changes were applied.

The generator checks the completed motion before saving, discarding the active
take on failure, then checks the final saved interval again. The stationary
pre-save inspection tail is included in that final interval. Export enforces
the same cap, so an invalid saved tail cannot bypass training quality checks.

The manifest was migrated only after asserting its previous strict quality
settings and exact equality of every other field against the current scene,
model, IO, task, and joint order. The original manifest is preserved as
`openyam-training.scene.strict-t7-backup.json`.

### ACT training

The requested model, optimizer, and training command are unchanged: 30,000 CUDA
steps, batch size 8, chunk size 50, execution prefix 7, four data workers, and
checkpoints every 10,000 steps. There is no Hub upload or W&B logging.
The model has 15,564,814 parameters. Video decoding uses PyAV because
TorchCodec is unavailable in this environment.

| Global step | Logged mean loss | Checkpoint / physical evaluation |
| --- | --- | --- |
| 5,000 | 0.054 | No checkpoint scheduled |
| 10,000 | 0.040 | `act-full/checkpoints/010000`; evaluation pending |
| 20,000 | Pending | Training in progress |
| 30,000 | Pending | Training in progress |

LeRobot rounds console step labels. `diagnostics/demo_training_progress.py`
uses the configured 500-step logging interval and explicit resume markers to
write exact step numbers to `t7-training-metrics.json`.

At the user's request, training was handed over at a verified complete 10k
checkpoint to `openyam-act-t7.service`, owned by the Linux user service manager.
The replacement process loaded model, optimizer, RNG, and dataset sampling state
and advanced beyond 10k. Its progress bar counts the remaining 20k updates.
Closing VS Code does not stop this service; the laptop must stay awake and the
user logged in. It does not automatically restart after reboot. Recovery uses
the last complete checkpoint, repeating any unsaved steps. See local `RESUME.md`.

### Pending physical evaluation and recovery

Evaluate the final checkpoint in ten trials with seed 1000 and 1.5 cm horizontal
spawn jitter. Success requires an actual lift of at least 5 cm, an open gripper,
at least one second of continuous bin containment, and complete sampled bottle
containment after stopping the policy and waiting another second. The evaluator
now explicitly requires that dwell even for a deposit immediately before timeout.

The threshold is 7/10. If the final checkpoint is below it, also evaluate the
10k and 20k checkpoints on the same ten seeds and report all three; do not infer
physical success from training loss or tune without examining the failures.
Then verify `run_policy`, `policy_status`, and `stop_policy` through the composed
agent stack, followed by reset and a successful classical pick/place recovery.
These checks have not yet been completed with this trained policy.

## Commands used

Run host Python from `/home/mustafa/dimos-wt/hub` using its linked `.venv` and
`PYTHONPATH=/home/mustafa/dimos-wt/hub`. Only the isolated LeRobot environment
uses the explicit `uv` commands below. GPU and shared-memory runs need host access;
the restricted tool namespace cannot see the NVIDIA driver.

```bash
nvidia-smi
MUJOCO_GL=egl PYTHONPATH=/home/mustafa/dimos-wt/hub timeout --kill-after=5s 45s /home/mustafa/dimos/.venv/bin/python -m dimos.robot.manipulators.dual_openyam.tool_check_sim_model
uv sync --project dimos/imitation/policy/lerobot/python --frozen
uv run --project dimos/imitation/policy/lerobot/python --frozen --with-editable . python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

After verifying and migrating the manifest's quality settings:

```bash
PYTHONPATH=/home/mustafa/dimos-wt/hub .venv/bin/python -m dimos.cli.dimos imitation prepare dual-openyam-sim \
  /home/mustafa/dimos/recordings/openyam-completion/openyam-training.db \
  --output /home/mustafa/dimos/recordings/openyam-completion/dataset-47

MUJOCO_GL=egl PYTHONPATH=/home/mustafa/dimos-wt/hub .venv/bin/python -m dimos.robot.manipulators.dual_openyam.tool_generate_demos \
  --resume --episodes 53 --max-attempts 110 --seed 47 \
  --zenoh-scout-addr 224.0.0.224:17467 \
  --recording /home/mustafa/dimos/recordings/openyam-completion/openyam-training.db \
  --report /home/mustafa/dimos/recordings/openyam-completion/t7-collection-resumed.jsonl

PYTHONPATH=/home/mustafa/dimos-wt/hub .venv/bin/python -m dimos.cli.dimos imitation prepare dual-openyam-sim \
  /home/mustafa/dimos/recordings/openyam-completion/openyam-training.db \
  --output /home/mustafa/dimos/recordings/openyam-completion/dataset-100
```

The full training command (already running; do not launch it again):

```bash
PYTHONPATH=/home/mustafa/dimos-wt/hub .venv/bin/python -m dimos.cli.dimos imitation train \
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

`diagnostics/demo_resume_act_background.py` launches the saved configuration
with `--resume=true` in the background service. It checks checkpoint completeness
and refuses duplicate training processes. All training logs append to
`t7-training-full.log`. Additional exact commands are in `t7-commands.md`.

## Previous validation and remaining stage work

The September 7 completion work established real contact-based classical
manipulation and corrected planner/simulator limits, TCP calibration, grasp
candidate fallback, measured motion settling, and reset stability. The previous
T5 checks moved objects to easier locations and counted displacement; those
results are not used as acceptance evidence here. Neither the diagnostic
constant-action checkpoint nor the 20-step CPU checkpoint establishes learned
pick/place success.

| Previous check | Result | Evidence |
| --- | --- | --- |
| Classical bimanual sequence with cameras | 5/5, both bottles wholly contained, each lifted at least 5 cm for two seconds | `openyam-bimanual-identical.jsonl` |
| Diagnostic ACT deployment | Isolated CUDA process, three camera streams, 14 joints, 12 accepted chunks; stop RPC 3.7 ms and reset worked | `openyam-policy-smoke3.log`, `smoke-checkpoint/` |
| CPU training compatibility | 20 steps; checkpoint reload returns finite 50×14 actions | `act-cpu-smoke/`, `trained-checkpoint-roundtrip.json` |
| Focused regressions | 185 passed, 3 deselected; separate 13 CPU simulation and 23 isolated-process tests | `openyam-regression-final.log`, `openyam-sim-cpu-final.log`, `openyam-isolated-tests.log` |
| Reset stability | Six tests including moving and never-settling scenes; 5/5 camera-free local MCP trials | `openyam-reset-settle-tests.log`, `openyam-local-mcp-settle.jsonl` |
| Recording portability | Six tests; geometry identity includes mesh contents and excludes checkout paths | `openyam-manifest-tests.log` |

Reset waits for measured scene stability before a scan can cache its geometry.
The new camera-enabled MCP result above also passes after this correction.
Fixture playback uses fixed tool calls with API credentials removed and validates
execution; live model judgment remains untested. A previous automatic approval
review rejected the external model call because payload and destination were not
explicitly authorized. The concrete call is documented in
[the model-call review](/docs/demos/openyam-model-call-review.md). The updated
project plan requests five live trials before rehearsal; those still need to run.

The September 7 Xid 79 GPU failure and driver log are preserved in
`gpu-failure.log`. The driver and EGL renderer respond after the user's reboot;
this does not establish the original fault's cause. Shared-memory resource-tracker
cleanup warnings remain unresolved. Targeted historical mypy checks covered 34
files with `--follow-imports=silent`, excluding the isolated LeRobot environment;
no repo-wide test or type-check claim is made. Applicable pre-commit hooks passed;
`lfs_check` was skipped for the downloaded scene reproducible by the pinned setup.

`openyam-classical-right.mp4` is a three-camera recording of an actual classical
right-arm take. The composed Rerun viewer and full stage flow need a visual dry
run. Pim's replacement scene has not been supplied and would require fresh
calibration and physical acceptance. See the
[operating guide](/docs/demos/openyam-simulation.md) for setup and stage commands.
