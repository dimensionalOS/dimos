# OpenYAM simulation validation, 2026-09-08

Current source is in `/home/mustafa/dimos-wt/hub`, branch `hackmit/t7-resume`,
based on fd94f07ec. The T7 implementation commits are b1d17eac6, 00351206e,
and 97ded8b38.
Nothing has been pushed. The 100-episode dataset is complete; the full 30,000-step ACT
training run has completed successfully. All three saved checkpoints scored
0/10 physical successes. Trained-policy controls and classical recovery passed;
ACT remains experimental.

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
| Trained policy MCP controls and recovery | Three accepted chunks; stop roundtrip 7.22 ms, inactive after one second; reset and classical pick/place/home/reset succeeded | `t7-trained-policy-controls.jsonl`, `.log`, `t7-trained-policy-controls-summary.json` |
| Camera viewer | All three views visibly rendered in headless Chromium with WebGL, no browser errors | `t7-viewer-webgl.png`, `.json`; initial WebGPU failure in `t7-viewer.png`, `.json` |

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
| 10,000 | 0.040 | `act-full/checkpoints/010000`; 0/10 physical successes |
| 20,000 | 0.028 | `act-full/checkpoints/020000`; 0/10 physical successes |
| 30,000 | 0.024 | `act-full/checkpoints/030000`; 0/10 physical successes |

LeRobot rounds console step labels. `diagnostics/demo_training_progress.py`
uses the configured 500-step logging interval and explicit resume markers to
write exact step numbers to `t7-training-metrics.json`.

Training finished with service exit status 0 and all 11 checkpoint files verified.
`t7-training-loss.png` and `.svg` plot all 60 logging intervals.

At the user's request, training was handed over at a verified complete 10k
checkpoint to `openyam-act-t7.service`, owned by the Linux user service manager.
The replacement process loaded model, optimizer, RNG, and dataset sampling state
and advanced beyond 10k. Its progress bar counts the remaining 20k updates.
Closing VS Code does not stop this service; the laptop must stay awake and the
user logged in. It does not automatically restart after reboot. Recovery uses
the last complete checkpoint, repeating any unsaved steps. See local `RESUME.md`.

### Physical evaluation and recovery

Physical evaluation uses ten trials with seed 1000 and 1.5 cm horizontal
spawn jitter. Success requires an actual lift of at least 5 cm, an open gripper,
at least one second of continuous bin containment, and complete sampled bottle
containment after stopping the policy and waiting another second. The evaluator
now explicitly requires that dwell even for a deposit immediately before timeout.

All checkpoints miss the 7/10 acceptance threshold:

| Checkpoint | Lift at least 5 cm | Gripper open at final check | Stable bin deposit / success | Runtime errors |
| --- | --- | --- | --- | --- |
| 10k | 10/10 | 0/10 | 0/10 | 0 |
| 20k | 10/10 | 10/10 | 0/10 | 0 |
| 30k | 10/10 | 3/10 | 0/10 | 0 |

Every trial produced a peak lift above 5 cm, but no trial achieved stable or
final bin containment. Peak-lift ranges were 0.11834–0.26405 m at 10k,
0.11926–0.31541 m at 20k, and 0.14743–0.30793 m at 30k. The summed trial wall
times were 501.59, 505.84, and 505.95 seconds, respectively. Reports, logs, and
summaries use `policy-eval-10000`, `policy-eval-20000`, and `policy-eval-30000`
with `.jsonl`, `.log`, and `-summary.json` suffixes.

Gripper readback in this diagnostic is normalized travel (0 closed, 1 open);
dataset state/action grippers use meters. An open gripper alone does not prove
a successful release into the bin. All 30 corrected trials had no policy
runtime error. ACT is **experimental, not ready for the reliable stage segment**.
Training loss does not establish physical success. No hyperparameters or model
weights were tuned after the benchmark failures. Inference mode and image
preprocessing were also checked in the installed LeRobot source; both match the
adapter's intended use. This does not identify the learned failure's root cause.

The final checkpoint passed the composed stack's local MCP controls check:
`run_policy` started, `policy_status` reported three accepted chunks, and
`stop_policy` returned inactive in 7.22 ms roundtrip, remaining inactive after a
one-second wait. A subsequent reset, classical right-arm pick (0.09975 m lift),
place, home, physical bin-containment check, and final reset all succeeded.
The diagnostic made 16 local tool calls and exited 0. It used an explicit local
model fixture with API credentials removed; it does not test live model judgment.

The first evaluation exposed a deployment bug: MuJoCo measured up to 0.048046 m
at a gripper's 0.0475 m soft limit. Predicted action targets were bounded, but the
runtime prepended the unbounded measured state as trajectory point zero, which
the controller correctly rejected. That diagnostic was stopped after five
completed rows and is preserved separately as `policy-eval-100.jsonl` and `.log`;
it is not the benchmark above. The command anchor now obeys the same backend
action bounds as later command points, while the model receives unchanged
measurements and the controller keeps its hard-limit and start-state guards.
The corrected complete benchmark restarted all ten original seeds.

The regression failed on the out-of-range anchor before the fix. All 15 focused
runtime/skill tests now pass, both changed production files pass targeted mypy,
and applicable pre-commit hooks pass. Evidence: `t7-start-boundary-red.log` and
`t7-start-boundary-tests.log`. The evaluator also records final pose and gripper
position to make physical failures inspectable without changing the score.

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

The completed training command (do not launch it again):

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

The composed camera layout was visually verified in headless Chromium. Its
WebGPU backend crashed while creating a mapped 32 MiB buffer; the error page and
browser log are preserved. Adding `&renderer=webgl` to the same viewer URL
rendered the table, left-wrist, and right-wrist views successfully, with no
browser errors. `t7-viewer-webgl.png` is the verified screenshot. This is a
headless browser check, not a native-viewer or full dress-rehearsal result.

`openyam-classical-right.mp4` remains a three-camera recording of an actual
classical right-arm take, not an ACT success video. Stage polish still needs a
3D layout, a full rehearsal script and two dry runs. Pim's replacement scene has
not been supplied and would require fresh calibration and physical acceptance.
See the [operating guide](/docs/demos/openyam-simulation.md) for setup and stage
commands. The updated handoff and local `RESUME.md` preserve the outstanding
live-model approval and next stage work; no training process remains active.
