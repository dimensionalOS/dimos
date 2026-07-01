# LeRobot LIBERO policy rollout gate handoff

This handoff captures the verified real 50-episode
`lerobot/VLA-JEPA-LIBERO` benchmark gate and the command needed to reproduce it.

## Goal

Run the real policy benchmark and verify:

```text
checkpoint = lerobot/VLA-JEPA-LIBERO
suite = libero_object
episodes = 50  # task_index 0..9 × init_state_index 0..4
pass condition = success_rate > 0.50
```

The gate is implemented in:

```text
scripts/benchmarks/demo_lerobot_libero_policy_rollout.py
```

Expected artifact directory:

```text
artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full
```

## Verified local result

The real non-fake 50-episode gate passed locally with checkpoint
`lerobot/VLA-JEPA-LIBERO` on CUDA:

```text
episodes=50 successes=45 success_rate=0.900 passed=True
```

Verified summary artifact:

```json
{
  "episodes": 50,
  "passed": true,
  "success_rate": 0.9,
  "success_threshold": 0.5,
  "successes": 45
}
```

Artifact directory:

```text
artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full
```

## Current repo state

Implementation is complete for the OpenSpec change
`add-lerobot-libero-policy-rollout`:

- runtime protocol `RuntimeActionFrame` union
- native LIBERO action mode in sidecar
- `RobotPolicyModule`
- `LeRobotBackend`
- `VlaJepaLiberoRobotContract`
- `BenchmarkPolicyEvalRunner`
- 50-episode gate script
- optional MP4 video artifacts
- standard LIBERO package asset auto-discovery
- noninteractive LIBERO config creation
- checkpoint-saved LeRobot processor loading for VLA-JEPA normalization stats
- LIBERO image orientation parity with LeRobot's `LiberoProcessorStep`
- native reset parity: 10 no-op settle steps, relative `OSC_POSE` control, and
  an effective simulator horizon that preserves the requested policy horizon

Validation already run locally:

```text
real 50-episode gate: 45/50 successes, success_rate=0.900, passed=True
5-episode smoke: 5/5 successes, success_rate=1.000, passed=True
targeted pytest: 25 passed
ruff: passed
pytest targeted: 44 passed earlier; 16 passed after LIBERO setup/doc updates
openspec validate add-lerobot-libero-policy-rollout --type change: passed earlier
```

## Important environment findings

### 1. Use LeRobot from GitHub main

PyPI `lerobot==0.5.1` is not sufficient for this gate in the current Python 3.12
environment. It did not provide a clean VLA-JEPA import path. Installing from
GitHub main produced `lerobot==0.5.2` and exposed:

```text
lerobot.policies.vla_jepa.modeling_vla_jepa
```

Use a GitHub install, ideally with the VLA-JEPA extra.

### 2. LIBERO may need a CMake compatibility variable

Installing `libero` can fail while building `egl-probe==1.0.2` under newer
CMake. This worked locally:

```bash
export CMAKE_POLICY_VERSION_MINIMUM=3.5
```

### 3. Standard LIBERO assets are package assets

The official standard LIBERO benchmark assets are part of the installed LIBERO
package:

```text
libero/libero/bddl_files
libero/libero/init_files
```

The runner/sidecar now auto-discovers those roots. Do **not** use
`zhouxueyang/LIBERO-Pro` for this gate: that dataset contains perturbation-style
folders such as `libero_object_task`, not the standard `libero_object` suite used
by `lerobot/VLA-JEPA-LIBERO`.

### 4. First startup may download LIBERO robot assets

The first LIBERO sidecar startup may download robot/assets from Hugging Face.
Use a longer startup timeout.

## Recommended remote command

From the repo root:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
MUJOCO_GL=egl \
uv run \
  --with libero \
  --with 'lerobot[vla_jepa] @ git+https://github.com/huggingface/lerobot.git' \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --device cuda \
  --save-videos \
  --startup-timeout-s 240 \
  --artifact-dir artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full
```

If the direct-reference extra syntax fails in `uv`, fall back to explicit deps:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
MUJOCO_GL=egl \
uv run \
  --with libero \
  --with git+https://github.com/huggingface/lerobot.git \
  --with diffusers \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --device cuda \
  --save-videos \
  --startup-timeout-s 240 \
  --artifact-dir artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full
```

If more missing VLA-JEPA dependencies appear, prefer the first command form with
`lerobot[vla_jepa]` from GitHub main rather than adding packages one by one.

## Local attempts already made

### Final real 50-episode gate

Command:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
MUJOCO_GL=egl \
uv run \
  --with libero \
  --with 'lerobot[vla_jepa] @ git+https://github.com/huggingface/lerobot.git' \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --device cuda \
  --save-videos \
  --startup-timeout-s 240 \
  --artifact-dir artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full
```

Result:

```text
episodes=50 successes=45 success_rate=0.900 passed=True
```

Key fixes needed to reach this result:

- `LeRobotBackend` loads checkpoint-saved policy pre/postprocessors so VLA-JEPA
  state normalization and action unnormalization use checkpoint stats.
- `VlaJepaLiberoRobotContract` flips LIBERO images on height and width axes and
  emits contiguous CHW float32 `[0, 1]` tensors.
- The sidecar native reset mirrors LeRobot's 10 no-op settle steps and relative
  `OSC_POSE` control; the simulator horizon is extended by those settle steps so
  the benchmark still gets the configured policy horizon.

### Setup-blocked attempt before fixes

Command:

```bash
uv run python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --allow-asset-bootstrap \
  --save-videos \
  --artifact-dir artifacts/benchmark/lerobot-vla-jepa-libero-real-gate
```

Result: setup artifact created, but policy did not execute.

Root cause:

```text
LIBERO_PRO_HF_REPO_ID was not set
```

This is no longer the recommended path for the standard LIBERO gate.

### Fake-backend LIBERO package preflight after fixes

Command:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
uv run --with libero \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --fake-backend \
  --episodes-limit 1 \
  --no-enforce-gate \
  --max-steps 1 \
  --startup-timeout-s 180 \
  --artifact-dir /tmp/opencode/lerobot-libero-standard-discovery-test-2
```

Result:

```text
episodes=1 successes=0 success_rate=0.000 passed=False
```

This proves sidecar startup, standard LIBERO package asset discovery, runtime
action protocol, and fake backend path work. It is not a policy quality check.

### Real-backend attempt with PyPI LeRobot

Command used `--with libero --with lerobot`.

Result: PyPI `lerobot==0.5.1` installed, but VLA-JEPA import was not usable in
Python 3.12 due an upstream policy-package import/dataclass error.

### Real-backend attempt with GitHub LeRobot main

`importlib.util.find_spec('lerobot.policies.vla_jepa.modeling_vla_jepa')`
succeeded with GitHub main (`lerobot==0.5.2`).

A full gate command then reached checkpoint load but failed because the VLA-JEPA
extra dependency `diffusers` was missing:

```text
ImportError: 'diffusers' is required but not installed.
Install it with: pip install 'lerobot[vla_jepa]'
```

That is why the recommended command now installs `lerobot[vla_jepa]` from GitHub
main.

## What to inspect after the run

Top-level artifacts should include:

```text
summary.json
episodes.jsonl
runtime_description.json
checkpoint_metadata.json
run_config.json
cleanup_status.json
```

For success, check:

```bash
python - <<'PY'
import json
from pathlib import Path
p = Path('artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full/summary.json')
print(json.dumps(json.loads(p.read_text()), indent=2))
PY
```

The benchmark only passes if:

```text
episodes == 50
success_rate > 0.50
passed == true
```

With `--save-videos`, per-episode MP4s should appear under:

```text
artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full/episodes/<episode_id>/videos/<episode_id>/
```

## If it fails remotely

Do not weaken the gate. Iterate on concrete failure causes until either:

1. `success_rate > 0.50`, or
2. there is a hard blocker outside repo control.

Useful places to inspect:

```text
artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full/setup_error.json
artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full/episodes/*/libero_sidecar.log
artifacts/benchmark/lerobot-vla-jepa-libero-real-gate-full/episodes.jsonl
```

Common likely issues:

- missing VLA-JEPA extra deps: use GitHub main with `[vla_jepa]`
- CUDA/device mismatch: pass `--device cuda` or the correct available device
- long first LIBERO asset download: increase `--startup-timeout-s`
- observation key mismatch: inspect per-episode artifacts
  failure reason
- simulator/headless rendering issue: ensure `MUJOCO_GL=egl` or equivalent is
  available on the remote machine

## Reminder

This gate is complete when a real non-fake run writes a 50-episode `summary.json`
with `success_rate > 0.50`. The verified local artifact currently satisfies that
condition with `success_rate=0.900`.
