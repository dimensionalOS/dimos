# LeRobot LIBERO policy rollout gate

This optional/manual gate evaluates LeRobot LIBERO checkpoints through DimOS
robot-learning seams. The default policy is the official VLA-JEPA LIBERO
checkpoint. The script also has an X-VLA LIBERO configuration for a two-stage
policy-agnostic gate. There are two runnable paths:

- **Fast benchmark path** (default): lockstep reset/snapshot/inference/action/step
  ownership stays in the benchmark runner so simulation can remain simple,
  parallelizable, and faster-than-realtime where possible.
- **Live policy stream path** (`--live-policy-stream`): runtime observation
  assembly, `RobotPolicyModule`, and `ControlCoordinator` run as DimOS modules.
  The policy module emits action chunks on a stream, the coordinator executes
  them via the policy chunk control task, and the demo forwards the resulting
  native LIBERO action row to the runtime. This is the realtime parity path.

Default fast benchmark flow:

```text
BenchmarkPolicyEvalRunner
  -> RuntimeSidecarClient.reset(...)
  -> LiberoRobotPolicyObservationBuilder.build(...)
  -> RobotPolicyModule.infer_action(RobotPolicyObservation)
     -> VlaJepaLiberoRobotContract.to_backend_batch(...)
     -> LeRobotBackend.infer_batch(...)
     -> VlaJepaLiberoRobotContract.from_backend_output(...)
  -> RobotPolicyAction
  -> BenchmarkPolicyEvalRunner adapts to RuntimeActionFrame
  -> RuntimeSidecarClient.step(...)
```

`RobotPolicyModule` is a first-class DimOS `Module`. It owns backend lifecycle,
policy reset, contract conversion, inference, and `RobotPolicyAction` emission.
Benchmark evaluation owns sidecar reset/step, sample construction, action-frame
adaptation, scoring, artifacts, videos, and the gate. `BenchmarkPolicyEvalModule`
wraps the runner for module/blueprint-compatible lockstep evaluation.

Policy backends and contracts are selected through lazy registries:

```text
RobotPolicyModule.blueprint(
    backend_type="lerobot",
    backend_params={"policy_family": "vla_jepa", "checkpoint_id": "lerobot/VLA-JEPA-LIBERO"},
    contract_type="vla_jepa_libero",
)
```

Supported policy-family defaults:

| `--policy-family` | Default checkpoint | Default contract | LeRobot extra |
| --- | --- | --- | --- |
| `vla_jepa` | `lerobot/VLA-JEPA-LIBERO` | `vla_jepa_libero` | `vla_jepa` |
| `xvla` | `lerobot/xvla-libero` | `xvla_libero` | `xvla` |

Override `--checkpoint` or `--contract-type` only when testing a known-compatible
variant. A checkpoint swap alone is not enough for policy-family changes because
LeRobot policy families can use different processor inputs, action modes, and
optional dependency extras.

The helper `lerobot_libero_policy_eval_blueprint(...)` returns a blueprint-shaped
composition of `RobotPolicyModule` and `BenchmarkPolicyEvalModule` for this gate.

The default fast benchmark path intentionally bypasses `ControlCoordinator`.
The sidecar still runs in native LIBERO action mode and accepts the official
relative end-effector delta + gripper action surface:

```text
space_id = libero.ee_delta_6d_gripper.normalized.v1
shape = (7,)
bounds = [-1, 1]
```

The live policy stream path preserves the same native LIBERO action semantics.
It does **not** reinterpret VLA-JEPA actions as Panda joint targets. Instead,
the policy chunk task claims a synthetic 7-DOF action surface, emits one
index-bounded normalized action row through `ControlCoordinator`, and the demo
uses that row as a `RuntimeActionFrame` with the same
`libero.ee_delta_6d_gripper.normalized.v1` space id.

Live stream flow:

```text
LiberoRobotPolicyObservationBuilder.build(...)
  -> RobotPolicyModule.update_observation(RobotPolicyObservation)
  -> RobotPolicyModule.trigger_action_chunk_inference()
     -> LeRobotBackend.infer_batch(..., use_action_chunk=True)
     -> VlaJepaLiberoRobotContract.chunk_from_backend_output(...)
  -> RobotPolicyModule.policy_action_chunk stream
  -> ControlCoordinator.robot_policy_action_chunk stream input
  -> PolicyChunkControlTask index-bounded execution
  -> RuntimeActionFrame(space_id=libero.ee_delta_6d_gripper.normalized.v1)
  -> LiberoProRuntimeModule.step(...)
```

## Dependencies

Run this only in an environment with:

- The standard LIBERO Python package installed. The runner auto-discovers the
  package `bddl_files` and `init_files` roots. You can still pass explicit
  `--bddl-root` and `--init-states-root` paths for custom or offline assets.
- LeRobot from GitHub main. The PyPI `lerobot` release may not include the
  newest policy families yet.
- The selected checkpoint accessible from Hugging Face, unless using the
  fake-backend smoke path.

On current Python/CMake environments, LIBERO's `egl-probe` dependency may need
the CMake compatibility policy set during install/run:

```bash
export CMAKE_POLICY_VERSION_MINIMUM=3.5
```

## Full 50-episode gate

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
uv run \
  --with libero \
  --with "lerobot[vla_jepa] @ git+https://github.com/huggingface/lerobot.git" \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --artifact-dir artifacts/benchmark/lerobot-vla-jepa-libero
```

The episode matrix is generated by the runner: `libero_object` task indices
`0..9` crossed with init state indices `[0, 1, 2, 3, 4]`, for 50 total episodes.
The run passes only when:

```text
success_rate > 0.50
```

Use `--device cuda` or another LeRobot-supported device when needed.

## X-VLA two-stage gate

X-VLA support is selected with `--policy-family xvla`. The backend resolves
`lerobot.policies.xvla.modeling_xvla.XVLAPolicy`, defaults to
`lerobot/xvla-libero`, and writes `policy_family`, `contract_type`, `gate_stage`,
and checkpoint metadata into artifacts.

Install X-VLA outside the main project dependencies. In the LIBERO sidecar or
another disposable environment, PyPI `lerobot[xvla]` is sufficient for the
current smoke/gate path:

```bash
uv pip install 'lerobot[xvla]'
PYTEST_VERSION=1 uv run python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py --help
```

If validating against LeRobot source instead, use an isolated `uv run --with`
invocation or sidecar virtualenv and verify dependency resolution before running
the gate. Do not add X-VLA dependencies to the main DimOS `pyproject.toml`.

Stage 1 validates the synchronous benchmark path without ControlCoordinator
policy-chunk execution:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
PYTEST_VERSION=1 uv run python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --policy-family xvla \
  --device cuda \
  --episodes-limit 10 \
  --gate-stage benchmark \
  --artifact-dir artifacts/benchmark/lerobot-xvla-libero-benchmark-10
```

Stage 2 runs the module-native live policy stream path after Stage 1 passes:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
PYTEST_VERSION=1 uv run python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --policy-family xvla \
  --live-policy-stream \
  --device cuda \
  --episodes-limit 10 \
  --save-videos \
  --gate-stage live \
  --artifact-dir artifacts/benchmark/lerobot-xvla-libero-live-10-videos
```

Both X-VLA stages use the same hard threshold:

```text
success_rate > 0.50
```

LeRobot's X-VLA docs identify the LIBERO checkpoint with `control_mode=absolute`
and `domain_id=3`. The DimOS `xvla_libero` contract prepares a 20D X-VLA state,
passes `domain_id=3`, flips the main camera by 180° to match LeRobot's LIBERO
processor convention, and emits absolute end-effector pose actions with:

```text
space_id = libero.ee_pose_axis_angle_gripper.absolute.v1
shape = (7,)
bounds = finite values; axis-angle entries are not normalized to [-1, 1]
```

The LIBERO runtime uses `native_absolute` for X-VLA so OSC pose control keeps
`use_delta=False`. Do not clamp or reinterpret X-VLA actions as
`libero.ee_delta_6d_gripper.normalized.v1`; that action space is reserved for
VLA-JEPA's normalized delta path. If the real gate fails during setup or
contract smoke, check whether the installed LeRobot checkpoint processor or the
local LIBERO runtime action mode changed before treating the failure as a
policy-quality regression.

Latest local Stage 1 artifact:

```text
artifacts/benchmark/lerobot-xvla-libero-benchmark-10-image-flip-videos/summary.json
episodes=10 successes=9 success_rate=0.9 passed=true
```

## Live policy stream parity gate

Run the realtime-style path by adding `--live-policy-stream`. Acceptance uses
the real `lerobot/VLA-JEPA-LIBERO` checkpoint over a 10-episode `libero_object`
slice and the same hard threshold:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
uv run \
  --with libero \
  --with "lerobot[vla_jepa] @ git+https://github.com/huggingface/lerobot.git" \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --live-policy-stream \
  --device cuda \
  --episodes-limit 10 \
  --save-videos \
  --artifact-dir artifacts/benchmark/lerobot-vla-jepa-libero-live-10-videos
```

`--fake-backend` is only a plumbing smoke test. Do not use it for live parity
acceptance. `--live-chunk-timeout-s` caps how long each tick waits for a
coordinator-emitted action, and `--live-max-stale-waits` aborts the episode
after repeated live action timeouts so inference-shape or backend failures fail
fast instead of spending `max_steps * timeout` seconds on one episode.

## Native-action smoke path

To test protocol/sidecar/native-action plumbing without downloading LeRobot:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 \
uv run --with libero \
  python scripts/benchmarks/demo_lerobot_libero_policy_rollout.py \
  --fake-backend \
  --live-policy-stream \
  --episodes-limit 1 \
  --no-enforce-gate
```

This still constructs the module-backed workflow and uses the selected real
contract (`vla_jepa_libero` by default, `xvla_libero` with
`--policy-family xvla`), but the backend returns a fixed 7D action or a
single-row action chunk when `--live-policy-stream` is set.

## Artifacts

The top-level artifact directory contains:

- `summary.json`
- `episodes.jsonl`
- `runtime_description.json`
- `checkpoint_metadata.json`
- `run_config.json`
- `cleanup_status.json`

Each per-episode subdirectory contains the runner artifacts for that episode and
the sidecar log. Full videos or image dumps are not saved by default. Passing
`--save-videos` writes lightweight MP4 previews for the configured camera streams
under each episode directory, for example:

```text
episodes/libero_object_task0_init0/videos/libero_object_task0_init0/agentview.mp4
```

Live stream runs also write `live_path_diagnostics.json` per episode. It records
chunk counts, accepted refill triggers, consumed actions, inference status
counts, stale waits/deactivations, and the last command sequence observed from
`ControlCoordinator`.
