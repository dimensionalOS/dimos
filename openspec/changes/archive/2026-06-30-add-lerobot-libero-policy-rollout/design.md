## Context

DimOS already has runtime sidecar infrastructure and a LIBERO sidecar package that can run registered LIBERO tasks behind a lightweight HTTP protocol. The current LIBERO demo proves reset/step/scoring, camera payload export, a local SHM motor plane, and ControlCoordinator-driven joint-position commands.

That path is not enough to verify a learned policy. Official LeRobot LIBERO checkpoints, including `lerobot/VLA-JEPA-LIBERO`, use LIBERO's native relative end-effector-delta-plus-gripper action mode rather than the current Panda joint-position motor surface. The policy rollout architecture therefore needs two separate but cooperating pieces:

1. a robot-learning rollout stack that owns policy lifecycle, contract conversion, and episode execution; and
2. a runtime sidecar native action mode that can accept LIBERO-native actions without pretending they are motor commands.

This change intentionally bypasses ControlCoordinator for v1 policy rollout. The ControlCoordinator action-surface gap is documented separately in `openspec/changes/support-control-coordinator-action-surfaces/control-coordinator-action-surface-gap.md` and should be handled by a later change.

## Goals / Non-Goals

**Goals:**

- Add a first `RobotPolicyModule` shape for policy inference and action emission.
- Add a separate benchmark/evaluation runner for episode matrix, runtime lifecycle, scoring, and artifacts.
- Add a batch-first `PolicyBackend` abstraction and an in-process `LeRobotBackend` for `lerobot/VLA-JEPA-LIBERO`.
- Add a narrow `VlaJepaLiberoRobotContract` that converts between LIBERO sidecar observations, LeRobot backend batches, backend outputs, and native runtime actions.
- Extend the runtime protocol with a native runtime action frame while preserving existing motor action frames.
- Extend the LIBERO sidecar with native LIBERO action mode for official relative end-effector delta plus gripper actions.
- Provide a 50-episode policy gate over `libero_object`, all 10 task indices, init states `[0, 1, 2, 3, 4]`, with `success_rate > 0.50`.
- Emit enough metadata and artifacts to debug policy, contract, sidecar, and version mismatches.

**Non-Goals:**

- No ControlCoordinator dispatch for v1 policy rollout.
- No `JointTrajectoryTask` or `EndEffectorDeltaTrajectoryTask` implementation in this change.
- No EE-delta-to-joint conversion.
- No overloading `MotorActionFrame` for end-effector actions.
- No dynamic observation binding/schema system.
- No general multi-policy registry or universal robot contract framework.
- No mandatory `lerobot-eval` comparison in the acceptance gate.
- No strict dependency version pinning beyond recording versions and resolved checkpoint metadata.

## Decisions

### 1. Split RobotPolicyModule from benchmark lifecycle

The demo must exercise the intended robot-learning architecture rather than directly calling LeRobot from a one-off script. However, the policy module should not own benchmark lifecycle. The v1 architecture should split responsibilities:

- `RobotPolicyModule` owns policy backend lifecycle, public policy reset, observation-to-action inference, and action emission.
- `BenchmarkPolicyEvalRunner` owns episode matrix selection, sidecar lifecycle, sidecar reset/step calls, scoring, success gate, metrics, artifacts, and cleanup.

`RobotPolicyModule` should expose a public reset method because LeRobot policies can keep episode-local state such as action queues or temporal context.

The core flow is:

```text
BenchmarkPolicyEvalRunner
  -> RuntimeSidecarClient.reset(...)
  -> RobotLearningSample / sidecar observation wrapper
  -> RobotPolicyModule.reset(...) at episode boundary
  -> RobotPolicyModule.infer_action(sample)
       -> VlaJepaLiberoRobotContract.to_backend_batch(...)
       -> LeRobotBackend.infer_batch(...)
       -> VlaJepaLiberoRobotContract.from_backend_output(...)
  -> RuntimeSidecarClient.step(RuntimeActionFrame)
```

Alternative considered: implement the gate as a plain LeRobot evaluation script. That would be faster but would not validate the DimOS policy-module/evaluation-runner/backend/contract seams.

### 2. Use a batch-first PolicyBackend interface

`PolicyBackend` should expose a batch-oriented inference method:

```python
class PolicyBackend(Protocol):
    def initialize(self) -> None: ...
    def reset_episode(self) -> None: ...
    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope: ...
    def close(self) -> None: ...
    def describe(self) -> PolicyBackendDescription: ...
```

`select_action` is not the rollout contract. `LeRobotBackend` may use LeRobot's `select_action`, `predict_action_chunk`, processor pipeline, or action queue internally, but `RobotPolicyModule` should only depend on `infer_batch(...)`.

Rationale: LeRobot, starVLA, and Dexbotic-like deployments all converge on explicit observation/action batch or capability surfaces. A batch-first backend also keeps training/inference IO conventions visible at the contract boundary.

### 3. Keep RobotContract as the semantic IO boundary

The v1 contract is backend-specific and runtime-specific:

```text
VlaJepaLiberoRobotContract
```

It owns semantic conversion:

```text
sidecar agentview      -> observation.images.image
sidecar eye-in-hand    -> observation.images.image2
sidecar 8D robot_state -> observation.state
sidecar language       -> task prompt

LeRobot backend output -> RuntimeActionFrame(space_id="libero.ee_delta_6d_gripper.normalized.v1", values=float32[7])
```

It also validates semantic mismatches and fails fast on missing observations, wrong shapes, wrong dtypes, invalid action shapes, non-finite values, or incompatible action range.

The contract does not load policies, schedule inference, call sidecar step, write artifacts, or know about ControlCoordinator.

### 4. Extend StepRequest with a RuntimeActionFrame union

Runtime protocol `StepRequest.action` should become a discriminated union:

```text
MotorActionFrame | RuntimeActionFrame
```

`RuntimeActionFrame` should carry a semantic action surface identity and values, for example:

```text
kind: "runtime_action"
space_id: "libero.ee_delta_6d_gripper.normalized.v1"
values: float32[7]
sequence: int
```

Rationale: one `/step` operation remains the runtime stepping concept, while individual sidecar modes can accept or reject action frame kinds. This avoids adding parallel step endpoints and avoids lying through `MotorActionFrame.q`.

### 5. Add native LIBERO action mode to the sidecar

The existing sidecar joint-position motor mode remains unchanged. Native LIBERO action mode should follow the official LeRobot LIBERO environment setup and validate the resulting environment action spec instead of hardcoding a guessed simulator controller string.

Native mode should:

- construct the LIBERO environment as official LeRobot LIBERO evaluation expects;
- inspect `env.action_spec`;
- require action dimension `(7,)`;
- require bounds compatible with `[-1, 1]`;
- advertise the native action surface and action mode in runtime description metadata;
- accept `RuntimeActionFrame` with matching `space_id`; and
- reject `MotorActionFrame` in native mode.

The term should be "LIBERO action mode", not "controller", because controller is overloaded with DimOS ControlCoordinator and control tasks.

### 6. Use an in-process LeRobotBackend for v1

`LeRobotBackend` runs in the DimOS policy rollout process for v1. It owns:

- checkpoint loading for `lerobot/VLA-JEPA-LIBERO`;
- policy/device initialization;
- LeRobot processor/preprocessor/postprocessor setup as required by the checkpoint;
- `policy.reset()` or equivalent episode reset;
- inference under no-grad/inference mode;
- conversion of raw backend output into `BackendOutputEnvelope`; and
- metadata for artifacts.

Future worker/venv isolation can be added later if dependency conflicts require it. The v1 abstraction seam is `PolicyBackend`, not a separate process boundary.

### 7. Define the 50-episode gate as a sidecar-native rollout

The gate uses the DimOS policy rollout stack, not `lerobot-eval`:

```text
checkpoint: lerobot/VLA-JEPA-LIBERO
suite: libero_object
task_indices: all 10 tasks
init_state_indices: [0, 1, 2, 3, 4]
episodes: 50
pass: successes / 50 > 0.50
```

`lerobot-eval` is only a debugging fallback if the DimOS path underperforms and the team needs to separate policy/checkpoint problems from DimOS sidecar/contract problems.

### 8. Treat setup/contract errors differently from policy failures

Policy failures count as episode failures and allow the run to continue:

- `success=false`;
- timeout;
- done without success.

Setup and contract failures abort the whole demo run:

- missing observation stream;
- wrong image/state shape;
- action spec mismatch;
- invalid action dtype/shape/range;
- checkpoint/backend load failure;
- sidecar protocol mismatch.

Rationale: contract/setup errors mean the integration is wired incorrectly; continuing 50 episodes would only generate noisy false failures.

### 9. Record metadata by default and videos only when requested

Always write:

- `summary.json`;
- `episodes.jsonl`;
- `runtime_description.json`;
- `contract_description.json`; and
- `checkpoint_metadata.json`.

Per episode, record task/init identity, success, steps, reward sum, done, failure reason, action min/max/shape, and observation streams.

Add `--save-videos` for optional rollout video artifacts. Do not save full images/videos by default because 50 episodes can become large quickly.

## Risks / Trade-offs

- **LeRobot observation mismatch** → Mirror the official VLA-JEPA LIBERO observation mapping in one hardcoded contract and fail fast on missing cameras/state.
- **LIBERO action-mode mismatch** → Inspect and validate `env.action_spec` in native mode before accepting step requests.
- **LeRobot dependency drift** → Record checkpoint revision and key package versions in artifacts; avoid strict pins until implementation proves the environment path.
- **Artifact bloat** → Default to JSON metadata and make videos opt-in via `--save-videos`.
- **Bypassing ControlCoordinator may look inconsistent with real-robot goals** → Keep the action-surface gap documented as a separate change and explicitly scope this change to policy/backend/runtime verification.
- **In-process LeRobot may conflict with DimOS dependencies** → Use `PolicyBackend` as the seam; move to a venv/worker backend later if necessary.

## Migration Plan

1. Extend runtime protocol types while preserving `MotorActionFrame` behavior.
2. Add sidecar native LIBERO action mode behind explicit configuration, keeping joint-position mode unchanged.
3. Add stubbed sidecar tests for native action frame validation without real LIBERO assets.
4. Add `PolicyBackend`, `LeRobotBackend`, `VlaJepaLiberoRobotContract`, `RobotPolicyModule`, and `BenchmarkPolicyEvalRunner` code paths.
5. Add a minimal sidecar-native smoke path with valid fixed actions before LeRobot inference.
6. Add one-episode policy smoke coverage where real dependencies are available.
7. Add the 50-episode manual/optional gate and artifact writing.

Rollback is straightforward: existing motor-frame demos remain separate, and the new policy demo can be disabled or skipped without changing the existing LIBERO joint-position workflow.

## Open Questions

- Exact source location and package boundaries for the first `RobotPolicyModule`, `BenchmarkPolicyEvalRunner`, `PolicyBackend`, and contract classes.
- Exact LeRobot processor API calls needed by the installed LeRobot version for `lerobot/VLA-JEPA-LIBERO`.
- Exact official camera stream names for VLA-JEPA LIBERO in the current LeRobot/LIBERO stack, especially the wrist/eye-in-hand name.
- Whether the native action mode should be added to the existing LIBERO-PRO sidecar name/package or trigger a later package rename once non-PRO LIBERO usage is clearer.
