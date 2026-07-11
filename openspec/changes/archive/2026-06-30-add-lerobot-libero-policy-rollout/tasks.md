## 1. Runtime Protocol

- [x] 1.1 Add a discriminated `RuntimeActionFrame` model to `packages/dimos-runtime-protocol` with semantic `space_id`, numeric `values`, and sequence or tick identity.
- [x] 1.2 Update `StepRequest.action` to accept `MotorActionFrame | RuntimeActionFrame` while preserving existing motor-frame JSON compatibility.
- [x] 1.3 Add protocol validation tests for valid motor frames, valid runtime action frames, non-finite runtime action values, and ambiguous or unsupported action frame payloads.
- [x] 1.4 Update `RuntimeSidecarClient.step(...)` call sites only as needed for the new union model without changing existing motor demo behavior.

## 2. Native LIBERO Sidecar Action Mode

- [x] 2.1 Add explicit LIBERO action mode configuration to the LIBERO sidecar while keeping existing joint-position motor mode as the default for current demos.
- [x] 2.2 Implement native LIBERO action mode startup so it follows the official LeRobot LIBERO environment action setup and validates `env.action_spec` shape `(7,)` and bounds compatible with `[-1, 1]`.
- [x] 2.3 Extend runtime description metadata to advertise native action surface id, action shape, action bounds, action mode, task metadata, language, horizon, and camera configuration.
- [x] 2.4 Implement native-mode step handling for `RuntimeActionFrame(space_id="libero.ee_delta_6d_gripper.normalized.v1")` and reject `MotorActionFrame` in native mode with a clear protocol error.
- [x] 2.5 Preserve motor-mode validation so motor-frame mode still requires Panda joint-position plus gripper action compatibility.
- [x] 2.6 Add stubbed sidecar tests for native action mode description, action-spec validation, runtime-action stepping, motor-frame rejection in native mode, and unchanged motor-mode behavior.

## 3. Robot Policy Module Core

- [x] 3.1 Add shared rollout data models for backend-ready batches, backend output envelopes, backend descriptions, contract descriptions, runtime action outputs, rollout episode records, and rollout summaries.
- [x] 3.2 Define the batch-first `PolicyBackend` protocol with `initialize`, `reset_episode`, `infer_batch`, `close`, and `describe` methods.
- [x] 3.3 Implement `RobotPolicyModule` for public `reset(...)`, sample-to-action inference, contract conversion, backend inference, and runtime action emission without sidecar reset/step, scoring, success gate, or artifact ownership.
- [x] 3.4 Add unit tests using fake backend and fake contract objects to prove `RobotPolicyModule` calls the expected seams and resets backend episode state through its public reset method.

## 4. Benchmark Policy Evaluation Runner

- [x] 4.1 Implement `BenchmarkPolicyEvalRunner` orchestration for sidecar lifecycle, episode matrix, sidecar reset/step, policy module reset, sample construction, episode failure handling, metrics, artifacts, and cleanup.
- [x] 4.2 Add unit tests using fake runtime and fake robot policy module objects to prove the evaluation runner owns runtime reset/step, artifacts, success gate, and episode continuation/abort behavior.

## 5. LeRobot Backend

- [x] 5.1 Implement `LeRobotBackend` as an in-process `PolicyBackend` that loads `lerobot/VLA-JEPA-LIBERO`, initializes device/eval mode, and prepares required LeRobot processors.
- [x] 5.2 Implement `LeRobotBackend.reset_episode()` so LeRobot policy/action queue state is reset between benchmark episodes.
- [x] 5.3 Implement `LeRobotBackend.infer_batch(...)` under inference/no-grad mode and return a backend output envelope with inference metadata.
- [x] 5.4 Implement `LeRobotBackend.describe()` with checkpoint id, resolved checkpoint metadata when available, policy class when available, device, and episode reset support.
- [x] 5.5 Add import-boundary or optional-dependency tests so normal CI can import rollout modules without requiring LeRobot unless the backend is instantiated.
- [x] 5.6 Add a mocked LeRobot backend test covering initialization, episode reset, inference envelope shape, and metadata description without downloading real checkpoints.

## 6. VLA-JEPA LIBERO Contract

- [x] 6.1 Implement `VlaJepaLiberoRobotContract.to_backend_batch(...)` for hardcoded VLA-JEPA LIBERO observation mapping from sidecar agent-view camera, wrist or eye-in-hand camera, 8D robot state, and task language.
- [x] 6.2 Implement contract validation for missing observation streams, wrong image shape or dtype, wrong state vector size, and missing task language.
- [x] 6.3 Implement `VlaJepaLiberoRobotContract.from_backend_output(...)` to validate finite `(7,)` action output and produce `RuntimeActionFrame` with `space_id="libero.ee_delta_6d_gripper.normalized.v1"`.
- [x] 6.4 Implement `VlaJepaLiberoRobotContract.describe()` for artifact output, including expected observation streams, state shape, action space id, action shape, and action range.
- [x] 6.5 Add contract unit tests for successful batch conversion, input validation failures, successful runtime action conversion, and invalid backend output rejection.

## 7. Policy Rollout Demo and Gate

- [x] 7.1 Add a LeRobot LIBERO policy rollout demo entrypoint that starts the LIBERO sidecar in native action mode and constructs `BenchmarkPolicyEvalRunner(RuntimeSidecarClient, RobotPolicyModule(LeRobotBackend, VlaJepaLiberoRobotContract))`.
- [x] 7.2 Generate the 50-episode matrix from `libero_object` task indices `0..9` and init state indices `[0, 1, 2, 3, 4]` rather than requiring 50 config files.
- [x] 7.3 Write required artifacts: `summary.json`, `episodes.jsonl`, `runtime_description.json`, `contract_description.json`, `checkpoint_metadata.json`, logs, and cleanup status.
- [x] 7.4 Add optional `--save-videos` or equivalent video artifact support without saving full videos or image dumps by default.
- [x] 7.5 Enforce the manual/optional gate pass condition `success_rate > 0.50` after all 50 episodes complete without setup or contract aborts.
- [x] 7.6 Add a small fixed-action or fake-backend smoke path that exercises native runtime actions without downloading the real LeRobot checkpoint.

## 8. Verification and Documentation

- [x] 8.1 Run targeted protocol tests for runtime action frame validation and existing motor frame compatibility.
- [x] 8.2 Run targeted sidecar tests for native action mode and unchanged motor mode using stubbed backend dependencies.
- [x] 8.3 Run targeted robot policy module, benchmark evaluation runner, backend, and contract unit tests without real LIBERO or LeRobot downloads.
- [x] 8.4 Document how to run the optional real LeRobot LIBERO 50-episode gate, including dependency/assets expectations and expected artifacts.
- [x] 8.5 Verify the existing LIBERO-PRO motor demo path remains unchanged by the native action mode work.
