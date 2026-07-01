## 1. Data Model Boundaries

- [x] 1.1 Add `RobotLearningSample` and related observation-role models for runtime-independent policy inference input.
- [x] 1.2 Add `RobotPolicyAction` for runtime-independent policy inference output.
- [x] 1.3 Update existing model exports and tests to cover sample/action serialization and validation.

## 2. Backend and Contract Registries

- [x] 2.1 Add lazy policy backend registry modeled after `ControlTaskRegistry`.
- [x] 2.2 Add lazy robot policy contract registry modeled after `ControlTaskRegistry`.
- [x] 2.3 Register the LeRobot backend under `lerobot` without importing LeRobot at registry discovery time.
- [x] 2.4 Register the VLA-JEPA LIBERO contract under `vla_jepa_libero`.
- [x] 2.5 Add registry tests for successful creation, lazy import behavior, duplicate detection, and unknown type errors.

## 3. RobotPolicyModule as DimOS Module

- [x] 3.1 Convert `RobotPolicyModule` to subclass `dimos.core.module.Module` with a `RobotPolicyModuleConfig`.
- [x] 3.2 Configure backend and contract through `backend_type`, `backend_params`, `contract_type`, and `contract_params`.
- [x] 3.3 Expose RPC methods for policy reset, inference, backend description, and contract description.
- [x] 3.4 Ensure inference accepts `RobotLearningSample` and returns `RobotPolicyAction`.
- [x] 3.5 Preserve backend lifecycle behavior: lazy or start-time initialization, episode reset, close/stop cleanup.
- [x] 3.6 Update existing RobotPolicyModule tests for DimOS module construction and RPC-callable methods.

## 4. Contract and Backend Adaptation

- [x] 4.1 Update `VlaJepaLiberoRobotContract` to consume `RobotLearningSample` instead of benchmark-specific runtime samples.
- [x] 4.2 Update `VlaJepaLiberoRobotContract` to return `RobotPolicyAction` instead of runtime action output/frame types.
- [x] 4.3 Keep `LeRobotBackend` batch-first and registry-created while preserving optional dependency behavior.
- [x] 4.4 Update contract/backend tests for the new sample and action models.

## 5. Module-Backed Benchmark Evaluation

- [x] 5.1 Add or refactor benchmark evaluation into a DimOS module-compatible component that owns episode lifecycle, scoring, artifacts, and cleanup.
- [x] 5.2 Add an explicit LIBERO sample-building seam that converts sidecar observations, payloads, task context, and metadata into `RobotLearningSample`.
- [x] 5.3 Adapt `RobotPolicyAction` to `RuntimeActionFrame` inside benchmark evaluation before sidecar stepping.
- [x] 5.4 Ensure the policy module no longer depends on benchmark-specific sample classes or runtime protocol action frames.
- [x] 5.5 Add tests proving benchmark evaluation builds samples, calls the policy module, adapts actions, and continues/aborts according to existing gate rules.

## 6. Blueprint and Demo Integration

- [x] 6.1 Add a blueprint or blueprint-compatible factory for the module-backed LeRobot LIBERO evaluation path.
- [x] 6.2 Update `demo_lerobot_libero_policy_rollout.py` to construct/run the module-backed workflow while preserving CLI behavior, artifacts, videos, fake-backend smoke path, and gate enforcement.
- [x] 6.3 Ensure existing fake, Robosuite, and LIBERO-PRO motor demos remain unchanged.

## 7. Verification and Documentation

- [x] 7.1 Update rollout documentation to describe the DimOS module workflow, registries, `RobotLearningSample`, and `RobotPolicyAction` boundaries.
- [x] 7.2 Run targeted unit tests for model, registry, RobotPolicyModule, contract, backend, and benchmark evaluation changes.
- [x] 7.3 Run Ruff on changed rollout, sidecar, protocol, and test files.
- [x] 7.4 Run `openspec validate moduleize-robot-policy-rollout --type change`.
- [x] 7.5 Confirm the existing 50-episode benchmark gate command and artifact expectations remain unchanged.
