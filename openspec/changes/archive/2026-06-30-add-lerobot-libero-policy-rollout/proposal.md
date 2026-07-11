## Why

DimOS has a LIBERO runtime sidecar workflow, but it only proves scripted joint/motor plumbing; it does not yet prove that a real robot-learning policy can load observations, run inference, and drive a benchmark episode through the runtime boundary. Official LeRobot LIBERO checkpoints provide a concrete high-signal target for validating a first policy rollout module, backend boundary, and runtime action path.

## What Changes

- Add a first robot-learning policy rollout path centered on a `RobotPolicyModule`, a separate benchmark/evaluation runner, a batch-first `PolicyBackend` interface, a `LeRobotBackend`, and a narrow `VlaJepaLiberoRobotContract`.
- Extend the runtime protocol so a step request can carry either the existing motor action frame or a native runtime action frame.
- Extend the LIBERO runtime sidecar with a native LIBERO action mode that follows the official LeRobot LIBERO action setup and accepts `float32[7]` relative end-effector delta plus gripper actions.
- Add a policy-driven LIBERO demo/gate using `lerobot/VLA-JEPA-LIBERO` over `libero_object`, all 10 task indices, init states `[0, 1, 2, 3, 4]`, for 50 total episodes and `success_rate > 0.50`.
- Record structured artifacts for rollout provenance, per-episode results, runtime description, contract description, checkpoint metadata, and optional videos.
- Keep the existing joint-position LIBERO demo path intact.
- Explicitly defer ControlCoordinator action-surface support; v1 bypasses ControlCoordinator for policy rollout because the current coordinator execution model is joint/motor oriented.

## Capabilities

### New Capabilities
- `robot-policy-module`: Defines reusable robot policy inference/action emission, including public reset, batch-first backend inference, robot policy contract conversion, and LeRobot backend integration.
- `benchmark-policy-evaluation`: Defines benchmark lifecycle orchestration for policy evaluation, including episode matrix execution, runtime reset/step ownership, scoring, artifacts, and the 50-episode LIBERO policy gate.

### Modified Capabilities
- `runtime-protocol`: Add native runtime action frames and allow step requests to carry either motor action frames or runtime action frames while preserving backend-neutral protocol boundaries.
- `runtime-libero-pro-sidecar`: Add native LIBERO action mode alongside the existing joint-position motor mode, validating official LIBERO action specs and accepting runtime action frames for native policy rollout.
- `runtime-scripted-demos`: Add a policy-driven LIBERO rollout demo/gate that validates real policy success rather than only scripted runtime plumbing.

## Impact

- Affected packages and modules:
  - `packages/dimos-runtime-protocol`
  - `packages/dimos-libero-pro-sidecar`
  - `dimos/simulation/runtime_client`
  - new robot-learning policy rollout/backend/contract code under the DimOS Python package
  - benchmark/demo scripts and tests under `scripts/benchmarks` and `dimos/benchmark/runtime`
- Adds an optional LeRobot dependency path for the policy backend environment.
- Requires prepared LIBERO assets for the real 50-episode gate; always-on tests should use stubs and import-boundary checks.
- Does not change the existing motor-frame sidecar demo behavior.
- Does not implement ControlCoordinator non-joint action-surface execution in this change.
