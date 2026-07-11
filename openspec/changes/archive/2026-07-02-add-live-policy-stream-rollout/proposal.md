## Why

The current LeRobot LIBERO policy rollout path is optimized for simulator benchmark evaluation: benchmark code synchronously builds observations, calls policy inference, adapts one action, and steps the runtime. Real robot rollout needs a live stream topology where policy observations, policy chunks, and ControlCoordinator execution are first-class, while preserving the fast synchronous benchmark path for parallel and faster-than-realtime simulation.

## What Changes

- Add a stream-native surface to `RobotPolicyModule` that stores the latest `RobotPolicyObservation`, accepts a fast inference trigger, runs chunk inference asynchronously, and publishes `RobotPolicyActionChunk` results.
- Make LeRobot VLA-JEPA inference chunk-first by using `predict_action_chunk` for the live path and preserving the chunk shape through backend output, contract conversion, and policy module emission.
- Add a first-class `RobotPolicyActionChunk` model so DimOS, not the backend, owns chunk execution policy.
- Add a ControlCoordinator input for policy action chunks and a policy chunk control task that owns index-bounded chunk execution, asks the policy module to refill when empty, and stops contributing commands when no fresh chunk is available.
- Route the live LIBERO policy path through ControlCoordinator rather than adapting policy actions directly into runtime `step()` calls.
- Add a real-policy LIBERO live parity gate that runs the live policy stream path with the actual VLA-JEPA checkpoint and requires the same hard success condition as the existing LIBERO policy gate: `success_rate > 0.50`.
- Preserve the existing synchronous benchmark evaluation path for fast, deterministic, parallel simulator rollout.

## Capabilities

### New Capabilities

- `policy-action-control`: ControlCoordinator support for receiving and executing robot policy action chunks through a policy-aware control task.

### Modified Capabilities

- `robot-policy-module`: Add stream-native observation ingestion, fast chunk inference triggering, chunk-first backend/contract output, and chunk stream emission while keeping synchronous inference available for benchmark evaluation.
- `runtime-scripted-demos`: Add a real-policy LIBERO live stream parity gate that validates policy rollout through ControlCoordinator rather than direct native runtime stepping.
- `benchmark-policy-evaluation`: Preserve the synchronous fast benchmark path and clarify its relationship to the live policy stream path.

## Impact

- Affected code: `dimos/robot_learning/policy_rollout/`, `dimos/control/`, `dimos/control/tasks/`, `scripts/benchmarks/`, LIBERO runtime demo/blueprint wiring, and associated tests.
- API impact: new policy action chunk model; new `RobotPolicyModule` stream/RPC surface; new ControlCoordinator policy chunk input; new control task type for policy chunk execution.
- Runtime impact: live LIBERO policy validation uses the real LeRobot VLA-JEPA checkpoint and ControlCoordinator path; the existing fast benchmark path remains available for simulator gates and parallel rollout.
