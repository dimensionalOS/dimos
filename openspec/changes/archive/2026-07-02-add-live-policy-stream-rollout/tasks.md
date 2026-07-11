## 1. Policy chunk models and backend contracts

- [x] 1.1 Add a first-class `RobotPolicyActionChunk` model with action-space id, ordered action values, sequence/timing metadata, and validation-friendly shape accessors.
- [x] 1.2 Extend backend output modeling so single-action inference and chunk inference preserve rank instead of flattening chunks into one action tuple.
- [x] 1.3 Add LeRobot backend support for chunk-first live inference using `predict_action_chunk` while preserving synchronous single-action inference for the fast benchmark path.
- [x] 1.4 Update the VLA-JEPA LIBERO contract to convert valid `N x 7` normalized backend chunks into `RobotPolicyActionChunk` values.
- [x] 1.5 Add unit tests for chunk model validation, backend chunk rank preservation, and contract chunk conversion failures.

## 2. RobotPolicyModule live stream surface

- [x] 2.1 Add typed stream outputs/inputs for live robot policy observations and robot policy action chunks on `RobotPolicyModule`.
- [x] 2.2 Store the latest live `RobotPolicyObservation` received by the module without making the policy module subscribe to raw camera or joint streams.
- [x] 2.3 Add a fast chunk inference trigger RPC that returns before slow backend inference completes and reports clear not-ready behavior when no observation is available.
- [x] 2.4 Run triggered chunk inference asynchronously with at most one in-flight backend inference per module instance by default.
- [x] 2.5 Publish successful chunk inference results on the policy action chunk stream and expose status/error metadata for diagnostics.
- [x] 2.6 Preserve existing synchronous `reset`, `infer_action`, `describe_backend`, and benchmark-facing behavior.
- [x] 2.7 Add module tests for latest-observation storage, trigger behavior, no-observation failure, duplicate trigger handling, chunk publication, reset, and backward-compatible synchronous inference.

## 3. ControlCoordinator policy chunk execution

- [x] 3.1 Add a ControlCoordinator input stream for `RobotPolicyActionChunk` values and route chunks to policy-action-aware tasks.
- [x] 3.2 Add a policy chunk control task registry entry and task configuration for accepted action-space id, claimed joints, priority, ticks per action, execute-first-N count, stale timeout, and gripper mapping.
- [x] 3.3 Implement policy chunk task validation for action-space id, shape, finite normalized values, and supported action dimensions.
- [x] 3.4 Implement index-bounded chunk execution that consumes a configured leading prefix and emits coordinator-compatible commands on each tick.
- [x] 3.5 Implement queue-empty refill behavior using the fast policy chunk inference trigger without blocking backend inference in the coordinator tick loop.
- [x] 3.6 Implement stale deactivation so the task stops contributing commands after configured staleness rather than continuing stale policy actions.
- [x] 3.7 Add ControlCoordinator/task tests for chunk routing, validation failures, prefix execution, refill trigger calls, stale deactivation, and arbitration behavior.

## 4. LIBERO live policy stream path

- [x] 4.1 Add a LIBERO live observation assembler that converts runtime image/state streams and task metadata into `RobotPolicyObservation` messages for the policy module.
- [x] 4.2 Wire LIBERO runtime observation streams, `RobotPolicyModule`, ControlCoordinator, and the policy chunk control task into a live parity demo path.
- [x] 4.3 Ensure the live path uses the real `lerobot/VLA-JEPA-LIBERO` checkpoint and chunk inference rather than fake/fixed policy actions for acceptance.
- [x] 4.4 Route policy execution through ControlCoordinator and the policy chunk task rather than direct benchmark evaluation adaptation into native runtime `step()` calls.
- [x] 4.5 Record live-path diagnostics including chunk counts, refill triggers, consumed actions, stale deactivations, inference status, and cleanup status.
- [x] 4.6 Add focused integration tests with fakes for the live observation-to-policy-to-coordinator-to-runtime control flow.
- [x] 4.7 Add module-native LIBERO live wiring so runtime observation assembly, `RobotPolicyModule`, and `ControlCoordinator` run as DimOS modules connected by streams/RPC rather than local object callbacks.
- [x] 4.8 Add tests proving `RobotPolicyModule.policy_action_chunk` connects to `ControlCoordinator.robot_policy_action_chunk` through module stream wiring and coordinator refill requests call the policy module trigger.

## 5. Benchmark preservation and validation

- [x] 5.1 Keep the existing synchronous benchmark policy evaluation path runnable without requiring ControlCoordinator policy chunk execution.
- [x] 5.2 Add tests proving the fast benchmark path still uses lockstep runtime reset/snapshot/inference/action-adaptation/step ownership.
- [x] 5.3 Run targeted unit tests for policy models, backend, contract, policy module, ControlCoordinator task, and benchmark evaluation.
- [x] 5.4 Run OpenSpec validation for `add-live-policy-stream-rollout` in strict mode.
- [x] 5.5 Run the module-native real-policy LIBERO live parity gate over a 10-episode `libero_object` slice and require `success_rate > 0.50`.
- [x] 5.6 Update developer documentation for the difference between the fast benchmark gate and live policy stream parity gate.
