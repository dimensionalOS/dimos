## Context

The current LeRobot LIBERO policy gate is a synchronous simulator-evaluation path. Benchmark evaluation owns runtime reset/step, stream snapshot collection, observation building, action adaptation, scoring, artifacts, and cleanup. `RobotPolicyModule` owns backend lifecycle, policy reset, contract conversion, backend inference, and action emission through a synchronous module-facing inference API.

That split is correct for fast and parallel simulation, but it does not model the real robot control topology. Real rollout should run from live policy observations, through policy chunk inference, into ControlCoordinator arbitration and control tasks. The live path should still be validated in LIBERO-PRO, but as a real-policy parity gate rather than a fake plumbing-only smoke test.

## Goals / Non-Goals

**Goals:**

- Add a live policy stream path where `RobotPolicyModule` stores the latest `RobotPolicyObservation`, receives a fast chunk inference trigger, runs chunk inference asynchronously, and publishes `RobotPolicyActionChunk` messages.
- Make the LeRobot VLA-JEPA live path chunk-first by using `predict_action_chunk` and preserving the chunk shape until DimOS control code chooses how much to execute.
- Add ControlCoordinator support for policy action chunks without adding a separate conversion module.
- Add a policy chunk control task that owns index-bounded chunk execution, refill triggering, and stale deactivation.
- Add a module-native LIBERO live blueprint/demo path where the LIBERO runtime, observation assembly, `RobotPolicyModule`, and `ControlCoordinator` are DimOS modules wired through streams/RPC rather than local object callbacks.
- Add a LIBERO live parity gate that runs the real VLA-JEPA policy through the live stream and ControlCoordinator path over 10 `libero_object` episodes with `success_rate > 0.50` as a hard pass condition.
- Preserve the current synchronous benchmark path for faster-than-realtime, deterministic, and parallel simulator rollout.

**Non-Goals:**

- No removal of synchronous `infer_action` or the existing benchmark evaluation runner.
- No temporal ensembling, RTC-style chunk blending, or receding-horizon smoothing in v1.
- No new standalone module whose only job is converting policy action chunks into control commands.
- No fake-backend-only acceptance for the live parity gate.
- No redesign of high-throughput image transport or shared-memory observation delivery.

## Decisions

1. **Use dual surfaces on `RobotPolicyModule`.**
   - Decision: keep synchronous inference for benchmark evaluation and add stream-native live rollout surfaces to the same module.
   - Rationale: `RobotPolicyModule` is already the first-class policy module and owns action emission. A separate stream wrapper would create ambiguity over which module owns backend lifecycle and policy reset.
   - Alternative considered: create `RobotPolicyStreamModule`. Rejected because it duplicates policy module responsibility and adds a wrapper layer without changing the core policy semantics.

2. **Keep `RobotPolicyObservation` as the stream input boundary.**
   - Decision: live `RobotPolicyModule` consumes ready `RobotPolicyObservation` messages, not raw camera or joint streams.
   - Rationale: freshness, stream alignment, camera role selection, and task-language assembly are robot/runtime-specific. The policy module should consume a policy-ready observation and delegate backend-specific conversion to its contract.
   - Alternative considered: make `RobotPolicyModule` subscribe directly to raw robot streams. Rejected because it would couple inference to sensor synchronization and robot-specific observation assembly.

3. **Make live inference chunk-first.**
   - Decision: live LeRobot inference uses `predict_action_chunk`, and DimOS introduces a first-class `RobotPolicyActionChunk` model. A single action is treated as a degenerate chunk of length one.
   - Rationale: chunked policies predict a short horizon; DimOS should own how much of that horizon is executed. Flattening chunks into a single action hides execution policy and prevents ControlCoordinator from managing chunk refill and stale behavior.
   - Alternative considered: continue using `select_action` for live rollout. Rejected because it leaves chunk execution inside the backend or policy library rather than the DimOS control stack.

4. **Route live policy chunks into ControlCoordinator directly.**
   - Decision: add a ControlCoordinator input for `RobotPolicyActionChunk` and route chunks to policy-aware control tasks.
   - Rationale: the live path should use the same coordinator arbitration and control-task model as real robots. A separate conversion module would move execution semantics outside the coordinator.
   - Alternative considered: adapt policy chunks to `JointState`, `PoseStamped`, or runtime action frames before ControlCoordinator. Rejected because it loses the semantic policy action surface too early.

5. **Use a policy chunk control task with index-bounded execution.**
   - Decision: the v1 task executes a configured number of leading actions from each chunk, advances by action index/tick-count convention, triggers refill when its queue is empty, and stops contributing commands when no fresh chunk is available.
   - Rationale: index-bounded execution is simple and sufficient for v1. It avoids blindly executing full horizons while avoiding the complexity of time-window execution and overlap blending.
   - Alternative considered: temporal ensembling or RTC-style blending. Rejected for v1 because it adds policy-specific complexity and makes the first live path harder to reason about.

6. **Use fast trigger RPC and stream return for chunk refill.**
   - Decision: when the policy chunk task needs a refill, it sends a fast trigger to `RobotPolicyModule`. The trigger returns early and does not perform backend inference synchronously. The inferred chunk returns on a stream.
   - Rationale: ControlCoordinator tasks run in the coordinator tick loop; slow policy inference must not block the tick. The task still owns execution and refill timing, but the actual inference runs asynchronously in the policy module.
   - Alternative considered: blocking inference RPC from task compute. Rejected because it can stall the entire coordinator tick loop.

7. **Validate live LIBERO with the real policy.**
   - Decision: the LIBERO live path acceptance run uses the real VLA-JEPA checkpoint over a 10-episode `libero_object` slice and passes only when `success_rate > 0.50`.
   - Rationale: fake action plumbing is not enough to validate that the live stream and ControlCoordinator path preserves policy behavior. A 10-episode real-policy gate is large enough to catch live topology regressions while keeping the manual validation loop practical.
   - Alternative considered: fake-backend smoke acceptance. Rejected because it does not validate policy timing, chunk inference, or semantic action execution.

8. **Use module-native blueprint wiring for acceptance.**
   - Decision: the accepting live parity path wires runtime observation assembly, `RobotPolicyModule`, and `ControlCoordinator` as DimOS modules. The coordinator/task requests refills from the policy module through the fast trigger RPC, and completed chunks return through the `RobotPolicyModule.policy_action_chunk` → `ControlCoordinator.robot_policy_action_chunk` stream.
   - Rationale: the live rollout topology being validated is a module/stream/RPC topology, not a local Python object harness. The local harness may remain as a quick smoke path, but it must not be the only accepting path for this change.
   - Alternative considered: keep direct local callback wiring in the parity script. Rejected because it bypasses blueprint stream wiring and does not exercise cross-module policy/coordinator boundaries.

## Risks / Trade-offs

- **Live path can stop between chunks while policy inference runs** → Accept for v1. The task stops contributing commands when empty/stale rather than continuing stale actions.
- **Chunk-first modeling changes backend and contract shapes** → Add explicit `RobotPolicyActionChunk` tests and keep single-action compatibility as a length-one chunk.
- **ControlCoordinator learns about robot-learning types** → Accept because policy chunks are a first-class live control input. Keep the dependency narrow to model types and task input routing.
- **Index-based execution assumes bounded coordinator tick jitter** → Use this for v1 simplicity and record tick/chunk diagnostics. Defer time-window execution until needed.
- **Real-policy live gate is expensive** → Keep the synchronous fast benchmark gate available and make the live gate explicit/manual like the existing LIBERO policy gate.
- **LIBERO live path may diverge from native direct-step performance** → Use a 10-episode success-rate gate and artifacts to diagnose chunk timing, refill delays, and stale deactivation.

## Migration Plan

1. Add model and interface support for `RobotPolicyActionChunk` while preserving `RobotPolicyAction` and synchronous inference.
2. Add stream and trigger surfaces to `RobotPolicyModule`, backed by the existing backend/contract lifecycle.
3. Update LeRobot backend/contract paths to preserve action chunk shape for live inference.
4. Add ControlCoordinator chunk input and policy chunk task registry entry.
5. Add LIBERO live observation assembly and ControlCoordinator runtime wiring.
6. Add module-native blueprint wiring for runtime observation assembly, policy module, and coordinator chunk routing.
7. Validate with unit tests, integration tests, OpenSpec validation, and the real-policy LIBERO live parity gate.

Rollback is straightforward: the existing synchronous benchmark path remains intact and can continue to gate policy performance if the live path is disabled or removed.

## Open Questions

- What exact task type name and config names should the policy chunk task use?
- Should the initial LIBERO policy chunk task internally share code with Cartesian IK/teleop tasks, or duplicate only the minimal mapping needed for the gate?
- What artifacts should record chunk refill latency and stale deactivation counts in the live parity gate?
