## Context

`dimos/manipulation/manipulation_module.py` currently owns planning and planned
execution. Its execution implementation selects a stored or explicit
`GeneratedPlan`, validates the plan start against live joints, splits a global
trajectory by robot, translates model joint names into coordinator names,
dispatches coordinator tasks, compensates partial dispatch, serializes cancellation,
tracks possibly active tasks, and projects results into `ManipulationState`.

The execution path calls `ControlCoordinator.task_invoke()` directly. The
coordinator already supports the required `execute` and `cancel` task commands, so
the extraction must not change its RPC contract. Existing blueprints, streams,
skills, CLI commands, and generated registries must remain source-compatible.

The design follows the manipulation glossary in `CONTEXT.md` and the replacement
decision in `docs/adr/0001-replace-active-planned-execution.md`.

## Goals / Non-Goals

**Goals:**

- Place complete-plan validation, preparation, dispatch, replacement, rollback,
  and cancellation behind one small in-process interface.
- Preserve hardware-safety behavior under execute/execute and execute/cancel races.
- Convert generic coordinator results into explicit typed outcomes.
- Keep existing `ManipulationModule` RPC and skill signatures.
- Make execution policy testable without constructing a planning world or generic
  RPC mock.

**Non-Goals:**

- Track physical completion, expose execution status, or return execution handles.
- Change `ControlCoordinator`, `JointTrajectoryTask`, streams, transports,
  blueprints, CLI commands, skills, or MCP schemas.
- Execute only part of a multi-robot generated plan.
- Move direct gripper actuation into planned execution.
- Add runtime enforcement that prevents diagnostic clients from writing directly
  to coordinator trajectory tasks.
- Redesign the public `ManipulationState` lifecycle.

## DimOS Architecture

`PlanExecutionManager` will be a regular class in
`dimos/manipulation/execution_manager.py`, owned by `ManipulationModule`. It will
not inherit `Module`, expose RPC methods, own streams, or run in another worker.

Its external interface will remain small:

```python
class PlanExecutionManager:
    def execute(self, plan: GeneratedPlan) -> ExecutionDispatchResult: ...
    def cancel(self) -> CancellationResult: ...
```

The same file will define immutable `ExecutionTarget` values, execution policy,
typed result models, and the adapter Protocol used by the manager. An execution
target contains only a robot name, configured local model joints, coordinator task
name, and a validated model-to-coordinator joint-name mapping.

The manager will receive three dependencies:

1. Immutable execution targets.
2. A callable that returns the current globally named `JointState`.
3. A `CoordinatorExecutionPort` adapter Protocol with typed `execute` and `cancel`
   methods.

A production `CoordinatorExecutionAdapter` will wrap the existing `RPCClient` and
translate `task_invoke()` behavior:

- Execute `True`, `False`, and `None` become accepted, rejected, and unknown.
- Cancel `True`, `False`, and `None` become cancelled, already stopped, and
  unknown.
- RPC exceptions become unknown outcomes.

`ManipulationModule` will create and close the RPC client and adapter. It will
construct the manager after robot and world-monitor initialization. During stop,
the module will ask the manager to cancel tracked tasks before closing the adapter.

The module's existing `execute()` and `execute_plan()` RPCs will select the explicit
or cached plan and delegate it to the manager. Existing boolean returns will project
the structured result. Planning cancellation remains in `ManipulationModule`;
planned-execution cancellation delegates to the manager. The module remains the
sole owner of the public `ManipulationState` projection.

No DimOS `Spec` Protocol, module ref, stream, transport, blueprint atom, skill/MCP
tool, CLI entry point, or generated registry changes are required.

## Decisions

### Execute complete successful plans

The manager accepts only a `GeneratedPlan` with successful planning status, a
non-empty path, and a non-empty timed trajectory. It dispatches every robot in the
plan or dispatches none. The legacy `robot_name` filter may remain in compatibility
wrappers only to validate a single-robot plan; it must reject filtering a
multi-robot plan.

This treats the generated plan as the atomic unit whose timing and robot
coordination must remain intact.

### Validate freshness immediately before dispatch

Structural validation and trajectory preparation may occur before acquiring the
dispatch lock. Under that lock, the manager first makes prior tracked tasks safe,
then reads the latest global joint state and compares every planned joint with the
trajectory's first waypoint. It retains the current `1e-6` tolerance during this
extraction.

Missing, duplicate, reordered, non-finite, stale, or mismatched planned joints
reject dispatch before any new coordinator execute call.

### Replace rather than overlap

The manager retains the names of tasks that may have accepted the prior plan. A new
execute request cancels those tasks before dispatch. Cancel `False` confirms that a
task was already stopped; `None` or an exception leaves the task unresolved. If any
prior task remains unresolved, the manager rejects the replacement.

Because this change does not observe completion, task names remain tracked until a
later replacement or explicit cancellation confirms safety.

### Serialize dispatch and prioritize cancellation

Private transaction facts include a dispatch lock, cancellation gate, generation
token, and possibly active task set. Concurrent execute calls fail fast rather than
queue. Cancellation publishes its gate before waiting for an in-flight dispatch,
then cancels every task that might have accepted. New execute calls fail while
cancellation is active. Concurrent cancellation is idempotent.

### Compensate partial multi-task dispatch

The manager validates and prepares every task trajectory before sending the first
one. It records a task as possibly active before issuing its execute RPC because an
exception can occur after remote acceptance. If a later task rejects or has an
unknown outcome, the manager cancels every possibly accepted task.

A fully confirmed rollback returns a safe rejection. Any unresolved cancellation
returns a fault result with the unresolved task names.

### Validate name alignment at construction

`RobotModelConfig.joint_name_mapping` maps coordinator names to local model names,
while dispatch needs the reverse direction. Manager construction will validate and
invert the mapping once. It will reject unknown local joints, duplicate reverse
targets, and duplicate resolved coordinator names. Unmapped model joints use
identity mapping.

Per-robot trajectory splitting preserves point timestamps, positions, velocities,
and the original trajectory timestamp. Name translation changes joint names only.

### Keep coordinator ownership conventional

The manager is conventionally the single writer for its configured trajectory
tasks. The coordinator cannot enforce this rule because `task_invoke()` has no
caller identity or ownership token. Adding task leases would expand coordinator
scope and introduce lease recovery. Diagnostic callers must not execute the same
tasks concurrently with manipulation.

## Safety / Simulation / Replay

The manager applies the same behavior to hardware and simulation because both use
coordinator trajectory tasks. Replay configurations without executable coordinator
tasks continue to reject execution through the compatibility wrapper.

Safety depends on these invariants:

- Reject an unsuccessful, malformed, stale, or partial plan before dispatch.
- Prepare every task before the first coordinator call.
- Treat unknown dispatch as possibly accepted.
- Cancel every possibly accepted task after partial failure.
- Block replacement when prior task safety is unknown.
- Give cancellation priority over new dispatch.

Unit tests will use an in-memory coordinator adapter to exercise every accepted,
rejected, already-stopped, unknown, and exception path. Integration tests will use
the existing mock coordinator. Manual hardware QA should execute and replace a
single-arm plan, cancel an active plan, and exercise a dual-arm plan if suitable
hardware is available. No physical test should intentionally create an unknown
cancellation outcome.

## Risks / Trade-offs

- A manual or diagnostic client can still replace a coordinator task behind the
  manager. Documentation will state the single-writer convention.
- Sequential multi-task RPC calls are not an atomic coordinator operation. The
  manager mitigates partial acceptance through best-effort cancellation.
- The public `COMPLETED` state continues to mean dispatch acceptance, not physical
  motion completion. The glossary and capability docs will state this distinction.
- Replacement may issue cancel after a prior trajectory already completed. The
  coordinator's `False` response confirms that the task is already safe.
- Requiring successful planning status may reveal test fixtures or callers that
  constructed inconsistent plans. Those callers must set the correct status or use
  the normal planning path.

## Migration / Rollout

1. Add manager, result, target, policy, Protocol, and adapter types with focused
   tests.
2. Construct the manager from existing robot configs and world-state access.
3. Replace direct execution and cancellation logic in `ManipulationModule` with
   delegation while retaining RPC signatures and public state mapping.
4. Move execution-policy tests to the manager interface and retain module
   integration tests for delegation and compatibility.
5. Update manipulation capability documentation and validate links and code blocks.

No blueprint variable changes are planned, so
`dimos/robot/all_blueprints.py` should not change. If implementation unexpectedly
changes a built-in blueprint, run
`pytest dimos/robot/test_all_blueprints_generation.py`.

Rollback consists of restoring module-local execution methods; no persisted data,
coordinator migration, or hardware configuration must be reversed.

## Open Questions

None. The design intentionally defers execution observation, coordinator task
leases, physical completion semantics, and gripper extraction to separate changes.
