# Design: Manipulation Execution Runtime

## Context

`ManipulationModule` currently owns planning, coordinator dispatch, cancellation, remote-task monitoring, and lifecycle bookkeeping. These responsibilities are interleaved with manually assigned state and separate execution flags, so concurrent requests can produce ambiguous dispatch, completion, or cancellation outcomes. Remote coordinator activity is also difficult to reconcile safely when a response is delayed, incomplete, or contradictory.

## Baseline inventory (historical, task 1.1)

The pre-runtime implementation used `ManipulationState.IDLE`, `PLANNING`, `EXECUTING`, `COMPLETED`, and `FAULT`, with module-owned `_state`, `_last_plan`, `_execution_generation`, `_planning_epoch`, `_execution_dispatch_lock`, and `_coordinator_client` fields. Planning entered through `plan_to_pose`, `plan_to_joints`, `plan_to_pose_targets`, and `plan_to_joint_targets`; execution entered through `execute`/`execute_plan` and split the stored plan into per-robot trajectories.

Module-owned coordinator RPC sites included trajectory `execute`, `cancel`, and `get_state` calls, plus fallback completion polling and direct coordinator-client setup. Status consumers read `get_state`, `get_error`, and `get_trajectory_status`, while the operator/UI adapters consumed those module methods.

The current replacement seams are `ExecutionRuntime` and its `LifecycleState`, `RuntimeSnapshot`, plan/operation/task records, and serialized `start_planning`/`complete_planning`/`execute_ready`/`execute_explicit`/`cancel`/`reset`/`poll` methods. The four planning APIs now submit planning epochs/results through those planning methods, and execution entry points submit prepared or ready plans through the runtime. `ControlCoordinatorGateway` now owns the runtime-facing `execute`/`cancel`/`status`/`reset` RPC boundary. `ManipulationModule.get_state`, `get_execution_snapshot`, `get_error`, `cancel`, `reset`, and execution entry points delegate to that snapshot/runtime seam; operator/UI status reads the atomic snapshot rather than legacy module flags.

This change introduces an internal execution runtime owned by `ManipulationModule`. The runtime is the sole lifecycle owner for manipulation execution. The module remains responsible for running planning algorithms and submitting planning and execution events to the runtime; it does not independently assign lifecycle state or maintain a second execution lifecycle.

The runtime provides the authoritative lifecycle and status snapshot for the manipulation layer while encapsulating coordinator trajectory-task interaction. The design is internal and implementation-agnostic: it specifies ownership, events, state semantics, and safety rules rather than a particular threading or queue library.

## Goals / Non-Goals

### Goals

- Define one authoritative lifecycle with exactly these public states: `IDLE`, `PLANNING`, `READY`, `DISPATCHING`, `RUNNING`, `CANCELLING`, and `FAULT`.
- Serialize planning results, execute, cancel, reset, remote reconciliation, and status publication through a single-owner queue and thread (or equivalent single serialized owner).
- Reject execution requests received during an active attempt rather than queueing them for later dispatch.
- Use a formal reducer to derive lifecycle state and runtime data from events, rather than ad hoc state assignments.
- Make the runtime authoritative for the ready plan, operation IDs, attempts, task knowledge, faults, and the coordinator client.
- Make plans single-use and make replacement of a ready plan explicit and atomic.
- Dispatch every robot trajectory represented by a generated plan; caller-selected partial-robot execution is not supported.
- Reconcile coordinator task activity conservatively, latching `FAULT` whenever safe remote activity cannot be proven.
- Require reset to establish that relevant remote activity is inactive before leaving `FAULT`.
- Build and validate coordinator-name inverse mapping once, using canonical configuration references.

### Non-Goals

- This change does not redesign planning algorithms or trajectory representations.
- It does not add a replacement for manipulation-layer trajectory start-state freshness validation.
- It does not introduce partial-robot execution or retain a duplicate direct trajectory-execution path owned by `ManipulationModule`. Standalone diagnostic/operator CLI direct-RPC tooling in `dimos/manipulation/control/coordinator_client.py` is out of scope; moving or removing it requires a separate compatibility-scoped change.
- It does not define a new external runtime or coordinator dependency.
- It does not prescribe a UI layout, transport, or concrete queue/thread implementation.

## Decisions

### Runtime ownership and event processing

The internal runtime is the sole lifecycle owner. `ManipulationModule` invokes planning algorithms and submits events such as planning started, plan produced, execute requested, cancel requested, reset requested, and coordinator observations. The module must not mutate lifecycle state, ready-plan ownership, operation bookkeeping, or fault state outside the runtime.

All lifecycle-affecting work is serialized by a single-owner queue and thread, or an equivalent mechanism with the same single-writer guarantee. This includes execute, cancel, reset, coordinator polling, task reconciliation, and status snapshot publication. External callers may enqueue requests concurrently, but event order at the runtime owner is the order used for lifecycle decisions.

Execution admission is state-based: while the runtime is `DISPATCHING`, `RUNNING`, or `CANCELLING`, a new execution request is completed as terminal `REJECTED` and is not retained for later dispatch. A coordinator refusal before a remote attempt is established is also a terminal `REJECTED` result and returns the runtime to `IDLE`; neither case is a safety fault.

A formal reducer is the authority for applying events. It validates each event against the current state and runtime context, performs the associated side effects through the runtime-owned coordinator client where needed, and produces the next state plus an atomic status snapshot. Invalid or unsafe events do not create an alternate state path; they are rejected or become a fault according to the event's safety semantics.

### Public lifecycle

The public state vocabulary is limited to:

| State | Meaning |
| --- | --- |
| `IDLE` | No ready plan or active manipulation operation is owned by the runtime. |
| `PLANNING` | A planning request is in progress; its eventual result is not yet executable. |
| `READY` | One complete plan is owned by the runtime and may be executed once. |
| `DISPATCHING` | The selected plan has been consumed and coordinator submission/reconciliation is in progress. |
| `RUNNING` | The current operation is known to be active remotely. |
| `CANCELLING` | Cancellation has been requested and the runtime is reconciling until inactivity or fault is established. |
| `FAULT` | The runtime cannot prove a safe, unambiguous lifecycle or remote-task condition; execution remains latched until reset succeeds. |

The snapshot is atomic from consumers' perspective and includes the public state together with the applicable operation, attempt, plan, task, and fault information. Consumers must use the snapshot rather than infer lifecycle from independent flags.

### Runtime-owned data and coordinator access

The runtime owns the ready plan, operation IDs, attempt records, task knowledge, fault details, and coordinator client. An operation ID identifies a logical execution request; attempts identify coordinator submission/reconciliation efforts within that operation. This separates local request identity from remote task identity and prevents stale responses from being treated as current activity.

The coordinator client used for `ManipulationModule` execution is accessed only by the runtime. A plan is dispatched through the runtime's single coordinator path; active module-owned duplicate direct trajectory execution paths are not retained. This does not require moving or removing `dimos/manipulation/control/coordinator_client.py`, which remains standalone diagnostic/operator CLI direct-RPC tooling; changing it is a separate compatibility-scoped change. The runtime reconciles submission responses and subsequent task observations against the current operation and attempt before changing state.

### Plan replacement and single-use semantics

Plans are single-use. A plan selected for dispatch is consumed before dispatch begins, so it cannot be executed again or reused after a failed or completed attempt. Planning while a ready plan exists may produce an explicit replacement event; applying that event atomically discards the prior ready plan and installs the new complete plan. There is never an externally observable interval in which both plans are executable.

Only a plan containing every robot trajectory represented by its planning groups is accepted. Partial-robot selection and a `robot_name` execution-selection argument are not part of this lifecycle. A plan that omits a trajectory required by its own planning groups is rejected rather than converted into a partial execution.

### Coordinator-name mapping

The coordinator-name inverse mapping is constructed once during runtime initialization from canonical configuration references. Initialization validates that references are complete, names are unambiguous, and the mapping is internally consistent. Runtime operation uses this validated mapping and does not rebuild or infer it from mutable per-request data.

### Freshness and uncertainty safety

Manipulation-layer trajectory start-state freshness validation is removed in this change and has no replacement here. The runtime instead focuses on lifecycle and remote-task certainty.

Remote uncertainty is fail-safe and latching. If coordinator task activity, ownership, submission outcome, cancellation outcome, or completion cannot be proven safe and attributable to the current operation, the reducer enters `FAULT`. A reset request may clear the fault only after the runtime proves that relevant remote activity is inactive; reset is not merely a local flag clear. Until then, the runtime remains in `FAULT` and does not dispatch a new plan.

## Risks / Trade-offs

- A single serialized owner simplifies race handling and makes reducer behavior deterministic, but polling, coordinator calls, or planning-result delivery can delay subsequent lifecycle events if not bounded or isolated appropriately.
- Conservative latching can expose `FAULT` for recoverable coordinator/network ambiguity and requires operator-visible reset, trading availability for avoidance of duplicate or uncontrolled motion.
- Consuming a plan before dispatch prevents accidental reuse but means a failed dispatch cannot silently retry the same plan; any retry must be represented as a new, explicit attempt under the runtime's rules.
- Atomic replacement makes plan ownership clear, but a newly completed plan can supersede a prior ready plan without execution of the prior one.
- Building the inverse mapping once improves consistency and detects configuration errors early, but configuration changes require runtime reinitialization rather than being picked up dynamically.
- Removing freshness validation leaves start-state correctness to lower layers or future work; this change must not imply that freshness is guaranteed elsewhere.

## Migration Plan

1. Add the internal runtime behind `ManipulationModule` and route all lifecycle-affecting requests, planning results, coordinator polling, cancellation, reset, and status publication through its event interface.
2. Move ready-plan storage, operation and attempt bookkeeping, task knowledge, fault handling, coordinator access, and state transitions into the runtime. Remove parallel module-level lifecycle flags and assignments.
3. Update manipulation state adapters and callers to consume the seven public states and atomic snapshots. Remove the `robot_name` execution-selection argument and any partial-robot execution paths.
4. Construct and validate the coordinator-name inverse mapping once from canonical configuration references during runtime initialization.
5. Remove manipulation-layer trajectory start-state freshness validation and active `ManipulationModule` duplicate direct trajectory execution paths, without adding replacement validation in this change. Leave `dimos/manipulation/control/coordinator_client.py` unchanged; moving or removing that standalone diagnostic/operator CLI direct-RPC tooling is a separate compatibility-scoped change.
6. Validate normal planning, atomic replacement, single-use dispatch, completion, cancellation, stale/ambiguous coordinator observations, and reset-after-inactivity behavior before removing the legacy lifecycle implementation.

## Open Questions

- What concrete polling interval and coordinator-call timeout best balance responsiveness with the runtime's single-owner serialization?
- Which coordinator observations constitute sufficient proof of inactivity for each reset path, and how long should that proof remain valid?
- How should faults and operation/attempt identifiers be exposed in existing UI and operator-facing diagnostics beyond the atomic snapshot?
- What future layer, if any, will own trajectory start-state freshness validation after it is removed from manipulation?
