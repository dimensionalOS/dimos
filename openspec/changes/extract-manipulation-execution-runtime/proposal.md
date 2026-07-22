## Why

`ManipulationModule` currently combines planning, coordinator dispatch, cancellation, task-status monitoring, and lifecycle bookkeeping. Its manually assigned state, separate execution flags, and ambiguous dispatch/completion semantics make concurrent cancellation and remote-task uncertainty difficult to reason about safely.

## What Changes

- Extract coordinator trajectory dispatch and its full remote-task lifecycle into an internal execution runtime owned by `ManipulationModule`.
- Replace manual lifecycle assignments with a single-owner, event-driven state machine and atomic status snapshots.
- Replace the public lifecycle vocabulary with `IDLE`, `PLANNING`, `READY`, `DISPATCHING`, `RUNNING`, `CANCELLING`, and `FAULT`.
- Make plans single-use: replacement planning discards the prior ready plan, and dispatch consumes the selected plan.
- Serialize execute, cancel, reset, and status-polling work through the runtime; latch faults when coordinator task activity cannot be proven safe.
- Remove manipulation-layer trajectory start-state freshness validation without adding a replacement in this change.
- Remove partial-robot execution and the duplicate direct trajectory execution path.

## Capabilities

### New Capabilities
- `manipulation-lifecycle-runtime`: Defines the authoritative manipulation plan and execution lifecycle, state transitions, snapshots, and fault/reset behavior.
- `coordinator-trajectory-execution`: Defines safe plan-to-coordinator dispatch, task reconciliation, completion monitoring, and cancellation semantics.

### Modified Capabilities

None.

## Impact

- Affects `dimos/manipulation/manipulation_module.py`, manipulation operator/UI state adapters, and manipulation execution tests.
- Changes public state strings and removes the `robot_name` execution-selection argument; callers and UI state handling must be updated.
- Centralizes use of `ControlCoordinator` trajectory task RPCs in the internal runtime; no new external dependency is introduced.
