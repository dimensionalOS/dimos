## Context

The Viser manipulation panel evaluates target feasibility as users drag Cartesian target controls. The current path is:

1. `ViserPanelGui._on_transform_update()` records the target and calls `PanelState.next_sequence_id()`.
2. `TargetEvaluationWorker.submit()` keeps only the latest queued request.
3. The worker calls the evaluation handler synchronously.
4. The handler calls IK through `ManipulationModule.inverse_kinematics()` and then applies the result if the sequence is still current.

This gives queue preemption and stale-result suppression, but it does not preempt a handler that is already blocked inside IK. If a target evaluation hangs, the worker cannot process later target updates, and the panel can remain in `CHECKING` indefinitely.

## Goals / Non-Goals

**Goals:**

- Bound Viser target-evaluation latency with a configurable timeout.
- Make timed-out target evaluations leave `CHECKING` and become an infeasible target with a diagnostic message.
- Let newer target updates run even if an older evaluation worker is stuck.
- Keep the existing latest-target-wins behavior and stale-result sequence checks.
- Cover timeout, restart, stale result, and normal success/failure flows with tests.

**Non-Goals:**

- Add cancellation tokens to IK solver interfaces.
- Kill native or Python IK calls directly.
- Move target evaluation into a separate process.
- Change plan, preview, execute, or cancel operation semantics.
- Change IK convergence behavior or solver configuration.

## Decisions

### Add a Viser target-evaluation timeout

Add a Viser visualization config value for target-evaluation timeout, defaulting to a small bounded value suitable for UI responsiveness. The target-evaluation worker will run each handler call behind a timeout and synthesize a failure result when the timeout expires.

Alternative considered: rely on IK solver iteration limits. This is insufficient because the problem is a blocked or unexpectedly slow call at the panel boundary, and some backends may not respect a Python-side iteration budget.

### Restart or replace stuck target-evaluation workers

If a request times out, treat the worker that owns the blocked handler as abandoned and create a fresh worker for future target requests. Existing sequence IDs remain the source of truth, so late results from abandoned work are ignored.

Alternative considered: only mark the current request timed out while keeping the same worker. That does not recover from a handler that remains blocked, because the worker thread still cannot dequeue future requests.

### Preserve sequence-based stale-result suppression

The current `sequence_id != latest_sequence_id` guard should remain. Timeout/restart adds recovery from stuck work; it should not let old results overwrite newer target state.

Alternative considered: cancel pending work by mutating shared panel state directly. That is more fragile than preserving the existing sequence contract.

### Keep the fix at the Viser worker boundary

The implementation should not thread deadlines through `ManipulationModule` or every kinematics backend. This keeps the change small and local to Viser target evaluation.

Alternative considered: cooperative cancellation through all IK APIs. That is a better long-term design for true cancellation, but it is broader than needed for UI recovery.

## Risks / Trade-offs

- Timed-out IK calls may continue running in abandoned daemon threads → cap restart behavior, log diagnostics, and treat this as a UI recovery mechanism rather than true cancellation.
- Too-short timeout can mark solvable targets infeasible → make the timeout configurable and choose a conservative default.
- Restarting workers can hide repeated backend hangs → surface timeout diagnostics in the panel and tests.
- Late stale successes could overwrite a newer target → preserve sequence checks before applying any result.

## Migration Plan

- Add the config field with a backward-compatible default.
- Keep existing behavior for successful and normal failed target evaluations.
- Add tests before implementation for timeout and restart behavior.
- No data migration or blueprint changes are required.

## Open Questions

- What exact default timeout should be used for Viser target evaluation: 1.0s, 2.0s, or an existing operation timeout value?
- Should repeated target-evaluation timeouts be rate-limited in logs?
