## Why

Viser target dragging can leave the manipulation panel stuck in `CHECKING` when an IK target evaluation hangs or takes too long. The current target-evaluation worker drops queued requests and ignores stale results, but it cannot preempt a handler that is already blocked inside IK, so newer target updates cannot recover the UI.

## What Changes

- Add bounded target-evaluation behavior for Viser IK feasibility checks.
- Mark timed-out target evaluations as infeasible with a clear diagnostic instead of leaving the panel in `CHECKING`.
- Restart or replace stuck target-evaluation workers so newer target requests can continue evaluating.
- Preserve existing latest-target-wins semantics: queued stale requests are dropped, and late stale results are ignored.
- Add tests for timeout recovery, worker restart behavior, stale result suppression, and normal target-evaluation success/failure paths.

## Capabilities

### New Capabilities

- `viser-target-evaluation-timeout`: Viser target IK feasibility evaluation must time out safely and continue accepting newer target requests.

### Modified Capabilities

<!-- No existing OpenSpec capability currently owns Viser target-evaluation worker behavior. -->

## Impact

- Affects Viser manipulation panel target evaluation in `dimos/manipulation/visualization/viser/`.
- Likely touches `TargetEvaluationWorker`, `PanelState`, `ViserPanelGui`, and Viser visualization tests.
- Adds a Viser visualization configuration value for target-evaluation timeout.
- No expected changes to planner, preview, execute, robot controller, or IK solver APIs in the first implementation.
