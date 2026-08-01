## 1. Target Evaluation Worker Behavior

- [x] 1.1 Add a configurable Viser target-evaluation timeout value with a backward-compatible default.
- [x] 1.2 Extend `TargetEvaluationWorker` so handler execution is bounded by the configured timeout.
- [x] 1.3 Convert target-evaluation timeout into a failure result that marks the current target infeasible and includes a timeout diagnostic.
- [x] 1.4 Add generation or restart handling so a timed-out worker cannot block newer target requests.
- [x] 1.5 Preserve latest-target-wins queue draining and stale sequence result suppression for normal and timeout paths.

## 2. Panel Integration

- [x] 2.1 Thread the target-evaluation timeout config from `ViserVisualizationConfig` into `ViserPanelGui` and its target worker.
- [x] 2.2 Ensure timeout results leave `TargetStatus.CHECKING` and refresh the panel state.
- [x] 2.3 Ensure a newer target submitted after a timeout can be evaluated without restarting the panel.
- [x] 2.4 Keep plan, preview, execute, cancel, and clear operation worker behavior unchanged.

## 3. Tests

- [x] 3.1 Add worker-level tests for target-evaluation timeout result generation.
- [x] 3.2 Add worker-level tests that a newer target can run after a timed-out handler remains blocked.
- [x] 3.3 Add tests that late stale results from abandoned or timed-out work do not overwrite newer panel state.
- [x] 3.4 Add regression coverage that normal target-evaluation success and normal IK failure still update the panel as before.
- [x] 3.5 Add config tests for the new timeout field and override alias if applicable.

## 4. Validation

- [x] 4.1 Run targeted Viser state, worker, and visualization panel tests.
- [x] 4.2 Run ruff on changed Python files.
- [x] 4.3 Run `openspec validate fix-viser-target-evaluation-timeout --strict`.
