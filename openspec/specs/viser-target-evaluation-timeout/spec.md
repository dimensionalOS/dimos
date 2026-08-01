## Purpose
Define Viser manipulation panel behavior for bounded target IK feasibility evaluation and recovery from stuck target-evaluation handlers.

## Requirements

### Requirement: Target Evaluation Timeout Recovery
The Viser manipulation panel SHALL bound target IK feasibility evaluation with a configurable timeout.

#### Scenario: Target evaluation times out
- **WHEN** a Viser target evaluation does not return before the configured timeout
- **THEN** the panel SHALL mark the current target infeasible and show a diagnostic message that the target evaluation timed out

#### Scenario: Timed-out evaluation exits checking state
- **WHEN** a Viser target evaluation times out while the target status is `CHECKING`
- **THEN** the panel SHALL leave `CHECKING` for that sequence without requiring a panel restart

### Requirement: Target Evaluation Worker Recovery
The Viser manipulation panel SHALL continue accepting and evaluating newer target requests after a target-evaluation timeout.

#### Scenario: Newer target after timeout
- **WHEN** one target evaluation times out and the user moves the Viser target again
- **THEN** the newer target request SHALL be evaluated by an available worker without waiting for the timed-out handler to return

#### Scenario: Abandoned worker result returns late
- **WHEN** a timed-out target evaluation eventually returns after a newer target sequence exists
- **THEN** the late result SHALL NOT overwrite the newer target status, target joints, feasibility state, or panel error

### Requirement: Existing Target Evaluation Semantics Preserved
The Viser manipulation panel SHALL preserve current latest-target-wins and normal target-evaluation behavior for non-timeout cases.

#### Scenario: Queued stale target is replaced
- **WHEN** multiple target requests are submitted before the worker starts evaluating them
- **THEN** the worker SHALL evaluate only the latest queued target request

#### Scenario: Successful target evaluation still applies
- **WHEN** the current target evaluation succeeds before the configured timeout
- **THEN** the panel SHALL mark the target feasible and update target joints as it does today

#### Scenario: Normal IK failure still applies
- **WHEN** the current target evaluation returns an IK failure before the configured timeout
- **THEN** the panel SHALL mark the target infeasible and show the returned failure diagnostic
