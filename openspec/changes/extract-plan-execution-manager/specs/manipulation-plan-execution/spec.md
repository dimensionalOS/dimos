## ADDED Requirements

### Requirement: Execute only successful fresh plans

The system SHALL dispatch only a generated plan whose planning status is successful,
whose path and timed trajectory are non-empty, and whose first waypoint matches the
latest available state for every planned joint within the configured tolerance.

#### Scenario: Successful fresh plan

- **GIVEN** a successful generated plan with a non-empty path and timed trajectory
- **AND** the latest robot joint state matches the plan's first waypoint
- **WHEN** the caller requests planned execution
- **THEN** the system attempts to dispatch the plan's robot tasks

#### Scenario: Unsuccessful plan carries residual trajectory data

- **GIVEN** a generated plan whose planning status is not successful
- **AND** the plan contains path or trajectory data
- **WHEN** the caller requests planned execution
- **THEN** the system rejects the plan before invoking any coordinator execute
  command

#### Scenario: Plan start is stale

- **GIVEN** a successful generated plan
- **AND** a planned joint is missing, duplicated, reordered, non-finite, stale, or
  outside the allowed start tolerance
- **WHEN** the caller requests planned execution
- **THEN** the system rejects the plan before invoking any coordinator execute
  command

### Requirement: Treat the generated plan as the atomic execution unit

The system SHALL dispatch every robot represented by a generated plan or reject the
request without selectively executing a robot subset.

#### Scenario: Coordinated multi-robot plan

- **GIVEN** a generated plan containing trajectories for two robots
- **WHEN** the caller requests planned execution
- **THEN** the system prepares and attempts to dispatch both robot trajectories
- **AND** both trajectories preserve the plan's shared timing

#### Scenario: Partial robot execution request

- **GIVEN** a generated plan containing trajectories for multiple robots
- **WHEN** a caller requests execution for only one of those robots
- **THEN** the system rejects the request before dispatch
- **AND** the caller must generate a plan for the intended robot set

### Requirement: Align execution joint names deterministically

The system SHALL validate model-to-coordinator joint-name alignment before
execution and SHALL preserve trajectory values and timing while translating names.

#### Scenario: Valid mapped joints

- **GIVEN** a plan expressed in globally scoped model joint names
- **AND** each robot has an unambiguous coordinator joint-name mapping
- **WHEN** the system prepares per-robot trajectories
- **THEN** each dispatched trajectory uses coordinator joint names
- **AND** positions, velocities, point timestamps, and trajectory timestamp remain
  unchanged

#### Scenario: Ambiguous mapping

- **GIVEN** two coordinator names map to the same local model joint
- **WHEN** execution targets are configured
- **THEN** the system rejects the ambiguous configuration before physical dispatch

#### Scenario: Unmapped model joint

- **GIVEN** a configured model joint without an explicit coordinator mapping
- **WHEN** the system prepares its trajectory
- **THEN** the system uses the local model joint name unchanged

### Requirement: Replace possibly active planned execution safely

Before dispatching a new generated plan, the system MUST confirm that every task
tracked from the previously accepted plan is safe to replace.

#### Scenario: Previous task is still executing

- **GIVEN** a coordinator task may still be executing the previous plan
- **WHEN** a caller requests a replacement plan
- **THEN** the system cancels the previous task before dispatching the replacement
- **AND** dispatch proceeds only after cancellation confirms safety

#### Scenario: Previous task already stopped

- **GIVEN** a tracked coordinator task reports that it was not executing
- **WHEN** a caller requests a replacement plan
- **THEN** the system treats the task as already safe
- **AND** it may dispatch the replacement plan

#### Scenario: Previous task safety is uncertain

- **GIVEN** cancellation of a tracked coordinator task returns no result or raises
  an error
- **WHEN** a caller requests a replacement plan
- **THEN** the system rejects the replacement
- **AND** reports the unresolved task

### Requirement: Compensate partial dispatch

If every task does not accept a multi-task plan, the system MUST attempt to cancel
every task that may have accepted it.

#### Scenario: Later task rejects

- **GIVEN** the first task accepts its trajectory
- **AND** a later task rejects its trajectory
- **WHEN** the plan is dispatched
- **THEN** the system attempts to cancel the first task
- **AND** reports a safe rejection if every possibly accepted task is confirmed
  stopped

#### Scenario: Dispatch outcome is uncertain

- **GIVEN** a coordinator execute call raises after the task may have accepted its
  trajectory
- **WHEN** the system handles the failure
- **THEN** it treats that task as possibly active
- **AND** attempts to cancel it

#### Scenario: Rollback cannot confirm safety

- **GIVEN** a partial dispatch failure
- **AND** cancellation cannot confirm that every possibly active task stopped
- **WHEN** the system returns the dispatch result
- **THEN** the result reports a fault
- **AND** identifies every unresolved task

### Requirement: Give cancellation priority over dispatch

The system MUST prevent new dispatch while cancellation is active and MUST cancel
every task that may have accepted an in-flight plan.

#### Scenario: Cancel races with dispatch

- **GIVEN** a coordinator execute call is in flight
- **WHEN** cancellation begins
- **THEN** the system blocks new dispatch
- **AND** waits for the in-flight call to resolve
- **AND** cancels every task that may have accepted the plan

#### Scenario: Concurrent execution requests

- **GIVEN** one plan is being dispatched
- **WHEN** another caller requests execution
- **THEN** the system rejects the concurrent request without queueing it

#### Scenario: Idempotent cancellation

- **GIVEN** no tracked task is executing
- **WHEN** cancellation is requested
- **THEN** the internal cancellation result reports a safe state

### Requirement: Preserve manipulation compatibility surfaces

The system SHALL retain existing manipulation RPC and skill signatures while
projecting structured execution outcomes into their existing result forms.

#### Scenario: Accepted dispatch through compatibility RPC

- **GIVEN** a valid plan whose tasks all accept their trajectories
- **WHEN** a caller uses the existing boolean execution RPC
- **THEN** the RPC returns `True`
- **AND** the public manipulation state becomes `COMPLETED`
- **AND** `COMPLETED` denotes execution acceptance rather than physical completion

#### Scenario: Safe pre-dispatch rejection

- **GIVEN** a plan is rejected before any new task dispatch
- **WHEN** a caller uses the existing boolean execution RPC
- **THEN** the RPC returns `False`
- **AND** the public manipulation state returns to its previous safe state

#### Scenario: Uncertain post-dispatch state

- **GIVEN** a task may have accepted a trajectory
- **AND** cancellation cannot confirm that it stopped
- **WHEN** the compatibility result is projected
- **THEN** the boolean execution RPC returns `False`
- **AND** the public manipulation state becomes `FAULT`
