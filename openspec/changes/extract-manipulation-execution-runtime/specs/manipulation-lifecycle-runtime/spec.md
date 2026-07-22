## ADDED Requirements

### Requirement: Authoritative lifecycle state
The manipulation execution runtime SHALL own the authoritative lifecycle state and SHALL expose only `IDLE`, `PLANNING`, `READY`, `DISPATCHING`, `RUNNING`, `CANCELLING`, and `FAULT` as lifecycle states.

The runtime SHALL publish an atomic status snapshot containing the lifecycle state and the currently relevant plan, attempt, task, and fault information. Readers SHALL observe one internally consistent snapshot rather than independently read lifecycle flags.

#### Scenario: Snapshot reports a coherent active execution
- **WHEN** a plan has been accepted for execution and its coordinator task is active
- **THEN** one status snapshot SHALL report `RUNNING` together with the associated plan/attempt and remote task identity
- **AND** the snapshot SHALL NOT combine fields from different plans or attempts

#### Scenario: Invalid lifecycle transition is rejected
- **WHEN** an event requests a transition that is not legal from the current lifecycle state
- **THEN** the runtime SHALL leave the lifecycle state unchanged
- **AND** it SHALL report the rejected transition without silently changing an execution flag

### Requirement: Legal lifecycle transitions
The runtime SHALL permit planning only from `IDLE` or `READY`, transitioning to `PLANNING`. A successful planning result SHALL transition to `READY`; a planning failure SHALL return to `IDLE` unless the runtime is already faulted. Dispatch of a selected ready plan SHALL transition through `DISPATCHING` and, after accepted dispatch, to `RUNNING`. A completed or cancelled attempt SHALL transition to `IDLE`. Cancellation of an active attempt SHALL transition to `CANCELLING` before reaching `IDLE` or `FAULT`. A safety-uncertain condition SHALL transition to and latch `FAULT`.

The runtime SHALL NOT transition out of `FAULT` except through reset reconciliation that proves the remote task state is safe.

#### Scenario: Replacement planning discards a ready plan
- **WHEN** the runtime is `READY` and a new planning request is accepted
- **THEN** the lifecycle SHALL become `PLANNING`
- **AND** the previous ready plan SHALL be discarded
- **AND** only the newly produced plan MAY become `READY`

#### Scenario: Completed execution returns to idle
- **WHEN** the active attempt is physically reported complete
- **THEN** the runtime SHALL consume the attempt and transition to `IDLE`
- **AND** its next snapshot SHALL contain no executable ready plan

### Requirement: Planning and single-use ready plans
The runtime SHALL serialize planning with dispatch, cancellation, reset, and status polling. It SHALL maintain at most one ready plan. Planning SHALL produce a plan with a unique plan identity and SHALL make it executable only after the planning operation succeeds. Dispatch SHALL consume the selected plan exactly once; a plan SHALL NOT be dispatched again after dispatch acceptance, completion, cancellation, or failure.

The runtime SHALL reject planning and dispatch requests while an attempt is actively executing remotely (`DISPATCHING`, `RUNNING`, or `CANCELLING`), except that cancellation and reset operations SHALL remain available through their defined serialized paths.

An execution request rejected because an attempt is active SHALL complete with a terminal `REJECTED` outcome, SHALL NOT be queued, and SHALL NOT transition the active runtime to `FAULT`.

#### Scenario: Dispatch consumes a plan
- **WHEN** a ready plan is dispatched and the coordinator accepts the task
- **THEN** the plan SHALL no longer be available as a ready plan
- **AND** a repeated dispatch request using that plan identity SHALL be rejected

#### Scenario: Planning is blocked during remote execution
- **WHEN** the runtime is `RUNNING` or `CANCELLING`
- **THEN** a planning request SHALL be rejected without creating or replacing a plan
- **AND** no trajectory execution request SHALL be issued

#### Scenario: Execution is rejected during remote execution
- **WHEN** the runtime is `DISPATCHING`, `RUNNING`, or `CANCELLING` and a second execution request is received
- **THEN** the request SHALL complete with a terminal `REJECTED` outcome
- **AND** it SHALL remain unqueued and issue no coordinator execute request
- **AND** the active lifecycle state SHALL remain unchanged

#### Scenario: Coordinator rejection returns to idle
- **WHEN** a valid execution request is rejected by the coordinator before an active remote attempt is established
- **THEN** the request SHALL complete with a terminal `REJECTED` outcome
- **AND** the lifecycle SHALL return to `IDLE`
- **AND** the runtime SHALL NOT enter `FAULT`

### Requirement: Dispatch acceptance is distinct from physical completion
The runtime SHALL distinguish coordinator dispatch acceptance from physical trajectory completion. Dispatch acceptance SHALL establish an active attempt and `RUNNING` state, but SHALL NOT be reported as physical completion. The runtime SHALL transition to completion only after coordinator task status or an equivalent authoritative result reports that the robot physically completed the task.

#### Scenario: Accepted dispatch remains running
- **WHEN** the coordinator accepts a trajectory task but has not reported physical completion
- **THEN** the runtime SHALL report `RUNNING`
- **AND** it SHALL continue task monitoring
- **AND** it SHALL NOT report the manipulation as complete

### Requirement: Remote safety uncertainty is latched
If the runtime cannot prove that a coordinator task is safely inactive or reconciled, it SHALL latch a `FAULT` state and SHALL prevent new planning or execution. The fault SHALL remain latched across subsequent polls and SHALL be cleared only by reset reconciliation that establishes a safe remote state.

#### Scenario: Unknown remote activity faults the runtime
- **WHEN** a status, cancellation, or reset operation returns an unknown/error result that does not prove the remote task is inactive
- **THEN** the runtime SHALL transition to `FAULT`
- **AND** subsequent plan and execute requests SHALL be rejected

#### Scenario: Safe reset clears a latched fault
- **WHEN** reset reconciliation confirms that no coordinator task is active and the remote state is safe
- **THEN** the runtime SHALL clear the latched fault
- **AND** it SHALL transition to `IDLE` with no ready plan or active attempt

### Requirement: No manipulation-layer start-state or partial-robot execution contract
The manipulation runtime SHALL NOT validate trajectory start-state freshness as a prerequisite for manipulation execution. It SHALL NOT accept a partial-robot execution selection or a `robot_name` execution argument; manipulation execution SHALL target every robot trajectory represented by the generated plan through the coordinator boundary.

#### Scenario: Stale manipulation start state does not block dispatch
- **WHEN** a ready manipulation plan is otherwise valid but its trajectory start state is not fresh according to the removed manipulation-layer check
- **THEN** the runtime SHALL not reject the plan solely for that freshness condition
- **AND** execution SHALL proceed subject to coordinator acceptance and safety rules

#### Scenario: Partial robot selection is rejected
- **WHEN** a caller attempts manipulation execution with a partial-robot selection or robot-specific execution argument
- **THEN** the request SHALL be rejected
- **AND** no coordinator trajectory request SHALL be issued
