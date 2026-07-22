## ADDED Requirements

### Requirement: Serialized coordinator task execution
The coordinator trajectory execution runtime SHALL serialize execute, cancel, reset, and status-polling operations. It SHALL allow at most one active plan and one active execution attempt at a time, and SHALL NOT issue planning or execution work while a remote execution is active.

The runtime SHALL reject a second execution request while an execution attempt is active; execution requests SHALL NOT be queued for later dispatch. The rejection SHALL be terminal for the rejected request and SHALL not fault or interrupt the already active attempt.

#### Scenario: Active execution rejects a second request
- **WHEN** an execution request arrives while the first attempt is active
- **THEN** the second request SHALL complete with a terminal `REJECTED` outcome without being queued
- **AND** the active attempt SHALL remain unchanged
- **AND** the coordinator SHALL receive no execute request for the rejected operation

#### Scenario: Rejected execution does not fault the idle runtime
- **WHEN** a valid execution request reaches dispatch and the coordinator rejects it
- **THEN** the request SHALL complete with a terminal `REJECTED` outcome
- **AND** the manipulation lifecycle SHALL return to `IDLE`
- **AND** the runtime SHALL NOT transition to `FAULT`

### Requirement: Global-to-local-to-coordinator mapping boundary
The runtime SHALL define an explicit mapping boundary from the global manipulation plan to the local coordinator trajectory/task request and back to the global manipulation result. The mapping SHALL preserve plan identity, attempt identity, trajectory/action data, and terminal outcome without exposing coordinator-specific task representation as the manipulation lifecycle contract.

The runtime SHALL perform coordinator dispatch only from a fully mapped local request and SHALL map every coordinator result into the normalized global outcome before publishing it to manipulation callers.

#### Scenario: Global plan is mapped for coordinator dispatch
- **WHEN** a global plan is selected for execution
- **THEN** the runtime SHALL create local coordinator requests containing every trajectory represented by the global plan and their correlation identities
- **AND** it SHALL dispatch that local request rather than passing an unbounded global plan object through the coordinator API

#### Scenario: Coordinator completion is mapped back
- **WHEN** the coordinator reports a terminal result for the active task
- **THEN** the runtime SHALL map it to the corresponding global plan/attempt outcome
- **AND** callers SHALL receive the normalized manipulation outcome rather than raw coordinator task details

### Requirement: Register the task before execute RPC
The runtime MUST register the active plan, attempt, and coordinator task correlation context before invoking the coordinator execute RPC. It MUST associate the RPC response with that pre-registered attempt and MUST retain the context when dispatch is accepted.

#### Scenario: Accepted task is correlated without a race
- **WHEN** the runtime is about to invoke the coordinator execute RPC
- **THEN** its active-attempt/task registration SHALL already exist
- **AND** an immediate acceptance or completion response SHALL be attributed to that attempt

### Requirement: Normalize cancellation and unknown outcomes
The runtime SHALL normalize coordinator cancellation responses into a cancellation outcome. It SHALL normalize unknown, malformed, unavailable, or contradictory coordinator task states into a safety-uncertain outcome rather than treating them as successful completion or ordinary cancellation.

Cancellation SHALL be idempotent for an already terminal attempt, but cancellation of an active task SHALL not be reported complete until the coordinator confirms cancellation or confirms that the task is no longer active. If that confirmation cannot be obtained, the runtime SHALL latch remote safety uncertainty and SHALL block new execution until reset reconciliation.

#### Scenario: Active cancellation waits for confirmation
- **WHEN** cancellation is requested for an active coordinator task
- **THEN** the runtime SHALL serialize the cancel request and enter cancellation handling
- **AND** it SHALL report cancellation only after the task is confirmed cancelled or inactive

#### Scenario: Unknown task state is unsafe
- **WHEN** coordinator polling returns an unknown task state or cannot establish whether the task remains active
- **THEN** the runtime SHALL normalize the result as safety uncertainty
- **AND** it SHALL latch the runtime in `FAULT` until safe reset reconciliation

### Requirement: Reset reconciles remote safety
Reset SHALL first reconcile coordinator task activity and SHALL clear local plan, attempt, and pending request state only after remote safety is proven. Reset SHALL leave the runtime faulted when remote activity is still active or cannot be determined, and SHALL be retryable without issuing a new trajectory execution.

#### Scenario: Reset clears local state after safe reconciliation
- **WHEN** reset confirms that the coordinator has no active task
- **THEN** the runtime SHALL clear the active attempt, ready plan, and pending execution-request bookkeeping
- **AND** it SHALL return the manipulation lifecycle to `IDLE`

#### Scenario: Reset does not hide active remote execution
- **WHEN** reset finds an active coordinator task
- **THEN** the runtime SHALL not clear the safety fault or start another execution
- **AND** it SHALL continue reconciliation or require cancellation before becoming idle

### Requirement: Plan-complete execution only
The coordinator execution interface SHALL require every robot trajectory represented by the generated plan and SHALL not accept a partial-robot argument or robot-name selector. The runtime SHALL maintain a single trajectory execution path through the coordinator and SHALL remove duplicate direct execution paths owned by `ManipulationModule`. This requirement does not remove or relocate `dimos/manipulation/control/coordinator_client.py`, whose standalone diagnostic/operator CLI direct-RPC tooling is outside the `ManipulationModule` execution path; changing that tooling is a separate compatibility-scoped change.

#### Scenario: Partial execution input has no dispatch effect
- **WHEN** an execution request omits a robot trajectory represented by its generated plan or supplies a robot selector
- **THEN** the runtime SHALL reject the request as invalid
- **AND** it SHALL issue no coordinator execute RPC

### Requirement: No manipulation start-state freshness gate
The coordinator execution runtime SHALL not add or restore manipulation-layer validation that rejects a plan because its trajectory start state is not fresh. Start-state acceptance and physical safety remain the responsibility of the coordinator and its execution path.

#### Scenario: Execution is not rejected by the removed freshness gate
- **WHEN** a complete mapped trajectory reaches the coordinator boundary with a non-fresh manipulation start-state observation
- **THEN** the runtime SHALL still submit the coordinator request
- **AND** it SHALL resolve the result according to coordinator acceptance, task status, cancellation, or safety uncertainty
