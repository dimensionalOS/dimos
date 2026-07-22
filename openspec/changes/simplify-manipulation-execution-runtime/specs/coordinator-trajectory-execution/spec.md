## ADDED Requirements

### Requirement: Use a fixed blocking-RPC executor and owner timer scan
The runtime SHALL execute blocking coordinator RPCs through one fixed executor with exactly four daemon workers returning futures. One owner-loop timer scan SHALL be authoritative for polling, deadline observation, cancellation reconciliation, deferred reset handling, and shutdown observation. The runtime SHALL NOT create poller, per-action deadline, reset, gateway-close, or module-side timeout-reconciliation threads.

The runtime SHALL include the module-configured `physical_operation_timeout: float = 60.0` safety deadline for a coordinator state that remains `RUNNING` indefinitely. Its expiry SHALL be handled by the owner loop, which schedules cancellation/reconciliation and reports safe terminal `FAILED` or a correlated `FAULT`; it SHALL not be confused with planning, per-RPC action, caller wait, or trajectory-duration timeouts.

The runtime constructor SHALL retain compatible callable injection through `monotonic_clock`, defaulting to `time.monotonic`, and SHALL route it through a dedicated serialized `ValidatedMonotonicClock` wrapper. The wrapper SHALL validate finite, nondecreasing samples and SHALL be the sole source used for deadline creation, deadline comparison, and relative wait calculations. An invalid sample SHALL fail the clock dependency contract, not create lifecycle `FAULT` state.

#### Scenario: RPC does not block lifecycle ownership
- **WHEN** a coordinator RPC blocks
- **THEN** the RPC SHALL run on the fixed executor
- **AND** the owner loop SHALL continue processing its serialized timer/request path

#### Scenario: Indefinite running state reaches the physical deadline
- **WHEN** a coordinator task remains `RUNNING` beyond `physical_operation_timeout`
- **THEN** the owner loop SHALL enqueue cancellation and reconciliation
- **AND** the eventual result SHALL be terminal `FAILED` or a correlated `FAULT`
- **AND** duration alone SHALL never be treated as physical completion

### Requirement: Preserve sequential dispatch and compensation
The gateway/actor boundary SHALL preserve sequential multi-task dispatch for complete plans. It SHALL register minimal operation, task, and method correlation before each RPC effect. It SHALL invoke compensation only when earlier tasks are known or potentially active; UNKNOWN activity SHALL fail closed and block replacement execution.

#### Scenario: Later task failure compensates earlier work
- **WHEN** a multi-task plan establishes an earlier task and a later task is rejected or becomes uncertain
- **THEN** the runtime SHALL preserve the existing compensation sequence
- **AND** the normalized operation result SHALL reflect compensation success or fail-closed safety uncertainty

### Requirement: Preserve physical completion polling
Dispatch acceptance SHALL remain distinct from physical completion. The timer scan SHALL poll active coordinator tasks and transition to the existing terminal outcome only after authoritative completion or safe cancellation/inactivity proof.

#### Scenario: Accepted task remains active
- **WHEN** a coordinator accepts a task without reporting physical completion
- **THEN** the runtime SHALL retain the active correlated task
- **AND** it SHALL report the existing running state rather than completion

#### Scenario: Immediate public polling is running-only
- **WHEN** public `poll` is called
- **THEN** it SHALL issue immediate physical polling only for a `RUNNING` operation
- **AND** the owner timer SHALL process deadlines and other timer work
- **AND** `STATUS` SHALL not be guaranteed in reset, shutdown, gateway, or other non-running states

### Requirement: Keep the gateway thin and synchronous at its boundary
The coordinator gateway SHALL contain only request/result adaptation and RPC invocation required by the actor. It SHALL not own lifecycle flags, timers, retries, or independent close state; futures and observer-only waits SHALL carry completion back to the owner loop. Unresolved work SHALL block replacement execution.

The policy, effects, auxiliary, and passive module boundaries SHALL remain explicit and testable. The runtime SHALL NOT require a numeric actor-size limit or an additional public-façade/private-owner layer; behavioral ownership and effect isolation are the contract.

#### Scenario: Gateway result is applied by owner
- **WHEN** the gateway future completes with acceptance, rejection, cancellation, completion, or uncertainty
- **THEN** the owner loop SHALL normalize and apply the result using minimal operation/task/method correlation
- **AND** the gateway SHALL not publish an independent lifecycle state
