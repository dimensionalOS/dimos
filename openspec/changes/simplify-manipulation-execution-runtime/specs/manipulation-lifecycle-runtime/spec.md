## ADDED Requirements

### Requirement: Preserve serialized lifecycle ownership
The simplified runtime SHALL preserve the existing public `ManipulationModule` lifecycle semantics and SHALL have exactly one owner loop that mutates lifecycle, ready-plan, operation, task, fault, and shutdown state. Any attempt/action/reset bookkeeping is compact local correlation, not a rich generation model.

The owner SHALL be the only writer of `RuntimeContext` and its mutable state. A caller-owned admission latch MAY be written outside the owner only to seal gateway admission before the owner processes shutdown; it SHALL not mutate lifecycle, operation, task, fault, or shutdown state.

The runtime SHALL use one owner loop, exactly four daemon RPC workers, and observer-only caller waits. It SHALL retain only a small current/recent result set; no exact result-history depth is part of the contract.

The policy, effects, auxiliary, and passive module separations SHALL remain explicit and testable. Success SHALL be assessed by reduced duplicated responsibility, tested ownership/effect boundaries, and no observable behavior regression, not by a hard actor LOC threshold or a required public-façade/private-owner split.

The runtime constructor SHALL retain compatible callable injection through `monotonic_clock`, defaulting to `time.monotonic`, and SHALL wrap it in a dedicated `ValidatedMonotonicClock`. The wrapper SHALL serialize source access and SHALL accept only finite, nondecreasing samples. All runtime deadline creation, deadline comparison, and relative wait calculations SHALL route through the wrapper. An invalid sample SHALL be a dependency-contract failure, not a lifecycle `FAULT`, and SHALL not be converted into a remote safety outcome.

#### Scenario: Concurrent lifecycle requests are serialized
- **WHEN** execute, cancel, reset, planning results, and timer events arrive concurrently
- **THEN** the owner loop SHALL apply them in one deterministic order
- **AND** no caller-visible snapshot SHALL combine state from different operations

### Requirement: Preserve correlation and normalized outcomes
The runtime SHALL retain minimal operation, task, and method correlation through planning, sequential dispatch, polling, cancellation, compensation, deadlines, faults, and shutdown. Registration SHALL occur before each RPC effect. Stale, mismatched, or unresolved results SHALL NOT mutate the current operation, and unresolved work SHALL block replacement execution; rich action/reset generations are out of scope.

#### Scenario: Late result cannot affect a new operation
- **WHEN** an RPC future from a prior operation completes after reset or a new operation begins
- **THEN** the owner loop SHALL classify it using its original correlation
- **AND** it SHALL not apply the result to the new operation

### Requirement: Preserve safety and shutdown semantics
Unknown, malformed, unavailable, contradictory, or deadline-uncertain remote results SHALL fail closed and preserve the existing safety-fault behavior. Unsafe or timed-out reset SHALL remain `FAULT` with no in-process retry; retryable reset generations SHALL be deferred. Shutdown SHALL report unresolved work truthfully.

#### Scenario: Deadline uncertainty faults safely
- **WHEN** a blocking coordinator result cannot be obtained before its established deadline
- **THEN** the runtime SHALL preserve the existing safety fault outcome
- **AND** it SHALL not report completion or safe reset without proof

#### Scenario: Unsafe reset remains faulted
- **WHEN** reset is requested while remote inactivity is active, uncertain, or timed out
- **THEN** the runtime SHALL retain the fault/safety condition and issue no new execution
- **AND** it SHALL not retry the reset generation in-process

#### Scenario: Shutdown is truthful
- **WHEN** shutdown begins with an unresolved RPC or remote attempt
- **THEN** the runtime SHALL not report that work as completed or cancelled without proof
- **AND** the public shutdown result SHALL reflect the unresolved condition

#### Scenario: Shutdown has one initiator
- **WHEN** a shutdown is already being processed by one caller
- **THEN** a concurrent shutdown caller SHALL be rejected
- **AND** expiry of the shutdown deadline SHALL report unresolved work without promising eventual success

### Requirement: Enforce a runtime-owned physical operation deadline
Each module-configured runtime SHALL expose `physical_operation_timeout: float = 60.0`. The owner SHALL start this safety deadline when a physical operation enters `RUNNING` and SHALL owner-schedule cancellation and reconciliation when it expires. This deadline is distinct from `planning_timeout`, per-RPC `action_timeout`, caller observation wait timeouts, and trajectory duration.

#### Scenario: Physical operation exceeds its safety deadline
- **WHEN** a correlated operation remains `RUNNING` until `physical_operation_timeout` expires
- **THEN** the owner SHALL schedule cancellation and reconciliation
- **AND** it SHALL produce a safe terminal `FAILED` result or a correlated `FAULT`
- **AND** it SHALL not infer completion from trajectory duration or permit module-side reconciliation

#### Scenario: Immediate polling is running-only
- **WHEN** the public `poll` method is called
- **THEN** it SHALL perform immediate physical polling only for a `RUNNING` operation
- **AND** the owner timer SHALL remain authoritative for deadlines, reset, shutdown, and other timer processing
- **AND** no special precheck SHALL guarantee `STATUS` during reset, shutdown, gateway, or other non-running states

### Requirement: No execution selection by freshness or robot name
The simplified runtime SHALL preserve the contract that execution is not selected by freshness or robot name. It SHALL dispatch the complete plan represented by the planning groups and SHALL reject partial or robot-name-selected execution without issuing coordinator work.

#### Scenario: Complete plan dispatch ignores freshness selection
- **WHEN** a complete ready plan reaches execution with a non-fresh manipulation-layer observation
- **THEN** the runtime SHALL not reject it solely for that observation
- **AND** it SHALL continue through normal coordinator acceptance and safety handling
