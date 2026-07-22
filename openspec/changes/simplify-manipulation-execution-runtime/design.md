# Design: Simplified Manipulation Execution Runtime

## Context

The just-added manipulation runtime is approximately 2,084 LOC. It currently spreads coordination across a large reducer, poller and action-specific threads, deadline/reset/gateway-close threads, duplicated flags and histories, and module-side timeout reconciliation. That structure obscures the single-owner lifecycle contract even though the required behavior is already defined and must remain unchanged.

The refactor is an internal, behavior-preserving rewrite with a compact initial-runtime scope. The public `ManipulationModule` API and all safety outcomes remain the compatibility boundary. Blocking coordinator RPCs must not block the owner loop, while lifecycle state must still have one writer and deterministic ordering.

## Goals / Non-Goals

### Goals

- Preserve four explicit seams and the achieved policy/effects/auxiliary/passive module separations:
  - plan materialization: 220–280 LOC;
  - coordinator gateway: 80–120 LOC;
  - models: 100–150 LOC in the final simplified architecture;
  - runtime owner: no hard LOC threshold; keep responsibilities reviewable without splitting a required public façade from its private owner.
- Preserve all existing public `ManipulationModule` semantics, including serialized lifecycle and ready-plan behavior.
- Preserve minimal operation/task/method RPC correlation, sequential multi-task dispatch, compensation only for known or potentially-active tasks, physical completion polling, UNKNOWN fail-closed safety faults, and truthful shutdown.
- Keep lifecycle state and mutable ownership in one runtime owner loop.
- Make blocking RPC completion explicit through futures and result waits.
- Enforce a module-configured `physical_operation_timeout: float = 60.0` safety deadline for operations that remain `RUNNING` indefinitely.
- Accept an additive runtime `monotonic_clock` seam defaulting to `time.monotonic` for deterministic deadline behavior.
- Keep the initial runtime compact: retryable reset generations, rich action/reset generations, exact result-history depth, and eventual post-deadline shutdown success are deferred rather than implemented promises.
- Define success by reduced duplicated responsibility, tested policy/effect/auxiliary/passive boundaries, and no observable behavior regression; LOC is an observation, not a release gate.

### Non-Goals

- No redesign of planning algorithms, trajectory data, coordinator protocol, or public manipulation semantics.
- No freshness-based or robot-name-based execution selection; neither is a replacement safety gate.
- No new poller, per-action deadline, reset, gateway-close, or module-side timeout-reconciliation thread.
- No UI/layout work, unrelated source cleanup, or changes outside the requested change during implementation.

## Decisions

### Four narrow implementation seams

Plan materialization converts planner output into a complete executable plan and owns no lifecycle state. Models contain immutable records/enums/results and the single mutable owner-state record. The gateway is a thin coordinator RPC adapter. The actor owns lifecycle handlers, minimal correlation, dispatch sequencing, polling, conditional compensation, and shutdown.

The alternative of retaining the existing reducer and extracting helpers was rejected: it would preserve duplicate state representations and make ownership and transition behavior difficult to verify. The alternative of splitting into many event-specific services was rejected because it would recreate cross-thread ordering problems.

### Fixed daemon blocking-RPC executor

All potentially blocking coordinator RPCs run on exactly four daemon executor workers. Each submission registers operation, task, and method correlation before the effect and returns a future; the owner loop consumes completed results at defined handler boundaries. RPC deadlines are represented as future/result outcomes and safety faults, not as per-action timer threads. Unresolved work blocks replacement execution; no rich action/reset generation model is required.

The executor is fixed for the runtime lifetime and is shut down truthfully: one shutdown initiator seals admission, shutdown stops accepting work, observer-only waits observe known futures, and incomplete/unknown remote work is reported rather than pretending it is complete. Concurrent shutdown callers are rejected. A shutdown deadline does not promise eventual success after the deadline; daemon workers prevent process teardown from being held hostage, but do not weaken fail-closed outcomes.

Admission has one exception to strict owner-only writes: a caller-owned latch may seal gateway admission before the owner processes shutdown. That latch is an admission gate only; all `RuntimeContext` writes, including shutdown transitions, remain owner-loop writes.

### Owner-loop timer scan

One daemon owner loop serializes requests, completed future results, and a monotonic timer scan. The owner timer is authoritative for physical-completion polling, deadline observation, cancellation reconciliation, deferred reset handling, and shutdown observation. It does not perform RPCs itself; it schedules blocking work on the fixed executor and applies results only after correlation checks.

This replaces poller, per-action deadline, reset, and gateway-close threads. A single timer scan is preferred over independent timers because it preserves ordering and avoids races between timeout, cancellation, reset, and shutdown.

### Monotonic clock seam and public polling

The runtime constructor keeps a compatible callable injection named `monotonic_clock`, defaulting to `time.monotonic`. It wraps that callable in a dedicated `ValidatedMonotonicClock` source wrapper. The wrapper serializes access, validates that every sample is finite and nondecreasing, and is the only clock exposed to runtime deadline code. An invalid sample violates the injected dependency contract and raises a dependency-contract error; it is not converted into lifecycle `FAULT` state or a remote safety outcome.

Every deadline creation, deadline comparison, and relative wait calculation routes through `ValidatedMonotonicClock`. Wall-clock time, mixed clock domains, and ad hoc elapsed-time calculations are not allowed for runtime deadlines. The compatible constructor surface remains callable injection/default behavior; validation and serialization are supplied by the wrapper rather than by changing callers.

The owner timer is the authoritative deadline processor. Public immediate polling is supported only while an operation is `RUNNING`; it does not provide a special narrow timer-precheck guarantee and does not promise `STATUS` in reset, shutdown, gateway, or other non-running states.

### Focused handlers and one mutable state

Phase 4 is decomposed into reviewable slices. Slice 1 adds the `_set_state` write
wrapper and extracts only planning, dispatch-admission, and lifecycle-deadline
branches. It intentionally retains the immutable context representation and all
Phase 3 scheduler/executor mechanics. Later slices will migrate state ownership,
remove redundant bookkeeping, and extract task/cancellation/reset/shutdown
handlers.

Handlers are grouped by concern: planning/ready-plan materialization, execute admission, sequential task dispatch, task-result reconciliation, physical-operation deadline expiry, conditional cancellation/compensation, deferred reset handling, and shutdown. Each handler mutates only the owner-owned state record and publishes the existing snapshot/result contract. Correlation IDs are limited to operation/task/method RPC identity; stale or unresolved results fail closed and cannot replace active work. Result retention is small and current/recent, with no exact history-depth promise.

The giant reducer, redundant flags, history/alias fields, and module-side timeout reconciliation are removed. Public methods remain adapters that submit owner-loop requests and wait for the corresponding future/result, rather than independently reconciling lifecycle state. This does not require a separate public façade/private owner split: the required boundary is behavioral ownership and tested effects, not a numeric actor size or an additional public layer.

The physical-operation deadline starts when the operation enters `RUNNING` and is 60 seconds by default. It is not planning time, an action/RPC deadline, a caller observation timeout, or trajectory duration. On expiry the owner schedules cancellation and reconciliation; only authoritative reconciliation can yield completion, otherwise the owner publishes safe terminal `FAILED` or a correlated `FAULT`. The module never reconciles this deadline.

### Sequential multi-task dispatch and compensation

The actor dispatches the complete plan one coordinator task at a time. It registers operation, task, and method correlation before each RPC. If a later task fails after an earlier task is known or potentially active, compensation is invoked in order; if activity is UNKNOWN, the outcome remains fail-closed and replacement is blocked. No second operation is admitted while unresolved work remains.

### Reset and shutdown scope

Reset generations that can safely be retried are deferred to a later scope. An unsafe or timed-out reset remains `FAULT` and is not retried in-process. Shutdown has one initiator; concurrent callers are rejected, observer-only waits do not mutate lifecycle state, and expiry reports unresolved work without promising eventual success. Admission sealing is the sole caller-side latch exception before owner shutdown processing.

## Risks / Trade-offs

- **Blocking RPC exceeds its expected duration** → represent the result as an explicit deadline/unknown safety outcome, let the owner loop continue its timer scan, and preserve reset proof requirements.
- **A stale or unresolved future completes after reset or shutdown** → retain minimal operation/task/method correlation until classification; never apply it to new work, and fail closed when activity is unknown.
- **Daemon executor hides unfinished work at process exit** → truthful shutdown reports unresolved work and does not claim cancellation or completion without proof.
- **LOC targets encourage over-compression** → treat measurements as maintainability evidence only; prioritize reduced duplicated responsibility, tested boundaries, and the required safety matrix over an arbitrary actor-size gate.
- **Phase 1 faithful extraction exceeds the final models budget** → retain the maintainable 190-LOC models baseline during pure extraction; defer reduction to the later simplification phases and do not compress records unsafely.
- **Changing test seams masks behavior changes** → run characterization and black-box compatibility tests before and after each extraction; decouple white-box tests only after equivalent observable coverage exists.
- **A clock seam hides an unsafe clock** → put finite/nondecreasing validation and serialized access in `ValidatedMonotonicClock`; treat invalid samples as dependency-contract failures, not lifecycle faults, and test the wrapper independently.
- **Public polling races the timer owner** → make the owner timer authoritative and limit immediate public polling to `RUNNING` operations.

## Migration Plan

1. Freeze characterization coverage and extract pure modules with re-exports; no behavior change.
2. Introduce futures/result waits behind the existing gateway and verify correlation and outcomes.
3. Replace thread topology with the fixed executor and owner-loop timer scan.
4. Move logic into focused handlers and one mutable owner state, deleting redundant reducer data.
5. Clean module/API adapters while preserving public semantics and removing freshness/robot-name selection paths.
6. Keep only compact seam tests and the required safety matrix; defer rich reset-generation/history coverage.

Rollback at each phase is a file-level revert before the next phase; no data migration or deployment migration is required.

## Open Questions

- None blocking implementation. Existing polling cadence, RPC deadline values, and reset proof criteria remain authoritative and must be carried over unchanged.
