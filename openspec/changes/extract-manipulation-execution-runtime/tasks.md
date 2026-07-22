## 1. Characterize existing behavior and update contracts

- [x] 1.1 Inventory current manipulation lifecycle states, execution flags, coordinator RPC call sites, planning APIs, and status consumers; see the historical baseline below.
- [x] 1.2 Add characterization tests for current planning, dispatch acceptance, physical completion, cancellation, reset, and coordinator uncertainty behavior.
- [x] 1.3 Define the runtime event, normalized outcome, operation/attempt identity, and atomic snapshot APIs in the manipulation execution test contract.
- [x] 1.4 Update API tests and call sites to use `IDLE`, `PLANNING`, `READY`, `DISPATCHING`, `RUNNING`, `CANCELLING`, and `FAULT` only.
- [x] 1.5 Update API tests to reject `robot_name` and all partial-robot execution selections.

## 2. Implement lifecycle state, reducer, and snapshots

- [x] 2.1 Add the lifecycle state model with exactly the seven public states and explicit legal-transition validation.
- [x] 2.2 Add runtime event types for planning, plan replacement, execute, cancel, reset, coordinator responses, task observations, and polling.
- [x] 2.3 Implement the formal reducer so invalid transitions leave state unchanged and produce a rejected-transition result.
- [x] 2.4 Implement atomic status snapshots containing consistent state, plan, operation, attempt, task, and fault data.
- [x] 2.5 Add reducer tests for legal transitions, rejected transitions, coherent snapshots, and dispatch acceptance versus physical completion.

## 3. Introduce the single-owner execution runtime

- [x] 3.1 Add the internal runtime owned by `ManipulationModule` with a single-owner event queue and serialized processing loop.
- [x] 3.2 Move coordinator client ownership, ready-plan storage, operation IDs, attempts, task knowledge, and fault details into the runtime.
- [x] 3.3 Ensure execute, cancel, reset, planning results, coordinator polling, and snapshot publication all use the serialized runtime path.
- [x] 3.4 Add runtime tests proving that concurrent requests cannot create overlapping execution attempts or inconsistent snapshots.
- [x] 3.5 Add tests proving that planning and execution work are blocked while a remote attempt is active, while cancellation and reset remain available.

## 4. Move planning results and ready-plan lifecycle

- [x] 4.1 Route planning start requests from `ManipulationModule` to the runtime and transition accepted requests to `PLANNING`.
- [x] 4.2 Route planning success and failure results to runtime events with unique plan identities and normalized outcomes.
- [x] 4.3 Implement atomic replacement so a new successful planning request discards the prior ready plan before exposing the new plan as `READY`.
- [x] 4.4 Enforce validation that every robot trajectory represented by a plan is present, and reject incomplete plans without creating executable ready state.
- [x] 4.5 Implement single-use plan consumption before dispatch and reject repeated dispatch of a consumed plan identity.
- [x] 4.6 Add tests for planning from `IDLE` and `READY`, replacement, planning failure, plan identity, and single-use dispatch semantics.

## 5. Implement mapping, dispatch, and task knowledge

- [x] 5.1 Build and validate the coordinator-name inverse mapping once from canonical configuration references during runtime initialization.
- [x] 5.2 Add the global-plan to local-coordinator-request mapping and normalized coordinator-result mapping while preserving plan and attempt identities.
- [x] 5.3 Register active plan, operation, attempt, and task-correlation context before invoking the coordinator execute RPC.
- [x] 5.4 Implement the single complete-robot coordinator dispatch path through `DISPATCHING` to `RUNNING` on acceptance.
- [x] 5.5 Record coordinator task knowledge and reject stale or mismatched responses without attributing them to the active attempt.
- [x] 5.6 Add tests for mapping validation, complete trajectory requests, pre-registration, immediate responses, accepted-but-not-complete tasks, and normalized terminal results.

## 6. Implement cancellation, polling, fault, and reset safety

- [x] 6.1 Implement serialized status polling and task reconciliation for active attempts, including physical completion detection.
- [x] 6.2 Implement cancellation through `CANCELLING`, reporting cancellation only after coordinator cancellation or inactivity is confirmed.
- [x] 6.3 Normalize unknown, malformed, unavailable, and contradictory coordinator results as safety uncertainty and latch `FAULT`.
- [x] 6.4 Ensure latched `FAULT` rejects new planning and execution and remains latched across subsequent polls.
- [x] 6.5 Implement reset reconciliation that clears plan, attempt, queue, and fault state only after remote inactivity is proven, returning to `IDLE`.
- [x] 6.6 Ensure reset remains retryable and does not issue new trajectory execution while remote activity is active or uncertain.
- [x] 6.7 Add tests for rejection of execution during an active attempt (without queueing), terminal coordinator rejection returning to `IDLE`, active cancellation confirmation, unknown task states, fault latching, and safe/unsafe reset.

## 7. Wire module, operator, and UI compatibility

- [x] 7.1 Replace direct lifecycle assignments and execution flags in `ManipulationModule` with runtime event submission and snapshot consumption.
- [x] 7.2 Update manipulation operators and callers for the new execution API, normalized outcomes, and removal of robot-specific selection.
- [x] 7.3 Update UI/state adapters to consume atomic snapshots and map all seven public states without legacy state names.
- [x] 7.4 Add compatibility tests covering module status publication, operator actions, UI state mapping, fault visibility, and reset behavior.

## 8. Remove legacy paths and validate the migration

- [x] 8.1 Remove legacy module-level lifecycle bookkeeping, duplicate coordinator access, and active `ManipulationModule` direct trajectory execution paths. Do not move or remove `dimos/manipulation/control/coordinator_client.py`: it is standalone diagnostic/operator CLI direct-RPC tooling, not `ManipulationModule` execution; changing it is a separate compatibility-scoped change.
- [x] 8.2 Remove manipulation-layer trajectory start-state freshness validation without adding a replacement gate in this change.
- [x] 8.3 Remove partial-robot execution handling and the `robot_name` argument from implementation, adapters, and tests.
- [x] 8.4 Run targeted lifecycle runtime, coordinator execution, manipulation module, operator, and UI compatibility tests.
- [x] 8.5 Run formatting, lint, type checks, and the relevant OpenSpec or repository validation checks. Verified with `uv run --with mypy mypy dimos/manipulation`: 54 source files, no issues found.
- [x] 8.6 Review test coverage for every lifecycle state, transition, safety fault, reset path, plan replacement, and single-use invariant.

## Verification note

Self-hosted and Drake-dependent verification is environment-limited for this change and is not claimed unless run on the corresponding self-hosted/Drake environment. Local artifact review and environment-independent checks may be run separately; unavailable self-hosted/Drake checks must be reported as not run rather than inferred as passing.
