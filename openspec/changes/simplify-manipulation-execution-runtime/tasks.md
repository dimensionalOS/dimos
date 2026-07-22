## 1. Pure extraction and re-exports

Baseline before Phase 1: `execution_runtime.py` was 2,084 LOC; topology/materialization occupied lines 82–337 and public models/results occupied lines 342–469. The public boundary was the `dimos.manipulation.execution_runtime` import path used by the manipulation module, tests, and helpers. Preserve the achieved policy/effects/auxiliary/passive module separations; LOC is recorded only as maintainability evidence, not as a hard actor gate or a requirement for a public-façade/private-owner split.

- [x] 1.1 Record current public `ManipulationModule` behavior and LOC/boundary baselines without changing source behavior.
- [x] 1.2 Extract pure plan materialization into a 220–280 LOC internal module with no lifecycle ownership.
- [x] 1.3 Extract a faithful models and correlation/result baseline into an internal module (190 LOC in Phase 1; the 100–150 LOC target remains a later simplification target, not a reason for unsafe compression).
- [x] 1.4 Add compatibility re-exports so existing runtime imports and public module semantics remain unchanged.
- [x] 1.5 **Validation gate:** run plan/materialization, model, import, and existing manipulation characterization tests; no behavior changes are allowed.

## 2. Futures and explicit result waits

- [x] 2.1 Extract the coordinator gateway into an 80–120 LOC adapter containing only RPC request/result conversion.
- [x] 2.2 Route blocking coordinator calls through retained futures while registering minimal operation/task/method correlation before each RPC effect. The existing auxiliary-thread topology remains intentionally unchanged until Phase 3.
- [x] 2.3 Replace action-thread completion signaling at the owner boundary with explicit correlated completion results and observer-only waits with normalized outcomes.
- [x] 2.4 **Validation gate:** verify immediate acceptance, rejection, completion, cancellation, malformed, unavailable, and stale-result cases against characterization tests.

## 3. Fixed executor and owner-loop timer scan

- [x] 3.1 Add one fixed blocking-RPC executor with exactly four daemon workers, bounded lifetime, and truthful unresolved-work shutdown.
- [x] 3.2 Add one owner-loop timer scan for polling, deadlines, cancellation reconciliation, deferred reset handling, and shutdown observation.
- [x] 3.3 Remove poller, per-action deadline, reset, and gateway-close threads without changing polling cadence or deadline semantics.
- [x] 3.4 Keep module-side timeout reconciliation out of the new path.
- [x] 3.5 **Validation gate:** run concurrency, deadline/UNKNOWN fail-closed, physical-completion, deferred-reset, and truthful-shutdown tests; inspect four-worker ownership.
- [x] 3.6 Add the module-configured `physical_operation_timeout: float = 60.0` safety deadline for indefinitely `RUNNING` coordinator state; owner-schedule cancellation/reconciliation on expiry and produce safe terminal `FAILED` or correlated `FAULT`. Keep it distinct from planning, per-RPC action, caller observation, and trajectory-duration timeouts.
- [x] 3.7 Add the additive compatible `monotonic_clock` callable injection, defaulting to `time.monotonic`, and route it through a dedicated serialized `ValidatedMonotonicClock` wrapper; validate finite/nondecreasing samples, treat invalid samples as dependency-contract failures rather than lifecycle `FAULT`, and route every deadline creation/comparison and relative wait calculation through the wrapper.

## 4. Focused handlers and mutable owner state

- [x] 4.1a Add a behavior-preserving `_set_state(**changes)` owner-state wrapper.
- [x] 4.1b Extract planning lifecycle transitions into focused owner-only handlers.
- [x] 4.1c Enforce owner-only `RuntimeContext` writes in dispatch admission and lifecycle transitions; permit only a caller-owned latch that seals gateway admission before owner shutdown processing.
- [ ] 4.1d **Deferred/out of scope:** rich handler decomposition for reset generations and action-level transitions; retain only the compact task dispatch, result reconciliation, deadline, conditional compensation, and shutdown seam.
- [x] 4.2a Introduce the mutable owner-state record.
- [x] 4.2b Migrate lifecycle state writes while preserving immutable snapshots and strict owner-only `RuntimeContext` ownership.
- [x] 4.2c Migrate minimal operation/task/method correlation state writes under the owner-only rule.
- [ ] 4.3 **Deferred/replaced:** exact history/alias removal and rich action/reset bookkeeping are out of scope; retain only small current/recent results and the existing compact policy/effects/auxiliary/passive boundaries.
- [x] 4.4 Preserve sequential multi-task dispatch, compensation only for known or potentially-active tasks, physical completion polling, and safe rejection of overlapping execution.
- [x] 4.5a Verify planning and ready-plan replacement behavior.
- [ ] 4.5b **Deferred/replaced:** rich dispatch-admission and single-use-plan bookkeeping is out of scope; verify only unresolved-work admission blocking.
- [x] 4.5c Verify the required compact safety matrix: UNKNOWN fail-closed, sequential dispatch, conditional compensation, 60-second physical deadline, cancellation, unsafe/timed-out reset remaining `FAULT`, and truthful shutdown.
- [x] 4.5e Verify immediate public polling is `RUNNING`-only; no special narrow timer-precheck or `STATUS` guarantee applies in reset/shutdown/gateway states.
- [x] 4.5d **Validation gate:** run the required compact Phase-4 safety matrix after all slices.

## 5. Module and API cleanup

- [x] 5.1 Keep `ManipulationModule` as a public adapter over the runtime and remove module-owned lifecycle mutation and reconciliation.
- [x] 5.2 Remove freshness- and robot-name-based execution selection and partial-plan paths without adding a replacement selection gate.
- [x] 5.3 Preserve all existing public `ManipulationModule` semantics, status snapshots, normalized outcomes, and truthful shutdown behavior.
- [x] 5.4 **Validation gate:** run module, operator, UI/state-adapter, API, and compatibility tests; confirm no unrelated source paths changed.

## 6. White-box test decoupling

- [x] 6.1 Move white-box tests from reducer/thread internals to plan, model, gateway, actor-handler, and owner-loop seams.
- [x] 6.2 Add focused tests for minimal correlation isolation, future completion ordering, owner timer scans, conditional compensation, deferred reset handling, executor shutdown, and the monotonic clock seam.
- [x] 6.2a Add deterministic tests for stale and cleared physical deadlines, including nondecreasing clock samples and no cancellation when the deadline is absent or not due.
- [x] 6.2b Add deterministic tests that retained faults remain observable after physical-deadline handling and that expiry yields safe `FAILED` or correlated `FAULT`.
- [x] 6.2c Add public poll tests proving immediate polling is `RUNNING`-only and that `STATUS` is not guaranteed in reset/shutdown/gateway states.
- [x] 6.2d Add poll-versus-shutdown tests covering clock-driven physical expiry, admission sealing, one shutdown initiator, concurrent rejection, and truthful unresolved shutdown without eventual-success promise.
- [x] 6.2e Add isolated `ValidatedMonotonicClock` unit tests for default/injected callables, serialized access, finite-sample rejection, nondecreasing-sample rejection, and dependency-contract error classification without lifecycle mutation.
- [x] 6.2f Add deterministic lifecycle tests proving all deadline and relative-wait paths use the wrapper, including stale/cleared deadlines, retained faults, physical expiry, deferred reset behavior, and state-dependent poll/shutdown behavior.
- [x] 6.3 Retain black-box characterization coverage for every public lifecycle and safety contract.
- [x] 6.4 **Validation gate:** run targeted manipulation runtime and coordinator test suites with no skipped contract scenarios.

## 7. Final verification

- [x] 7.1 Record the compact seam measurement as maintainability evidence and verify reduced duplicated responsibility across the policy/effects/auxiliary/passive boundaries; there is no hard actor LOC gate or required public-façade/private-owner split.
- [x] 7.2 Verify strict owner-only `RuntimeContext` writes, the sole admission-latch exception, exactly four daemon RPC workers, absence of forbidden thread types, and absence of module-side timeout reconciliation.
- [x] 7.3 Verify physical-operation deadline expiry schedules owner cancellation/reconciliation and yields safe terminal `FAILED` or correlated `FAULT`, never duration-inferred completion.
- [x] 7.4 Run formatting, lint, type checks, targeted manipulation/coordinator tests, the full repository test, and OpenSpec strict validation; formatting/lint/type checks, the manipulation target suite (364 passed, 10 skipped, 16 deselected), OpenSpec validation, and diff review passed. The full repository suite was environment-limited by an unrelated MCP `localhost:9990` port conflict/connection resets (2910 passed, 30 skipped, 289 deselected, 2 failed, 24 errors).
- [x] 7.5 Review the final diff and task ledger so no source implementation task is marked complete merely because work was planned, and only supported completed work remains checked.
