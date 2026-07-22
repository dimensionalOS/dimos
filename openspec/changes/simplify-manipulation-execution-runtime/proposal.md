## Why

The newly added manipulation execution runtime preserves the required safety and lifecycle behavior, but its 2,084 lines concentrate planning materialization, RPC adaptation, mutable lifecycle state, polling, deadlines, reset handling, and shutdown mechanics in one over-engineered implementation. Simplifying those seams now will make the behavior easier to audit and maintain without changing the public `ManipulationModule` contract or safety semantics.

## What Changes

- Retain and document the achieved separations between policy, effects, auxiliary, and passive modules, with explicit boundaries for plan materialization, the coordinator gateway, runtime models, and the runtime owner.
- Replace per-action/poller/deadline/reset/gateway-close threads with one fixed daemon blocking-RPC executor and one owner-loop timer scan.
- Replace the giant reducer and redundant flags/history/aliases with focused handlers over one owner-owned mutable runtime state.
- Use futures and observer-only waits for blocking coordinator operations, preserving minimal operation/task/method correlation and serialized lifecycle decisions. Unresolved work blocks replacement execution.
- Enforce a runtime-owned `physical_operation_timeout: float = 60.0` safety deadline for operations that remain `RUNNING`, with owner-scheduled cancellation/reconciliation and safe terminal failure or correlated fault.
- Enforce strict owner-only `RuntimeContext` writes; permit only a caller-owned admission latch to seal gateway admission before owner shutdown processing.
- Use a dedicated serialized `ValidatedMonotonicClock` wrapper behind compatible `monotonic_clock` callable injection/default behavior; reject invalid samples as dependency-contract failures rather than lifecycle faults.
- Preserve sequential multi-task dispatch, compensation only for known or potentially-active tasks, physical-completion polling, UNKNOWN fail-closed safety outcomes, the 60-second physical deadline, admission sealing, and truthful shutdown.
- Defer retryable reset generations; unsafe or timed-out reset remains `FAULT` with no in-process retry. Retain only a small current/recent result set rather than promising exact history depth.
- Permit one shutdown initiator only: concurrent shutdown callers are rejected, and no eventual success is promised after the shutdown deadline.
- Make the owner timer authoritative. Public immediate polling is only for `RUNNING` operations and has no special narrow timer-precheck guarantee.
- **BREAKING** Remove implementation-only freshness/robot-name execution selection paths; retain the existing public manipulation semantics without freshness- or robot-name-based execution selection.
- Keep all changes confined to the new internal runtime structure; do not add execution selection based on freshness or robot name.
- Do not impose a hard runtime-actor LOC threshold or require a public-façade/private-owner split; judge the refactor by reduced duplicated responsibility, tested boundaries, and unchanged behavior.

## Capabilities

### New Capabilities
- `manipulation-lifecycle-runtime`: Compact fail-closed lifecycle ownership, minimal correlation, safety, reset, and shutdown semantics for the simplified runtime.
- `coordinator-trajectory-execution`: Compact gateway, sequential dispatch, conditional compensation, polling, and blocking-RPC execution semantics.

### Modified Capabilities

## Impact

- Affects only the implementation and tests of the recently added manipulation execution runtime and its `ManipulationModule` integration.
- Introduces no external dependency and no source changes outside the eventual runtime refactor.
- Preserves public `ManipulationModule` methods, lifecycle outcomes, serialized ready-plan behavior, and operator-visible safety results.
- Success is reduced duplicated responsibility across the existing policy/effects/auxiliary/passive separations, tested ownership and effect boundaries, and no behavior regression in the safety and lifecycle contract.
