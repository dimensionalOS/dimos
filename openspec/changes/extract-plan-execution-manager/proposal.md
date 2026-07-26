## Why

`ManipulationModule` currently mixes motion planning with coordinator dispatch,
joint-name translation, cancellation, rollback, and execution concurrency. This
mix makes the module difficult to reason about and forces execution safety tests
through a large planning surface.

Extract planned execution behind a small in-process interface so execution policy
has one owner and one focused test surface. Preserve existing module RPCs while
making dispatch and cancellation outcomes explicit inside the module.

## What Changes

- Add a plan execution manager that validates and dispatches complete generated
  plans and cancels tasks that may still be active.
- Add a generic owning control-coordinator client for task, status, and gripper
  RPC mechanics, plus a narrow manipulation adapter that converts raw results into
  typed execution and cancellation outcomes.
- Add immutable per-robot execution targets with validated model-to-coordinator
  joint-name mappings.
- Use replacement execution: confirm prior tasks safe through cancellation before
  dispatching another plan.
- Preserve existing `ManipulationModule` boolean RPCs and its public state
  projection while moving execution transaction facts into the manager.
- **BREAKING**: Reject partial execution of a multi-robot generated plan. The
  complete plan becomes the atomic execution unit.
- **BREAKING**: Reject generated plans whose planning status is not successful,
  even if they contain residual path or trajectory data.
- Keep direct gripper actuation, execution observation, and physical completion
  tracking outside this change.

## Affected DimOS Surfaces

- Modules/streams: Refactors `ManipulationModule` internals; adds an owning
  coordinator client plus in-process execution policy and adapter classes. No
  stream contract changes.
- Blueprints/CLI: Existing manipulation blueprints and CLI commands remain
  source-compatible; no registry regeneration is expected.
- Skills/MCP: Existing manipulation skills retain their signatures and boolean or
  `SkillResult` behavior.
- Hardware/simulation/replay: Hardware and simulated manipulators use the same
  coordinator task commands. Dispatch and cancellation safety behavior changes as
  described above; no coordinator protocol changes are required.
- Docs/generated registries: Update manipulation capability documentation. No
  generated blueprint registry changes are expected.

## Capabilities

### New Capabilities

- `manipulation-plan-execution`: Defines validation, complete-plan dispatch,
  replacement, cancellation, and failure-safety behavior for generated
  manipulation plans.

### Modified Capabilities

None.

## Impact

Users keep the existing manipulation RPC and skill surfaces. Developers gain a
focused execution interface and typed results, but callers that relied on
per-robot filtering of a multi-robot plan or executable non-success plans must
generate a valid plan for the intended robot set instead.

The change adds no external dependency and does not modify the
`ControlCoordinator` protocol. Unit tests will cover the manager and adapter
interfaces, including concurrent execution, cancellation priority, replacement,
partial dispatch rollback, uncertain cancellation, freshness validation,
multi-robot timing preservation, and joint-name alignment. Existing manipulation
integration tests will verify compatibility delegation.
