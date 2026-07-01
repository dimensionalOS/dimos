## Why

Once planning groups exist, the backend query layer must answer group-scoped FK, Jacobian, joint-state, and collision questions. This is the second reviewable PR in the stack: it consumes the foundation without exposing the public `ManipulationModule` API or UI rewrite.

## What Changes

- Teach Drake and RoboPlan world backends to resolve planning groups.
- Add group FK/Jacobian APIs and group-local joint ordering behavior.
- Update world monitor and robot state monitor logic for group-aware state access.
- Preserve legacy robot-scoped wrappers only where needed for compatibility, resolving through a unique pose-targetable group.
- Add backend and monitor tests.

## Capabilities

### New Capabilities
- `planning-world-monitor-groups`: World and monitor support for planning-group queries.

### Modified Capabilities
- `manipulation-world-backends`: Drake/RoboPlan backends understand group-local chains and target frames.

## Impact

- Base branch: PR 1 `planning-groups-foundation`.
- Reference implementation: `cc/spec/movegroup`.
- Primary files: `dimos/manipulation/planning/world/*`, `dimos/manipulation/planning/monitor/*`, RoboPlan world tests, Drake planning-group tests.
- Out of scope: IK/RRT algorithm migration, `ManipulationModule`, Viser UI, and control integration.
