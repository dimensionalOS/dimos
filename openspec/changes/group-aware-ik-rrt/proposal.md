## Why

After world backends understand planning groups, the solver and planner layer must consume group IDs and group-local targets. This PR keeps algorithm review separate from both backend plumbing and public module/UI migration.

## What Changes

- Update PinkIK, Jacobian IK, and Drake optimization IK to resolve target frames from planning groups.
- Update RRT planning to handle group-local joint targets, group candidate selection, collision checking, and result projection.
- Add focused IK/planner tests.

## Capabilities

### New Capabilities
- `group-aware-ik-rrt`: IK and RRT planning support explicit planning groups and group-local targets.

### Modified Capabilities
- `manipulation-planning-algorithms`: Planners and IK solvers no longer rely on robot-scoped end-effector metadata.

## Impact

- Base branch: PR 2 `planning-world-monitor-groups`.
- Reference implementation: `cc/spec/movegroup`.
- Primary files: `dimos/manipulation/planning/kinematics/*`, `dimos/manipulation/planning/planners/rrt_planner.py`, related tests.
- Out of scope: public `ManipulationModule` API migration and Viser UI.
