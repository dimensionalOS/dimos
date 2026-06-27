## Why

`RoboPlanWorld` is already the RoboPlan-backed planning world and native planner for DimOS manipulation, but inverse kinematics still routes through separate backends such as Pink or generic Jacobian IK. That forces RoboPlan stacks to cross model/context boundaries for IK even though `RoboPlanWorld` already owns the RoboPlan scene, planning-group metadata, current belief state, joint-limit ordering, collision checks, and group FK/Jacobian queries.

RoboPlan exposes `roboplan.optimal_ik` bindings locally, including the Oink task solver used by robokin's RoboPlan wrapper. DimOS can use those bindings directly inside `RoboPlanWorld` to provide a RoboPlan-native `KinematicsSpec` without adding a separate adapter object or reviving the deferred robokin backend.

## What Changes

- Add `backend="roboplan"` to manipulation kinematics configuration with Oink solver tunables.
- Extend factory validation so `kinematics_name="roboplan"` requires `world_backend="roboplan"`.
- Make `create_kinematics(...)` return the existing `RoboPlanWorld` instance as `KinematicsSpec` for RoboPlan kinematics, matching the current `PlannerSpec` pattern.
- Add `KinematicsSpec.solve(...)` and `solve_pose_targets(...)` behavior to `RoboPlanWorld` using `roboplan.optimal_ik.Oink` with one `FrameTask` per pose target.
- Support multi-target IK for non-overlapping selected planning groups, including multi-robot selections and same-robot disjoint groups represented by generated RoboPlan composite groups.
- Keep IK collision behavior narrow: verify final candidate collision freedom, but do not implement obstacle-aware IK optimization in this slice.
- Keep the deferred robokin backend out of scope.

## Capabilities

### New Capabilities

- `roboplan-oink-kinematics`: RoboPlanWorld provides RoboPlan Oink-backed inverse kinematics through the existing DimOS `KinematicsSpec` surface.

### Modified Capabilities

- `roboplan-composite-multi-robot-planning`: RoboPlan composite group metadata is reused by IK target selections; no planner behavior change is required.

## Impact

- Affected code areas:
  - `dimos/manipulation/planning/kinematics/config.py`
  - `dimos/manipulation/planning/factory.py`
  - `dimos/manipulation/planning/world/roboplan_world.py`
  - `dimos/manipulation/test_planning_factory.py`
  - `dimos/manipulation/test_roboplan_world.py`
- Public configuration impact:
  - `kinematics_name="roboplan"` and/or `kinematics.backend="roboplan"` become valid only with `world_backend="roboplan"`.
- Dependency impact:
  - No new dependency is introduced; the change uses existing `roboplan.optimal_ik` bindings when RoboPlan is installed.
- Behavior impact:
  - RoboPlan planning remains wired as `planner_name="roboplan" -> RoboPlanWorld`.
  - RoboPlan IK uses the same single `RoboPlanWorld` object rather than a separate adapter.
