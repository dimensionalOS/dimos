## Why

RoboPlan currently cannot plan coupled motion for multiple registered robots because `RoboPlanWorld` builds one RoboPlan `Scene` from one robot model and rejects additional robots. Dual-arm manipulation needs one collision-aware planning scene where selected planning groups can move together while the Planning world remains the authoritative belief state.

## What Changes

- Add multi-robot RoboPlan support by finalizing registered robot models into one Composite RoboPlan model: a generated RoboPlan-facing URDF and SRDF.
- Generate deterministic Composite planning groups for all non-overlapping planning-group combinations of size two or greater, subject to a configurable safety cap.
- Preserve DimOS public names (`robot/joint`, planning-group IDs) while mapping them to RoboPlan-native prefixed names inside the composite model.
- Support coupled RoboPlan-native planning for multi-group selections by routing each supported selection to one generated Composite planning group.
- Set RoboPlan `Scene` current full joint positions from the Planning world before invoking group RRT so non-selected joints are held at the authoritative belief state.
- Return `PlanningResult.path` as global `JointState` waypoints in the caller's requested `PlanningGroupSelection.joint_names` order.
- Keep single-robot RoboPlan behavior compatible, including direct use of a provided `RobotModelConfig.srdf_path` when only one robot is registered.

## Capabilities

### New Capabilities
- `roboplan-composite-multi-robot-planning`: Covers generated Composite RoboPlan models, Composite planning groups, multi-robot RoboPlan naming/order mappings, and coupled selected-group planning semantics.

### Modified Capabilities
- None. There are no existing repo-level OpenSpec capabilities to modify.

## Impact

- Affects `RoboPlanWorld` robot registration, scene construction/finalization, SRDF/URDF preparation, joint-order conversion, collision checks, FK/Jacobian routing, and `plan_selected_joint_path`.
- Affects tests for RoboPlan world behavior, dual-arm planning, generated SRDF/URDF content, and factory wiring.
- Uses existing RoboPlan Python APIs: `Scene`, `JointConfiguration`, group metadata, group limit queries, explicit RRT options, and full-scene current joint state.
- Adds implementation complexity around prefixing, base placement, collision-disable rewriting, and global/local/native name conversion.
