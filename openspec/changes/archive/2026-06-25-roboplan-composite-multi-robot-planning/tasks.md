## 1. Composite model scaffolding

- [x] 1.1 Add RoboPlan-world configuration fields for `max_generated_composite_groups` and selected-start tolerance.
- [x] 1.2 Split RoboPlan robot registration from RoboPlan `Scene` construction so multiple robots can be collected before finalization.
- [x] 1.3 Preserve existing single-robot behavior, including direct `RobotModelConfig.srdf_path` pass-through.
- [x] 1.4 Add an explicit world-finalization path that constructs the RoboPlan `Scene` exactly once before planning/query use.
- [x] 1.5 Add clear errors when planning or scene queries are requested before finalization.

## 2. Native name mapping

- [x] 2.1 Implement deterministic RoboPlan-native prefixing for robot-local joint, link, frame, and group names.
- [x] 2.2 Store mappings between DimOS robot-local names, DimOS global names, and RoboPlan-native names.
- [x] 2.3 Update selected-joint conversion helpers to map caller-order global/local inputs to native group order.
- [x] 2.4 Update path extraction to map native RoboPlan paths back to global `JointState` waypoints in caller selection order.
- [x] 2.5 Add duplicate-name and missing-name validation with actionable error messages.

## 3. Composite URDF generation

- [x] 3.1 Generate a synthetic world root for multi-robot RoboPlan models.
- [x] 3.2 Rewrite each registered robot model into the composite URDF using RoboPlan-native prefixed names.
- [x] 3.3 Attach each robot to the synthetic world root using `RobotModelConfig.base_pose` exactly once.
- [x] 3.4 Preserve `strip_model_world_joint` behavior to avoid double-applying model-authored world/base joints.
- [x] 3.5 Persist generated composite URDF files in the same temporary/resource lifecycle used by existing generated RoboPlan inputs.

## 4. Composite SRDF generation

- [x] 4.1 Generate single-robot SRDF groups from each registered `PlanningGroupDefinition` using native joint names.
- [x] 4.2 Generate Composite planning groups for every non-overlapping Planning-group combination of size at least two.
- [x] 4.3 Use canonical `PlanningGroupRegistry` order for Composite planning group identity and native joint order.
- [x] 4.4 Enforce `max_generated_composite_groups` and fail finalization when the cap is exceeded.
- [x] 4.5 Preserve `RobotModelConfig.collision_exclusion_pairs` as native disable-collisions entries.
- [x] 4.6 Rewrite per-robot SRDF disable-collisions entries when referenced links can be mapped unambiguously.
- [x] 4.7 Keep inter-robot collisions enabled unless an explicit future configuration disables them.

## 5. Planning world state and scene queries

- [x] 5.1 Assemble full composite RoboPlan q vectors from the Planning world's per-robot belief state.
- [x] 5.2 Set RoboPlan `Scene` current joint positions from the full Planning world q before group RRT.
- [x] 5.3 Validate selected `start` states against the Planning world's selected state using the configured tolerance.
- [x] 5.4 Update collision checks to evaluate the full composite scene while overlaying candidate selected robot/group states as needed.
- [x] 5.5 Update FK and Jacobian queries to use native prefixed frame names and reorder returned Jacobian columns by DimOS group order.
- [x] 5.6 Keep dynamic obstacle state outside generated URDF/SRDF and route obstacle changes through existing scene-state mechanisms.

## 6. RoboPlan-native planning

- [x] 6.1 Resolve `PlanningGroupSelection` sets to generated single or Composite RoboPlan group names.
- [x] 6.2 Return `PlanningStatus.UNSUPPORTED` when a selection cannot map to exactly one generated RoboPlan group.
- [x] 6.3 Build RoboPlan `JointConfiguration` start and goal values in native group order.
- [x] 6.4 Set RoboPlan RRT options explicitly, including group name, timeout, and collision-check behavior.
- [x] 6.5 Convert RoboPlan planner output from native order to caller-order global `JointState` waypoints.
- [x] 6.6 Preserve existing error/status mapping for invalid starts, invalid goals, no-solution paths, and backend exceptions.

## 7. Tests

- [x] 7.1 Add unit tests for native name prefixing and global/local/native mapping round trips.
- [x] 7.2 Add unit tests for composite URDF generation with two robots and non-identity base poses.
- [x] 7.3 Add unit tests for `strip_model_world_joint` preventing double base placement.
- [x] 7.4 Add unit tests for composite SRDF group generation, canonical ordering, and safety-cap failure.
- [x] 7.5 Add tests that collision exclusions are preserved and inter-robot collisions remain enabled by default.
- [x] 7.6 Add tests that `plan_selected_joint_path` sets full scene current q before RoboPlan RRT.
- [x] 7.7 Add tests that selected-start disagreement returns `PlanningStatus.INVALID_START` without invoking RoboPlan.
- [x] 7.8 Add tests for coupled dual-arm planning path output in caller selection order.
- [x] 7.9 Add regression tests proving existing single-robot RoboPlan behavior still passes.

## 8. Verification and documentation

- [x] 8.1 Run RoboPlan world and planning factory tests.
- [x] 8.2 Run dual xArm blueprint tests relevant to RoboPlan/Viser manual QA.
- [x] 8.3 Run ruff on modified RoboPlan, planning, and test files.
- [x] 8.4 Update developer-facing docs or manual QA notes with the supported dual-arm RoboPlan command and expected constraints.
- [x] 8.5 Verify `openspec status --change roboplan-composite-multi-robot-planning` reports all artifacts complete before implementation starts.
