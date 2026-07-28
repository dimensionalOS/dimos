## 1. Dependencies and contracts

- [x] 1.1 Upgrade the RoboPlan manipulation dependency to a verified release exposing
  the official Cartesian path planner, refresh `uv.lock`, and add a binding-level
  import/API smoke test.
- [x] 1.2 Add immutable `CartesianDelta` and `CartesianTarget` planning models with
  world-frame defaults, validation-focused unit tests, and public exports alongside the
  existing planning models.
- [x] 1.3 Add backend-discriminated RRT and RoboPlan planner configs, including the
  direct RoboPlan `linear_cartesian` subconfig and its validated official-option fields.
- [x] 1.4 Add `ManipulationModuleConfig.planner`, retain explicit `planner_name` as a
  deprecated compatibility override, and update planner factories plus configuration
  tests to follow the existing typed-kinematics migration convention.
- [x] 1.5 Extend the internal `PlannerSpec` Protocol with
  `plan_linear_cartesian_path` using the agreed selection/start/targets/auxiliary-groups
  signature and no timeout parameter.

## 2. Planner implementations

- [x] 2.1 Implement `RRTConnectPlanner.plan_linear_cartesian_path` as an explicit
  `PlanningStatus.UNSUPPORTED` result and cover it with a focused protocol behavior
  test.
- [x] 2.2 Implement RoboPlan request validation for non-empty targets, exact disjoint
  target-plus-auxiliary coverage, target group TCP availability, normalized global
  start state, and authoritative-scene start matching.
- [x] 2.3 Resolve absolute world poses and world-frame `CartesianDelta` values from the
  explicit start state, including mixed target kinds and
  `R_target = R_delta @ R_start`; return `UNSUPPORTED` for other frames.
- [x] 2.4 Map the typed linear Cartesian config to official RoboPlan options, force
  bounded-speed mode, construct one official path per target group, and invoke the
  official planner once for the full synchronized selection.
- [x] 2.5 Convert the official timed trajectory into selection-ordered, globally named
  combined `JointState` waypoints with positions and velocities plus
  `PlanningResult.timestamps`; omit accelerations rather than writing them to effort.
- [x] 2.6 Post-validate every combined waypoint and synchronized interpolated edge
  sample atomically in a scratch world context, returning `NO_SOLUTION` without
  free-space fallback on collision.
- [x] 2.7 Add fake-binding unit coverage for fixed orientation, rotating orientation,
  mixed absolute/relative multi-group targets, shorter-track hold timing, auxiliary
  groups, invalid requests, official planner failures, output conversion, and
  synchronized collision rejection.
- [x] 2.8 Add a real-RoboPlan integration test that plans a collision-free
  fixed-orientation linear move and, where the existing multi-robot fixture permits,
  verifies a synchronized multi-arm request.

## 3. Documentation

- [x] 3.1 Update `docs/capabilities/manipulation/index.md` with the typed `planner`
  configuration, RoboPlan `linear_cartesian` options, `planner_name` deprecation, and
  the explicit statement that no public linear-motion API is introduced yet.
- [x] 3.2 Keep code docstrings and type documentation aligned with the OpenSpec
  terminology for linear Cartesian paths, Cartesian targets, and world-frame deltas.

## 4. Verification

- [x] 4.1 Run `openspec validate add-linear-cartesian-planning`.
- [x] 4.2 Run focused model, config, factory, RRT, and RoboPlan tests with
  `uv run pytest dimos/manipulation/planning/spec/test_models.py
  dimos/manipulation/test_planning_factory.py
  dimos/manipulation/planning/planners/test_rrt_planner_selection.py
  dimos/manipulation/test_roboplan_world.py`.
- [x] 4.3 Run the real-binding integration test in an environment with the upgraded
  RoboPlan wheel installed and record any required test marker separately from the
  default unit suite.
- [x] 4.4 Run `uv run mypy dimos/manipulation` and the repository's configured formatting
  and lint checks for all changed Python files.
- [x] 4.5 Run `uv lock --check`, then verify the resolved RoboPlan version and supported
  platform wheels in the lockfile.
- [x] 4.6 Run `uv run doclinks docs/capabilities/manipulation/index.md` and, if executable
  examples changed, `uv run md-babel-py run
  docs/capabilities/manipulation/index.md`.
- [x] 4.7 Manually exercise the internal library-driver surface against a RoboPlan-backed
  test scene: plan a world-relative fixed-orientation move, inspect global joint names
  and timestamps, then add a blocking obstacle and confirm the same request returns
  `NO_SOLUTION`.
