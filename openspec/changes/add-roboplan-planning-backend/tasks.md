## 1. Backend registration and dependency handling

- [x] 1.1 Add `dimos/manipulation/planning/backends/roboplan/` package skeleton with lazy RoboPlan import helpers, option parsing, and conversion placeholders.
- [x] 1.2 Register `"roboplan"` in `dimos/manipulation/planning/backends/registry.py` while preserving `"drake"` as the default backend.
- [x] 1.3 Add backend availability errors that identify missing required RoboPlan modules only when `planning_backend="roboplan"` is selected.
- [x] 1.4 Add optional TOPPRA availability handling so missing `roboplan.toppra` fails only when TOPPRA retiming is required.
- [x] 1.5 Add dependency or install documentation hooks for RoboPlan without making default Drake-backed manipulation stacks require RoboPlan at import time.

## 2. RoboPlan backend configuration and lifecycle

- [x] 2.1 Implement RoboPlan backend config validation for URDF/model path, SRDF if required, package paths, planning group or active joint names, base frame, end-effector frame, joint limits, and retiming mode.
- [x] 2.2 Implement one-time RoboPlan scene, active joint order, collision model, RRT planner, IK solver, and optional TOPPRA construction during backend finalization.
- [x] 2.3 Reject unsupported multi-robot RoboPlan configurations with a clear initialization error unless multi-robot support is fully implemented.
- [x] 2.4 Populate RoboPlan `BackendCapabilities` and `BackendDiagnostics` from runtime feature probing instead of desired feature parity.
- [x] 2.5 Ensure RoboPlan `world`, `world_monitor`, `legacy_planner`, and `legacy_kinematics` compatibility properties return `None` or equivalent non-Drake diagnostics safely.

## 3. Scene state, joint mapping, and robot queries

- [x] 3.1 Implement joint-state ingestion that maps coordinator joint names through `RobotModelConfig` into RoboPlan active joint order.
- [x] 3.2 Implement current-state, velocity, wait-for-state, and stale-state behavior for RoboPlan without relying on Drake `WorldMonitor`.
- [x] 3.3 Implement joint-limit lookup in DimOS public joint order and fail fast on incomplete limits.
- [x] 3.4 Implement RoboPlan-backed state validity and path validity checks with explicit interpolation/step-size behavior.
- [x] 3.5 Implement FK, link pose, Jacobian, and distance query support where RoboPlan exposes stable APIs, with diagnostics for unsupported queries.

## 4. Planning and trajectory normalization

- [x] 4.1 Implement `plan_to_joints()` using the latest state, DimOS-to-RoboPlan joint mapping, explicit RRT options, RoboPlan planning, path validation, and DimOS-order result normalization.
- [x] 4.2 Implement `plan_to_pose()` through RoboPlan-native pose planning or IK plus joint planning, with clear unsupported diagnostics when unavailable.
- [x] 4.3 Centralize pose conversion for RoboPlan frame IDs and quaternion ordering.
- [x] 4.4 Implement `retiming="dimos"` using the existing `JointTrajectoryGenerator` for RoboPlan paths.
- [x] 4.5 Implement `retiming="toppra"` using RoboPlan TOPPRA with explicit grid/discretization and limit options, or fail clearly when selected but unavailable.
- [x] 4.6 Ensure failed, invalid, or stale RoboPlan plans do not overwrite the current executable plan or leave stale trajectories executable.

## 5. Obstacles, perception, and diagnostics

- [x] 5.1 Implement supported primitive obstacle projection into RoboPlan collision state, starting with boxes where available.
- [x] 5.2 Implement unsupported or approximated obstacle diagnostics for spheres, cylinders, meshes, pointcloud layers, and attached objects when RoboPlan support is missing or disabled.
- [x] 5.3 Implement obstacle add, remove, update pose, clear, and list behavior with `SceneUpdateResult` statuses that distinguish applied, remove/re-add, approximated, missing, and unsupported updates.
- [x] 5.4 Preserve existing perception object cache and refresh behavior for `PickAndPlaceModule` callers when RoboPlan is active.
- [x] 5.5 Ensure path validation and planning use the active RoboPlan collision scene for applied obstacles.

## 6. Manipulation surfaces and optional demos

- [ ] 6.1 Verify existing manipulation RPC and `@skill` method names and result shapes continue to work with RoboPlan-selected stacks.
- [x] 6.2 Surface RoboPlan planning, configuration, and unsupported-feature failures through existing manipulation error or `SkillResult` surfaces.
- [ ] 6.3 Add a mock or simulated RoboPlan manipulation demo/driver only if enough assets are available for reliable manual QA.
- [x] 6.4 If new runnable blueprints are exported or renamed, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py` rather than editing it manually.

## 7. Documentation

- [x] 7.1 Update `docs/capabilities/manipulation/readme.md` to describe RoboPlan as an optional manipulation planning backend, including safety and coordinator-execution behavior.
- [x] 7.2 Update `dimos/manipulation/planning/README.md` with RoboPlan backend selection, required robot assets, `planning_backend_options`, supported/unsupported features, and troubleshooting import checks.
- [x] 7.3 Update manipulation blueprint docs if a RoboPlan demo blueprint is added; otherwise document RoboPlan as a module/blueprint configuration option.
- [x] 7.4 Update `docs/development/dimos_run.md` only if a new `dimos run` RoboPlan blueprint is exported.
- [x] 7.5 Update `docs/development/testing.md` if RoboPlan-specific markers, skips, or integration test commands are added.
- [x] 7.6 Update `docs/coding-agents/index.md` or a manipulation coding-agent guide to mention active backend Protocol usage, lazy RoboPlan imports, explicit RoboPlan options, and generated registry rules when relevant.

## 8. Verification

- [x] 8.1 Run `openspec validate add-roboplan-planning-backend`.
- [x] 8.2 Run focused backend unit tests covering registry selection, lazy import errors, config validation, joint mapping, capability diagnostics, and trajectory normalization.
- [x] 8.3 Run focused manipulation tests proving Drake/default backend behavior remains unchanged when RoboPlan is not selected.
- [ ] 8.4 Run RoboPlan-selected tests with RoboPlan installed to cover joint planning, pose planning or unsupported pose diagnostics, collision/path validation, and missing optional feature diagnostics.
- [ ] 8.5 Run `pytest dimos/robot/test_all_blueprints_generation.py` if blueprint exports or generated registry inputs changed.
- [x] 8.6 Run `uv run doclinks` after documentation updates.
- [x] 8.7 Run `uv run md-babel-py run docs/capabilities/manipulation/readme.md` if that doc gains executable examples.
- [x] 8.8 Run `uv run md-babel-py run dimos/manipulation/planning/README.md` if that doc gains executable examples.
- [x] 8.9 Run relevant lint/type checks for touched Python modules, including `uv run mypy dimos/manipulation` if type contracts or backend Protocol usage changed.

## 9. Manual QA

- [ ] 9.1 Start a Drake-backed mock or simulated manipulation stack through the normal `dimos run` or library-driver surface and confirm existing joint planning still succeeds.
- [ ] 9.2 Start a RoboPlan-backed mock or simulated manipulation stack through the documented user surface and confirm backend startup reports RoboPlan capabilities.
- [ ] 9.3 Through the existing manipulation RPC/skill surface, run a RoboPlan joint-plan request, inspect the stored path/trajectory, and execute through the coordinator-mediated path in mock/sim.
- [ ] 9.4 Through the existing manipulation RPC/skill surface, run a RoboPlan pose-plan request or confirm the documented unsupported-pose diagnostic.
- [ ] 9.5 Add a supported primitive obstacle and confirm RoboPlan planning or validation accounts for it.
- [ ] 9.6 Try one invalid target, missing state, or unsupported obstacle and confirm the user-facing failure is clear and no stale plan executes.
- [x] 9.7 Verify missing RoboPlan dependency behavior by selecting RoboPlan in an environment without required modules and confirming the failure is scoped to RoboPlan selection.
