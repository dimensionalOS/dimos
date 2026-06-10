## 1. Milestone 1: Planning backend abstraction with full Drake support

- [x] 1.1 Create `dimos/manipulation/planning/backends/` with backend-facing protocols/models for active backend lifecycle, scene facade, planner facade, capabilities, diagnostics, planning results, IK results, and scene update results.
- [x] 1.2 Add startup-time backend selection config to manipulation planning with Drake as the default and no runtime backend-switching API.
- [x] 1.3 Implement a `DrakePlanningBackend` compatibility layer that wraps the current `WorldMonitor`, `DrakeWorld`, RRT planner, Jacobian IK, Drake optimization IK, and trajectory generation surfaces.
- [x] 1.4 Refactor `ManipulationModule` initialization to construct one active backend, register robots once, finalize/prepare the scene once, and keep existing public RPCs and skills unchanged.
- [x] 1.5 Route joint-state sync through the active backend scene while preserving current joint-name mapping, init-joint capture, stale-state behavior, and multi-robot splitting.
- [x] 1.6 Route `plan_to_joints`, `plan_to_pose`, stored paths, trajectory generation, and execute through backend facades while preserving the current Drake-backed behavior and coordinator execution path.
- [x] 1.7 Route current scene APIs through the active backend: add/remove/update/clear/list obstacles, floor obstacle startup behavior, collision checks, path validation, minimum distance, current joint state, end-effector pose, arbitrary link pose, and Jacobian access.
- [x] 1.8 Refactor perception obstacle integration to target the scene facade while preserving current semantics: ObjectDB objects are cached by stable ID, `refresh_obstacles()` adds box obstacles by default, optional convex-hull mesh obstacles remain pointcloud-derived only when enabled, and clear/list/status methods keep the same observable behavior.
- [x] 1.9 Preserve Drake-specific behavior behind the Drake backend for scratch contexts, Drake optimization IK, dynamic-obstacle caveats, and TF extra-link publishing.
- [x] 1.10 Add backend capability and diagnostic reporting for unsupported, approximated, failed, or backend-specific scene/planning operations without changing existing successful Drake user flows.
- [x] 1.11 Add focused tests proving the default backend is Drake, existing Drake planning paths still work through the new abstraction, and current Drake-only features remain reachable after the refactor.
- [x] 1.12 Manually QA milestone 1 by running a Drake-backed mock/sim manipulation surface such as `dimos run xarm7-planner-coordinator` with the interactive manipulation client to plan, inspect the stored path, add/remove an obstacle, query state/pose, and execute through the coordinator path.

## 2. Milestone 2: MPlib integration on the stable backend surface

- [ ] 2.1 Add MPlib robot configuration fields and validation for URDF, SRDF, package path, move group, link ordering, joint ordering, end-effector link, collision resolution, and pointcloud policy.
- [ ] 2.2 Add MPlib to the manipulation dependency surface and implement import/availability errors that only trigger when MPlib is selected.
- [ ] 2.3 Implement `MPlibPlanningBackend` startup so it validates robot config, constructs native MPlib planner objects once per active robot/group, and reuses them across plans.
- [ ] 2.4 Implement MPlib joint-state mapping so DimOS start/goal/path values convert between DimOS robot joint order and MPlib `user_joint_names` order without losing names.
- [ ] 2.5 Implement joint-space planning through native MPlib joint planning and normalize success, timeout, invalid start/goal, collision, and no-solution results into DimOS planning results.
- [ ] 2.6 Implement pose planning through native MPlib pose planning when available, including pose/frame/quaternion conversion and clear unsupported diagnostics when unavailable.
- [ ] 2.7 Implement MPlib scene projection for all safely representable native features: current box obstacles at minimum, other primitive/mesh shapes when available, explicit diagnostics for unsupported shapes, and current perception object-to-box/default semantics.
- [ ] 2.8 Implement optional raw pointcloud collision projection through native MPlib pointcloud APIs when enabled, frame-compatible, and available; report disabled, frame-mismatched, missing-data, and unavailable-API cases.
- [ ] 2.9 Implement attached-object attach/detach behavior through native MPlib APIs when available, with explicit unsupported diagnostics otherwise.
- [ ] 2.10 Implement MPlib-native FK, IK, Jacobian, collision, distance, and path-validation queries wherever the installed MPlib API exposes them; report true gaps through backend capabilities.
- [ ] 2.11 Normalize MPlib timing/path outputs into stored DimOS paths and executable `JointTrajectory` data, preserving native timing where compatible and falling back to existing trajectory generation only by explicit policy.
- [ ] 2.12 Add xArm6/xArm7 MPlib config defaults and optional MPlib mock/sim blueprint or documented config path; regenerate blueprint registry only if new blueprint exports are added.
- [ ] 2.13 Add focused unit tests with fake MPlib modules for config validation, missing dependency errors, one-time planner construction, joint-order mapping, joint planning, pose planning, scene projection, pointcloud policy, attached-object diagnostics, and result normalization.
- [ ] 2.14 Manually QA milestone 2 through an MPlib-selected manipulation surface by starting the stack, planning to joints, planning to a pose, adding an obstacle, refreshing perception-style box obstacles, exercising pointcloud policy diagnostics, and confirming execution still routes through the existing coordinator trajectory task.

## 3. Documentation

- [ ] 3.1 Update `docs/capabilities/manipulation/readme.md` to describe backend-selectable planning, default Drake behavior, optional MPlib behavior, and unchanged coordinator execution.
- [ ] 3.2 Add or update manipulation planning docs for MPlib robot assets, joint/link ordering, move group, package paths, end-effector link, collision resolution, pointcloud policy, attached-object support, and troubleshooting.
- [ ] 3.3 Document current perception obstacle behavior accurately: objects become box obstacles by default, optional convex-hull mesh obstacles are pointcloud-derived only when enabled, and raw MPlib pointcloud collision is an explicit backend option.
- [ ] 3.4 Update blueprint docs only if implementation adds or renames user-runnable MPlib demo blueprints; otherwise document backend selection on existing manipulation/xArm planner blueprints.
- [ ] 3.5 Update `docs/coding-agents/` only if implementation introduces new agent-facing backend conventions; update `AGENTS.md` only if the rule is repo-wide.

## 4. Verification

- [ ] 4.1 Run `openspec validate new-planning-interface-mplib-integration`.
- [ ] 4.2 Run focused manipulation unit tests, including `pytest dimos/manipulation/test_manipulation_unit.py -v` or updated equivalent targets for the backend abstraction and MPlib adapter.
- [ ] 4.3 Run Drake-backed integration or e2e tests relevant to current manipulation planning, such as `pytest dimos/e2e_tests/test_manipulation_module.py -v` when the environment supports Drake.
- [ ] 4.4 Run `pytest dimos/robot/test_all_blueprints_generation.py` if any blueprint exports or generated registry inputs change.
- [ ] 4.5 Run type/lint checks required by the touched area, including `uv run mypy dimos/manipulation` if backend protocols or typed config models change.
- [ ] 4.6 Run docs validation for changed docs: `md-babel-py run docs/capabilities/manipulation/readme.md` when examples change, plus the repo doc-link checker if available.
- [ ] 4.7 Manually QA milestone 1 before starting milestone 2: use the Drake-backed planner through its CLI/client surface and confirm all current Drake features still work after the abstraction refactor.
- [ ] 4.8 Manually QA milestone 2 after MPlib integration: use the MPlib-selected planner through the same user surface and confirm joint planning, pose planning, scene diagnostics, and coordinator execution behavior.
