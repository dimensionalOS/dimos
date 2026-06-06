## 1. Implementation

- [x] 1.1 Add optional Viser dependencies to the project dependency configuration without making existing manipulation installs require Viser.
- [x] 1.2 Extend `ManipulationModule.get_robot_info()` with panel-needed metadata: model path, base pose, package paths, xacro args, and joint limits when available.
- [x] 1.3 Add `ManipulationModule.get_planned_path(robot_name=None)` to return the current stored planned joint path through the public RPC surface.
- [x] 1.4 Add `ManipulationModule.solve_ik_preview(pose, robot_name=None)` to run the configured `KinematicsSpec` solver without storing paths, previewing, executing, or publishing commands.
- [x] 1.5 Add `ManipulationModule.solve_fk_preview(joints, robot_name=None)` to return the candidate end-effector pose and feasibility data for joint-slider targets without planning or executing.
- [x] 1.6 Add focused unit coverage for the new manipulation RPCs, including no-plan side effects for IK/FK preview and enriched `get_robot_info()` response shape.
- [x] 1.7 Create the optional Viser panel package/module under `dimos/manipulation/viser_panel/` with a dedicated worker module that hosts a `viser.ViserServer`.
- [x] 1.8 Implement panel RPC connection handling to discover a running `ManipulationModule`, show a disconnected state when unavailable, and keep plan/execution controls disabled while disconnected.
- [x] 1.9 Implement current robot rendering as a solid live-state `ViserUrdf` and target ghost rendering as a visually distinct translucent or color-coded target `ViserUrdf`.
- [x] 1.10 Implement the Viser end-effector target transform control as a separate draggable gizmo, visually distinct from both current robot and target ghost.
- [x] 1.11 Implement target preset controls for Current, Init, and Home targets, applying presets once and returning the selector to a neutral state.
- [x] 1.12 Implement always-visible synchronized Cartesian and joint controls, including IK-driven slider updates and FK-driven target-gizmo updates.
- [x] 1.13 Implement the non-blocking preview worker: debounce/coalesce IK/FK/collision preview requests, tag them with sequence IDs, and ignore stale results.
- [x] 1.14 Implement automatic feasibility display and gating so infeasible targets turn target controls/ghost red and disable Plan, Preview, and Execute.
- [x] 1.15 Implement Plan, local Preview, Execute, Cancel, and Clear Plan UI actions through public `ManipulationModule` RPCs with stale-plan gating.
- [x] 1.16 Ensure Execute remains disabled unless the plan is fresh, target is feasible, selected robot matches, and current state matches the plan start constraints.
- [x] 1.17 Add the supported launch surface: companion `python -m dimos.manipulation.viser_panel` entrypoint, optional blueprint, or both as finalized during implementation.
- [x] 1.18 If a new runnable blueprint is added, regenerate and verify the blueprint registry with `pytest dimos/robot/test_all_blueprints_generation.py`.

## 2. Documentation

- [x] 2.1 Update manipulation user docs under `docs/capabilities/manipulation/` or the canonical manipulation guide with install, launch, UI workflow, visual semantics, and safety notes.
- [x] 2.2 Document the optional Viser dependency extra and supported launch command once implementation finalizes the command or blueprint name.
- [x] 2.3 Add contributor notes only if implementation introduces reusable extension points, such as named target presets or preview RPC response schemas. No separate contributor notes were needed.
- [x] 2.4 Add coding-agent guidance only if implementation establishes a reusable Viser module pattern; otherwise explicitly leave coding-agent docs unchanged.

## 3. Verification

- [x] 3.1 Run `openspec validate add-viser-manipulation-panel`.
- [x] 3.2 Run focused tests for manipulation RPC changes, including existing manipulation unit tests and any new panel/RPC tests.
- [x] 3.3 Run focused tests or a lightweight driver for the Viser panel state model, target preset application, async preview worker, stale-result dropping, and execution gating.
- [x] 3.4 Run `pytest dimos/robot/test_all_blueprints_generation.py` if any runnable blueprint or generated blueprint input changes.
- [x] 3.5 Run type/lint checks required for the touched Python modules.
- [x] 3.6 Run documentation validation for changed docs, including link validation where available and `md-babel-py run <doc>` for any changed docs with executable Python snippets.
- [x] 3.7 Manually QA the static alignment mock or equivalent rendered panel surface to verify solid current robot, translucent target ghost, separate EEF gizmo, target presets, sliders, feasibility state, and plan controls are understandable.
- [x] 3.8 Manually QA the running panel against a mock or simulation manipulation stack: open the panel, select a robot, apply Current/Init/Home presets, drag the EEF target, move joint sliders, observe async feasibility updates, plan, preview, execute when supported, cancel, reset, and verify stale-plan gating.
- [x] 3.9 Manually QA the disconnected/faulted states: start the panel without a compatible manipulation stack and with a faulted manipulation state, then verify unavailable/fault messages and disabled planning/execution controls.
- [x] 3.10 For hardware validation, follow the project hardware safety procedure: require explicit confirmation before Execute, verify Cancel remains visible, and do not execute on real hardware until mock/sim QA passes.
