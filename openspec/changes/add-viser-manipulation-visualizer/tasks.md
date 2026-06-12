## 1. Configuration and backend selection

- [x] 1.1 Add manipulation visualization backend config to `ManipulationModuleConfig`, including `visualization_backend`, Viser host/port/browser/panel fields, preview timing, polling rate, and `allow_plan_execute`.
- [x] 1.2 Implement backend resolution semantics: explicit backend wins; otherwise `enable_viz=True` maps to Meshcat and `enable_viz=False` maps to none.
- [x] 1.3 Add or extend a manipulation visualization factory that returns `None`, the existing Meshcat visualization path, or a lazy-imported Viser implementation.
- [x] 1.4 Add clear validation/error handling for unsupported manipulation visualization backend values.

## 2. WorldMonitor visualization integration

- [x] 2.1 Refactor `WorldMonitor` so `_visualization` is selected explicitly instead of only inferred by `isinstance(self._world, VisualizationSpec)`.
- [x] 2.2 Preserve Meshcat compatibility by keeping `DrakeWorld(enable_viz=True)` as the Meshcat visualization backend for now.
- [x] 2.3 Ensure `visualization_backend="viser"` creates the planning world with Drake Meshcat disabled and attaches the Viser `VisualizationSpec` implementation.
- [x] 2.4 Keep `get_visualization_url`, `publish_visualization`, `show_preview`, `hide_preview`, `animate_path`, and `close` safe no-ops when visualization is disabled.
- [x] 2.5 Rename generic visualization thread logging/thread names if needed so they are not Meshcat-specific once Viser is supported.

## 3. Lean in-process adapter boundary

- [x] 3.1 Add a small in-process Viser adapter over existing `ManipulationModule` and `WorldMonitor` methods for robot metadata, current joint state, stale-state checks, planned path copies, trajectory duration, and panel actions.
- [x] 3.2 Ensure the adapter avoids broad snapshot DTOs and instead copies mutable containers only at the read boundary before rendering.
- [x] 3.3 Add an operation worker path that runs plan, preview, execute, cancel, clear, and target-evaluation operations through existing manipulation methods or bounded helper methods.
- [x] 3.4 Ensure Viser GUI callbacks only enqueue backend operations and never call `WorldSpec`, kinematics, planner, or live Drake contexts directly.
- [x] 3.5 Ensure Viser rendering does not hold Viser scene/GUI locks while calling manipulation/world accessors and renders only after any needed values are locally copied.
- [x] 3.6 Ensure any FK data needed by Viser display is obtained through `WorldMonitor` helpers with bounded link lists, not through direct Viser callback access to `WorldSpec`.
- [x] 3.7 Preserve preview worker coalescing, stale-request handling, timeout behavior, and busy-state reporting from the previous Viser panel design.

## 4. Viser runtime and protocol implementation

- [x] 4.1 Add the new manipulation Viser package and lazy optional imports for `viser`, `viser.extras.ViserUrdf`, and URDF support dependencies.
- [x] 4.2 Implement `ViserManipulationVisualizer.get_visualization_url()` and runtime lifecycle startup/close behavior.
- [x] 4.3 Implement `publish_visualization(ctx=None)` using the lean adapter/current robot accessors and current robot scene updates.
- [x] 4.4 Implement `show_preview(robot_id)` and `hide_preview(robot_id)` with ghost/preview visibility semantics.
- [x] 4.5 Implement `animate_path(robot_id, path, duration)` with initial Meshcat-compatible blocking semantics unless a cancellable worker is chosen during implementation.
- [x] 4.6 Ensure `close()` stops panel workers, refresh loops, animation workers if present, scene handles, and the Viser server without requiring robot motion.

## 5. Viser scene and panel feature retention

- [x] 5.1 Port reusable Viser scene helpers from the previous `cc/viser-vis` implementation for URDF preparation, current robot rendering, ghost rendering, and target visual state.
- [x] 5.2 Preserve DimOS robot joint-name to Viser/yourdfpy actuated-joint mapping and add tests for mismatched joint orders.
- [x] 5.3 Render planned paths with Viser line segments using the correct `(N, 2, 3)` points shape.
- [x] 5.4 Port optional GUI controls for robot selection, status/error display, feasibility display, preset controls, joint sliders, plan, preview, execute, cancel, and clear-plan actions.
- [x] 5.5 Keep panel execution disabled unless `allow_plan_execute=True`, and route enabled execution through existing manipulation execution behavior.
- [x] 5.6 Preserve clear missing-dependency hints, port-conflict reporting, and browser-opening behavior from the draft panel where applicable.

## 6. Dependencies and packaging

- [x] 6.1 Add a `manipulation-viser` optional dependency extra with Viser URDF support.
- [x] 6.2 Verify existing manipulation, Meshcat, and no-visualization imports do not import Viser unless the Viser backend is selected.
- [x] 6.3 If new blueprints are added for Viser examples, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`.

## 7. Documentation

- [x] 7.1 Update user-facing visualization docs to distinguish global Rerun visualization from manipulation planning visualization.
- [x] 7.2 Document manipulation visualization backend choices: Meshcat, Viser, and none.
- [x] 7.3 Document Viser optional installation, example config/blueprint usage, and the panel execution opt-in gate.
- [x] 7.4 Add contributor guidance documenting the lean adapter boundary and the rule that Viser callbacks must not access `WorldSpec`/IK/planner objects directly.

## 8. Verification

- [x] 8.1 Run `openspec validate add-viser-manipulation-visualizer`.
- [x] 8.2 Add and run focused tests for backend resolution and `WorldMonitor` visualization delegation for Meshcat, Viser, and none.
- [x] 8.3 Add and run fake-server/fake-URDF Viser scene tests for current robot rendering, ghost visibility, line segment path shape, joint mapping, and close behavior.
- [x] 8.4 Add and run Viser panel regression tests for status refresh, preview worker coalescing, timeout handling, stale requests, planning/preview controls, execution gating, cancel, and clear-plan behavior.
- [x] 8.5 Add and run tests proving Viser refresh/publish uses the lean adapter/accessors and does not call raw `WorldSpec`, IK, planner, or live context APIs from GUI callbacks.
- [x] 8.6 Run focused manipulation tests, for example `uv run pytest dimos/manipulation -k "visualization or viser or world_monitor"` once the implementation test names exist.
- [x] 8.7 Run type/lint checks for touched manipulation packages, such as `uv run mypy dimos/manipulation` and the repo's ruff/pre-commit checks as appropriate.
- [x] 8.8 Run documentation validation commands available in the repo, including doc link checks and executable markdown checks for changed docs where supported.
- [ ] 8.9 Manually QA a mock or simulation manipulation setup with `visualization_backend="viser"`: start module, open Viser URL, observe current robot state, preview a path, replan to dismiss stale preview, verify panel execution is blocked by default, and stop cleanly.
- [ ] 8.10 Manually QA compatibility with `enable_viz=True` and no explicit backend to confirm Meshcat remains the default behavior.

## 9. Full `cc/viser-vis` control panel parity

- [x] 9.1 Restore rich panel session state from the previous panel: runtime/backend status, target status, action status, feasibility status/message, plan status, selected robot, current joints, current EE pose, cartesian target, joint target, latest sequence id, and sync source.
- [x] 9.2 Add a dedicated preview worker separate from the generic operation worker, with debounce, single-slot coalescing, target-evaluation timeout handling, busy-state reporting, and sequence-id stale-result rejection.
- [x] 9.3 Add Viser end-effector transform controls for cartesian target editing and route transform updates through adapter-based pose target evaluation.
- [x] 9.4 Preserve joint-slider target editing and route slider updates through adapter-based joint target evaluation without direct `WorldSpec`, IK, planner, or live-context access from GUI callbacks.
- [x] 9.5 Add adapter methods needed for panel parity, including pose target evaluation, planned-path pose lookup/render support, module state/error reads, and any bounded helper needed to replace old RPC calls without broad snapshot DTOs.
- [x] 9.6 Restore target synchronization between cartesian target, joint target, transform control, joint sliders, and ghost joints while preventing callback feedback loops.
- [x] 9.7 Restore target visual state feedback so feasible targets and infeasible/collision targets color the transform control and ghost robot distinctly.
- [x] 9.8 Restore preset parity: `Current`, `Init`, and `Home` when the corresponding robot data exists.
- [x] 9.9 Restore button disabled/enabled gating using `can_plan`, `can_preview`, `can_execute`, and `can_cancel` semantics rather than only reporting errors after clicks.
- [x] 9.10 Restore plan freshness tracking: target changes mark fresh plans stale, successful planning stores target, start joint snapshot, robot, and planned path, and preview/execute operate only on the fresh selected-robot plan.
- [x] 9.11 Restore safe execute gating: require `allow_plan_execute=True`, feasible target, fresh selected-robot plan, idle/completed manipulation state, and current joints within `viser_current_match_tolerance` of the plan start snapshot.
- [x] 9.12 Restore cancel and clear-plan behavior to update panel session state and Viser path/ghost visuals consistently with the old panel.
- [x] 9.13 Add/port regression tests from `cc/viser-vis` for transform target updates, pose/joint preview evaluation, stale preview result rejection, plan stale/fresh transitions, button gating, execute start-joint tolerance checks, target visual coloring, Init/Home presets, cancel, and clear-plan behavior.
- [x] 9.14 Run focused Viser panel parity tests plus existing visualization tests after the parity implementation.

## 10. Planning-target inverse operability

- [x] 10.1 Extend the manipulation `VisualizationSpec` with semantic planning-target lifecycle methods (`set_planning_target`, `clear_planning_target`) without adding GUI/panel-specific state.
- [x] 10.2 Add `WorldMonitor` passthrough methods for planning-target lifecycle updates.
- [x] 10.3 Update successful `ManipulationModule` planning to publish the final planned target joint state, optional target pose, and feasibility to visualization.
- [x] 10.4 Clear the planning target visualization when `clear_planned_path()` clears the stored plan.
- [x] 10.5 Implement the Viser backend hook by updating the persistent target ghost and pose selector while keeping the preview ghost transient.
- [x] 10.6 Add focused regressions for `WorldMonitor`, `ManipulationModule`, and Viser target synchronization, then run focused tests/lint/type validation.
