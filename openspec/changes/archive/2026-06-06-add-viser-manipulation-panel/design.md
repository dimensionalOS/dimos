## Context

The manipulation stack already exposes the core operator loop through `ManipulationModule`: robot discovery, robot state, end-effector pose, collision checking, planning to pose/joints, path preview, execution, cancel, reset, trajectory status, and obstacle add/remove RPCs. Existing runnable manipulation blueprints wire `ManipulationModule` to `ControlCoordinator` and route `joint_state: In[JointState]` over LCM, for example `xarm7_planner_coordinator` uses `/coordinator/joint_state`.

The current operator experience is fragmented: Meshcat/Drake handles backend visualization, Rerun handles general stream visualization, and `dimos.manipulation.planning.examples.manipulation_client` provides a Python shell workflow for planning and execution. There is no Viser dependency or Viser UI module in the repo today. Viser can provide the missing browser-native operator surface with APIs that exist in Viser today: `ViserServer`, `server.gui.add_button`, `add_slider`, `add_dropdown`, `add_checkbox`, tab/folder grouping, `server.scene.add_transform_controls`, scene frames/lines/meshes, and mutable scene handle color/visibility.

The requested scope is phase 1 and phase 2 only: a read-only live viewer and a basic planning panel. Perception-driven pick/place, rich obstacle editing, camera feeds, point clouds, full MoveIt RViz parity, and planner benchmarking are out of scope.

## Goals / Non-Goals

**Goals:**

- Provide a Viser browser panel that can inspect a running manipulation stack without replacing Meshcat or Rerun.
- Render the configured manipulator robot from its model file and update it from live joint state.
- Show current module state, error text, end-effector pose, trajectory task status, and selected robot metadata.
- Provide explicit Viser-supported UI controls for the basic planning workflow: choose robot, move the end-effector target control or joint sliders, see synchronized target state, plan, preview, execute, cancel, reset.
- Keep all robot-facing actions routed through `ManipulationModule` RPCs so the existing planning and coordinator safety behavior remains authoritative.
- Add only small read-only or compatibility-preserving RPCs where Viser needs data not currently exposed, such as model paths, joint limits, and planned path samples.

**Non-Goals:**

- Do not implement pick/place, object scanning, grasp candidate visualization, perception obstacle refresh, or camera/point-cloud overlays in this change.
- Do not replace DrakeWorld/Meshcat visualization or the Rerun bridge.
- Do not add new MCP tools or change agent skills.
- Do not introduce custom planner algorithms, trajectory generators, or benchmarking UI.
- Do not make Viser a required dependency for all manipulation users; it should remain optional.

## DimOS Architecture

### New module and launch surfaces

Add a dedicated optional module, tentatively `ViserManipulationPanelModule`, under `dimos/manipulation/viser_panel/`. It should be a `Module` with `dedicated_worker = True` because it hosts a web server and long-running polling/UI callbacks.

The module should have no required typed stream inputs or outputs in phase 1/2. It communicates with the manipulation stack by RPC:

```text
ViserManipulationPanelModule
  In streams:  none required
  Out streams: none required
  RPC clients:
    RPCClient.remote(ManipulationModule)
  Hosts:
    viser.ViserServer(host, port)
```

Configuration fields should include:

- `host: str = "127.0.0.1"`
- `port: int = 0` or a deterministic default if the project has an available Viser port convention
- `poll_hz: float = 5.0` for state refresh
- `preview_duration: float = 3.0`
- `open_browser: bool = False`
- `default_robot: str | None = None`
- `hardware_confirm_required: bool = True`

Expose two launch paths:

1. A companion Python entrypoint for developers:
   - `python -m dimos.manipulation.viser_panel`
   - This builds only the panel module and connects to an already-running manipulation stack through RPC.
2. Optional blueprint composition for convenience:
   - `viser_manipulation_panel = ViserManipulationPanelModule.blueprint(...)`
   - Optional combined blueprints can be added later, such as `xarm7_planner_coordinator_viser`, if useful.

If new runnable blueprint variables are added, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`.

### Existing RPCs used by the panel

The panel should call these existing `ManipulationModule` RPCs:

| Panel need | RPC |
|---|---|
| Discover robot list | `list_robots()` |
| Load robot metadata | `get_robot_info(robot_name)` |
| Read module state | `get_state()` |
| Read last error | `get_error()` |
| Read current joints | `get_current_joints(robot_name)` |
| Read current EE pose | `get_ee_pose(robot_name)` |
| Check target joints | `is_collision_free(joints, robot_name)` |
| Plan Cartesian target | `plan_to_pose(pose, robot_name)` |
| Plan joint target | `plan_to_joints(joint_state, robot_name)` |
| Read stored plan for preview | `get_planned_path(robot_name)` |
| Check if a plan exists | `has_planned_path()` |
| Clear current plan | `clear_planned_path()` |
| Execute stored plan | `execute(robot_name)` |
| Read execution status | `get_trajectory_status(robot_name)` |
| Cancel motion | `cancel()` |
| Reset fault | `reset()` |

The panel should not call private attributes or `world_monitor` directly. That keeps the panel usable both as a separate process and as an optional module in a larger blueprint.

### Additive RPCs needed for Viser-specific rendering

Phase 1/2 needs data not fully exposed by the current RPC surface. Add compatibility-preserving RPCs to `ManipulationModule` rather than letting the panel inspect private fields:

1. Extend existing `get_robot_info(robot_name: str | None = None) -> dict[str, Any] | None`
   - Do not add a separate `get_robot_model_info` RPC. The existing `get_robot_info` already returns `name`, `world_robot_id`, `joint_names`, `end_effector_link`, `base_link`, velocity/acceleration limits, coordinator task name, home joints, and init joints. Extend that same dictionary with Viser/planning metadata: `model_path`, `base_pose`, `joint_limits` if configured or discoverable, `package_paths`, and `xacro_args` if needed for URDF loading.
   - Purpose: one robot-info call should initialize `ViserUrdf`, build joint sliders, set limits, place the base frame, and avoid splitting robot metadata across two RPC names.

2. `get_planned_path(robot_name: str | None = None) -> list[JointState] | None`
   - Returns the currently stored planned joint path for preview rendering.
   - Purpose: draw Viser path ghosts/scrubber without relying on backend-native preview.

3. `solve_ik_preview(pose: Pose, robot_name: str | None = None) -> dict[str, Any]`
   - Computes IK for the candidate end-effector pose without planning, storing, previewing, or executing a path.
   - Internally this belongs in `ManipulationModule` next to `plan_to_pose()`, because `ManipulationModule` already owns the configured IK solver as `self._kinematics`. That solver is created by `create_kinematics(config.kinematics_name)`, so it uses `JacobianIK` by default or `DrakeOptimizationIK` when `kinematics_name="drake_optimization"` is configured. `solve_ik_preview()` should call the same `KinematicsSpec.solve(...)` path as `plan_to_pose()`: `self._kinematics.solve(world=self._world_monitor.world, robot_id=robot_id, target_pose=target_pose, seed=current_or_preview_seed, check_collision=True)`.
   - The preview RPC should return `success`, `joint_state`, `status`, `position_error`, `orientation_error` if available, and a collision/feasibility flag.
   - Purpose: support real-time IK as the operator moves the end-effector control. The panel calls this at a throttled UI rate, updates joint sliders from the returned joint state on success, and marks the target red on IK/collision failure.

4. `solve_fk_preview(joints: JointState, robot_name: str | None = None) -> dict[str, Any]`
   - Computes FK for candidate joint-slider values without planning or executing.
   - Purpose: support bidirectional synchronization. When the user moves a joint slider, the panel updates the end-effector target control from FK and runs collision checking automatically.

5. Optional: `get_panel_state(robot_name: str | None = None) -> dict[str, Any]`
   - Bundles state, error, current joints, EE pose, trajectory status, and `has_planned_path` into one call.
   - Purpose: reduce UI polling fan-out. This can be deferred if individual RPC polling is simpler.

These RPCs are additive and should not change existing skill/MCP behavior.

### UI layout and exact control wiring

Use one Viser server with a fixed/collapsible control panel and a 3D scene.

Viser UI callbacks must not call blocking IK/FK/planning RPCs directly. `on_update` handlers should only update local target state, enqueue a preview request, and return. A small background worker inside `ViserManipulationPanelModule` should process the latest queued preview request and publish the result back to Viser handles. Requirements:

- Debounce high-frequency end-effector-control and slider updates, e.g. target 10-20 Hz preview requests instead of one RPC per browser event.
- Coalesce stale requests: keep only the newest `(sequence_id, source, target)` per selected robot and drop older pending work.
- Apply results only if their `sequence_id` still matches the latest local target; late IK/FK responses must not overwrite newer UI state.
- Keep `Plan`, `Preview`, and `Execute` disabled while feasibility for the latest target is unknown.
- Long operations (`Plan`, `Preview`, `Execute`) should also run through the same non-UI worker pattern so the Viser event loop remains responsive.

#### Scene nodes

- `/world/grid`: static grid with +Z up.
- `/robots/<robot_name>/current`: `ViserUrdf` root for the selected robot's live/current state. Render it as the primary solid robot: normal visual mesh opacity/color, current joint values from `get_current_joints`, and a small labeled frame at `/robots/<robot_name>/current/ee` for the current EE pose.
- `/targets/<robot_name>/ee_control`: Viser `add_transform_controls` handle for the target end-effector pose. This is the draggable 6-DOF gizmo only; it is not the robot ghost. Render it with a compact axes/handle scale so it reads as the target pose control, not as another robot.
- `/targets/<robot_name>/ghost`: optional second `ViserUrdf` instance rendered at the current target joint values. Render it visually distinct from the current robot: translucent/wireframe if Viser URDF material controls allow it, otherwise use lighter ghost mesh colors and disable heavy visual detail. Color-code feasibility: blue/green for feasible, red for IK/collision failure, gray while feasibility is unknown.
- `/plans/<robot_name>/path`: line/frames/ghost robot samples for the currently planned path returned by `get_planned_path`.

The current robot, target gizmo, and target ghost must be visually separate concepts:

- **Current robot:** solid, authoritative live state, never colored red for target infeasibility.
- **Target gizmo:** small 6-DOF pose handle the user drags; color follows target feasibility.
- **Target ghost robot:** translucent target configuration computed from IK or joint sliders; color follows target feasibility.

#### Header/status controls

- **Robot dropdown**
  - Created from `list_robots()`.
  - `on_update`: set `PanelSession.selected_robot`, call `get_robot_info`, rebuild URDF/sliders if model changed, call `clear_planned_path()` only if switching robot and local plan state belongs to another robot, disable `Preview`/`Execute`, and refresh state.

- **Refresh button**
  - Calls `list_robots`, `get_robot_info`, `get_current_joints`, `get_ee_pose`, `get_state`, `get_error`, and `get_trajectory_status`.
  - Updates all read-only labels and scene handles.

- **State label / error label**
  - Populated from `get_state()` and `get_error()` on each poll.
  - If state is `FAULT`, show the error text and keep only safe controls enabled: `Refresh`, `Cancel`, `Reset Fault`, target editing.

#### Viewer tab controls

- **Target Preset dropdown**
  - Purpose: give the operator clear one-shot target initializers instead of an always-on “follow current pose” mode.
  - Implemented with Viser `server.gui.add_dropdown`, populated after `get_robot_info(selected_robot)` and state refresh.
  - Required options:
    - `Current`: calls `get_ee_pose(selected_robot)` and `get_current_joints(selected_robot)`, then sets the target gizmo, target ghost, and joint sliders to the live robot state.
    - `Init`: uses `get_robot_info().init_joints` when available, calls `solve_fk_preview`, then updates target gizmo/ghost/sliders.
    - `Home`: uses `get_robot_info().home_joints` when available, calls `solve_fk_preview`, then updates target gizmo/ghost/sliders.
  - Optional options can be added from robot-specific named presets in `RobotModelConfig` if that field is introduced later.
  - `on_update`: applies the selected preset once, runs automatic feasibility checking, invalidates any existing plan, and sets the dropdown value back to a neutral `Select preset...` item so it does not imply ongoing tracking.
  - If a preset is unavailable, show it disabled or omit it; do not create a clickable preset that fails at runtime.

- **Show Robot Visual checkbox**
  - Toggles `ViserUrdf.show_visual` or root visibility.

- **Show Collision Visual checkbox**
  - Toggles `ViserUrdf.show_collision` if collision meshes are loaded and available.

#### Target tab controls

There is no target mode switch. Cartesian and joint controls are both always visible because Viser supports both primitives directly: `server.scene.add_transform_controls` for the end-effector target and `server.gui.add_slider` for joints. Editing either one automatically updates the other.

- **End-effector target control**
  - Implemented with Viser `server.scene.add_transform_controls("/targets/<robot>/ee_target", ...)`.
  - This is needed because Viser has no normal React-style form component for dragging a 6-DOF pose in the 3D scene. The transform-control handle is the supported Viser interaction primitive for Cartesian pose editing: the operator drags/translates/rotates the desired EEF pose in the scene instead of typing all six pose values manually.
  - `on_update`: copy handle position/quaternion into read-only numeric labels, enqueue an async `solve_ik_preview(pose, selected_robot)` request, and mark the local plan stale. The callback must not wait for IK.
  - On IK success: update all joint sliders from returned `joint_state`, update `/targets/<robot>/target_robot`, call/consume automatic feasibility state, and color the target visuals normal/blue.
  - On IK or collision failure: do not update joint sliders from a failed result; keep the previous feasible joint target, set feasibility status to `Infeasible`, disable `Plan`/`Preview`/`Execute`, and color the target visuals red.
  - Programmatic updates from joint sliders must set a local `sync_source = "joints"` guard so the transform-control callback does not recurse.

- **Joint sliders**
  - Created in `get_robot_info().joint_names` order with limits from `get_robot_info().joint_limits` when available.
  - Initial values come from `get_current_joints` or init/home fallback if current state is unavailable.
  - `on_update`: update local joint target list, enqueue async `solve_fk_preview(joint_target, selected_robot)` plus feasibility checking, mark the local plan stale, and disable `Preview`/`Execute`. The callback must not wait for FK or collision checks.
  - On FK success: update the end-effector target control pose from returned FK and refresh numeric labels.
  - On collision failure: keep the sliders at the operator-selected values, set feasibility status to `In collision / invalid`, disable `Plan`/`Preview`/`Execute`, and color the target robot red.
  - Programmatic updates from IK must set a local `sync_source = "cartesian"` guard so slider callbacks do not recurse.

- **Apply Preset button**
  - Optional if the dropdown should not apply immediately on selection.
  - Runs the same preset application path described above. Prefer immediate dropdown application if it feels reliable in Viser; prefer this explicit button if users need to inspect the preset name before applying it.

- **Feasibility status readout**
  - No manual collision-check button is needed.
  - Whenever the operator moves the end-effector target control or a joint slider, the panel automatically runs IK/FK synchronization and collision feasibility.
  - Feasible targets are shown with normal/blue target visuals and enabled planning controls. Infeasible targets are shown red with planning controls disabled.

#### Planning/action buttons

- **Plan button**
  - Disabled while a plan/execution RPC is in flight or when module state is not `IDLE`/`COMPLETED`.
  - Uses the last feasible synchronized target. Prefer `plan_to_joints(joint_state, selected_robot)` because real-time IK has already converted the end-effector control into a joint target; this avoids solving IK twice with possibly different seeds. If no feasible joint target exists but a Cartesian target exists, fall back to `plan_to_pose(pose, selected_robot)`.
  - On success: store local `PanelPlanState(robot, target_pose, target_joints, start_joints_snapshot, planned_at_time)`, call `get_planned_path`, render Viser preview, enable `Preview`, enable `Execute` if current joints still match `start_joints_snapshot` within tolerance.
  - On failure: call `get_error`, show failure message, keep `Preview`/`Execute` disabled.

- **Preview button**
  - Re-renders the local Viser path from `get_planned_path` when available.
  - Does not execute motion.

- **Execute button**
  - Enabled only when local plan state is fresh, selected robot matches the plan, and current joints match the plan's stored start snapshot within tolerance.
  - If `hardware_confirm_required` is true, opens a confirmation modal showing robot, target pose/joints summary, and latest module state.
  - On confirmation, calls `execute(selected_robot)` and starts polling `get_trajectory_status` at a short interval until idle/completed/fault/timeout.

- **Plan & Execute button**
  - Runs the same handler as `Plan`; if planning succeeds and safety gates pass, opens the same execution confirmation modal before calling `execute`.
  - This button should be disabled by default for real hardware unless explicitly enabled in configuration.

- **Cancel button**
  - Always visible.
  - Calls `cancel()` immediately.
  - Clears local in-flight operation state and refreshes `get_state`/`get_trajectory_status`.

- **Reset Fault button**
  - Calls `reset()`.
  - On success, clears local error display, refreshes module state, and leaves any target unchanged but invalidates the plan.

- **Clear Plan button**
  - Calls `clear_planned_path()`.
  - Removes `/plans/<robot>` scene nodes and disables `Preview`/`Execute`.

### User workflow

Typical motion-planning workflow:

1. Start a manipulation stack, e.g. `dimos run xarm7-planner-coordinator` or the mock/sim equivalent.
2. Start the Viser panel via the companion entrypoint or optional panel blueprint.
3. Open the Viser URL in a browser.
4. Select the robot from the robot dropdown.
5. Confirm the status panel shows live joints, EE pose, and `IDLE` or `COMPLETED` state.
6. Choose a target preset such as `Current`, `Init`, or `Home` to initialize the target gizmo, target ghost, and joint sliders.
7. Refine the target with either control surface:
   - Drag/rotate the Viser end-effector target control in the 3D scene. The panel computes IK in real time through `ManipulationModule.solve_ik_preview`, updates the joint sliders automatically, and marks infeasible targets red.
   - Move joint sliders. The panel computes FK through `ManipulationModule.solve_fk_preview`, updates the end-effector target control automatically, runs collision checking, and marks infeasible targets red.
8. Click `Plan`.
9. Inspect the result: status message, rendered path/ghost preview, optional Meshcat preview via `Preview`.
10. Click `Execute` if the plan is fresh and the target is correct.
11. Confirm execution in the modal when running against hardware.
12. Watch trajectory status until complete. Use `Cancel` if needed.
13. If planning fails or the module enters `FAULT`, read the error, adjust target or click `Reset Fault`, then retry.

### Internal state model

The panel should maintain local UI/session state independent of `ManipulationModule` internals:

```text
PanelSession
  selected_robot: str | None
  robot_info: dict | None
  current_joints: list[float] | None
  current_ee_pose: Pose | None
  cartesian_target: Pose | None
  joint_target: list[float] | None
  feasibility:
    status: unknown | feasible | ik_failed | collision | invalid
    message: str
    sequence_id: int
  preview_queue:
    latest_sequence_id: int
    worker_busy: bool
  sync_source: None | cartesian | joints
  in_flight_operation: None | Refresh | Plan | Preview | Execute | Cancel | Reset
  plan_state:
    status: none | fresh | stale | executing | failed
    robot: str | None
    target_pose: Pose | None
    target_joints: list[float] | None
    start_joints_snapshot: list[float] | None
    planned_path: list[JointState] | None
```

Plan freshness is local and conservative. Any target edit, robot change, reset, clear-plan, failed RPC, current-joint mismatch, or infeasible target state invalidates `Execute`.

## Decisions

1. **Implement as an optional DimOS companion module, not inside `ManipulationModule`.**
   - Rationale: Viser is an operator UI dependency and should not affect core manipulation startup or planning.
   - Alternative rejected: embedding Viser into `ManipulationModule`, which couples robot planning lifecycle to browser/server behavior.

2. **Use RPC rather than direct module references or streams for phase 1/2.**
   - Rationale: `RPCClient.remote(ManipulationModule)` works as a separate process and avoids requiring blueprint coupling.
   - Alternative rejected: subscribing directly to `joint_state`, because robot selection, EE pose, planning state, and trajectory status still require manipulation/coordinator semantics.

3. **Extend `get_robot_info` instead of adding `get_robot_model_info`.**
   - Rationale: Viser needs more robot metadata, but the existing robot-info RPC is already the natural owner for model path, base pose, package paths, joint limits, and xacro args. One enriched robot-info shape is simpler for clients than two overlapping metadata calls.
   - Alternative rejected: adding `get_robot_model_info`, which would split robot metadata and force clients to reconcile duplicate fields.

4. **Add small preview RPCs for path rendering and UI synchronization.**
   - Rationale: Viser requires planned path samples and real-time IK/FK synchronization data that are not safely exposed today.
   - Alternative rejected: reading `_robots`, `_planned_paths`, or `world_monitor` internals from the panel.

5. **Run real-time IK in `ManipulationModule`, not in Viser.**
   - Rationale: IK must use the same `KinematicsSpec`, `WorldSpec`, current joint seed, and collision context as `plan_to_pose()`. The panel is only the UI; it should request `solve_ik_preview()` and render the result.
   - Alternative rejected: loading a separate Viser-side IK solver, which risks mismatched limits, seeds, collision state, and backend-specific behavior.

6. **Do not use a target-mode switch.**
   - Rationale: Viser supports both 3D transform controls and GUI sliders. Keeping both visible lets operators choose whichever is natural and keeps Cartesian/joint state synchronized continuously.
   - Alternative rejected: a `Cartesian`/`Joints` dropdown, which hides useful state and adds unnecessary workflow branching.

7. **Run collision/feasibility checks automatically on every target change.**
   - Rationale: The operator should not need to click a separate collision-check button after every drag/slider edit. The panel can throttle checks and mark infeasible targets red immediately.
   - Alternative rejected: a manual `Collision Check` button.

8. **Keep execution safety server-authoritative.**
   - Rationale: The panel can disable buttons and warn users, but `ManipulationModule` and `ControlCoordinator` must remain the source of truth for planning/execution acceptance.
   - Alternative rejected: bypassing `execute()` by sending trajectories directly to coordinator tasks from the panel.

9. **Defer perception/pick/place controls.**
   - Rationale: Object detection snapshots, grasp generation, and place workflows introduce additional stale-state risks outside the phase 1/2 scope.

## Safety / Simulation / Replay

- Hardware execution must be treated as safety-sensitive. `Execute` and `Plan & Execute` require a fresh local plan and hardware confirmation when configured.
- `Cancel` must remain visible and callable regardless of selected tab or in-flight operation.
- The panel must never send raw joint commands directly to hardware; it only calls `ManipulationModule` planning/execution RPCs.
- Real-time IK preview is not execution. `solve_ik_preview()` must not store paths, publish commands, or move hardware; it only returns candidate joints and feasibility for UI synchronization.
- The panel should handle missing hardware/coordinator configuration by showing execution unavailable rather than failing startup.
- Simulation and mock stacks should use the same UI workflow with confirmation optionally disabled.
- Replay mode should support read-only viewing if manipulation state is available, but execution controls must remain disabled unless a live manipulation/coordinator stack accepts execution RPCs.
- Manual QA should cover mock/sim before real hardware: launch a manipulation blueprint, open panel, verify live updates, plan Cartesian and joint targets, preview, execute, cancel/reset, and verify stale-plan gating after target edits.

## Risks / Trade-offs

- **Stale internal plan risk:** `execute()` runs the module's currently stored plan, which can be changed by another client. Mitigation: local start-state comparison and optional future `plan_id`/`execute_plan(plan_id)` RPC if multi-client use becomes common.
- **Real-time IK load:** Solving IK on every drag event can overload the backend or make the UI lag. Mitigation: throttle/debounce `solve_ik_preview()` calls, cancel stale UI requests locally, and only render the latest response.
- **IK seed drift:** Repeated Cartesian edits can produce discontinuous joint solutions if the seed changes unexpectedly. Mitigation: seed preview IK from the current synchronized `joint_target` when available, falling back to live current joints.
- **Joint order mismatch:** Viser sliders and URDF updates must use `joint_names` from robot info in planner order. Mitigation: only build targets from `get_robot_info().joint_names` and keep names with every `JointState`.
- **Missing model resources:** Viser URDF loading may fail if package paths or meshes are unavailable. Mitigation: show clear model-load errors and keep status/RPC controls available.
- **Blocking RPC callbacks:** Planning and preview can block. Mitigation: run long RPCs in a worker thread or background task queue and keep the Viser event loop responsive.
- **Dependency weight:** Viser and URDF extras add optional dependencies. Mitigation: add them to an optional extra and keep existing manipulation installs working without the panel.
- **Dual visualization confusion:** Multiple preview concepts can confuse operators. Mitigation: label Viser preview as the operator panel preview and keep backend-native preview out of the planning backend abstraction.

## Migration / Rollout

- Add the Viser panel behind optional dependencies, likely under a manipulation/web extra, so existing manipulation users are unaffected.
- Add the new panel package and module under `dimos/manipulation/viser_panel/`.
- Extend `ManipulationModule.get_robot_info` with model/Viser metadata and add preview RPCs for planned path, IK, and FK synchronization.
- Add a companion entrypoint documented as the first supported launch surface.
- Add optional blueprint(s) only if they improve discoverability; regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py` if any runnable blueprint is added.
- Document the phase 1/2 workflow in manipulation capability docs and note that pick/place/perception controls are intentionally out of scope.
- Rollback is straightforward: remove the optional panel launch path and dependency without changing core manipulation planning/execution behavior.

## Open Questions

- Which default Viser port should DimOS reserve, if any, and should it be added to `GlobalConfig`?
- Should the first implementation add only the companion `python -m` entrypoint, or also a named `dimos run` blueprint?
- Should planned path preview expose `JointPath` only, or a time-parameterized `JointTrajectory` for scrubber playback fidelity?
- Should `solve_ik_preview()` return just the best solution, or include multiple IK branches when available for redundant arms?
- Do real hardware sessions need an explicit `--allow-execute` flag in addition to browser confirmation?
