## Context

The current manipulation planning stack has a backend-neutral `VisualizationSpec` for URL lookup, publishing, preview visibility, path animation, and cleanup. In practice, Meshcat visualization is still embedded in `DrakeWorld`, and `WorldMonitor` discovers visualization by checking whether the world object implements `VisualizationSpec`. `ManipulationModule` only has `enable_viz: bool`; when enabled it creates `WorldMonitor(enable_viz=True)`, starts the visualization thread, and reports the URL through `get_visualization_url()`.

The previous Viser prototype exists on the `cc/viser-vis` branch as a full manipulation panel module. It includes a Viser server, URDF rendering, current and ghost robot scene handling, path rendering, GUI widgets, preview coalescing, plan/preview/execute/cancel actions, and tests. Its data access was mostly through an RPC-like `PanelBackend`, which created a useful boundary between GUI callbacks and manipulation planning state.

This change intentionally accepts the risk of running Viser inside the manipulation process. The design must still preserve the old backend boundary concept so Viser callbacks do not directly mutate `WorldSpec`, run IK unsafely, or hold locks while rendering.

## Goals / Non-Goals

**Goals:**

- Add Viser as a selectable manipulation visualization backend that implements `VisualizationSpec`.
- Preserve current Meshcat behavior and no-visualization behavior.
- Retain the useful behavior from the previous Viser panel, including URDF rendering, ghost preview, path rendering, GUI controls, preview worker, status display, and guarded execution.
- Replace old RPC reads with a small in-process adapter over existing `ManipulationModule` and `WorldMonitor` methods, avoiding a parallel visualization data model.
- Ensure Viser GUI callbacks and preview workers call existing manipulation operations through that adapter rather than directly using `WorldSpec`, live contexts, planner, or kinematics objects.
- Keep Viser dependencies optional and lazily imported.

**Non-Goals:**

- Do not make Viser a global `vis_module()` replacement for Rerun stream visualization.
- Do not extract Meshcat out of `DrakeWorld` in this change unless needed for a minimal adapter seam.
- Do not expose Viser panel actions as agent skills or MCP tools.
- Do not bypass existing manipulation planning, collision checking, or execution state gates.
- Do not require Viser dependencies for users who keep Meshcat or no visualization.

## DimOS Architecture

### Configuration and factory shape

Add manipulation-specific visualization selection to `ManipulationModuleConfig`:

- `visualization_backend: Literal["meshcat", "viser", "none"] | None`
- `enable_viz: bool` remains for compatibility.
- Viser-specific config fields cover host, port, browser opening, panel enablement, preview timing, polling rate, and `allow_plan_execute`.

Compatibility semantics:

- `enable_viz=False` and no explicit backend means no manipulation visualization.
- `enable_viz=True` and no explicit backend keeps existing Meshcat behavior.
- `visualization_backend="viser"` creates a Viser visualization and does not enable Drake Meshcat.
- `visualization_backend="meshcat"` keeps current DrakeWorld/Meshcat behavior.
- `visualization_backend="none"` disables visualization even if callers request URL/publish/preview operations.

Add a manipulation visualization factory near `dimos/manipulation/planning/factory.py`, conceptually:

```text
create_visualization(
  backend,
  world,
  world_monitor,
  manipulation_module,
  config,
) -> VisualizationSpec | None
```

The factory should lazy-import Viser only when the Viser backend is selected and should raise a clear install hint such as `uv sync --extra manipulation-viser` when dependencies are missing.

### WorldMonitor integration

Refactor `WorldMonitor` so visualization is selected explicitly instead of inferred only from the world object:

```text
WorldMonitor
  ├── _world: WorldSpec
  ├── _visualization: VisualizationSpec | None
  ├── _lock: RLock
  ├── state/obstacle monitors
  └── optional visualization robot-registration hook
```

For Meshcat, `DrakeWorld(enable_viz=True)` may continue to be assigned as `_visualization` because it already implements `VisualizationSpec`. For Viser, `DrakeWorld(enable_viz=False)` remains the planning world and `_visualization` becomes the new Viser object. For no visualization, `_visualization` is `None` and existing delegation methods remain safe no-ops.

`WorldMonitor.add_robot()` should also notify the Viser scene of robot metadata, either by calling an internal registration hook when the visualizer supports it or by letting the visualizer read robot configs from `WorldMonitor`/`ManipulationModule` through its adapter. This is an internal adapter concern, not a change to the public `VisualizationSpec` protocol.

### Viser package layout

Introduce a manipulation-local package, for example:

```text
dimos/manipulation/visualization/viser/
  config.py        # Viser config models and defaults
  visualizer.py    # ViserManipulationVisualizer implements VisualizationSpec
  runtime.py       # ViserServer lifecycle and optional browser opening
  scene.py         # URDF/current/ghost/path rendering helpers
  gui.py           # optional panel GUI widgets and callbacks
  state.py         # panel session, enums, preview request/worker
  adapter.py       # small in-process adapter over existing module/world APIs
  animation.py     # interpolation and ghost animation helpers
```

The top-level `ViserManipulationVisualizer` owns the Viser runtime, scene, optional GUI, and optional preview/action workers. It implements:

- `get_visualization_url()`
- `publish_visualization(ctx: Any | None = None)`
- `show_preview(robot_id)`
- `hide_preview(robot_id)`
- `animate_path(robot_id, path, duration=3.0)`
- `close()`

### Replacing old RPC reads with a lean in-process adapter

The old panel retrieved data through RPC calls such as robot listing, robot info, current joints, EE pose, target evaluation, planning, preview, execution, cancellation, and clear-plan actions. Because Viser now runs in-process, this change should not introduce a large `VisualizationDataProvider`, broad snapshot DTOs, or a second visualization data model.

Instead, preserve the useful part of the old boundary with a small internal adapter over existing `ManipulationModule` and `WorldMonitor` state/methods:

```text
InProcessViserAdapter
  ├── list_robots()
  ├── get_robot_config(robot_name)
  ├── get_current_joint_state(robot_name)
  ├── is_state_stale(robot_name)
  ├── get_ee_pose(robot_name)
  ├── get_planned_path(robot_name)
  ├── get_planned_trajectory_duration(robot_name)
  ├── plan_to_pose(...)
  ├── plan_to_joints(...)
  ├── preview_path(...)
  ├── execute(...)
  └── clear_planned_path()
```

This adapter is not a public RPC surface and should mostly delegate to existing code:

- robot metadata comes from `RobotModelConfig` already stored in `ManipulationModule._robots` and retrievable from `WorldMonitor.get_robot_config()`;
- current joints come from `WorldMonitor.get_current_joint_state()`;
- stale-state checks come from `WorldMonitor.is_state_stale()`;
- EE pose display, if needed, comes from `WorldMonitor.get_ee_pose()`;
- planned paths and trajectory duration can be copied from `ManipulationModule._planned_paths` and `_planned_trajectories`;
- planning, preview, execution, and clear-plan actions call existing `ManipulationModule` methods.

The adapter's job is callback discipline, not data translation. GUI callbacks should enqueue work or call adapter methods from controlled worker/poll contexts. They should not reach around the adapter to call raw `WorldSpec`, `world.get_live_context()`, `world.set_joint_state()`, `kinematics.solve()`, or planner methods directly.

Use small local copies at the read boundary where mutable containers are involved. For example, copy a planned path with `list(path)` before rendering. Do not add broad immutable snapshot types unless a specific feature cannot be expressed through existing accessors.

### Locking and callback contract

Viser introduces server threads, GUI callbacks, preview workers, and optional animation work. The lock contract is:

- Viser event callbacks may read widget values and enqueue work, but must not run IK/planning or take the world lock directly.
- The adapter/operation worker is the only place where panel actions call manipulation operations.
- The operation worker must not hold Viser scene/GUI locks while calling manipulation methods.
- Manipulation/world accessors must not call back into Viser.
- `WorldMonitor` owns access to `WorldSpec`; Viser does not store or mutate raw live contexts.
- Prefer scratch contexts for FK/IK-related read calculations. Do not reuse live contexts for Viser rendering.
- If rendering needs world-derived data, obtain it through existing `WorldMonitor` accessors, copy only what is needed, release any manipulation/world locks, then render.
- Avoid lock order cycles by enforcing this order when both are unavoidable: manipulation/world lock first for data copy, release it, then Viser scene/GUI update. Never hold Viser locks while waiting on manipulation locks.

Panel operations should use an internal request/response object with timeout and stale-request handling, preserving the behavior of the previous preview worker. If an operation times out or conflicts with `PLANNING`/`EXECUTING`, the GUI reports the error and does not retry in a tight loop.

### Viser scene behavior

The scene layer should reuse the previous panel concepts:

- Prepare/expand URDF input for Viser.
- Map DimOS robot joint names to Viser/yourdfpy actuated joint order.
- Render current robot state.
- Render a translucent preview/ghost robot.
- Render planned paths with Viser line segments.
- Animate ghost joints along a path.
- Maintain handles and remove/update them during close or robot changes.

Known Viser details to preserve in tests:

- `scene.add_line_segments()` points must have shape `(N, 2, 3)`.
- `ViserUrdf.update_cfg()` must receive values in Viser/yourdfpy actuated joint order.
- Transparent material overrides should be applied through Viser-supported mesh color/opacity handles.

### Skills/MCP, streams, blueprints, and generated registries

No new skill or MCP tool is required. No new stream contract is required for the in-process Viser backend. Existing manipulation blueprints may pass config to `ManipulationModule` to select Viser; no generated blueprint registry update is required unless implementation adds a new blueprint. If a new blueprint is added, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`.

## Decisions

1. **Use an in-process Viser `VisualizationSpec` implementation.**
   - Rationale: The user explicitly wants Viser swappable with Meshcat through the new protocol and accepts the process-coupling risk.
   - Alternative rejected: Keep Viser only as a separate dedicated-worker panel.

2. **Preserve the old backend boundary as a tiny in-process adapter, not RPC or broad DTO snapshots.**
   - Rationale: The old RPC boundary prevented GUI callbacks from directly reaching world/IK internals. In-process Viser still needs callback discipline, but most required data already exists on `ManipulationModule`, `RobotModelConfig`, and `WorldMonitor`.
   - Alternative rejected: Let GUI callbacks call `WorldSpec` and kinematics objects directly.

3. **Use copy-at-read-boundary rendering instead of a new visualization snapshot model.**
   - Rationale: Viser rendering should consume small local copies and should not hold manipulation locks while sending Viser updates, but a large `ManipulationVisualizationSnapshot` would duplicate existing module/world state.
   - Alternative rejected: Render directly from live world contexts.

4. **Keep Meshcat embedded initially.**
   - Rationale: Existing Meshcat behavior works and should remain compatible. Extracting a separate Meshcat adapter can happen later.

5. **Do not overload global `GlobalConfig.viewer`.**
   - Rationale: Global Rerun visualization and manipulation planning visualization are separate surfaces.

6. **Make the Viser panel optional but available.**
   - Rationale: Protocol behavior should work without full panel controls, while the previous panel functionality remains available when enabled.

## Safety / Simulation / Replay

Viser must not authorize motion. Execution from the optional panel remains behind `allow_plan_execute` and existing manipulation execution/state checks. Preview and rendering must not alter stored plans, generated trajectories, collision checking, or robot command outputs.

Simulation, replay, and non-hardware workflows should be able to select Viser for inspection without changing planner behavior. Hardware users must opt into any panel execution path. Missing Viser dependencies should fail clearly only when Viser is selected; Meshcat and no-visualization workflows should remain available.

Manual QA should include a simulated or mock manipulation setup, a path preview, stale preview dismissal before replanning, optional panel planning/preview/execute gating, and shutdown while the browser is connected.

## Risks / Trade-offs

- **Process coupling:** Viser server or GUI failures may affect `ManipulationModule`. Mitigate with lazy import, clear close order, worker timeouts, broad callback exception logging, and tests for shutdown.
- **Deadlocks/reentrancy:** GUI callbacks can conflict with world/planning locks. Mitigate with the in-process adapter, operation queues, copy-at-read-boundary rendering, and strict lock ordering.
- **IK/world access mistakes:** The panel may need feasibility or target evaluation that previously crossed RPC. Mitigate by adding explicit backend methods that call existing manipulation operations or bounded helper methods, never raw `WorldSpec` from GUI callbacks.
- **Joint mapping bugs:** Viser/yourdfpy actuated joint order may differ from DimOS robot config order. Mitigate with unit tests using fake URDFs and known joint orders.
- **Dependency footprint:** Viser/URDF dependencies are optional. Mitigate with extras and lazy imports.
- **Backend naming confusion:** Users may confuse global Rerun viewer selection with manipulation visualization backend. Mitigate in docs and config names.

## Migration / Rollout

1. Add config fields and backend resolution while keeping `enable_viz=True` mapped to Meshcat by default.
2. Refactor `WorldMonitor` to accept/create an explicit visualization backend while preserving existing Meshcat delegation.
3. Add Viser package with minimal runtime and `VisualizationSpec` implementation.
4. Add the lean in-process adapter and operation worker before porting panel controls.
5. Port Viser scene rendering and path animation from the previous branch.
6. Port optional panel UI at minimal usable level.
7. Complete `cc/viser-vis` panel parity: transform target control, pose/joint preview worker, rich session states, plan freshness, safe execute gating, and parity regression tests.
8. Add optional `manipulation-viser` dependency extra.
9. Add unit/regression tests and update docs.
10. If new blueprints are added, regenerate `dimos/robot/all_blueprints.py`.

Rollback path: keep Meshcat as the default and guard Viser behind explicit backend selection. If Viser proves unstable, users can set the backend to Meshcat or none without changing planning code.

## Open Questions

- Exact Viser default port should be selected to avoid common Rerun/MCP/websocket ports.
- Whether Viser path animation should remain blocking like the current Meshcat implementation or immediately move to a cancellable animation worker.
- Which obstacle/perception objects should be included in the first Viser snapshot beyond robot and path rendering.
- Whether a dedicated-worker Viser panel should be reintroduced later as an additional deployment mode using the same adapters.
