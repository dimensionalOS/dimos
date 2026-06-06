## Context

The optional Viser manipulation panel was added as a companion UI over `ManipulationModule` RPCs. Its current implementation lives mostly in `dimos/manipulation/viser_panel/module.py`, while `state.py` already holds the orthogonal lifecycle axes (`PanelRuntime`, backend readiness, target status, plan status, action status), `PanelSession`, and `PreviewWorker`.

`module.py` currently owns all of these responsibilities at once:

- DimOS `Module` lifecycle, config, RPC methods, and fallback RPC-client creation.
- Polling the manipulation backend and translating transient startup errors into panel state.
- Creating Viser GUI controls and wiring callbacks.
- Creating Viser scene handles, transform controls, line segments, and current/ghost `ViserUrdf` models.
- Preparing xacro/URDF model paths and mapping DimOS joint names to Viser/youdfpy actuated joint names.
- Handling target updates, preset application, IK/FK preview request dispatch, result application, and recursion guards.
- Running long operations (`Plan`, local Viser `Preview`, `Execute`, `Cancel`, `Clear Plan`) off the Viser callback path.
- Interpolating planned joint paths for local ghost animation.

The file is cohesive at the feature level but too broad at the implementation level. Future additions such as more scene overlays, richer viewer controls, or more safety readouts would likely make the module harder to test and easier to regress.

Viser's public examples and API docs are handle/callback oriented: controls are created with `server.gui.add_*`, callbacks are registered with `on_update`/`on_click`, layout uses folders/tabs/forms, and live state is applied by mutating handle fields such as `.value`, `.options`, `.visible`, `.disabled`, `.position`, and `.wxyz`. Public examples also use helper functions/dataclasses for grouped handles rather than a React-style declarative renderer. That should shape this refactor.

## Goals / Non-Goals

**Goals:**

- Keep `ViserManipulationPanelModule` as the public DimOS module and blueprint surface.
- Keep `PanelSession` as the single mutable source of truth for panel state and gating.
- Split implementation concerns into small internal collaborators with explicit ownership of backend access, GUI handles, scene handles, workflow operations, and preview animation.
- Preserve existing public behavior, config, optional dependency behavior, CLI entrypoint, blueprint compatibility, and hardware execution opt-in.
- Make the tests describe component responsibilities instead of relying only on a partially initialized `ViserManipulationPanelModule`.
- Isolate Viser-specific quirks such as line-segment shape, transform-control quaternion order, `ViserUrdf` mesh material mutation, and actuated-joint-name mapping.

**Non-Goals:**

- Do not redesign the operator workflow or add new panel controls.
- Do not introduce a custom declarative UI framework.
- Do not change `ManipulationModule` RPC names, behavior, or return shapes.
- Do not change DimOS streams, blueprints, generated registry entries, or optional dependency extras unless the refactor reveals stale docs.
- Do not change execution safety gates, plan freshness logic, or the `allow_plan_execute` opt-in.

## DimOS Architecture

The refactor stays within `dimos/manipulation/viser_panel/` and should keep these public imports stable:

- `ViserManipulationPanelConfig`
- `ViserManipulationPanelModule`
- `viser_manipulation_panel`

No new typed streams, transports, DimOS Python `Spec` Protocols, skills/MCP tools, or generated blueprint registry entries are expected.

Recommended package layout:

```text
dimos/manipulation/viser_panel/
├── __init__.py
├── __main__.py
├── module.py          # DimOS Module shell and composition root
├── state.py           # Existing PanelSession, state axes, PreviewWorker
├── backend.py         # Manipulation backend client facade + retry/timeout helpers
├── gui.py             # Viser GUI handle ownership and callback wiring
├── scene.py           # Viser scene, URDF, path, target visual ownership
├── controller.py      # Robot selection, target/preset sync, operations orchestration
└── animation.py       # Local preview interpolation/ghost animation helpers
```

Suggested runtime composition:

```text
ViserManipulationPanelModule
  ├─ PanelSession
  ├─ PanelBackend
  │    ├─ module-ref client when composed in a blueprint
  │    └─ RPCClient.remote fallback when launched as companion process
  ├─ PanelScene
  │    ├─ current robot URDF
  │    ├─ target ghost URDF
  │    ├─ EE transform controls
  │    └─ planned-path line segments
  ├─ PanelGui
  │    ├─ status/error/feasibility handles
  │    ├─ robot/preset controls
  │    ├─ plan/preview/execute/cancel/clear buttons
  │    └─ dynamic joint sliders
  ├─ PanelController
  │    ├─ refresh/select/apply preset
  │    ├─ target changed from Cartesian or joints
  │    └─ plan/preview/execute/cancel/clear
  └─ PreviewAnimator
       └─ interpolate sparse joint paths for local Viser preview
```

### Component responsibilities

`module.py` should become the composition root:

- Import Viser and Viser URDF support, preserving current fail-fast install hints.
- Create `ViserServer` and collaborators in `start()`.
- Start/stop `PreviewWorker` and the polling thread.
- Expose `refresh_panel_state()` and `get_panel_snapshot()` as RPCs.
- Own the shared lock and pass it to collaborators, or ensure collaborators are only called while the module holds the lock.

`backend.py` should provide a small facade over either a composed `ManipulationModule` reference or `RPCClient.remote(ManipulationModule)`:

- `reset_client()` / `close()`
- `list_robots()` with stale-client timeout retry
- state reads: robot info, joints, EE pose, manipulation state, error
- preview calls: IK/FK with explicit timeout result mapping
- operations: plan to joints/pose, get path, execute, cancel, clear plan

The backend facade should not own Viser handles or mutate `PanelSession` directly, except through returned values. That keeps it testable with fake clients.

`gui.py` should own GUI handles and expose intent callbacks instead of business logic:

- Build static controls and dynamic joint sliders.
- Store handles in typed-ish dataclasses where practical (`StatusHandles`, `ActionHandles`, `JointSliderHandles`) rather than a single unstructured `_handles` dictionary.
- Use small declarative data only for repetitive controls, for example action-button metadata: label, handle key, action status, callback name.
- Provide methods such as `set_robot_options()`, `set_preset_options()`, `set_joint_slider_values()`, `set_target_pose_handle()`, and `apply_session_state()`.
- Convert Viser events into controller calls: selected robot, selected preset, transform-control changed, joint slider changed, action button clicked.

`scene.py` should own Viser scene handles:

- Ensure scene nodes for the selected robot.
- Prepare expanded URDFs through `prepare_urdf_for_drake()`.
- Create current and ghost `ViserUrdf` instances.
- Update current/ghost joints using Viser/youdfpy actuated joint names.
- Apply target feasibility colors/material opacity.
- Render/remove planned path line segments with the real Viser `(N, 2, 3)` segment shape.

`controller.py` should be the workflow coordinator over `PanelSession`, `PanelBackend`, `PanelGui`, and `PanelScene`:

- Refresh backend state and update GUI/scene.
- Select robot, invalidate plans, and rebuild robot-specific UI/scene as needed.
- Apply Current/Init/Home presets and submit preview requests.
- Handle target changes from Cartesian controls or joint sliders using `sync_source` recursion guards.
- Apply preview results only when sequence IDs match.
- Run plan, local preview, execute, cancel, and clear plan operations.
- Maintain the distinction between UI enablement gates and in-operation gates to avoid self-blocking actions.

`animation.py` should hold pure or nearly pure preview helpers:

- Interpolate sparse planned joint paths to visual frames.
- Animate a ghost target through `PanelScene.set_ghost_joints()`.
- Avoid backend preview calls; local preview remains Viser-only.

## Decisions

1. **Use composition as the primary structure.**
   - Rationale: the current complexity is from several real domains, not repeated syntax alone.
   - Alternative rejected: keep one module and add comments/regions. That would preserve coupling and not improve test seams.

2. **Do not build a general declarative UI framework.**
   - Rationale: Viser APIs and examples are imperative handle/callback APIs. A custom reconciler would add abstraction over mutable handles and make debugging harder.
   - Limited declarative use is still useful for static metadata such as action buttons and status bindings.

3. **Keep `PanelSession` as the state model.**
   - Rationale: the current orthogonal state axes already address lifecycle complexity. The refactor should not replace them with distributed state across GUI/scene/backend objects.
   - Collaborators may cache handles/resources, but business state stays in `PanelSession`.

4. **Keep the module as the public compatibility boundary.**
   - Rationale: imports, blueprints, and the companion entrypoint should not churn for users.
   - Internal files can be added freely as long as `__init__.py` exports remain stable.

5. **Split tests by responsibility as code moves.**
   - State/gating tests stay against `PanelSession` and `PreviewWorker`.
   - Backend retry/timeout tests should target `PanelBackend`.
   - Scene tests should target URDF joint mapping, target colors, and line-segment shape.
   - GUI/controller tests should target callback-to-intent wiring, target sync, stale preview results, plan/execute gating, and snapshots.

## Safety / Simulation / Replay

This is a behavior-preserving refactor. It must not broaden hardware execution availability. `Execute` remains disabled unless `allow_plan_execute` is enabled and the current joints still match the fresh plan start snapshot within tolerance.

Manual QA should use the same surface as the existing panel:

- Focused tests: `uv run pytest dimos/manipulation/test_viser_panel.py dimos/manipulation/test_manipulation_unit.py -q`.
- Lightweight launch/snapshot check against the mock panel when no conflicting DimOS coordinator is running: `uv run --extra manipulation-viser dimos run xarm7-viser-panel-mock` and inspect the Viser URL.
- If another DimOS run owns the default coordinator/LCM bus, do not stop it casually; either defer live launch or isolate the run if supported.

Simulation/replay behavior is unchanged because no new robot-facing commands or streams are introduced.

## Risks / Trade-offs

- **Regression from moving coupled code.** Mitigate with small commits/tasks and existing tests before and after each extraction.
- **Over-abstraction.** Keep collaborators concrete and panel-specific. Avoid generic GUI frameworks, registries, or plugin systems.
- **Locking/thread assumptions.** Preserve current lock discipline. If collaborators are called from poll threads, preview workers, and Viser callbacks, document whether each method expects the caller to hold the lock.
- **Handle lifecycle leaks.** Scene/GUI owners should provide explicit remove/reset paths for robot switches and plan clearing.
- **Test churn.** Some tests currently patch partially initialized `ViserManipulationPanelModule`. The refactor should reduce that pattern by testing collaborators directly, but public module-level regression tests should remain.

## Migration / Rollout

Roll out in extraction order from lowest-risk to highest-risk:

1. Move pure helpers and preview interpolation into `animation.py` without behavior changes.
2. Extract scene handle/URDF/path rendering into `scene.py` and update scene-focused tests.
3. Extract backend client facade and timeout/retry handling into `backend.py`.
4. Extract GUI handle creation and session-to-handle rendering into `gui.py`.
5. Extract workflow orchestration into `controller.py`, leaving `module.py` as lifecycle shell.
6. Run focused tests, LSP diagnostics for changed files, pre-commit, and manual QA where available.

No data migration, generated registry update, or dependency lockfile change is expected.

## Open Questions

- Should `PanelGui` use Viser folders/tabs during extraction, or should layout remain visually unchanged until a later UI polish change?
- Should lock ownership be centralized in `module.py`, or should collaborators receive a small `locked()` helper/context to make thread expectations explicit?
- Should compatibility tests keep importing private helper methods from `ViserManipulationPanelModule`, or should they move immediately to component-level APIs as each component is extracted?
