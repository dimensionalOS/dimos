## 1. Materialize Generated Plans

- [x] 1.1 Extend `GeneratedPlan` with a required synchronized `JointTrajectory`, migrate construction sites, and add model/import/round-trip tests proving path and trajectory remain distinct representations.
- [x] 1.2 Add one post-planner materialization path in `ManipulationModule` that validates selected global waypoints, resolves velocity/acceleration limits by exact global name and selection order, and invokes one trajectory generator across all planned joints.
- [x] 1.3 Validate generated trajectory names, dimensions, finite values, timestamps, positive duration, and ordered waypoint boundaries before caching; ensure planning/parameterization failure exposes no generated plan.
- [x] 1.4 Remove non-dispatch lazy path-to-trajectory generation from status/completion timing and isolate the remaining dispatch-only bridge for Phase 2 removal after partial trajectory task support lands.
- [x] 1.5 Add materialization tests for reordered groups, heterogeneous multi-robot limits, malformed/non-finite waypoints, invalid limits, generator failure, exactly one generation call, shared timing, and zero generation after caching.

## 2. Execute Partial Joint Trajectories

- [x] 2.1 Update the existing `JointTrajectoryTask.execute` validation to accept only non-empty unique configured subsets with valid point widths, finite values, and strictly increasing timing while preserving full-width behavior.
- [x] 2.2 Make task computation emit only the active trajectory joint names and sampled values while leaving `claim()` on the full configured joint set and clearing active subset state on replacement, completion, cancellation, fault, and reset.
- [x] 2.3 Update manipulation execution to split the stored global trajectory by robot, preserve every timestamp/value, translate selected names only at coordinator dispatch, and never fill or command omitted joints.
- [x] 2.4 Add control/manipulation tests for valid subsets, unknown/duplicate/empty names, malformed positions/velocities/timing, full-width compatibility, full configured claims, replacement/reset lifecycle, coordinator translation, omitted-joint non-commanding, and multi-robot shared clocks.

## 3. Introduce the Visualization Session Boundary

- [x] 3.1 Extend `PlanningSceneInfo` with resolved immutable planning groups and add visualization session/state-frame DTOs containing static topology, the optional concrete operator, and pushed current joint states respectively.
- [x] 3.2 Replace `VisualizationSpec` pull/group/plan methods with one-shot initialization, `update_state`, raw globally named `animate_trajectory`, cancellation, URL, and close; add structural/import tests for the complete protocol.
- [x] 3.3 Make `WorldMonitor` initialize visualization once, push current joint-state frames at visualization cadence, serialize renderer mutations, forward raw stored trajectories, and route cancellation without exposing freshness policy.
- [x] 3.4 Migrate Drake/Meshcat visualization to static topology and raw trajectory playback, preserving stored non-uniform timestamps, optional playback scaling, synchronized multi-robot ticks, generation cancellation, and automatic ghost cleanup.
- [x] 3.5 Add WorldMonitor and Drake tests for immutable group metadata, pushed state frames, no state pull, raw timing/scaling, trajectory immutability, multi-robot shared ticks, replacement/cancel/error/close cleanup, and removal of legacy visualization methods.

## 4. Add the Concrete Manipulation Operator

- [x] 4.1 Add concrete operator status, canonical joint/pose request, target evaluation, plan summary, action result, and visualization session types without introducing a new operator Protocol or Spec.
- [x] 4.2 Implement exact ordered global-joint request validation and explicit world-frame pose request validation, rejecting aliases, duplicates, omissions, extras, unsupported groups/frames, and malformed seeds.
- [x] 4.3 Move joint/pose target composition, IK, FK, per-robot state validity, and selected-domain evaluation results from Viser-local helpers into the operator while keeping complete robot states private.
- [x] 4.4 Implement synchronous stateless planning methods that consume original requests, delegate to manipulation APIs, and return summaries without exposing `GeneratedPlan`; implement compact slow-changing status and action methods.
- [x] 4.5 Move execute freshness enforcement into `ManipulationModule` by comparing current planned global joints with the stored trajectory start immediately before dispatch; remove dependence on frontend robot snapshots and stale-state gates.
- [x] 4.6 Construct one operator after module/world setup, bind it into visualization initialization, preserve indirect module-to-world-to-visualization cancellation, and add lifecycle/action/freshness tests.

## 5. Migrate Viser to the Clean Boundary

- [x] 5.1 Initialize Viser from static robot/group metadata and build deterministic global-joint-to-robot/local-joint maps without retaining `ManipulationModule` or `WorldMonitor` dependencies.
- [x] 5.2 Consume pushed state frames for current URDFs, joint readout, target initialization, and Current presets; remove direct state/freshness polling and Viser-side stale policy.
- [x] 5.3 Replace generated-plan preview projection and preview-track resampling with raw stored-trajectory projection/playback; support duration scaling only, hide ghosts on completion/cancel/error/replacement/close, and prevent stale frames from mutating scene handles.
- [x] 5.4 Migrate the panel to the concrete operator's typed status, target evaluation, planning, preview, execution, cancellation, clear, and reset calls while keeping target drafts and callback generations frontend-owned.
- [x] 5.5 Remove `GroupPanelBackend`, direct module/monitor calls, local/global alias normalization, complete-state composition, robot snapshot matching, and obsolete preview show/hide state from Viser code.
- [x] 5.6 Preserve the normative Viser UI controls and interactions exactly and add granular tests for group selection, presets, sliders, pose gizmos, advisory evaluation, action results, pushed current state, raw preview timing, auto-hide, cancellation races, and absence of forbidden dependencies.

## 6. Cleanup, Documentation, and Validation

- [x] 6.1 Remove all production/test references to `animate_plan`, `show_preview`, `hide_preview`, lazy path projection/parameterization, Viser `ManipulationModule`/`WorldMonitor` access, and legacy panel adapter APIs.
- [x] 6.2 Update manipulation/planning-group/visualization documentation and `CONTEXT.md` to describe materialized generated plans, partial planned-joint commands, the concrete operator, pushed visualization state, and transient previews.
- [x] 6.3 Run focused planner/materialization, manipulation module/unit/reservation, WorldMonitor, Drake, Viser/factory/lifecycle, control task/coordinator, and multi-robot end-to-end tests with no relevant skips.
- [x] 6.4 Run targeted mypy over every changed production file, Ruff check/format, `git diff --check`, and `openspec validate clean-manipulation-visualization-boundary --strict`; audit the final diff for unrelated planner/control/UI changes.
- [x] 6.5 Complete a final architecture and safety review confirming one trajectory-generation call, stored-timing preview/execution parity, partial-output/full-claim semantics, module-owned freshness, renderer isolation, exact Viser interaction parity, and no compatibility shims.
