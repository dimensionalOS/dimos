## Context

Viser currently receives `ManipulationModule` and `WorldMonitor` directly. Its GUI and visualization adapter call a broad set of module and monitor APIs for topology, current state, FK, collision checks, IK, planning, preview, execution, cancellation, and plan freshness. The Viser layer also accepts local and global joint-name variants, composes complete robot states, projects generated paths for preview, and duplicates safety policy that belongs in manipulation.

The visualization protocol reinforces this coupling: backends pull current state, receive `GeneratedPlan`, resolve planning groups at runtime, and separately show, animate, cancel, and hide preview ghosts. Execution independently turns the stored geometric path into trajectories, so preview and execution need not share one timing authority.

The agreed domain language in `CONTEXT.md` distinguishes paths, synchronized trajectories, generated plans, target drafts, target evaluations, plan freshness, manipulation operators, and transient previews. This design applies those distinctions while preserving the existing visible Viser interaction contract.

## Goals / Non-Goals

**Goals:**

- Materialize one synchronized trajectory immediately after geometric planning and store it with the path in `GeneratedPlan`.
- Make the stored trajectory the sole motion and timing authority for preview and execution.
- Let the existing `JointTrajectoryTask` execute a configured subset without commanding omitted joints.
- Replace Viser's direct module/monitor dependencies with one concrete, UI-neutral `ManipulationOperator`.
- Make visualization topology immutable after initialization and push current joint-state frames instead of allowing renderers to pull world state.
- Pass raw globally named trajectories to visualization backends and keep projection backend-local.
- Move execution freshness and authoritative action validation out of Viser.
- Remove obsolete adapters and representation conversions rather than retaining compatibility shims.

**Non-Goals:**

- Changing planner algorithms, IK algorithms, collision ownership, or world-backend kinematics.
- Providing transport-level simultaneous multi-robot state snapshots; planning keeps the current static-world assumption and existing state APIs.
- Supporting dynamic robot or planning-group topology after visualization initialization.
- Adding atomic multi-task coordinator dispatch, dynamic subset resource claims, or concurrent joint ownership semantics.
- Correcting the existing lack of coordinator task cancellation when manipulation execution is cancelled.
- Changing Viser controls, wording, layout, group selection behavior, target ghosts, or pose-gizmo interactions.
- Adding arbitrary pose-target frame transforms; operator pose requests initially require the world frame.

## Decisions

### 1. Materialize `GeneratedPlan` once after planner success

`PlannerSpec.plan` continues to produce the planner's geometric `PlanningResult`. `ManipulationModule` validates the successful path, resolves selected global joint limits by exact name and requested order, calls one trajectory generator across all planned joints, validates the result, and constructs a complete `GeneratedPlan` containing both `path` and `trajectory`.

Only complete generated plans are cached or exposed. Preview, execution, status, and completion timing MUST NOT regenerate or re-parameterize the path. The path remains geometric/diagnostic data; the trajectory is motion/timing authority.

This keeps planner backends waypoint-focused and avoids a transient incomplete `GeneratedPlan` contract.

### 2. Store only planned global joints and no separate baseline

Both `GeneratedPlan.path` and `GeneratedPlan.trajectory` use canonical global joint names in selected planning-group order. `path[0]` and the first trajectory point represent the planned-joint start state. Joints outside the selected groups are not stored or commanded.

No plan baseline is added. Under the accepted single-commander/static-state assumption, renderers overlay planned joints onto their latest current visual state. Execution sends only planned joints through the existing trajectory task.

### 3. Extend the existing `JointTrajectoryTask` for partial trajectories

The task validates that incoming trajectory names are a non-empty, unique subset of its configured joints and that every point matches the active width with finite values and valid timing. During execution it emits `JointCommandOutput` using the incoming active names, not the task's full configured list.

`claim()` continues to claim the full configured joint set. This intentionally preserves current arbitration/preemption behavior while preventing commands to omitted joints. Dynamic claims are deferred.

### 4. Initialize a static visualization session

`PlanningSceneInfo` is extended with resolved `PlanningGroup` values in addition to robot configs keyed by world robot ID. `ManipulationModule` constructs one concrete `ManipulationOperator` after world setup and asks `WorldMonitor` to initialize the visualization with a session containing:

- immutable scene/robot/planning-group metadata; and
- the optional concrete operator used by interactive backends.

There is no `ManipulationOperatorSpec` or protocol because only one implementation exists. `VisualizationSpec` remains justified by its multiple backend implementations.

### 5. Push fast state and keep slow operator status small

`WorldMonitor` periodically builds a `VisualizationStateFrame` containing current joint states keyed by world robot ID and calls `VisualizationSpec.update_state(frame)`. The frame contains no staleness policy. Viser uses it for current URDF rendering, target initialization, current presets, and joint readout.

`ManipulationOperator.status()` returns only slow-changing manipulation state, error, and stored-plan summary. It does not return topology or current joint telemetry.

### 6. Use a raw trajectory visualization API

The runtime visualization API is reduced to raw globally named trajectory animation and cancellation. Backends receive no `GeneratedPlan`, path, or runtime group IDs. They derive affected robots from trajectory joint names using static initialization metadata, preserve stored timestamps by default, and apply an explicit duration only as playback-rate scaling without mutating the trajectory.

Preview ghosts are transient. Animation shows affected ghosts, plays the trajectory, and hides them in `finally`; cancellation invalidates playback and hides ghosts immediately. `show_preview` and `hide_preview` are removed.

### 7. Use one concrete stateless manipulation operator facade

The Viser panel receives only `ManipulationOperator` and its local scene/controller objects. The facade composes `ManipulationModule` and `WorldMonitor` and exposes synchronous typed operations:

- compact `status`;
- joint/pose target evaluation;
- joint/pose planning;
- preview, execute, cancel, clear-plan, and reset.

Target drafts, group selection, and callback generation IDs remain frontend-owned. Operator evaluation and planning calls are stateless with respect to drafts.

Joint requests contain ordered group IDs and one exact globally named `JointState` whose names equal the selected groups' concatenated joints in group order. Pose requests contain explicit world-frame `PoseStamped` targets, optional auxiliary group IDs, and an optional globally named seed.

Target evaluations are advisory and expose only selected global joints, group poses, and group diagnostics. Complete per-robot states used for FK/collision checks stay private. Planning consumes the original target request rather than a prior evaluation result. Planning returns a typed summary, never the complete `GeneratedPlan`.

### 8. Keep authoritative freshness in manipulation

Viser no longer polls `is_state_stale`, records complete robot snapshots, compares snapshots, or decides whether execution is fresh. Immediately before dispatch, `ManipulationModule.execute` resolves current planned joints and compares them against the first stored trajectory point within the configured tolerance. Missing, malformed, or mismatched state rejects execution.

Omitted joints are intentionally ignored under the accepted single-commander assumption.

### 9. Route preview cancellation through the existing outbound port

`ManipulationOperator.cancel` calls `ManipulationModule.cancel`. The module routes preview cancellation through `WorldMonitor`, which calls `VisualizationSpec.cancel_preview_animation`; no layer knows whether Viser or another backend implements the port.

The actual coordinator execution-cancellation gap remains explicitly out of scope.

## Risks / Trade-offs

- **Partial trajectories change task validation/output behavior** → Preserve full task claims, add strict subset/shape/timing validation, and retain full-width trajectory tests.
- **No stored baseline means omitted joints can differ between preview moments** → Accepted under the stated static/single-commander assumption; planned joints and timing remain identical.
- **Raw global trajectories make renderers aware of global joint naming** → Supply immutable robot/group metadata once and centralize backend-local name projection helpers.
- **A visualization callback can re-enter the same backend through operator preview** → Keep operator calls synchronous but run them from Viser's existing worker lane; retain visualization mutation/cancellation locking and generation tests.
- **Removing frontend freshness checks changes immediate button behavior** → Keep authoritative module rejection and return clear action errors; do not duplicate policy in state frames.
- **Direct cutover touches several layers simultaneously** → Land model/materialization and task subset support first, then protocol/session/operator migration, then delete legacy methods and adapters after all backends compile.
- **Exact API names may need adjustment against construction code** → Preserve the directional contracts and responsibilities even if final concrete class or method names change during implementation.

## Migration Plan

1. Add trajectory materialization and partial-task support with focused tests while preserving existing visualization calls temporarily inside the implementation branch.
2. Add static visualization session/state-frame DTOs and the concrete operator; initialize them after world setup.
3. Migrate WorldMonitor, Drake, and Viser to `initialize`, `update_state`, raw `animate_trajectory`, and cancellation.
4. Migrate the Viser panel to typed operator requests/results and pushed current state; remove direct module/monitor access, snapshot freshness logic, and `GroupPanelBackend`.
5. Remove old visualization methods and all lazy path-to-trajectory/preview projection paths.
6. Run full manipulation, planning, visualization, control, lifecycle, and multi-robot tests plus mypy, Ruff, diff, and strict OpenSpec validation.

Rollback is a source revert because this is an internal breaking boundary with no persisted data migration.

## Open Questions

- Final concrete DTO and method names are provisional until implementation verifies construction and lifecycle call sites; their responsibilities and directionality are fixed by this design.
