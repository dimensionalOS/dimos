## Context

`ManipulationModule._materialize_generated_plan()` currently validates a planner path, resolves selected-joint limits, directly constructs `JointTrajectoryGenerator`, and stores its output beside the source path in `GeneratedPlan`. `JointTrajectoryGenerator` creates an independent trapezoidal profile for each adjacent waypoint pair, so every interior waypoint is a stop. Dense paths consequently execute much more slowly than their geometric length and robot limits imply.

The current execution architecture is intentionally atomic: a cached `GeneratedPlan` contains both its source path and executable `JointTrajectory`, and `PlanExecutionManager` dispatches that stored trajectory without regenerating it. This design preserves that contract while introducing a deep path-to-trajectory adapter seam.

RoboPlan 0.5.1 provides TOPP-RA with Hermite, cubic, adaptive, and linear-blend curve-fitting modes. Its parameterizer owns collision preservation for fitted curves and obtains absolute velocity and acceleration limits from its RoboPlan scene. The DimOS `RobotModelConfig` motion-limit fields are currently informal and are not authoritative for this backend.

## Goals / Non-Goals

**Goals:**

- Select one parametrization backend at manipulation-stack startup.
- Preserve the existing simple segmented-trapezoid behavior as a compatibility backend.
- Add RoboPlan TOPP-RA for any planner path represented in `RoboPlanWorld`.
- Convert and validate a path before constructing or caching `GeneratedPlan`.
- Preserve the source path while allowing bounded backend interpolation between waypoints.
- Keep preview and execution on the exact trajectory accepted during planning.
- Use URDF-backed RoboPlan velocity and acceleration limits and fail clearly when they are unavailable.
- Retain current public manipulation RPC, skill, MCP, stream, and execution signatures.

**Non-Goals:**

- Path shortcutting, waypoint simplification, or path-class-specific resampling.
- Linear-TCP constraint metadata or constraint-aware geometric post-processing.
- Reparametrizing one stored geometric path at multiple speeds.
- Runtime backend switching or cross-backend fallback.
- Formal per-joint DimOS motion-limit models or RoboPlan YAML overrides.
- Independent DimOS collision resampling of trajectories already checked by RoboPlan.
- Jerk-limited execution or changes to the trajectory message schema.

## DimOS Architecture

### Configuration and startup

Add a typed `TrajectoryParametrizationConfig` under manipulation planning configuration. It selects `simple_trapezoid` or `roboplan_toppra` and carries backend-specific options:

- common operating scales and output sample period;
- simple-backend point density/minimum segment controls needed for compatibility;
- RoboPlan spline-fitting mode and its adaptive/blend controls.

The configuration factory validates the complete backend combination during startup. `roboplan_toppra` requires a finalized `RoboPlanWorld`; a non-RoboPlan world is rejected before planning. The selected parametrizer is constructed once and retained by `ManipulationModule`.

### Adapter Protocol

Introduce a small adapter `Protocol`, distinct from an RPC-oriented DimOS `Spec`, for path-to-trajectory conversion. Its input contains:

- selected planning-group IDs;
- exact global joint ordering;
- the validated source `JointState` path;
- backend configuration.

Its output is the canonical `JointTrajectory`, or it raises/returns a typed failure that plan materialization converts into the module's existing planning error surface. The adapter must not mutate the input path.

The simple adapter wraps `JointTrajectoryGenerator` and receives its existing DimOS-resolved limits. The RoboPlan adapter owns a finalized `RoboPlanWorld`/`RoboPlanModel` reference, resolves the selected group from the model, converts global names to native RoboPlan ordering, invokes `PathParameterizerTOPPRA`, and converts the native result back to exact selected global ordering.

The RoboPlan adapter may cache one native TOPP-RA parameterizer per selected group set. This is internal optimization; construction and use must remain safe under the manipulation module's existing planning concurrency rules.

### Plan materialization

Retain `GeneratedPlan` as the canonical accepted aggregate:

```text
PlanningResult.path
        │
        ▼
canonical input validation
        │
        ▼
startup-selected TrajectoryParametrizer
        │
        ▼
canonical timed-output validation
        │
        ▼
GeneratedPlan(path + trajectory)
```

Replace the direct `JointTrajectoryGenerator` construction inside materialization with the selected adapter. A failure at either parametrization or validation leaves `_last_plan` unset and follows the existing planning-epoch failure path. No separate public `GeneratedTrajectory` lifecycle is added.

Canonical validation retains the current strong invariants: exact global joint ordering, finite and dimensionally aligned positions/velocities, first time at zero, strictly increasing times, positive duration for non-noop motion, and preserved start/goal. It also checks returned motion against the applicable velocity and acceleration limits with a documented numerical tolerance. Where RoboPlan exposes native accelerations, validate them before converting to the current positions/velocities-only message; otherwise derive the acceleration check consistently from velocity samples.

### RoboPlan limits and fitting

Pin RoboPlan to `0.5.1`. `RoboPlanModel.scene` remains the source of native joint order and absolute TOPP-RA limits:

- velocity from the URDF model;
- acceleration from extended URDF joint limits.

DimOS `max_velocity`, `velocity_limits`, and `max_acceleration` do not override RoboPlan TOPP-RA in this change. Startup or first selected-group construction fails explicitly when a required URDF limit is missing or invalid. Common TOPP-RA velocity and acceleration scales may reduce, but never increase, the scene limits.

Default RoboPlan fitting is `LinearBlend`, subject to confirming the 0.5.1 Python binding names during implementation. Other supported modes remain startup-selectable. Curve fitting is part of path-to-trajectory conversion, but preprocessing that rewrites the source waypoint sequence is not.

RoboPlan owns collision checking for a fitted curve against its authoritative scene. DimOS does not repeat that expensive collision pass. RoboPlan's documented internal transition to a safe fitting mode remains within the selected `roboplan_toppra` backend and is allowed; failure of the backend as a whole does not invoke `simple_trapezoid`.

### Other DimOS surfaces

No streams, transports, module references, blueprint composition, RPC signatures, skills, MCP tools, CLI commands, or generated registry inputs change. Existing preview and execution flows consume the stored `GeneratedPlan.trajectory`. No `all_blueprints.py` regeneration is expected.

## Decisions

### Keep `GeneratedPlan` as the accepted aggregate

The feature does not introduce separately cached geometric-plan and timed-trajectory artifacts because no current caller retimes one plan multiple ways. A narrow internal adapter provides extensibility without changing the public lifecycle.

Alternative: restore frontier's public `GeneratedPlan`/`GeneratedTrajectory`/dispatch split. Rejected because it conflicts with the newer atomic execution architecture and solves no current use case.

### Select one backend for the run

Backend selection is startup configuration. A selected backend's failure fails materialization; no other backend is attempted.

Alternative: fall back to the simple backend after TOPP-RA failure. Rejected because it silently changes timing and stop behavior.

### Scope RoboPlan TOPP-RA to `RoboPlanWorld`

TOPP-RA accepts paths from any planner, but it reuses the authoritative RoboPlan scene and planning groups rather than building and synchronizing a second RoboPlan model for other worlds.

Alternative: make RoboPlan TOPP-RA work with Drake by constructing a shadow RoboPlan scene. Deferred because model, naming, group, and limit synchronization add complexity without a current deployment need.

### Keep geometric post-processing outside this feature

The parametrizer may fit a bounded continuous curve while producing a trajectory. It does not shortcut, simplify, resample, or replace the source waypoint sequence.

Alternative: port frontier's adaptive uniform waypoint decimator. Rejected because RoboPlan provides path-shortcutting and path-specific resampling facilities, and those operations change planning geometry.

### Trust backend collision preservation

RoboPlan owns collision checking introduced by its fitting mode. DimOS validates representation and motion constraints without repeating collision checks.

Alternative: sample and collision-check the complete returned trajectory again in DimOS. Rejected for duplicated cost and competing backend logic.

### Use URDF limits for RoboPlan

RoboPlan scene limits are authoritative. Generic existing DimOS defaults are not injected.

Alternative: wire current scalar/list DimOS fields into RoboPlan. Rejected because their ordering, provenance, defaults, and test coverage are insufficient. Formal globally named per-joint overrides are future work.

## Safety / Simulation / Replay

- Hardware never receives a trajectory that failed canonical validation or whose selected backend failed.
- Missing or invalid URDF motion limits fail rather than selecting generic defaults.
- The TOPP-RA reduction scales are constrained to safe ranges and cannot raise URDF limits.
- Simulation uses the same materialized trajectory path as hardware and is the primary manual QA surface.
- Preview must show the exact stored trajectory later dispatched by execution.
- Replay behavior is unaffected because no stream or replay-data format changes.
- Manual QA should compare simple and TOPP-RA trajectories for the same RoboPlan-world path, check smooth traversal of interior waypoints, and verify explicit failures for missing limits and incompatible startup configuration before any hardware trial.

## Risks / Trade-offs

- Existing robot URDFs may lack acceleration attributes. Mitigation: inventory relevant manipulation models, add valid model limits where authoritative, and test the failure diagnostic.
- RoboPlan's Python binding may expose names or return shapes different from the C++ documentation. Mitigation: add a focused API contract test against pinned 0.5.1 before integrating.
- `LinearBlend` can deviate from the waypoint polyline. Mitigation: bound deviation through backend configuration and rely on RoboPlan's scene collision check/internal safe-mode behavior.
- Finite-difference acceleration validation can be sensitive to output sampling. Mitigation: prefer native accelerations when available and document a numerical tolerance.
- Pinning RoboPlan 0.5.1 upgrades Pinocchio/Coal transitive dependencies. Mitigation: retain the regenerated lockfile and run focused RoboPlan world/planning tests.
- Simple and RoboPlan backends use different absolute-limit sources. Mitigation: document this explicitly; formal unified limit overrides remain separate work.

## Migration / Rollout

1. Land the RoboPlan 0.5.1 pin and compatible lock update.
2. Add configuration, adapter protocol, factory validation, and the wrapped simple backend while preserving its default behavior.
3. Add the RoboPlan TOPP-RA adapter and URDF-limit validation.
4. Route plan materialization through the startup-selected adapter.
5. Update manipulation planning docs with backend compatibility, limit requirements, and configuration examples.
6. Run focused manipulation/RoboPlan tests and manual simulation preview/execute QA before enabling TOPP-RA on hardware.

Rollback is configuration-only while the simple backend remains available. No generated blueprint registry update or persistent data migration is required.

## Open Questions

None. Implementation must verify the exact RoboPlan 0.5.1 Python binding surface and choose numerical validation tolerances, but the intended behavior and ownership boundaries are decided.
