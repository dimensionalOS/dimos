## Context

Linear TCP planning currently samples a straight Cartesian segment into dense IK waypoints and returns those samples as the geometric `GeneratedPlan` path. RoboPlan TOPP-RA consumes a positions-only `JointPath`; public RoboPlan behavior and local observations indicate dense intermediate waypoints can behave like stop-like knots. This makes valid linear TCP motions much slower than comparable free-space plans even when velocity and acceleration scales are already at their maximum allowed values.

The agreed product direction is to keep the practical implementation under RoboPlan trajectory parametrization for now, but structure the internals like a future-splittable trajectory post-processing pipeline. Smoothing is enabled by default for RoboPlan, conservative, validated before use, and non-blocking: if smoothing cannot produce an accepted path, parametrization falls back to the original geometric path.

## Goals / Non-Goals

**Goals:**

- Add minimal optional path-constraint metadata to geometric plan artifacts.
- Preserve linear TCP intent by carrying TCP-line constraints from linear Cartesian planning into trajectory post-processing.
- Introduce a staged RoboPlan post-processing pipeline inside the existing parametrization backend boundary.
- Provide conservative default smoothing that reduces dense stop-like waypoints when validation accepts the refined path.
- Make smoothing failure non-blocking: fall back to original path and preserve the existing slow-but-valid behavior.
- Keep the structure easy to split later into independent post-processing, validation, retiming, and execution smoothing stages.

**Non-Goals:**

- Do not build a full Tesseract/TrajOpt/Drake-style constrained trajectory optimizer in this change.
- Do not add Ruckig or another jerk-limited external dependency in this change.
- Do not guarantee constant TCP speed along linear paths in this change.
- Do not alter the public planner method signatures unless needed for metadata propagation.
- Do not make smoothing a correctness gate for plan validity.

## Decisions

### Keep the facade as trajectory parametrization, but model internals as a pipeline

RoboPlan smoothing will run from `RoboPlanWorld.parametrize(...)` through internal pipeline-style stages before TOPP-RA conversion. The first implementation can remain private to the RoboPlan backend, but the code should separate stage responsibilities: input interpretation, preprocessing, validation, retiming, conversion, and fallback.

Alternatives considered:

- Put smoothing directly inside linear TCP planning. Rejected because mature stacks separate geometric planning from post-processing/retiming, and smoothing should also be available for other dense geometric paths.
- Add a top-level post-processing abstraction immediately. Deferred because the current implementation only needs the RoboPlan backend and the practical boundary should stay under parametrization for now.

### Add optional path-constraint metadata to plan artifacts

`GeneratedPlan` should gain an optional metadata field that declares constraints downstream post-processing must preserve. `PlanningResult` should carry the same optional metadata so planners can return it and `ManipulationModule` can preserve it when constructing `GeneratedPlan`.

The initial constraint model should be minimal and typed. For linear TCP it should identify the constrained planning group/TCP frame and the line segment from start pose to target pose, plus translational and rotational tolerance values used for validation.

Alternatives considered:

- Infer linear constraints from path shape. Rejected because inference is brittle and cannot distinguish intentional straight-line constraints from merely dense joint-space samples.
- Delay metadata. Rejected because production-safe smoothing for linear TCP needs explicit Cartesian validation, not only joint-space deviation checks.

### Validate refined paths before retiming them

The pipeline should only accept a refined geometric path if validation succeeds. Baseline validation should cover endpoints, joint limits, collision, waypoint count/order sanity, and maximum joint-space deviation from the original path. When path-constraint metadata is present, validation must also check those declared constraints, including linear TCP Cartesian deviation for linear TCP plans.

Alternatives considered:

- Trust waypoint simplification because it only selects original waypoints. Rejected because reducing waypoints can change the interpolated path that TOPP-RA/spline fitting follows between retained knots.
- Validate only after time parametrization. Rejected as the first gate because geometric invalidity should be detected before relying on retimed trajectories.

### Start with adaptive waypoint simplification as one pipeline stage

The first concrete preprocessing stage should reduce dense waypoint chains by preserving endpoints and adaptively selecting fewer original waypoints. If validation fails, the stage retries with a more conservative selection that preserves more waypoints. It must not invent new IK states in the first version.

This is intentionally only a stage, not the architecture. Future stages may add spline fitting, IK refinement, or constraint-aware optimization after the same validation contract exists.

Alternatives considered:

- Fit new joint-space splines immediately. Deferred because generated joint states can silently violate Cartesian path and collision constraints unless paired with stronger validation and resampling.
- Leave all waypoints intact and rely on RoboPlan spline modes. Rejected because all current modes remain slow on dense linear TCP paths.

### Smoothing failure is non-blocking

If every smoothing attempt fails validation or preprocessing raises a recoverable error, the backend must parametrize the original path. The original path can still fail if RoboPlan TOPP-RA itself cannot parametrize it, but smoothing failure alone must not fail the plan.

Alternatives considered:

- Strict smoothing mode. Rejected for default behavior because operators need continuity and the worst expected smoothing outcome is the current slow trajectory.

### Enable RoboPlan smoothing by default, conservatively

RoboPlan smoothing should be enabled by default with conservative thresholds. Configuration should allow disabling smoothing and tuning attempts, minimum waypoint count, and deviation tolerances, but the default should preserve correctness and fallback behavior.

Alternatives considered:

- Opt-in first. Rejected because the user-facing problem is the default behavior of linear TCP plans crawling despite full speed settings.

## Risks / Trade-offs

- Refined path violates linear TCP intent → Mitigation: require path-constraint metadata for linear TCP plans and validate Cartesian deviation before accepting the refined path.
- Refined path passes sparse validation but collides between samples → Mitigation: collision-check retained/refined samples now and keep validation structure ready for denser resampling checks; fallback to original path on failure.
- Conservative defaults provide little speedup → Mitigation: make attempts and tolerances configurable while preserving correctness tolerances.
- Internal pipeline becomes a permanent hidden abstraction → Mitigation: keep stage boundaries and names explicit so future work can split it out without redesigning behavior.
- Additional metadata creates model churn across tests → Mitigation: make metadata optional and default to `None`, so existing free-space plans remain valid.

## Migration Plan

1. Add optional metadata/config model fields with safe defaults.
2. Propagate metadata from `PlanningResult` to `GeneratedPlan` without changing existing no-metadata behavior.
3. Attach linear TCP constraints in RoboPlan linear Cartesian planning results.
4. Add RoboPlan post-processing pipeline internals behind the existing parametrization backend.
5. Validate and fallback on smoothing failure before calling TOPP-RA on the accepted path.
6. Update tests to cover metadata propagation, linear TCP validation, smoothing acceptance, adaptive retry, and fallback to the original path.

Rollback is straightforward: disable RoboPlan smoothing in configuration or remove the preprocessing stage while leaving optional metadata harmlessly unused.

## Open Questions

- Exact default tolerance values should be chosen from existing linear TCP tolerances where possible, or conservatively introduced in config if no single source exists.
- Whether future hardware execution should add a final jerk-limited smoothing stage remains out of scope for this change.
