## 1. Path Constraint Metadata

- [x] 1.1 Add typed optional path-constraint metadata models for geometric plans, including a minimal linear TCP constraint model with constrained group/frame, start pose, target pose, and Cartesian tolerances.
- [x] 1.2 Add optional path-constraint metadata fields to `PlanningResult` and `GeneratedPlan` with safe `None` defaults.
- [x] 1.3 Preserve path-constraint metadata when `ManipulationModule` converts successful `PlanningResult` values into stored `GeneratedPlan` artifacts.
- [x] 1.4 Add unit tests proving existing no-metadata plans remain valid and metadata is preserved into parametrization.

## 2. Linear TCP Metadata Propagation

- [x] 2.1 Attach linear TCP constraint metadata to successful RoboPlan `path_mode="linear"` Cartesian planning results.
- [x] 2.2 Ensure RoboPlan `path_mode="free"` and standard joint/pose planning results do not claim linear TCP constraints unless explicitly validated.
- [x] 2.3 Add tests for absolute and relative linear TCP planning metadata, including resolved start-to-target segment data.
- [x] 2.4 Add tests that free Cartesian and standard plans leave path-constraint metadata unset.

## 3. RoboPlan Post-Processing Pipeline Structure

- [x] 3.1 Add RoboPlan trajectory-parametrization smoothing configuration with conservative defaults, disable switch, minimum waypoint threshold, retry count, and deviation tolerances.
- [x] 3.2 Refactor RoboPlan parametrization internals into separable pipeline responsibilities for input interpretation, preprocessing, validation, TOPP-RA retiming, and fallback handling.
- [x] 3.3 Implement the first preprocessing stage as adaptive waypoint simplification that preserves original endpoints and selects only original waypoints.
- [x] 3.4 Implement adaptive-conservative retry behavior that preserves more original waypoints after validation failure.

## 4. Validation and Fallback

- [x] 4.1 Implement generic candidate validation for selected-joint compatibility, endpoints, joint limits, collision acceptability, and configured joint-space deviation.
- [x] 4.2 Implement linear TCP candidate validation that evaluates constrained TCP poses and rejects candidates outside declared translational or rotational tolerance.
- [x] 4.3 Ensure rejected smoothing candidates are never passed to RoboPlan TOPP-RA.
- [x] 4.4 Ensure smoothing/preprocessing failures fall back to parametrizing the original geometric path and do not fail `GeneratedTrajectory` by themselves.
- [x] 4.5 Preserve existing RoboPlan TOPP-RA failure reporting when fallback to the original path still cannot be parametrized.

## 5. Tests and Documentation

- [x] 5.1 Add RoboPlan parametrization tests for smoothing enabled by default, smoothing disabled, short-path skip, accepted smoothing, adaptive retry, and fallback to original path.
- [x] 5.2 Add linear TCP tests proving accepted smoothing preserves declared Cartesian line tolerance and violating candidates trigger fallback.
- [x] 5.3 Update relevant manipulation capability docs or examples to mention conservative default RoboPlan smoothing and non-blocking fallback.
- [x] 5.4 Run focused tests for RoboPlan world, manipulation module, linear TCP planning, Viser preview/execution behavior, and trajectory parametrization.
- [x] 5.5 Run formatting and lint checks on modified Python files and OpenSpec validation/status checks for this change.
