## ADDED Requirements

### Requirement: Linear TCP plans declare path-constraint metadata
Successful linear TCP plans SHALL carry path-constraint metadata that declares the TCP-line constraint downstream post-processing must preserve.

#### Scenario: Absolute linear TCP plan records constraint metadata
- **WHEN** `plan_linear_to_pose_targets(...)` succeeds for a linear TCP path
- **THEN** the stored `GeneratedPlan` MUST include path-constraint metadata describing the constrained TCP frame, start pose, target pose, and Cartesian deviation tolerances

#### Scenario: Relative linear TCP plan records resolved absolute constraint metadata
- **WHEN** `plan_linear_relative_to_pose_targets(...)` succeeds for a linear TCP path
- **THEN** the stored `GeneratedPlan` MUST include path-constraint metadata describing the resolved absolute start-to-target TCP segment and Cartesian deviation tolerances

#### Scenario: Standard and free plans do not claim linear TCP constraints
- **WHEN** standard pose planning or free Cartesian planning succeeds
- **THEN** the stored `GeneratedPlan` MUST NOT declare linear TCP path constraints unless that planner explicitly validated those constraints

### Requirement: Linear TCP post-processing preserves line tolerance
Post-processing of a linear TCP plan SHALL preserve the declared straight-line TCP constraint within configured Cartesian tolerance.

#### Scenario: Smoothed linear TCP path remains within tolerance
- **WHEN** trajectory post-processing accepts a smoothed path for a linear TCP plan
- **THEN** the accepted path MUST keep the constrained TCP within the declared translational and rotational tolerances of the start-to-target Cartesian segment

#### Scenario: Smoothed linear TCP path exceeds tolerance
- **WHEN** a smoothed candidate for a linear TCP plan exceeds declared Cartesian tolerance
- **THEN** the candidate MUST be rejected and non-blocking smoothing fallback MUST apply
