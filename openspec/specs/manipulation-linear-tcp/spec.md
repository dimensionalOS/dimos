## Purpose

Define public manipulation APIs and Viser behavior for explicit linear TCP path planning from pose targets.

## Requirements

### Requirement: ManipulationModule exposes intentful linear and relative pose planning APIs
The system SHALL expose public `ManipulationModule` methods for explicit linear TCP path planning and relative pose-target planning while preserving the existing standard pose-target planning API.

#### Scenario: Standard pose planning remains unchanged
- **WHEN** a caller invokes `plan_to_pose_targets(...)`
- **THEN** the system SHALL keep the existing behavior of solving IK for the absolute pose targets and planning a selected joint path
- **AND** the call SHALL NOT require planner-level Cartesian path support

#### Scenario: Absolute linear pose planning uses planner linear Cartesian mode
- **WHEN** a caller invokes `plan_linear_to_pose_targets(pose_targets, auxiliary_groups=..., timeout=...)`
- **THEN** the system SHALL resolve the selected planning groups from `pose_targets` and `auxiliary_groups`
- **AND** it SHALL call the planner-level absolute Cartesian method with `path_mode="linear"`
- **AND** it SHALL store the resulting plan when planning succeeds

#### Scenario: Relative free pose planning uses relative Cartesian planner mode
- **WHEN** a caller invokes `plan_relative_to_pose_targets(delta_targets, auxiliary_groups=..., timeout=...)`
- **THEN** the system SHALL resolve the selected planning groups from `delta_targets` and `auxiliary_groups`
- **AND** it SHALL call the planner-level relative Cartesian method with `path_mode="free"`
- **AND** it SHALL store the resulting plan when planning succeeds

#### Scenario: Relative linear pose planning uses relative linear Cartesian planner mode
- **WHEN** a caller invokes `plan_linear_relative_to_pose_targets(delta_targets, auxiliary_groups=..., timeout=...)`
- **THEN** the system SHALL resolve the selected planning groups from `delta_targets` and `auxiliary_groups`
- **AND** it SHALL call the planner-level relative Cartesian method with `path_mode="linear"`
- **AND** it SHALL store the resulting plan when planning succeeds

#### Scenario: Explicit public methods fail without fallback
- **WHEN** one of the explicit linear or relative public planning methods receives a non-success `PlanningResult`
- **THEN** the method SHALL return `False`
- **AND** it SHALL expose an explanatory module error
- **AND** it SHALL NOT silently fall back to IK plus joint-space planning

### Requirement: Viser exposes linear TCP path planning as a next-plan option
The Viser manipulation panel SHALL provide a simple `Linear TCP path` option that affects the next Plan action without changing existing standard planning behavior.

#### Scenario: Linear TCP checkbox unchecked uses standard planning
- **WHEN** the `Linear TCP path` checkbox is unchecked and the user clicks Plan
- **THEN** Viser SHALL keep the existing behavior of planning from IK-evaluated joint targets through `plan_to_joint_targets(...)`

#### Scenario: Linear TCP checkbox checked uses absolute pose targets
- **WHEN** the `Linear TCP path` checkbox is checked and the user clicks Plan with feasible active pose targets
- **THEN** Viser SHALL call `plan_linear_to_pose_targets(...)` using the current active absolute pose targets
- **AND** it SHALL pass the current auxiliary planning group IDs

#### Scenario: Linear TCP checkbox does not add relative UI
- **WHEN** the Viser panel renders phase-1 linear TCP planning controls
- **THEN** it SHALL NOT expose relative-motion controls
- **AND** it SHALL NOT expose a broader Cartesian free/linear planning selector

#### Scenario: Missing pose target fails recoverably
- **WHEN** the `Linear TCP path` checkbox is checked and Plan is requested without active pose targets
- **THEN** Viser SHALL report a recoverable planning error
- **AND** it SHALL NOT attempt to plan from joint slider targets as a linear TCP path

### Requirement: Viser preserves current plan recipe independently from next-plan settings
The Viser panel SHALL track the recipe used to create the current fresh plan separately from the checkbox setting used for future Plan actions.

#### Scenario: Changing checkbox does not stale current plan
- **WHEN** a fresh plan exists
- **AND** the user toggles the `Linear TCP path` checkbox
- **THEN** the fresh plan SHALL remain fresh
- **AND** preview and execute eligibility SHALL continue to be based on the current plan state and normal freshness checks

#### Scenario: Current plan records standard recipe
- **WHEN** a standard plan succeeds through `plan_to_joint_targets(...)`
- **THEN** the current plan state SHALL record the plan recipe as standard

#### Scenario: Current plan records linear TCP recipe
- **WHEN** a linear TCP plan succeeds through `plan_linear_to_pose_targets(...)`
- **THEN** the current plan state SHALL record the plan recipe as linear TCP

#### Scenario: Status distinguishes current and next plan recipe
- **WHEN** Viser displays plan status
- **THEN** it SHALL expose the recipe used by the current plan
- **AND** it SHALL not imply that toggling the next-plan checkbox changed the current plan recipe

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
