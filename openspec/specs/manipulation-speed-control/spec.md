## Purpose

Define next-plan manipulation speed control semantics and the minimal Viser slider used to update future trajectory generation without mutating existing generated trajectories.

## Requirements

### Requirement: Next-plan speed setting
The system SHALL treat manipulation motion speed as a next-plan trajectory generation setting.

#### Scenario: Setting speed before planning
- **WHEN** an operator sets a valid speed scale before generating a manipulation plan
- **THEN** the next generated trajectory MUST use that speed scale during parametrization

#### Scenario: Setting speed after planning
- **WHEN** an operator changes the speed scale after a `GeneratedTrajectory` has already been produced
- **THEN** the existing generated trajectory MUST remain unchanged and executable at the speed scale captured when it was generated

#### Scenario: Invalid speed is rejected
- **WHEN** an operator sets a speed scale that is not greater than zero or is greater than one
- **THEN** the system MUST reject the update and keep the previous next-plan speed setting

### Requirement: Generated trajectory speed metadata
The system SHALL record the speed scale used to create each `GeneratedTrajectory`.

#### Scenario: Trajectory records generation speed
- **WHEN** trajectory parametrization produces a `GeneratedTrajectory`
- **THEN** the generated trajectory MUST include the speed scale used for that parametrization invocation

#### Scenario: Current and next speeds can differ
- **WHEN** the next-plan speed setting changes after a plan is generated
- **THEN** consumers MUST be able to inspect the frozen generated trajectory speed separately from the next-plan speed setting when they need that distinction

### Requirement: Option-one backend speed scaling
The system SHALL apply next-plan speed by multiplying configured velocity and acceleration scales by the same speed scale.

#### Scenario: Simple trapezoid backend scales limits
- **WHEN** `simple_trapezoid` parametrizes a plan with `speed_scale = S`
- **THEN** it MUST use `configured_velocity_scale * S` and `configured_acceleration_scale * S` for effective velocity and acceleration limits

#### Scenario: RoboPlan backend scales options
- **WHEN** the RoboPlan backend parametrizes a plan with `speed_scale = S`
- **THEN** it MUST set RoboPlan velocity and acceleration options to configured scales multiplied by `S`

#### Scenario: Reduced speed slows in-house trajectory
- **WHEN** the same geometric plan is parametrized by `simple_trapezoid` with a reduced speed scale
- **THEN** the generated trajectory duration MUST be no shorter than the duration produced with the larger speed scale while preserving the final waypoint

### Requirement: Viser next-plan speed control
The Viser manipulation panel SHALL expose a minimal next-plan speed slider that updates future trajectory generation without mutating the current plan.

#### Scenario: Slider updates next-plan speed
- **WHEN** the operator changes the Viser next-plan speed slider
- **THEN** the panel MUST call the manipulation module speed setter with the selected value

#### Scenario: Existing plan remains fresh
- **WHEN** the next-plan speed slider changes while a fresh plan exists
- **THEN** the panel MUST NOT mark the current plan stale solely because the speed setting changed

#### Scenario: Slider remains visually minimal
- **WHEN** the panel displays the next-plan speed control
- **THEN** it MUST show only the `Next plan speed` slider near the Plan action without additional motion-settings heading, helper copy, or current-plan speed text
