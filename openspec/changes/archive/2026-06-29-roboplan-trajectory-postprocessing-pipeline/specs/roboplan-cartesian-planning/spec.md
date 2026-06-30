## ADDED Requirements

### Requirement: RoboPlan linear Cartesian planning returns constraint metadata
RoboPlan linear Cartesian planning SHALL return enough path-constraint metadata for downstream trajectory post-processing to validate straight-line TCP preservation.

#### Scenario: Linear Cartesian result includes TCP segment metadata
- **WHEN** RoboPlan-backed Cartesian planning succeeds with `path_mode="linear"`
- **THEN** the returned planning result MUST include metadata for the constrained planning group, constrained TCP frame, start TCP pose, target TCP pose, and Cartesian deviation tolerances

#### Scenario: Linear Cartesian metadata matches validated path
- **WHEN** RoboPlanWorld validates a linear Cartesian path before returning success
- **THEN** the returned path-constraint metadata MUST describe the same start and target TCP segment that was validated

#### Scenario: Free Cartesian result omits linear constraint metadata
- **WHEN** RoboPlan-backed Cartesian planning succeeds with `path_mode="free"`
- **THEN** the returned planning result MUST NOT include linear TCP path-constraint metadata

### Requirement: RoboPlan post-processing can validate linear TCP constraints
RoboPlanWorld SHALL provide the backend operations needed for trajectory post-processing to validate linear TCP constraints against candidate geometric paths.

#### Scenario: Candidate path is checked against TCP line
- **WHEN** RoboPlan trajectory post-processing validates a candidate path with linear TCP constraint metadata
- **THEN** RoboPlanWorld MUST evaluate the constrained TCP poses along the candidate path and compare them with the declared Cartesian line tolerance

#### Scenario: Candidate path cannot be validated
- **WHEN** RoboPlanWorld cannot evaluate the constrained TCP poses or constraint metadata is inconsistent with the selected planning groups
- **THEN** the candidate path MUST be rejected and non-blocking smoothing fallback MUST apply
