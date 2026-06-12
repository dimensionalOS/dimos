## MODIFIED Requirements

### Requirement: Optional planning visualization remains available independently from world planning behavior

The manipulation planning stack SHALL expose visualization behavior through a dedicated planning visualization capability without requiring backend-agnostic planning code to depend on visualization methods, and SHALL allow callers to select an available visualization backend independently from the world planning backend.

#### Scenario: Visualization enabled
- **GIVEN** a manipulation planning stack with visualization enabled
- **WHEN** a caller asks for the visualization URL
- **THEN** the stack returns the active visualization URL when one is available
- **AND** planning, collision checking, and kinematics callers can continue to depend on world planning behavior separately from visualization behavior

#### Scenario: Visualization disabled
- **GIVEN** a manipulation planning stack with visualization disabled
- **WHEN** visualization publishing or URL lookup is requested
- **THEN** the request completes without disrupting planning behavior
- **AND** URL lookup returns no URL

#### Scenario: Meshcat remains the default compatibility backend
- **GIVEN** a manipulation planning stack that enables visualization without explicitly selecting a backend
- **WHEN** the stack starts
- **THEN** the existing Meshcat-backed visualization behavior remains available when supported by the world backend
- **AND** existing callers using the previous visualization enablement behavior do not need to select Viser

#### Scenario: Viser is selected independently from the world backend
- **GIVEN** a manipulation planning stack using the existing world planning backend
- **AND** the Viser visualization backend is explicitly selected
- **WHEN** the stack starts visualization
- **THEN** visualization behavior is provided by Viser instead of the world backend's Meshcat visualization
- **AND** world planning, collision checking, and kinematics behavior continue to use the configured world backend

#### Scenario: Unknown visualization backend is rejected
- **GIVEN** a manipulation planning stack configured with an unsupported manipulation visualization backend
- **WHEN** the stack attempts to create visualization
- **THEN** the stack reports the unsupported backend clearly
- **AND** it lists or documents the supported manipulation visualization backend choices
