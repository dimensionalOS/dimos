## ADDED Requirements

### Requirement: Project runtime placement supports real GPD grasp detector Modules
The system SHALL support placing a real GPD grasp detector Module into a Python project runtime while preserving normal DimOS blueprint wiring and worker lifecycle behavior.

#### Scenario: Blueprint places GPD generator into project runtime
- **WHEN** the GPD MuJoCo demo blueprint is built
- **THEN** the GPD grasp detector Module is assigned to the named GPD Python project runtime and other demo Modules remain in their normal runtime placements

#### Scenario: GPD placement uses existing worker protocol
- **WHEN** the placed GPD generator receives lifecycle calls, stream wiring, or RPC calls
- **THEN** those operations use the same DimOS worker protocol semantics as other project-runtime Modules

#### Scenario: GPD demo remains opt-in
- **WHEN** existing xArm perception or VGN demo blueprints are loaded
- **THEN** they do not implicitly include the GPD grasp generator or prepare/launch its project runtime
