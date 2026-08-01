## ADDED Requirements

### Requirement: Universal agentic manipulation facade
The system SHALL provide an `AgenticManipulationModule` that exposes a universal agent-facing manipulation primitive surface through DimOS skills.

#### Scenario: Facade delegates to manipulation provider
- **WHEN** a caller invokes a primitive skill on `AgenticManipulationModule`
- **THEN** the module MUST delegate the operation to an injected manipulation provider through a DimOS Spec/RPC contract

#### Scenario: Facade remains simulator independent
- **WHEN** `AgenticManipulationModule` is imported or unit tested
- **THEN** the module MUST NOT require Robosuite, runtime sidecar clients, or benchmark-specific APIs

### Requirement: Initial primitive skill surface
The system SHALL expose robot state, joint motion, open gripper, and close gripper as the initial agentic manipulation primitive skills.

#### Scenario: Robot state primitive is available
- **WHEN** a caller invokes the robot state primitive
- **THEN** the system MUST return the injected manipulation provider's robot state result

#### Scenario: Joint motion primitive is available
- **WHEN** a caller invokes the joint motion primitive with a target joint configuration
- **THEN** the system MUST forward the target to the injected manipulation provider's joint motion operation

#### Scenario: Gripper primitives are available
- **WHEN** a caller invokes the open or close gripper primitive
- **THEN** the system MUST forward the command to the injected manipulation provider's gripper operation

### Requirement: Simulator-free primitive tests
The system SHALL include default unit tests for the agentic manipulation facade that do not depend on Robosuite or other heavy simulator runtimes.

#### Scenario: Default test execution
- **WHEN** the default Python test suite runs without Robosuite installed
- **THEN** the agentic manipulation primitive unit tests MUST be able to execute using a fake injected manipulation provider

### Requirement: Script-hosted Robosuite API validation
The system SHALL provide a script-hosted Robosuite validation path that calls the `AgenticManipulationModule` API through the full DimOS manipulation stack.

#### Scenario: Full stack validation
- **WHEN** the Robosuite validation script runs in an environment with the Robosuite sidecar dependencies available
- **THEN** it MUST construct a stack containing the Robosuite sidecar, benchmark runtime SHM adapter, `ControlCoordinator`, `ManipulationModule`, and `AgenticManipulationModule`

#### Scenario: API smoke assertions
- **WHEN** the Robosuite validation script calls the agentic manipulation primitives
- **THEN** it MUST fail if robot state, joint motion, open gripper, or close gripper does not report success through the API path

#### Scenario: Script-hosted artifacts
- **WHEN** the Robosuite validation script completes or fails
- **THEN** it MUST write artifacts describing the episode config, runtime description, resolved runtime plan, API call summary, motor trace, score when available, sidecar log, and cleanup status
