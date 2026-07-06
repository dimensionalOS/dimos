## Purpose

Define a simulator-independent agent-facing manipulation primitive facade and its validation path through the DimOS manipulation stack.

## Requirements

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

### Requirement: Grasp-capable agentic manipulation facade
The system SHALL provide a grasp-capable agent-facing manipulation facade separate from the universal manipulation facade for blueprints that include perception and grasp-generation dependencies.

#### Scenario: Universal facade remains dependency-light
- **WHEN** a blueprint only provides the universal manipulation provider dependency
- **THEN** `AgenticManipulationModule` MUST remain usable without requiring object-scene registration, GPD, or grasp orchestration providers

#### Scenario: Grasp facade exposes additional dependencies
- **WHEN** a blueprint wires the grasp-capable facade
- **THEN** the facade MUST be able to delegate object scanning, grasp generation, and motion/gripper execution through injected DimOS Spec/RPC dependencies

### Requirement: Round 1 grasp demo skill surface
The grasp-capable facade SHALL expose the Round 1 agent-facing skills needed for the manual sim grasp demo and useful motion debugging.

#### Scenario: Scene and grasp skills are available
- **WHEN** a caller inspects the grasp-capable facade skill schema
- **THEN** `scan_objects(...)`, `generate_grasps(...)`, and `execute_grasp(candidate_index=0)` MUST be available as skills

#### Scenario: Motion and gripper skills are available
- **WHEN** a caller inspects the grasp-capable facade skill schema
- **THEN** `move_to_pose(...)`, `move_relative(...)`, `move_along_axis(...)`, `go_home()`, `open_gripper()`, `close_gripper()`, and `set_gripper(...)` MUST be available as skills

### Requirement: Cached Grasp candidate execution
The grasp-capable facade SHALL execute a selected cached Grasp candidate without implicitly rescanning or regenerating grasps.

#### Scenario: Execute selected cached candidate
- **WHEN** `generate_grasps(...)` has cached one or more Grasp candidates and the caller invokes `execute_grasp(candidate_index)` with a valid index
- **THEN** the facade MUST select that cached candidate and command a conservative execution sequence that opens the gripper, moves to pregrasp, moves to the grasp pose, closes the gripper, and lifts or retracts

#### Scenario: Execute fails without cached candidates
- **WHEN** a caller invokes `execute_grasp(...)` before any Grasp candidates are cached
- **THEN** the facade MUST return a clear failure explaining that `generate_grasps(...)` must be called first
- **AND** it MUST NOT implicitly call `scan_objects(...)` or `generate_grasps(...)`

#### Scenario: Execute rejects invalid candidate index
- **WHEN** a caller invokes `execute_grasp(candidate_index)` with an index outside the cached Grasp candidate range
- **THEN** the facade MUST return a clear invalid-input failure without commanding robot motion

### Requirement: World-frame relative motion skills
The agent-facing relative motion skills SHALL default to world-frame Cartesian deltas for Round 1.

#### Scenario: Relative motion defaults to world frame
- **WHEN** a caller invokes `move_relative(dx, dy, dz)` without a frame argument
- **THEN** the requested translation MUST be interpreted in the world/base frame

#### Scenario: Axis motion defaults to world frame
- **WHEN** a caller invokes `move_along_axis(axis, distance)` without a frame argument
- **THEN** the requested axis motion MUST be interpreted in the world/base frame

#### Scenario: Unsupported relative frame fails clearly
- **WHEN** a caller invokes relative motion with a frame that the underlying planner does not support
- **THEN** the facade MUST return a clear unsupported-frame failure instead of silently changing frames
