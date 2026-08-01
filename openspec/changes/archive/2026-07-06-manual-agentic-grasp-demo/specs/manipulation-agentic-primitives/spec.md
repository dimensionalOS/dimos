## ADDED Requirements

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
