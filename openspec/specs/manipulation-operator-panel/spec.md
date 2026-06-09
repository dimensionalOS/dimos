# manipulation-operator-panel Specification

## Purpose
TBD - created by archiving change add-viser-manipulation-panel. Update Purpose after archive.
## Requirements
### Requirement: Browser-based manipulation operator panel
The system SHALL provide a browser-based manipulation operator panel for supported manipulation stacks that lets an operator inspect robot state, set a motion-planning target, plan, preview, execute, cancel, and reset through a graphical interface.

#### Scenario: Open panel for a running manipulation stack
- **GIVEN** a supported manipulation stack is running and exposes manipulation state through its public control surface
- **WHEN** an operator opens the manipulation operator panel
- **THEN** the panel displays the available robot choices and the selected robot's current manipulation state
- **AND** the panel provides controls for target selection, planning, previewing, execution, cancel, and reset

#### Scenario: Missing manipulation service
- **GIVEN** the panel is opened when no compatible manipulation stack is reachable
- **WHEN** the panel attempts to load robot state
- **THEN** the panel displays a clear unavailable or disconnected state
- **AND** the panel keeps planning and execution controls disabled

### Requirement: Current and target visualization
The system SHALL visually distinguish the live current robot state from the editable target state in the operator panel.

#### Scenario: Display current robot and target ghost
- **GIVEN** a robot is selected and current state is available
- **WHEN** the panel renders the 3D scene
- **THEN** the live current robot is rendered as the solid authoritative state
- **AND** the target robot configuration is rendered as a visually distinct translucent or ghosted target
- **AND** the draggable end-effector target control is visually separate from both robot renderings

#### Scenario: Target is infeasible
- **GIVEN** an operator has changed the target pose or joint target
- **WHEN** the target is infeasible because IK fails, collision checking fails, or the target is otherwise invalid
- **THEN** the panel marks the target controls and target ghost as infeasible using a red visual state
- **AND** the live current robot remains visually unchanged as the current robot state

### Requirement: Target presets
The system SHALL provide one-shot target presets that initialize the editable target without creating an ongoing follow mode.

#### Scenario: Apply current-position preset
- **GIVEN** current robot joints and end-effector pose are available
- **WHEN** the operator selects the Current target preset
- **THEN** the panel sets the target gizmo, target ghost, and joint controls to the current robot state
- **AND** the preset selection does not continue tracking future current-state changes after it is applied

#### Scenario: Apply configured init or home preset
- **GIVEN** the selected robot exposes init or home target data
- **WHEN** the operator applies the Init or Home preset
- **THEN** the panel updates the target ghost, target gizmo, and joint controls to represent that preset target
- **AND** the panel evaluates target feasibility before enabling planning controls

### Requirement: Synchronized Cartesian and joint target controls
The system SHALL keep Cartesian end-effector target controls and joint target controls synchronized for the selected robot.

#### Scenario: Drag end-effector target
- **GIVEN** the operator is viewing a selected robot in the panel
- **WHEN** the operator drags or rotates the end-effector target control
- **THEN** the panel computes a candidate joint target for that end-effector target
- **AND** the joint controls update when the candidate target is feasible
- **AND** the existing plan is marked stale until the operator creates a new plan

#### Scenario: Move joint slider
- **GIVEN** the operator is viewing a selected robot in the panel
- **WHEN** the operator changes a joint target control
- **THEN** the panel updates the target robot visualization and end-effector target pose to match the joint target
- **AND** the existing plan is marked stale until the operator creates a new plan

### Requirement: Non-blocking target feasibility preview
The system SHALL keep the operator panel responsive while evaluating IK, FK, and collision feasibility for target edits.

#### Scenario: High-frequency target edits
- **GIVEN** the operator is dragging an end-effector target control or moving joint controls repeatedly
- **WHEN** feasibility preview requests are generated faster than they can complete
- **THEN** the panel remains responsive to further UI input
- **AND** stale preview results do not overwrite a newer target state
- **AND** planning and execution controls remain disabled while the latest target feasibility is unknown

#### Scenario: Preview result completes
- **GIVEN** the panel has requested feasibility for the latest target
- **WHEN** the latest feasibility result completes successfully
- **THEN** the panel updates the target visualization and controls from that latest result
- **AND** the panel enables planning controls only when the target is feasible and the manipulation state permits planning

### Requirement: Planning, preview, and execution gating
The system SHALL require a fresh feasible target and a fresh plan before enabling execution from the operator panel.

#### Scenario: Create and preview plan
- **GIVEN** a feasible synchronized target exists for the selected robot
- **WHEN** the operator requests a plan
- **THEN** the panel requests a motion plan for the selected robot
- **AND** the panel displays whether planning succeeded or failed
- **AND** a successful plan can be previewed before execution

#### Scenario: Target changes after planning
- **GIVEN** a plan exists for the selected robot
- **WHEN** the operator changes the target pose, target joints, target preset, or selected robot
- **THEN** the panel marks the plan stale
- **AND** the panel disables execution until a new plan is created for the latest feasible target

#### Scenario: Execute fresh plan
- **GIVEN** a fresh plan exists for the selected robot and the current robot state still matches the plan start constraints
- **WHEN** the operator confirms execution
- **THEN** the panel requests execution through the manipulation stack's public execution surface
- **AND** the panel displays trajectory status until execution completes, fails, or is canceled

### Requirement: Safety controls remain available
The system SHALL keep safety-relevant controls visible and usable during planning and execution states.

#### Scenario: Cancel during execution
- **GIVEN** the selected robot is executing a trajectory requested from the panel
- **WHEN** the operator activates Cancel
- **THEN** the panel requests cancellation through the manipulation stack's public cancel surface
- **AND** the panel refreshes and displays the resulting manipulation and trajectory status

#### Scenario: Reset after fault
- **GIVEN** the manipulation stack reports a fault state
- **WHEN** the operator activates Reset Fault
- **THEN** the panel requests reset through the manipulation stack's public reset surface
- **AND** planning and execution controls remain disabled until the refreshed state and target feasibility allow them

### Requirement: Optional dependency and compatibility boundary
The system SHALL keep the Viser manipulation panel optional and compatible with existing manipulation workflows.

#### Scenario: Use manipulation without panel
- **GIVEN** a user installs or runs an existing manipulation workflow without the Viser panel dependencies
- **WHEN** the user runs existing manipulation planning, preview, or execution flows
- **THEN** those existing flows continue to work without requiring the panel
- **AND** existing Meshcat, Rerun, and Python/RPC workflows remain valid

#### Scenario: Panel does not add agent tools
- **GIVEN** the manipulation operator panel is enabled
- **WHEN** agent or MCP tools are listed for the robot stack
- **THEN** the panel does not add or change agent/MCP tools as part of this phase
- **AND** operator UI actions remain separate from agent skill exposure

### Requirement: Stable panel compatibility boundary during internal refactors
The system SHALL preserve the optional manipulation operator panel's public launch, import, configuration, and safety behavior while reorganizing internal implementation details.

#### Scenario: Existing panel launch surfaces continue to work
- **GIVEN** a user has installed the optional Viser manipulation panel dependencies
- **WHEN** the user launches the panel through the existing companion entrypoint or an existing blueprint that includes the panel module
- **THEN** the launch surface accepts the same configuration fields as before
- **AND** the panel connects to the manipulation stack through the same public manipulation control surface

#### Scenario: Existing developer imports remain stable
- **GIVEN** developer code imports the documented Viser panel module, config, or blueprint symbols
- **WHEN** the panel implementation is reorganized into internal collaborators
- **THEN** those documented imports continue to resolve
- **AND** callers do not need to import the internal backend, GUI, scene, controller, or animation helpers

#### Scenario: Safety gates remain unchanged
- **GIVEN** the panel implementation has been reorganized internally
- **WHEN** an operator plans, previews, executes, cancels, or clears a plan
- **THEN** execution remains gated by the same operator opt-in and fresh-plan checks
- **AND** no new robot-facing action is enabled solely because of the internal refactor
