## ADDED Requirements

### Requirement: Explicit manipulation backend selection
DimOS SHALL allow manipulation stacks to select the world backend explicitly and independently from the planner selection.

#### Scenario: Default backend remains Drake
- **GIVEN** a manipulation stack with no RoboPlan backend option set
- **WHEN** the stack starts
- **THEN** DimOS uses the existing Drake-backed manipulation world behavior
- **AND** existing planner and kinematics defaults remain compatible.

#### Scenario: RoboPlan world selected explicitly
- **GIVEN** a manipulation stack configured with the RoboPlan world backend
- **WHEN** the stack starts with a compatible planner selection
- **THEN** DimOS constructs a RoboPlan-backed manipulation world
- **AND** the planner selection is not changed implicitly.

### Requirement: Generic planner compatibility with RoboPlan world
DimOS SHALL support collision-checked joint-path planning with the generic RRT planner when the RoboPlan world backend is selected.

#### Scenario: Generic RRT plans through RoboPlan collision checks
- **GIVEN** a finalized RoboPlan-backed manipulation world with a supported robot model and supported obstacles
- **AND** the generic RRT planner is selected
- **WHEN** a joint-space plan is requested between valid start and goal joint states
- **THEN** DimOS returns a planning result produced through the generic planner path
- **AND** sampled configurations and path edges are checked against the RoboPlan-backed world for collisions.

#### Scenario: Unsupported planning-critical input fails before planning
- **GIVEN** a RoboPlan-backed manipulation world configuration containing an unsupported robot, joint, obstacle, or collision feature
- **WHEN** the stack is initialized or the unsupported input is added
- **THEN** DimOS rejects the input with an actionable error
- **AND** DimOS MUST NOT silently ignore or approximate the unsupported planning-critical feature.

### Requirement: RoboPlan native planner requires RoboPlan world
DimOS SHALL allow RoboPlan-native planning only when the RoboPlan world backend is selected.

#### Scenario: Valid native RoboPlan combination
- **GIVEN** a manipulation stack configured with the RoboPlan world backend
- **AND** the RoboPlan-native planner is selected
- **WHEN** the stack starts
- **THEN** DimOS accepts the configuration
- **AND** native RoboPlan planning uses the RoboPlan-backed world representation.

#### Scenario: Invalid native RoboPlan combination fails fast
- **GIVEN** a manipulation stack configured with a non-RoboPlan world backend
- **AND** the RoboPlan-native planner is selected
- **WHEN** the stack starts
- **THEN** DimOS fails startup with an error explaining that the RoboPlan-native planner requires the RoboPlan world backend.

### Requirement: Backend combination validation
DimOS SHALL validate world, planner, and kinematics backend combinations during startup before accepting plan requests.

#### Scenario: Incompatible kinematics rejected
- **GIVEN** a manipulation stack configured with the RoboPlan world backend
- **AND** a kinematics implementation that requires the Drake world backend
- **WHEN** the stack starts
- **THEN** DimOS fails startup with an actionable incompatibility error.

#### Scenario: Unknown backend name rejected
- **GIVEN** a manipulation stack configured with an unknown world, planner, or kinematics backend name
- **WHEN** the stack starts
- **THEN** DimOS fails startup with an actionable error listing or indicating supported choices.

### Requirement: Optional RoboPlan dependency behavior
DimOS SHALL keep RoboPlan dependencies optional and provide clear installation guidance when the RoboPlan backend is requested without them.

#### Scenario: RoboPlan backend requested without dependencies
- **GIVEN** an environment without RoboPlan packages installed
- **AND** a manipulation stack configured with the RoboPlan world backend
- **WHEN** the stack starts
- **THEN** DimOS fails with an error that identifies the missing RoboPlan dependency path
- **AND** the error explains which optional installation path is required.

#### Scenario: Non-RoboPlan stacks do not require RoboPlan imports
- **GIVEN** an environment without RoboPlan packages installed
- **WHEN** a manipulation stack starts with the default Drake world backend
- **THEN** DimOS does not require RoboPlan packages to be importable.

### Requirement: Explicit unsupported query behavior
DimOS SHALL report unsupported non-critical RoboPlan world query methods explicitly rather than returning misleading values.

#### Scenario: Unsupported query method called
- **GIVEN** a RoboPlan-backed manipulation world
- **AND** a non-critical query method has no verified RoboPlan semantic equivalent
- **WHEN** the method is called
- **THEN** DimOS raises an explicit unsupported-method error
- **AND** planning-critical collision checks remain unaffected.
