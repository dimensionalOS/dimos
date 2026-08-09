## ADDED Requirements

### Requirement: One logical robot per manipulation module

The manipulation system SHALL expose exactly one logical robot backed by one prepared robot model and SHALL NOT require a robot identifier in planning, state, execution, or visualization APIs.

#### Scenario: Plan for one arm
- **GIVEN** a manipulation module configured with one prepared arm model
- **WHEN** a caller requests motion for an available planning group
- **THEN** the module plans without a robot selector
- **AND** returned state and trajectory data contain canonical joint names rather than robot IDs

#### Scenario: Use a prepared bimanual model
- **GIVEN** one prepared model containing left-arm and right-arm joints
- **WHEN** a caller plans for either arm or both arms
- **THEN** the system treats the model as one robot
- **AND** the caller selects the controlled subset through planning groups only

### Requirement: Static prepared model input

The manipulation system SHALL accept a statically prepared URDF and optional SRDF as the canonical model input for the logical robot. It SHALL NOT require runtime composition of component models.

#### Scenario: Start a dual-arm mock blueprint
- **GIVEN** a pre-generated dual-arm URDF/SRDF fixture
- **WHEN** the blueprint starts its manipulation module
- **THEN** the module loads the fixture as one model
- **AND** it does not assemble two arm models during startup

#### Scenario: Model input is invalid
- **GIVEN** a configured model whose canonical joints or planning groups cannot be loaded
- **WHEN** the manipulation module initializes
- **THEN** startup fails with a diagnostic identifying the invalid model or group
- **AND** no partial manipulation service becomes available

### Requirement: Canonical joint namespace

The system SHALL use the same canonical joint names in the prepared model, planning groups, live state, trajectories, control interfaces, gripper handling, and visualization.

#### Scenario: Execute a left-arm trajectory
- **GIVEN** the prepared model contains `left/j1`
- **WHEN** planning produces and execution consumes a trajectory for that joint
- **THEN** both trajectory and control-facing state use `left/j1`
- **AND** no caller-visible local/global or robot-prefix conversion occurs

#### Scenario: A backend needs a restricted native name
- **GIVEN** a planning backend cannot accept the canonical name syntax
- **WHEN** it loads and evaluates the model
- **THEN** any encoding is private and reversible inside that backend boundary
- **AND** all public inputs and outputs still use the canonical name

### Requirement: Planning groups select controllable subsets

The system SHALL define planning groups by stable group name, canonical joint names, and group-specific base and tip links. It SHALL support several groups and several end-effectors within one robot.

#### Scenario: Select one end effector
- **GIVEN** a bimanual model with `left_arm` and `right_arm` planning groups
- **WHEN** a caller selects `left_arm`
- **THEN** planning controls the left group’s declared joints and uses its declared base and tip links
- **AND** no model-wide singular end-effector property is consulted

#### Scenario: Plan coordinated dual-arm motion
- **GIVEN** a group containing canonical joints from both arms
- **WHEN** a caller requests a coordinated plan for that group
- **THEN** planning operates on one model state and the combined joint subset
- **AND** it does not split the request by robot identity

### Requirement: Direct state and trajectory flow

Joint state, trajectories, execution, collision queries, forward kinematics, and visualization SHALL operate on one logical model and canonical names without per-robot maps or trajectory splitting.

#### Scenario: Execute a trajectory spanning two groups
- **GIVEN** a valid trajectory containing left and right canonical joints
- **WHEN** manipulation submits it for execution
- **THEN** it preserves one canonical trajectory representation
- **AND** execution does not manufacture per-robot targets

#### Scenario: Visualize current state
- **GIVEN** current state containing canonical joints from several connections
- **WHEN** visualization renders the robot
- **THEN** it applies one joint-state set to one logical model
- **AND** it does not require a map keyed by robot ID

### Requirement: Grippers are controllable joints

The manipulation system SHALL represent every gripper degree of freedom as an ordinary canonical joint with the same state, limits, units, planning-group selection, and control-interface conventions as other joints.

#### Scenario: Select a gripper group
- **GIVEN** a group containing `left/gripper`
- **WHEN** a caller commands the gripper through its public manipulation API
- **THEN** the control claim targets that joint’s declared interface
- **AND** manipulation does not derive a task name or limits from hardware identity

#### Scenario: Control a multi-DOF gripper
- **GIVEN** a gripper group with multiple canonical joints
- **WHEN** its task commands native, normalized, or sweep positions
- **THEN** every degree of freedom participates in arbitration
- **AND** the task retains the reference-pose rules defined by the gripper API
