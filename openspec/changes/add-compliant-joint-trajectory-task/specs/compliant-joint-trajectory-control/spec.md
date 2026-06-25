## ADDED Requirements

### Requirement: Compliant joint trajectory execution

DimOS SHALL provide a joint trajectory execution mode that follows a nominal joint trajectory while allowing bounded compliant deviation from the reference under tracking resistance or contact.

#### Scenario: Free-space trajectory behaves like nominal tracking
- **GIVEN** a compliant joint trajectory is active for a set of manipulator joints
- **AND** the measured joint positions track the nominal trajectory within the configured deadband or tolerance
- **WHEN** the controller computes the next joint command
- **THEN** the commanded joint positions SHALL remain close to the nominal trajectory reference
- **AND** the compliant offset SHALL remain near zero within configured tolerance.

#### Scenario: Contact or resistance produces bounded backoff
- **GIVEN** a compliant joint trajectory is active for a set of manipulator joints
- **AND** measured joint feedback indicates sustained resistance to the nominal motion
- **WHEN** the controller computes subsequent joint commands
- **THEN** the commanded joint positions SHALL include a compliant offset that reduces continued fighting against the resistance
- **AND** the compliant offset and offset velocity SHALL remain within configured bounds.

#### Scenario: Existing rigid trajectory behavior remains available
- **GIVEN** a robot stack or test selects the existing rigid joint trajectory behavior
- **WHEN** a joint trajectory is executed
- **THEN** DimOS SHALL execute the trajectory without applying compliant offset behavior
- **AND** existing rigid trajectory users SHALL NOT be required to change their configuration.

### Requirement: Position-servo compatibility

The compliant joint trajectory capability SHALL produce position-servo joint commands in its initial supported mode and MUST NOT require torque command output from the manipulator hardware path.

#### Scenario: Hardware supports position commands but not torque commands
- **GIVEN** manipulator hardware can read joint positions and accept position-servo commands
- **AND** the hardware does not expose torque command output through the generic manipulator command path
- **WHEN** a compliant joint trajectory is selected
- **THEN** the controller SHALL be able to operate using position-servo commands
- **AND** it SHALL NOT require torque-mode hardware routing.

#### Scenario: Effort feedback is unavailable or disabled
- **GIVEN** manipulator joint positions are available
- **AND** joint effort feedback is unavailable, placeholder-valued, or not configured for use
- **WHEN** a compliant joint trajectory is executed
- **THEN** DimOS SHALL still compute compliance from configured non-effort feedback behavior
- **AND** it SHALL NOT infer reliable effort feedback solely from zero-valued effort readings.

#### Scenario: Effort feedback is explicitly enabled
- **GIVEN** a hardware or simulation backend provides reliable joint effort feedback
- **AND** effort feedback use is explicitly enabled in the compliant trajectory configuration
- **WHEN** a compliant joint trajectory is executed
- **THEN** DimOS MAY use effort feedback as part of its resistance estimate
- **AND** safety bounds SHALL still limit the resulting commanded offsets.

### Requirement: Safe bounded behavior

DimOS SHALL expose and enforce safety bounds for compliant trajectory execution, including maximum compliant offset and maximum offset velocity per controlled joint.

#### Scenario: Offset reaches configured limit
- **GIVEN** a compliant joint trajectory is active
- **AND** the computed compliant offset exceeds a configured joint offset limit
- **WHEN** the command is emitted
- **THEN** DimOS SHALL clamp the commanded offset to the configured limit
- **AND** the saturation condition SHALL be observable to tests or diagnostics.

#### Scenario: Invalid timing input
- **GIVEN** a compliant joint trajectory is active
- **AND** the controller receives a zero or negative control timestep
- **WHEN** the controller computes the next command
- **THEN** DimOS SHALL avoid integrating an unstable offset update
- **AND** it SHALL produce a documented safe outcome such as pass-through, hold, or no command.

#### Scenario: Required joint state is missing
- **GIVEN** a compliant joint trajectory is active for one or more joints
- **AND** required joint position state is missing for a controlled joint
- **WHEN** the controller computes the next command
- **THEN** DimOS SHALL avoid producing arbitrary compliant offsets for that joint
- **AND** it SHALL use a documented safe behavior such as pass-through, hold, fault, or no command.

### Requirement: Single owner of final joint command

DimOS SHALL expose compliant trajectory execution to the coordinator as a single owner of the controlled joints, rather than requiring separate trajectory and compliance controllers to compete for the same joints.

#### Scenario: Coordinator arbitrates compliant trajectory joints
- **GIVEN** a compliant joint trajectory is active for a set of joints
- **WHEN** the coordinator arbitrates active joint commands
- **THEN** it SHALL see one final command stream for those joints from the compliant trajectory behavior
- **AND** it SHALL NOT need to schedule separate competing trajectory and compliance tasks for the same joints.

#### Scenario: Higher-priority safety behavior preempts compliant trajectory
- **GIVEN** a compliant joint trajectory is active
- **AND** a higher-priority safety or control behavior claims one or more of the same joints
- **WHEN** the coordinator preempts the compliant trajectory behavior for those joints
- **THEN** the compliant trajectory behavior SHALL stop or transition to a documented preempted state
- **AND** internal compliant offset state SHALL be reset or made safe before later reuse.

### Requirement: MuJoCo verification support

DimOS SHALL support verification of compliant joint trajectory behavior in simulation using joint-space metrics, with optional contact-scene integration tests or demos.

#### Scenario: Free-space simulation smoke test
- **GIVEN** a simulated manipulator can execute joint trajectories in MuJoCo
- **WHEN** a compliant joint trajectory runs in free space
- **THEN** the trajectory SHALL complete without large compliant offsets
- **AND** the observed joint positions and efforts SHALL remain within configured verification thresholds.

#### Scenario: Rigid contact comparison
- **GIVEN** a MuJoCo scene includes a table or rigid obstacle in the manipulator workspace
- **WHEN** an equivalent rigid trajectory and compliant trajectory are compared while attempting motion into contact
- **THEN** the compliant trajectory run SHOULD show reduced fighting behavior using metrics such as peak effort, tracking error, commanded offset, or saturation time
- **AND** the compliant trajectory run SHALL remain within configured safety bounds.

#### Scenario: Soft contact comparison
- **GIVEN** a MuJoCo scene includes a soft or cushion-like contact object with tuned contact parameters
- **WHEN** a compliant trajectory makes contact with the object
- **THEN** DimOS SHOULD be able to demonstrate a bounded compliant equilibrium or safe saturation
- **AND** failure to tune soft contact parameters SHALL NOT invalidate deterministic unit-level compliance tests.
