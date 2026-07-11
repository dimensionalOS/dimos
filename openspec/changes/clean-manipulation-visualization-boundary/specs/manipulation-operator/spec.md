## ADDED Requirements

### Requirement: Interactive manipulation must use one concrete operator facade
The system MUST provide one concrete UI-neutral manipulation operator that composes `ManipulationModule` and `WorldMonitor`. It MUST NOT introduce an operator Spec or Protocol when no alternate implementation exists.

#### Scenario: Visualization session is created
- **WHEN** ManipulationModule completes world initialization
- **THEN** it constructs one operator and binds that concrete instance into the visualization session

### Requirement: Operator dynamic reads must remain compact
The operator MUST expose slow-changing manipulation state, error, and stored-plan summary through one compact status read. Static topology and high-rate joint states MUST NOT be returned by operator status.

#### Scenario: Panel refreshes status
- **WHEN** Viser refreshes module and plan status
- **THEN** it reads compact operator status without copying robot configs or joint telemetry

### Requirement: Joint target requests must use canonical global joints
Joint target requests MUST contain ordered planning-group IDs and one globally named `JointState` whose names exactly equal the selected groups' concatenated joint names in group order. Local aliases, suffix matching, missing joints, duplicate joints, and extraneous joints MUST be rejected.

#### Scenario: Multiple groups submit a joint draft
- **WHEN** a frontend submits targets for ordered groups across one or more robots
- **THEN** the operator validates one flat global joint state in that exact selection order

### Requirement: Pose target requests must carry explicit frames
Pose target requests MUST contain `PoseStamped` values keyed by pose-capable planning-group ID, optional ordered auxiliary group IDs, and an optional globally named seed. Initial operator support MUST require every pose target to use the world frame.

#### Scenario: World-frame pose target is evaluated
- **WHEN** a frontend submits a world-frame target for a pose-capable group
- **THEN** the operator evaluates it without inventing or rewriting its frame

#### Scenario: Unsupported pose frame is submitted
- **WHEN** a pose target uses a frame other than world
- **THEN** the operator rejects the request with a clear validation result

### Requirement: Target evaluations must be advisory and selected-domain only
Target evaluation MUST return feasibility, status, message, selected globally named joints when available, group poses, and per-group diagnostics. Complete per-robot states used for FK or collision checks MUST remain internal, and an evaluation MUST NOT authorize planning or execution.

#### Scenario: Joint target is evaluated
- **WHEN** the operator composes complete robot states for collision and FK checks
- **THEN** the frontend receives only selected-domain results and diagnostics

#### Scenario: Pose target is evaluated
- **WHEN** IK produces selected joints
- **THEN** the operator returns those selected global joints and group results without exposing internal complete states

### Requirement: Planning must consume original complete target requests
Operator planning methods MUST consume the original joint or pose target request and MUST NOT accept a prior advisory evaluation as planning authority. Planning MUST return a typed action summary rather than the complete `GeneratedPlan`.

#### Scenario: Target changes after evaluation
- **WHEN** a frontend plans a newer target draft after an older evaluation completed
- **THEN** planning validates and plans the submitted request independently of the older evaluation

#### Scenario: Planning succeeds
- **WHEN** manipulation caches a complete generated plan
- **THEN** the operator returns success, message, group IDs, waypoint count, and duration without returning path or trajectory data

### Requirement: Target drafts and scheduling must remain frontend-owned
The operator MUST be synchronous and stateless with respect to editable target drafts, group selection, request generations, and frontend worker scheduling.

#### Scenario: Evaluation results arrive out of order
- **WHEN** Viser runs synchronous operator calls in workers and a newer target generation exists
- **THEN** Viser discards the stale result using its own generation identity

### Requirement: Operator actions must preserve domain ownership
Preview, execute, cancel, clear-plan, and reset methods MUST delegate to manipulation behavior and return typed action results. Cancellation MUST route preview cancellation through WorldMonitor's configured visualization without directly addressing Viser.

#### Scenario: Operator cancels during preview
- **WHEN** the frontend calls operator cancel
- **THEN** ManipulationModule cancellation routes through WorldMonitor to visualization cancellation

#### Scenario: Operator clears a plan
- **WHEN** the frontend calls clear-plan
- **THEN** the stored generated plan is discarded without introducing persistent preview state
