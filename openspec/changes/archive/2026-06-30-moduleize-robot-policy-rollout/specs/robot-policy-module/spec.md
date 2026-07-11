## ADDED Requirements

### Requirement: Policy backend registry
The system SHALL provide a lazy policy backend registry that maps backend type names to importable backend factory paths.

#### Scenario: Module creates backend from registry
- **WHEN** a robot policy module is configured with a registered backend type and backend parameters
- **THEN** it resolves the backend factory lazily and constructs the policy backend from configuration rather than requiring a live backend object in the blueprint

#### Scenario: Unknown backend type fails clearly
- **WHEN** a robot policy module is configured with an unknown backend type
- **THEN** backend construction fails with a message that includes the unknown type and the available backend types

### Requirement: Robot policy contract registry
The system SHALL provide a lazy robot policy contract registry that maps contract type names to importable contract factory paths.

#### Scenario: Module creates contract from registry
- **WHEN** a robot policy module is configured with a registered contract type and contract parameters
- **THEN** it resolves the contract factory lazily and constructs the robot policy contract from configuration rather than requiring a live contract object in the blueprint

#### Scenario: Unknown contract type fails clearly
- **WHEN** a robot policy module is configured with an unknown contract type
- **THEN** contract construction fails with a message that includes the unknown type and the available contract types

### Requirement: Robot learning sample boundary
The system SHALL define a reusable robot learning sample model for policy inference inputs that is not tied to a benchmark sidecar response type.

#### Scenario: Policy module accepts runtime-independent samples
- **WHEN** a robot policy module receives an inference request
- **THEN** the request carries a robot learning sample with semantically named observation roles, task context, timestamps, and metadata rather than a benchmark-specific runtime observation sample

### Requirement: Robot policy action boundary
The system SHALL define a reusable robot policy action model for policy inference outputs before adaptation to runtime frames or real robot control commands.

#### Scenario: Policy module returns runtime-independent action
- **WHEN** a robot policy module completes inference and contract conversion
- **THEN** it returns a robot policy action with action-space identity, numeric values, and metadata rather than a runtime sidecar action frame

## MODIFIED Requirements

### Requirement: Robot policy module inference boundary
The system SHALL provide a robot policy module that is a first-class DimOS `Module` and owns policy backend lifecycle, public policy reset, robot policy contract conversion, backend inference, and robot policy action emission without owning benchmark episode lifecycle, runtime reset or step calls, scoring, success gates, or artifact writing.

#### Scenario: Robot policy module emits one action through the architecture seams
- **WHEN** the robot policy module receives a ready robot learning sample and has an initialized policy backend
- **THEN** it converts the sample through the robot policy contract, invokes the backend through `infer_batch`, converts the backend output back through the contract, and returns or emits the resulting robot policy action

#### Scenario: Robot policy module exposes public reset
- **WHEN** a benchmark runner starts a new policy episode
- **THEN** it can call the robot policy module public reset method, and the robot policy module resets backend episode state before the next inference

#### Scenario: Robot policy module is blueprint-configured
- **WHEN** a blueprint constructs the robot policy module
- **THEN** the blueprint supplies backend and contract types and parameters as module configuration rather than passing live backend or contract objects

### Requirement: VLA-JEPA LIBERO robot policy contract
The system SHALL provide a narrow robot policy contract for the VLA-JEPA LIBERO rollout that converts robot learning samples into LeRobot backend batches and converts LeRobot backend output into robot policy actions.

#### Scenario: Contract maps robot learning sample to LeRobot batch
- **WHEN** the contract receives a ready robot learning sample for VLA-JEPA LIBERO rollout
- **THEN** it maps agent-view camera data to `observation.images.image`, wrist or eye-in-hand camera data to `observation.images.image2`, the 8D robot state to `observation.state`, and task language to the backend prompt field expected by the backend path

#### Scenario: Contract rejects semantic input mismatch
- **WHEN** a supposedly ready sample is missing a required role or has incompatible image, state, dtype, or shape semantics
- **THEN** the contract raises a contract conversion failure before backend inference runs

#### Scenario: Contract converts backend output to robot policy action
- **WHEN** the contract receives a backend output envelope containing a valid VLA-JEPA LIBERO action
- **THEN** it returns a robot policy action with action-space id `libero.ee_delta_6d_gripper.normalized.v1` and finite `float32[7]` values compatible with the native LIBERO action range
