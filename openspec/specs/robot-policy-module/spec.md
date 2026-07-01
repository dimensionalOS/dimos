# robot-policy-module Specification

## Purpose
TBD - created by archiving change add-lerobot-libero-policy-rollout. Update Purpose after archive.

## Requirements

### Requirement: Robot policy module inference boundary
The system SHALL provide a robot policy module that is a first-class DimOS `Module` and owns policy backend lifecycle, public policy reset, robot policy contract conversion, backend inference, and robot policy action emission without owning benchmark episode lifecycle, runtime reset or step calls, scoring, success gates, or artifact writing.

#### Scenario: Robot policy module emits one action through the architecture seams
- **WHEN** the robot policy module receives a ready robot policy observation and has an initialized policy backend
- **THEN** it converts the observation through the robot policy contract, invokes the backend through `infer_batch`, converts the backend output back through the contract, and returns or emits the resulting robot policy action

#### Scenario: Robot policy module exposes public reset
- **WHEN** a benchmark runner starts a new policy episode
- **THEN** it can call the robot policy module public reset method, and the robot policy module resets backend episode state before the next inference

#### Scenario: Robot policy module is blueprint-configured
- **WHEN** a blueprint constructs the robot policy module
- **THEN** the blueprint supplies backend and contract types and parameters as module configuration rather than passing live backend or contract objects

### Requirement: Batch-first policy backend
The system SHALL define a policy backend interface whose primary inference method accepts backend-ready batches and returns backend output envelopes containing flat numeric action tuples and inference metadata.

#### Scenario: Robot policy module does not depend on LeRobot select_action
- **WHEN** the robot policy module needs a policy action
- **THEN** it calls the backend batch inference method rather than depending on LeRobot-specific `select_action` or `predict_action_chunk` APIs

#### Scenario: Backend describes loaded policy metadata
- **WHEN** a policy backend is initialized
- **THEN** it can describe backend type, checkpoint identifier, resolved checkpoint metadata when available, device, policy class when available, and episode reset support for rollout artifacts

#### Scenario: Backend output is constrained
- **WHEN** a policy backend completes inference
- **THEN** it returns a backend output envelope whose output is a flat tuple of finite numeric action values suitable for contract validation

### Requirement: LeRobot backend for VLA-JEPA LIBERO
The system SHALL provide an in-process LeRobot backend capable of loading and running the official `lerobot/VLA-JEPA-LIBERO` checkpoint for LIBERO policy rollout using official LeRobot policy and processor APIs.

#### Scenario: Backend initializes official checkpoint
- **WHEN** the LeRobot backend is configured with `lerobot/VLA-JEPA-LIBERO`
- **THEN** it loads the policy through the official VLA-JEPA policy API, prepares required LeRobot preprocessing and postprocessing through official processor factories, moves the policy to the configured device, and enters inference/evaluation mode

#### Scenario: Backend returns an output envelope
- **WHEN** the LeRobot backend completes inference for a backend-ready batch
- **THEN** it returns a backend output envelope containing a flat action tuple and inference metadata needed for validation and artifacts

### Requirement: VLA-JEPA LIBERO robot policy contract
The system SHALL provide a narrow robot policy contract for the VLA-JEPA LIBERO rollout that converts robot policy observations into LeRobot backend batches and converts LeRobot backend output into robot policy actions.

#### Scenario: Contract maps robot policy observation to LeRobot batch
- **WHEN** the contract receives a ready robot policy observation for VLA-JEPA LIBERO rollout
- **THEN** it maps agent-view camera data to `observation.images.image`, wrist or eye-in-hand camera data to `observation.images.image2`, the 8D robot state to `observation.state`, and language metadata or observation data to the backend prompt field expected by the backend path

#### Scenario: Contract rejects semantic input mismatch
- **WHEN** a supposedly ready observation is missing a required role or has incompatible image, state, dtype, shape, or language semantics
- **THEN** the contract raises a contract conversion failure before backend inference runs

#### Scenario: Contract converts backend output to robot policy action
- **WHEN** the contract receives a backend output envelope containing a valid VLA-JEPA LIBERO action tuple
- **THEN** it returns a robot policy action with action-space id `libero.ee_delta_6d_gripper.normalized.v1` and finite `float32[7]` values compatible with the native LIBERO action range

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
The system SHALL define a reusable robot policy observation model for policy inference inputs that is not tied to a benchmark sidecar response type or benchmark episode lifecycle identifiers.

#### Scenario: Policy module accepts runtime-independent observations
- **WHEN** a robot policy module receives an inference request
- **THEN** the request carries a robot policy observation with semantically named observation roles, timestamps, and metadata rather than a benchmark-specific runtime observation sample or benchmark episode identifiers

#### Scenario: Language remains contract-specific input data
- **WHEN** a policy contract requires a language prompt
- **THEN** it reads the prompt from observation metadata or observation roles rather than from a generic top-level task field on every policy observation

### Requirement: Robot policy action boundary
The system SHALL define a reusable robot policy action model for policy inference outputs before adaptation to runtime frames or real robot control commands.

#### Scenario: Policy module returns runtime-independent action
- **WHEN** a robot policy module completes inference and contract conversion
- **THEN** it returns a robot policy action with action-space identity, numeric values, and metadata rather than a runtime sidecar action frame
