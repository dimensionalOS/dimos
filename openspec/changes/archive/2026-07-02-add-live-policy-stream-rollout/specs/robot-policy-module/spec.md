## MODIFIED Requirements

### Requirement: Robot policy module inference boundary
The system SHALL provide a robot policy module that is a first-class DimOS `Module` and owns policy backend lifecycle, public policy reset, robot policy contract conversion, backend inference, robot policy action emission, and live policy action chunk emission without owning benchmark episode lifecycle, runtime reset or step calls, ControlCoordinator control execution, scoring, success gates, or artifact writing.

#### Scenario: Robot policy module emits one action through the architecture seams
- **WHEN** the robot policy module receives a ready robot policy observation through its synchronous inference API and has an initialized policy backend
- **THEN** it converts the observation through the robot policy contract, invokes the backend through `infer_batch`, converts the backend output back through the contract, and returns or emits the resulting robot policy action

#### Scenario: Robot policy module emits action chunks through live stream seams
- **WHEN** the robot policy module receives a fast live inference trigger and has a latest ready robot policy observation
- **THEN** it converts the observation through the robot policy contract, invokes chunk-capable backend inference asynchronously, converts the backend output into a robot policy action chunk, and publishes the chunk on its live action chunk stream

#### Scenario: Robot policy module exposes public reset
- **WHEN** a benchmark runner or live rollout path starts a new policy episode
- **THEN** it can call the robot policy module public reset method, and the robot policy module resets backend episode state before the next inference

#### Scenario: Robot policy module is blueprint-configured
- **WHEN** a blueprint constructs the robot policy module
- **THEN** the blueprint supplies backend and contract types and parameters as module configuration rather than passing live backend or contract objects

### Requirement: Batch-first policy backend
The system SHALL define a policy backend interface whose inference methods accept backend-ready batches and return backend output envelopes containing finite numeric action values or finite numeric action chunks plus inference metadata.

#### Scenario: Robot policy module does not depend on LeRobot select_action
- **WHEN** the robot policy module needs a policy action or action chunk
- **THEN** it calls the backend batch inference method rather than depending directly on LeRobot-specific `select_action` or `predict_action_chunk` APIs

#### Scenario: Backend describes loaded policy metadata
- **WHEN** a policy backend is initialized
- **THEN** it can describe backend type, checkpoint identifier, resolved checkpoint metadata when available, device, policy class when available, episode reset support, and chunk inference support for rollout artifacts

#### Scenario: Backend output is constrained
- **WHEN** a policy backend completes inference
- **THEN** it returns a backend output envelope whose output is finite numeric action data suitable for contract validation, preserving chunk rank when chunk inference is requested

### Requirement: VLA-JEPA LIBERO robot policy contract
The system SHALL provide a narrow robot policy contract for the VLA-JEPA LIBERO rollout that converts robot policy observations into LeRobot backend batches and converts LeRobot backend output into robot policy actions or robot policy action chunks.

#### Scenario: Contract maps robot policy observation to LeRobot batch
- **WHEN** the contract receives a ready robot policy observation for VLA-JEPA LIBERO rollout
- **THEN** it maps agent-view camera data to `observation.images.image`, wrist or eye-in-hand camera data to `observation.images.image2`, the 8D robot state to `observation.state`, and language metadata or observation data to the backend prompt field expected by the backend path

#### Scenario: Contract rejects semantic input mismatch
- **WHEN** a supposedly ready observation is missing a required role or has incompatible image, state, dtype, shape, or language semantics
- **THEN** the contract raises a contract conversion failure before backend inference runs

#### Scenario: Contract converts backend output to robot policy action
- **WHEN** the contract receives a backend output envelope containing a valid VLA-JEPA LIBERO action tuple
- **THEN** it returns a robot policy action with action-space id `libero.ee_delta_6d_gripper.normalized.v1` and finite `float32[7]` values compatible with the native LIBERO action range

#### Scenario: Contract converts backend output to robot policy action chunk
- **WHEN** the contract receives a backend output envelope containing a valid VLA-JEPA LIBERO action chunk
- **THEN** it returns a robot policy action chunk with action-space id `libero.ee_delta_6d_gripper.normalized.v1` and finite normalized `float32[N,7]` values compatible with DimOS-owned chunk execution

### Requirement: Robot policy action boundary
The system SHALL define reusable robot policy action and robot policy action chunk models for policy inference outputs before adaptation to runtime frames, ControlCoordinator task commands, or real robot control commands.

#### Scenario: Policy module returns runtime-independent action
- **WHEN** a robot policy module completes single-action inference and contract conversion
- **THEN** it returns a robot policy action with action-space identity, numeric values, and metadata rather than a runtime sidecar action frame

#### Scenario: Policy module publishes runtime-independent action chunk
- **WHEN** a robot policy module completes live chunk inference and contract conversion
- **THEN** it publishes a robot policy action chunk with action-space identity, ordered numeric action values, and metadata rather than controller-ready joint commands or runtime action frames

## ADDED Requirements

### Requirement: Live policy observation stream
The system SHALL allow a robot policy module to receive live robot policy observations through a stream and retain the latest ready observation for asynchronously triggered chunk inference.

#### Scenario: Module stores latest live observation
- **WHEN** a robot policy observation arrives on the live policy observation stream
- **THEN** the robot policy module records it as the latest candidate observation for future chunk inference triggers

#### Scenario: Trigger fails clearly without observation
- **WHEN** a live chunk inference trigger arrives before any ready robot policy observation has been received
- **THEN** the robot policy module reports a clear not-ready result and does not run backend inference

### Requirement: Fast live chunk inference trigger
The system SHALL expose a fast trigger on the robot policy module that requests asynchronous chunk inference from the latest live observation without returning the chunk as the trigger response.

#### Scenario: Trigger starts asynchronous inference
- **WHEN** the robot policy module receives a live chunk inference trigger and has a latest ready observation
- **THEN** it starts or schedules backend chunk inference and returns before the chunk result is available

#### Scenario: Chunk result returns on stream
- **WHEN** asynchronous chunk inference completes successfully
- **THEN** the robot policy module publishes the resulting robot policy action chunk on its chunk output stream

#### Scenario: Duplicate trigger does not start concurrent backend inference
- **WHEN** a trigger arrives while chunk inference is already in flight for the same module instance
- **THEN** the robot policy module does not start an unbounded concurrent backend inference request
