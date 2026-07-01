## ADDED Requirements

### Requirement: Robot policy module inference boundary
The system SHALL provide a robot policy module that owns policy backend lifecycle, public policy reset, robot policy contract conversion, backend inference, and policy action emission without owning benchmark episode lifecycle, runtime reset or step calls, scoring, success gates, or artifact writing.

#### Scenario: Robot policy module emits one action through the architecture seams
- **WHEN** the robot policy module receives a ready robot learning sample and has an initialized policy backend
- **THEN** it converts the sample through the robot policy contract, invokes the backend through `infer_batch`, converts the backend output back through the contract, and returns or emits the resulting runtime action frame

#### Scenario: Robot policy module exposes public reset
- **WHEN** a benchmark runner starts a new policy episode
- **THEN** it can call the robot policy module public reset method, and the robot policy module resets backend episode state before the next inference

### Requirement: Batch-first policy backend
The system SHALL define a policy backend interface whose primary inference method accepts backend-ready batches and returns backend output envelopes.

#### Scenario: Robot policy module does not depend on LeRobot select_action
- **WHEN** the robot policy module needs a policy action
- **THEN** it calls the backend batch inference method rather than depending on LeRobot-specific `select_action` or `predict_action_chunk` APIs

#### Scenario: Backend describes loaded policy metadata
- **WHEN** a policy backend is initialized
- **THEN** it can describe backend type, checkpoint identifier, resolved checkpoint metadata when available, device, policy class when available, and episode reset support for rollout artifacts

### Requirement: LeRobot backend for VLA-JEPA LIBERO
The system SHALL provide an in-process LeRobot backend capable of loading and running the official `lerobot/VLA-JEPA-LIBERO` checkpoint for LIBERO policy rollout.

#### Scenario: Backend initializes official checkpoint
- **WHEN** the LeRobot backend is configured with `lerobot/VLA-JEPA-LIBERO`
- **THEN** it loads the policy, prepares required LeRobot preprocessing or postprocessing, moves the policy to the configured device, and enters inference/evaluation mode

#### Scenario: Backend returns an output envelope
- **WHEN** the LeRobot backend completes inference for a backend-ready batch
- **THEN** it returns a backend output envelope containing the backend-native action result and inference metadata needed for validation and artifacts

### Requirement: VLA-JEPA LIBERO robot policy contract
The system SHALL provide a narrow robot policy contract for the VLA-JEPA LIBERO rollout that converts sidecar observations into LeRobot backend batches and converts LeRobot backend output into a native runtime action frame.

#### Scenario: Contract maps sidecar observation to LeRobot batch
- **WHEN** the contract receives a ready LIBERO sidecar observation for VLA-JEPA LIBERO rollout
- **THEN** it maps agent-view camera data to `observation.images.image`, wrist or eye-in-hand camera data to `observation.images.image2`, the 8D robot state to `observation.state`, and task language to the backend prompt field expected by the backend path

#### Scenario: Contract rejects semantic input mismatch
- **WHEN** a supposedly ready sample is missing a required stream or has incompatible image, state, dtype, or shape semantics
- **THEN** the contract raises a contract conversion failure before backend inference runs

#### Scenario: Contract converts backend output to runtime action
- **WHEN** the contract receives a backend output envelope containing a valid VLA-JEPA LIBERO action
- **THEN** it returns a runtime action frame with `space_id` `libero.ee_delta_6d_gripper.normalized.v1` and finite `float32[7]` values compatible with the native LIBERO action range
