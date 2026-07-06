## ADDED Requirements

### Requirement: LeRobot backend supports X-VLA LIBERO policy
The system SHALL allow RobotPolicyModule to load and run a LeRobot X-VLA LIBERO policy through the same policy backend interface used by existing LIBERO policy rollout paths.

#### Scenario: Backend initializes X-VLA checkpoint
- **WHEN** a robot policy module is configured with the LeRobot backend, the X-VLA policy family, and the `lerobot/xvla-libero` checkpoint
- **THEN** the backend loads the X-VLA policy through its LeRobot policy API, prepares LeRobot preprocessing and postprocessing through official processor factories, moves the policy to the configured device, and enters inference/evaluation mode

#### Scenario: Backend reports X-VLA metadata
- **WHEN** the X-VLA backend is initialized or described for rollout artifacts
- **THEN** the backend description includes the configured checkpoint identifier, resolved policy class, device, policy family, episode reset support, and chunk inference support

#### Scenario: Missing X-VLA dependency fails clearly
- **WHEN** X-VLA policy loading fails because the required LeRobot policy package or optional dependency is unavailable
- **THEN** backend construction or initialization fails with an actionable message naming the X-VLA policy family and the dependency setup needed to run it

### Requirement: X-VLA LIBERO chunk inference preserves policy boundary semantics
The system SHALL convert X-VLA LIBERO policy outputs into runtime-independent robot policy action chunks before any runtime or ControlCoordinator execution mapping.

#### Scenario: X-VLA chunk output is published as robot policy chunk
- **WHEN** X-VLA live inference produces a finite LIBERO action chunk compatible with the normalized 7D LIBERO action surface
- **THEN** RobotPolicyModule publishes a `RobotPolicyActionChunk` with action-space id `libero.ee_delta_6d_gripper.normalized.v1`, ordered action values, policy metadata, and no controller-specific command mapping

#### Scenario: X-VLA semantic mismatch is rejected before execution
- **WHEN** X-VLA backend output has an incompatible shape, non-finite value, unsupported action-space semantics, or unsupported action range
- **THEN** the robot policy contract rejects the output before ControlCoordinator receives an executable policy chunk
