## ADDED Requirements

### Requirement: X-VLA LIBERO live policy stream parity gate
The system SHALL include a script-based X-VLA LIBERO live policy stream parity gate that validates `lerobot/xvla-libero` through the same module-native RobotPolicyModule live chunk inference, ControlCoordinator policy chunk execution, LIBERO runtime observation streams, score collection, artifact output, and teardown used by the VLA-JEPA live gate after the X-VLA synchronous benchmark stage has passed.

#### Scenario: X-VLA gate runs the real policy
- **WHEN** a developer runs the X-VLA LIBERO live policy stream parity gate with compatible LeRobot dependencies and prepared LIBERO assets
- **THEN** the gate loads the actual `lerobot/xvla-libero` policy rather than using fake, fixed, or VLA-JEPA policy actions as the acceptance path

#### Scenario: X-VLA gate uses existing live topology
- **WHEN** the X-VLA live parity gate executes policy actions
- **THEN** observations flow into RobotPolicyModule, inferred X-VLA robot policy action chunks flow through ControlCoordinator, and the policy chunk control task drives the runtime control path without adding X-VLA-specific control execution logic

#### Scenario: X-VLA gate enforces 10-episode real-policy success
- **WHEN** the 10-episode X-VLA live parity gate over `libero_object` completes without setup or contract aborts
- **THEN** the gate passes only if the recorded success rate is greater than `0.50`

#### Scenario: X-VLA live gate is second acceptance stage
- **WHEN** X-VLA acceptance is evaluated for policy-agnostic rollout support
- **THEN** the live parity gate runs as the second stage after a passing X-VLA synchronous benchmark gate for the same policy family and checkpoint

#### Scenario: X-VLA artifacts identify policy family and diagnostics
- **WHEN** the X-VLA LIBERO live parity gate runs
- **THEN** it writes rollout artifacts that identify the X-VLA policy family, checkpoint, and live stage and include live-path diagnostics such as chunk counts, refill triggers, inference status counts, consumed actions, stale deactivations, and cleanup status
