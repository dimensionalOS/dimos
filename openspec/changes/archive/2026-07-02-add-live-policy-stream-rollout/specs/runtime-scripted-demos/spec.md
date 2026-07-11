## ADDED Requirements

### Requirement: LeRobot LIBERO live policy stream parity gate
The system SHALL include a script-based LeRobot LIBERO live policy stream parity gate that validates the real VLA-JEPA policy through module-native RobotPolicyModule live chunk inference, ControlCoordinator policy chunk execution, LIBERO runtime observation streams, score collection, artifact output, and teardown.

#### Scenario: Live gate runs the real policy
- **WHEN** a developer runs the LeRobot LIBERO live policy stream parity gate with compatible LeRobot dependencies and prepared LIBERO assets
- **THEN** the gate loads the actual `lerobot/VLA-JEPA-LIBERO` policy rather than using fake or fixed policy actions as the acceptance path

#### Scenario: Live gate routes policy through ControlCoordinator
- **WHEN** the live parity gate executes policy actions
- **THEN** observations flow into RobotPolicyModule, inferred robot policy action chunks flow through ControlCoordinator, and the policy chunk control task drives the runtime control path rather than benchmark evaluation adapting actions directly into native runtime `step()` calls

#### Scenario: Live gate uses module-native wiring
- **WHEN** the live parity gate wires the runtime observation stream, policy module, and coordinator
- **THEN** `RobotPolicyModule.policy_action_chunk` is connected to `ControlCoordinator.robot_policy_action_chunk` through a module stream boundary, and the coordinator requests refills through the policy module fast trigger RPC rather than through direct local callback execution

#### Scenario: Live gate enforces 10-episode real-policy success
- **WHEN** the 10-episode live parity gate over `libero_object` completes without setup or contract aborts
- **THEN** the gate passes only if the recorded success rate is greater than `0.50`

#### Scenario: Live gate records chunk diagnostics
- **WHEN** the LeRobot LIBERO live parity gate runs
- **THEN** it writes rollout artifacts that include policy success metrics and live-path diagnostics such as chunk counts, refill triggers, consumed actions, stale deactivations, and cleanup status
