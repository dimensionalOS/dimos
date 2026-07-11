## MODIFIED Requirements

### Requirement: LeRobot LIBERO policy rollout demo
The system SHALL include a module-backed LeRobot LIBERO policy rollout demo that validates policy loading, contract conversion, native runtime actions, placed runtime-module stepping, stream snapshot collection, score collection, artifact output, and teardown through the same module-backed policy evaluation path used by DimOS workflow.

#### Scenario: Policy rollout demo starts native runtime module and policy rollout stack
- **WHEN** a developer runs the LeRobot LIBERO policy rollout demo with compatible LeRobot dependencies and prepared LIBERO assets
- **THEN** the demo starts the placed LIBERO runtime module in native LIBERO action mode, initializes module-backed benchmark evaluation with a robot policy module, LeRobot backend, and VLA-JEPA LIBERO contract, runs the configured episode matrix, writes artifacts, and tears down all resources

#### Scenario: Policy rollout demo bypasses ControlCoordinator
- **WHEN** the LeRobot LIBERO policy rollout demo executes policy actions
- **THEN** actions flow from the robot policy module through benchmark evaluation to the runtime module as native runtime action frames without using ControlCoordinator, JointTrajectoryTask, EndEffectorDeltaTrajectoryTask, the SHM motor bridge, or motor action frames

#### Scenario: Policy rollout demo consumes runtime streams directly
- **WHEN** the runtime module publishes configured camera images and robot-state events during reset or step
- **THEN** the policy rollout demo captures those DimOS stream outputs as the source of policy observations and optional videos without constructing HTTP payload references

#### Scenario: Policy rollout demo enforces success gate
- **WHEN** the 50-episode policy rollout gate completes without setup or contract aborts
- **THEN** the demo passes only if the recorded success rate is greater than `0.50`

#### Scenario: Policy rollout demo records required artifacts
- **WHEN** the LeRobot LIBERO policy rollout demo runs
- **THEN** it writes rollout summary, per-episode JSONL records, runtime description, checkpoint metadata, logs, and cleanup status to the artifact directory

#### Scenario: Existing scripted demos remain unchanged
- **WHEN** the fake, Robosuite, or existing LIBERO-PRO motor demos are run
- **THEN** they continue to validate scripted runtime plumbing without requiring LeRobot policy dependencies or policy task success
