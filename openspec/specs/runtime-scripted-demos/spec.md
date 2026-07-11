## Purpose

Define script-based simulator runtime demos that validate module-native control, motor-state, observation, visualization, artifact, and teardown plumbing without adding a new DimOS CLI command or requiring agent task success.

## Requirements

### Requirement: Fake simulator runtime smoke demo
The system SHALL include a script-based fake simulator runtime smoke demo that validates module-native control behavior, resolved runtime plan generation, typed DimOS stream publication, and artifact output without requiring Robosuite.

#### Scenario: Fake demo completes in normal DimOS environment
- **WHEN** a developer runs the fake simulator runtime demo script in the normal DimOS development environment
- **THEN** the demo completes a configured number of synchronous module steps and writes episode config, resolved plan, module trace summary, motor trace, score output, and logs

#### Scenario: Fake demo exercises motor command/state flow
- **WHEN** the fake demo runs a scripted motor command sequence
- **THEN** commands flow through the module-native step path and motor states flow back into the DimOS side through the accepted module/stream contract

### Requirement: Robosuite Panda Lift plumbing demo
The system SHALL include a script-based Robosuite Panda Lift plumbing demo that validates the placed Robosuite Simulator Runtime Module, runtime description, derived motor mapping, synchronous step RPC, DimOS observation streams, score collection, and artifact output.

#### Scenario: Robosuite demo starts placed runtime module
- **WHEN** a developer runs the Robosuite Panda Lift demo script with a compatible prepared Robosuite Python project runtime environment
- **THEN** the script uses the package-local blueprint helper, starts the placed Simulator Runtime Module, obtains runtime metadata, runs the scripted sequence, collects artifacts, and tears down the runtime

#### Scenario: Robosuite joint state changes from scripted command
- **WHEN** the Robosuite demo sends a scripted motor position command sequence through module `step()`
- **THEN** the returned Robosuite-derived motor state metadata or published state stream changes consistently with the command sequence and is recorded in the motor trace

#### Scenario: Robosuite observation stream is exported as Image and CameraInfo
- **WHEN** the Robosuite demo enables a camera observation stream
- **THEN** DimOS-side artifacts or logs show that at least one `Image` frame and associated `CameraInfo` were published through the module's DimOS stream outputs

#### Scenario: Robosuite camera appears in Rerun through DimOS streams
- **WHEN** a developer runs the Robosuite demo with Rerun stream visualization enabled
- **THEN** Rerun displays camera output consumed from normal DimOS streams rather than from direct runtime-boundary Rerun SDK logging or HTTP payload fetching

#### Scenario: Rerun demo stream is isolated and bounded
- **WHEN** a developer runs the Robosuite demo with Rerun stream visualization enabled repeatedly or alongside other DimOS publishers
- **THEN** the demo uses isolated Rerun and local DimOS transport settings for its visualization path and applies a bounded Rerun memory limit so old recordings or unrelated camera topics do not mix with the demo stream

### Requirement: LIBERO-PRO full-control runtime demo
The system SHALL include a script-based LIBERO-PRO runtime demo that validates the placed LIBERO-PRO Simulator Runtime Module, runtime description, registered task reset, synchronous step RPC, ControlCoordinator integration, camera stream export, score collection, artifact output, and teardown.

#### Scenario: LIBERO-PRO demo starts placed runtime module
- **WHEN** a developer runs the LIBERO-PRO demo script with a compatible prepared Python project runtime environment and prepared registered-suite assets
- **THEN** the script uses the package-local blueprint helper, obtains runtime metadata, resolves the runtime plan, starts the DimOS control path, runs the configured step loop, collects artifacts, and tears down the runtime

#### Scenario: LIBERO-PRO demo exercises motor command and state flow
- **WHEN** the LIBERO-PRO demo sends scripted Panda motor position targets through module `step()`
- **THEN** commands flow through the module-native step path and runtime-returned or stream-published motor states flow back into the DimOS side and are recorded in the motor trace

#### Scenario: LIBERO-PRO camera stream is exported as Image and CameraInfo
- **WHEN** the LIBERO-PRO demo enables a configured camera observation stream
- **THEN** the demo observes at least one `Image` frame and associated `CameraInfo` through DimOS stream outputs rather than through HTTP payload fetching

#### Scenario: LIBERO-PRO score is recorded
- **WHEN** the LIBERO-PRO demo completes, times out, or reaches done
- **THEN** the demo requests module-owned score output and writes success, reward or score, step count, task metadata, runtime trace summary, motor trace, and logs to the artifact directory

#### Scenario: LIBERO-PRO demo does not require agent task success
- **WHEN** the scripted LIBERO-PRO demo does not solve the selected task successfully
- **THEN** the demo can still pass if module control flow, motor flow, observation flow, score collection, and teardown satisfy the demo acceptance checks

### Requirement: LIBERO-PRO asset preparation remains explicit
The LIBERO-PRO scripted demo SHALL NOT download or mutate benchmark assets unless the developer passes an explicit asset preparation flag or runs an explicit preparation command.

#### Scenario: Demo validates assets by default
- **WHEN** a developer runs the LIBERO-PRO demo without an asset preparation option
- **THEN** the demo validates prepared asset paths and fails clearly if required assets are missing

#### Scenario: Demo prepares assets only when requested
- **WHEN** a developer runs the LIBERO-PRO demo with an explicit asset preparation option
- **THEN** the demo may run the runtime asset bootstrap before module reset and still validates the prepared layout before episode stepping

### Requirement: No agent success requirement
The scripted demos SHALL verify runtime plumbing and MUST NOT require an LLM, MCP skill policy, or successful task completion by an agent.

#### Scenario: Robosuite task is not solved
- **WHEN** the scripted Robosuite demo does not lift the object successfully
- **THEN** the demo can still pass if protocol, motor flow, observation flow, score collection, and teardown satisfy the demo acceptance checks

### Requirement: No DimOS CLI integration
The scripted demos SHALL be launched through plain scripts and MUST NOT require a new `dimos benchmark` command.

#### Scenario: Developer runs demo script directly
- **WHEN** a developer invokes the demo script with a config path
- **THEN** the script performs orchestration directly rather than delegating to a new DimOS CLI subcommand

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
