## Purpose

Define script-based runtime sidecar demos that validate protocol, motor-control, observation, visualization, artifact, and teardown plumbing without adding a new DimOS CLI command or requiring agent task success.

## Requirements

### Requirement: Fake sidecar smoke demo
The system SHALL include a script-based fake sidecar smoke demo that validates protocol handshake, prelaunch orchestration, resolved runtime plan generation, local motor bridge behavior, ControlCoordinator integration, and artifact output without requiring Robosuite.

#### Scenario: Fake demo completes in normal DimOS environment
- **WHEN** a developer runs the fake sidecar demo script in the normal DimOS development environment
- **THEN** the demo completes a configured number of ticks and writes episode config, resolved plan, protocol trace summary, motor trace, score output, and logs

#### Scenario: Fake demo exercises motor command/state flow
- **WHEN** the fake demo runs a scripted motor command sequence
- **THEN** commands flow from the ControlCoordinator-facing surface through the local bridge and protocol client, and motor states flow back into the DimOS side

### Requirement: Robosuite Panda Lift plumbing demo
The system SHALL include a script-based Robosuite Panda Lift plumbing demo that validates the real Robosuite sidecar, runtime description, derived motor mapping, network protocol, local motor bridge, observation stream export, score collection, and artifact output.

#### Scenario: Robosuite demo starts sidecar and DimOS blueprint
- **WHEN** a developer runs the Robosuite Panda Lift demo script with a compatible Robosuite sidecar environment
- **THEN** the script starts the sidecar, derives the resolved runtime plan from live sidecar metadata, starts the DimOS blueprint, runs the scripted sequence, collects artifacts, and tears down both runtimes

#### Scenario: Robosuite joint state changes from scripted command
- **WHEN** the Robosuite demo sends a scripted motor position command sequence
- **THEN** the returned Robosuite-derived motor state changes consistently with the command sequence and is recorded in the motor trace

#### Scenario: Robosuite observation stream is exported
- **WHEN** the Robosuite demo enables a camera observation stream
- **THEN** DimOS-side artifacts or logs show that at least one observation frame was received from the sidecar and published or recorded by the runtime client

#### Scenario: Robosuite camera appears in Rerun through DimOS streams
- **WHEN** a developer runs the Robosuite demo with Rerun stream visualization enabled
- **THEN** the demo fetches referenced `.npy` camera payloads, applies the declared image convention, publishes `color_image` and `camera_info` through DimOS streams, and Rerun displays the camera image through its normal stream bridge rather than through direct Rerun SDK logging at the runtime boundary

#### Scenario: Rerun demo stream is isolated and bounded
- **WHEN** a developer runs the Robosuite demo with Rerun stream visualization enabled repeatedly or alongside other DimOS publishers
- **THEN** the demo uses isolated Rerun and local DimOS transport settings for its visualization path and applies a bounded Rerun memory limit so old recordings or unrelated camera topics do not mix with the demo stream

### Requirement: LIBERO-PRO full-control runtime demo
The system SHALL include a script-based LIBERO-PRO runtime demo that validates the real LIBERO-PRO sidecar, runtime description, registered task reset, local SHM motor bridge, ControlCoordinator integration, camera payload export, score collection, artifact output, and teardown.

#### Scenario: LIBERO-PRO demo starts sidecar and DimOS control path
- **WHEN** a developer runs the LIBERO-PRO demo script with a compatible sidecar environment and prepared registered-suite assets
- **THEN** the script starts the sidecar, obtains runtime metadata, resolves the runtime plan, starts the DimOS control path, runs the configured tick loop, collects artifacts, and tears down both runtimes

#### Scenario: LIBERO-PRO demo exercises motor command and state flow
- **WHEN** the LIBERO-PRO demo sends scripted Panda motor position targets through the ControlCoordinator-facing path
- **THEN** commands flow through the local SHM motor bridge to the runtime protocol client, and sidecar-returned motor states flow back into the DimOS side and are recorded in the motor trace

#### Scenario: LIBERO-PRO camera payload is exported
- **WHEN** the LIBERO-PRO demo enables a configured camera observation stream
- **THEN** the demo fetches referenced `.npy` camera payloads and publishes or records at least one camera observation through the runtime observation path

#### Scenario: LIBERO-PRO score is recorded
- **WHEN** the LIBERO-PRO demo completes, times out, or reaches done
- **THEN** the demo requests sidecar-owned score output and writes success, reward or score, step count, task metadata, protocol trace summary, motor trace, and logs to the artifact directory

#### Scenario: LIBERO-PRO demo does not require agent task success
- **WHEN** the scripted LIBERO-PRO demo does not solve the selected task successfully
- **THEN** the demo can still pass if protocol, motor flow, observation flow, score collection, and teardown satisfy the demo acceptance checks

### Requirement: LIBERO-PRO asset preparation remains explicit
The LIBERO-PRO scripted demo SHALL NOT download or mutate benchmark assets unless the developer passes an explicit asset preparation flag or runs an explicit preparation command.

#### Scenario: Demo validates assets by default
- **WHEN** a developer runs the LIBERO-PRO demo without an asset preparation option
- **THEN** the demo validates prepared asset paths and fails clearly if required assets are missing

#### Scenario: Demo prepares assets only when requested
- **WHEN** a developer runs the LIBERO-PRO demo with an explicit asset preparation option
- **THEN** the demo may run the runtime asset bootstrap before launching the sidecar and still validates the prepared layout before episode reset

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
