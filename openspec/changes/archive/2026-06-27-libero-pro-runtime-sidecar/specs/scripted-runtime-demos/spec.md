## ADDED Requirements

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
