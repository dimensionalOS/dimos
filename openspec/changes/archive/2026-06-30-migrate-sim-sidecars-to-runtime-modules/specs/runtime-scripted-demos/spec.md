## MODIFIED Requirements

### Requirement: Fake sidecar smoke demo
The system SHALL include a fake simulator runtime smoke demo that validates module-native protocol/control behavior, runtime placement where applicable, DimOS stream data flow, ControlCoordinator integration, and artifact output without requiring Robosuite.

#### Scenario: Fake demo completes in normal DimOS environment
- **WHEN** a developer runs the fake simulator runtime demo in the normal DimOS development environment
- **THEN** the demo completes a configured number of synchronous module steps and writes episode config, resolved runtime plan, module trace summary, motor trace, score output, and logs

#### Scenario: Fake demo exercises motor command/state flow
- **WHEN** the fake demo runs a scripted motor command sequence
- **THEN** commands flow through the module-native step path and motor states flow back into the DimOS side through the accepted module/stream contract

### Requirement: Robosuite Panda Lift plumbing demo
The system SHALL include a Robosuite Panda Lift plumbing demo that validates the placed Robosuite Simulator Runtime Module, runtime description, derived motor mapping, synchronous step RPC, DimOS observation streams, score collection, and artifact output.

#### Scenario: Robosuite demo starts placed runtime module
- **WHEN** a developer runs the Robosuite Panda Lift demo with a prepared compatible Robosuite Python project runtime environment
- **THEN** the demo uses the package-local blueprint helper, starts the placed Simulator Runtime Module, obtains runtime metadata, runs the scripted sequence, collects artifacts, and tears down the runtime

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
The system SHALL include a LIBERO-PRO runtime demo that validates the placed LIBERO-PRO Simulator Runtime Module, runtime description, registered task reset, synchronous step RPC, ControlCoordinator integration, camera stream export, score collection, artifact output, and teardown.

#### Scenario: LIBERO-PRO demo starts placed runtime module
- **WHEN** a developer runs the LIBERO-PRO demo with a compatible prepared Python project runtime environment and prepared registered-suite assets
- **THEN** the demo uses the package-local blueprint helper, obtains runtime metadata, resolves the runtime plan, starts the DimOS control path, runs the configured step loop, collects artifacts, and tears down the runtime

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
