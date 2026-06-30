## ADDED Requirements

### Requirement: Robosuite sidecar package
The system SHALL provide a first-class Robosuite sidecar package in the monorepo that depends on the runtime protocol package and Robosuite-specific dependencies without depending on the main DimOS package.

#### Scenario: Robosuite sidecar installs in isolated environment
- **WHEN** a developer installs the Robosuite sidecar package in a Robosuite-compatible environment
- **THEN** the sidecar can start and import runtime protocol models without installing the main DimOS package

### Requirement: Baked Robosuite task instantiation
The Robosuite sidecar SHALL instantiate baked Robosuite tasks from episode config fields such as env name, robot name, controller profile, control frequency, horizon, renderer options, camera options, and seed.

#### Scenario: Panda Lift task starts
- **WHEN** the episode config requests `env_name: Lift` and `robots: Panda`
- **THEN** the sidecar creates the corresponding Robosuite environment and exposes its runtime description

### Requirement: Runtime-derived motor surface
The Robosuite sidecar SHALL derive robot motor surface metadata from the live Robosuite environment and controller setup rather than requiring every benchmark config to manually enumerate Robosuite action indices.

#### Scenario: Panda motor order is described
- **WHEN** the sidecar creates a Panda Lift environment
- **THEN** it reports a stable ordered motor surface suitable for DimOS whole-body motor control

#### Scenario: Unsupported controller profile fails
- **WHEN** the selected Robosuite controller profile cannot be mapped to a supported DimOS motor command mode
- **THEN** the sidecar rejects the episode setup with a protocol error that identifies the unsupported profile

### Requirement: Robosuite step ownership
The Robosuite sidecar SHALL own backend-native `env.reset()` and `env.step(action)` calls and SHALL translate between runtime protocol action/state frames and Robosuite action/observation structures.

#### Scenario: Motor position step advances Robosuite
- **WHEN** DimOS sends a motor position action frame for the described Panda motor surface
- **THEN** the sidecar maps it to the Robosuite action vector, steps the environment, and returns motor state, reward, done, success if available, and observation metadata

### Requirement: Observation export
The Robosuite sidecar SHALL expose configured Robosuite camera and state observations through runtime protocol observation frames that DimOS can publish as observation streams.

#### Scenario: Agentview camera is available
- **WHEN** the episode config enables the `agentview` camera
- **THEN** step responses include observation frames with `.npy` payload references that allow DimOS to fetch and publish the camera output as a stream

### Requirement: Score and artifact export
The Robosuite sidecar SHALL provide score and artifact outputs for each episode, including reward/done/success metadata, backend timing, and sidecar logs or trace summaries.

#### Scenario: Score is collected after demo
- **WHEN** the demo completes or times out
- **THEN** the runner can request score output from the sidecar and write it with the episode artifacts
