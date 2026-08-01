## ADDED Requirements

### Requirement: Benchmark episode config
The system SHALL support a benchmark episode config that declares benchmark intent, including backend selection, task identity, robot profile, control constraints, observation needs, evaluator expectations, and artifact destination before a DimOS blueprint is launched.

#### Scenario: Robosuite task is declared as benchmark intent
- **WHEN** an episode config names backend `robosuite`, env name `Lift`, robot `Panda`, controller profile, control frequency, horizon, and seed
- **THEN** the config is treated as portable benchmark intent rather than as a precomputed DimOS hardware component

### Requirement: Runtime prelaunch orchestration
The system SHALL provide a prelaunch orchestrator that starts the simulator sidecar first, obtains live sidecar metadata, derives concrete DimOS launch material, and then launches the DimOS blueprint.

#### Scenario: Sidecar describes runtime before blueprint launch
- **WHEN** prelaunch starts a sidecar for an episode
- **THEN** it waits for sidecar health and runtime description before creating the resolved runtime plan used to launch DimOS

#### Scenario: Sidecar health fails
- **WHEN** the sidecar does not become healthy within the configured timeout
- **THEN** prelaunch fails without launching the DimOS blueprint and writes a failure artifact

### Requirement: Resolved runtime plan
The system SHALL derive a resolved runtime plan from live sidecar metadata and the episode config, including hardware components, simulator connection config, observation stream config, evaluator config, and artifact routing.

#### Scenario: Hardware component is derived from sidecar metadata
- **WHEN** the sidecar reports an ordered whole-body motor surface for `panda`
- **THEN** the resolved runtime plan includes a matching `HardwareComponent` projection for the ControlCoordinator

#### Scenario: Mismatched robot profile fails
- **WHEN** the episode config requests a robot profile that is incompatible with the sidecar-described motor surface
- **THEN** prelaunch fails before starting the DimOS blueprint and records the mismatch

### Requirement: Runner owns both runtime lifetimes
The benchmark runner SHALL remain the parent owner of both the simulator sidecar process/environment and the DimOS blueprint process/environment for the duration of a demo or benchmark episode.

#### Scenario: DimOS blueprint exits early
- **WHEN** the DimOS blueprint process exits before the episode completes
- **THEN** the runner tears down the sidecar and records failure attribution and logs from both runtimes

#### Scenario: Sidecar exits early
- **WHEN** the sidecar exits before the episode completes
- **THEN** the runner tears down the DimOS blueprint and records sidecar failure details

### Requirement: Local SHM is not a remote sidecar protocol
The system SHALL restrict SHM usage to local DimOS motor control plumbing between the runtime client module and ControlCoordinator-facing WholeBodyAdapter.

#### Scenario: Sidecar runs remotely
- **WHEN** the sidecar endpoint is configured for another host
- **THEN** DimOS communicates with it through the runtime network protocol and does not require remote SHM access

### Requirement: Plain script entrypoints
The system SHALL provide plain script entrypoints for v1 demos and MUST NOT require a new `dimos` CLI command for this change.

#### Scenario: Demo is launched from script
- **WHEN** a developer runs the fake sidecar or Robosuite demo script
- **THEN** the script performs prelaunch orchestration and calls or builds the relevant DimOS blueprint directly
