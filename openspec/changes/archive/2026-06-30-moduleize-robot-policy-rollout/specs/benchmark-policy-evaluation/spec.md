## ADDED Requirements

### Requirement: Benchmark sample builder
The system SHALL provide an explicit LIBERO benchmark sample-building seam that converts runtime sidecar observations and payloads into robot learning samples.

#### Scenario: Evaluation builds sample from runtime observations
- **WHEN** benchmark evaluation receives LIBERO runtime observations and payload references for an episode tick
- **THEN** it resolves the required payloads and produces a robot learning sample with policy-facing observation roles, task context, timestamps, and metadata

#### Scenario: Sample builder remains outside policy module
- **WHEN** LIBERO runtime observations must be converted to robot learning samples
- **THEN** the conversion is owned by benchmark evaluation or its sample-builder seam rather than by the robot policy module

### Requirement: Benchmark action adapter
The system SHALL adapt robot policy actions into native runtime action frames inside benchmark evaluation before stepping the runtime sidecar.

#### Scenario: Evaluation adapts policy action to runtime frame
- **WHEN** the robot policy module returns a robot policy action for the LIBERO native action surface
- **THEN** benchmark evaluation converts it into a runtime action frame with the matching action-space identity before calling the runtime sidecar step method

## MODIFIED Requirements

### Requirement: Benchmark policy evaluation runner
The system SHALL provide a module-backed benchmark policy evaluation runner that owns episode matrix selection, runtime sidecar lifecycle, runtime reset and step calls, scoring, success gates, metrics, artifacts, and cleanup for policy-driven benchmark rollouts.

#### Scenario: Evaluation runner uses robot policy module for actions
- **WHEN** the evaluation runner has a current sidecar observation during an episode
- **THEN** it builds a robot learning sample, sends the sample to the robot policy module through its module-facing inference API, adapts the returned robot policy action to a runtime action frame, and uses that frame for the sidecar step call

#### Scenario: Evaluation runner owns episode lifecycle
- **WHEN** the evaluation runner starts an episode
- **THEN** it resets the runtime sidecar, calls the robot policy module public reset method, steps the runtime with adapted policy actions, records episode metrics, and applies the success gate outside the robot policy module

#### Scenario: Evaluation runner can be launched through DimOS module composition
- **WHEN** a developer launches the LeRobot LIBERO policy evaluation path
- **THEN** benchmark evaluation, policy inference, and runtime sidecar access are represented as DimOS modules or blueprint-compatible module configuration rather than only as directly wired plain Python service objects
