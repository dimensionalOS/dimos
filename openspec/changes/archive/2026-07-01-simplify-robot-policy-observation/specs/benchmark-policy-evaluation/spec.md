## MODIFIED Requirements

### Requirement: Benchmark policy evaluation runner
The system SHALL provide a module-backed benchmark policy evaluation runner that owns episode matrix selection, runtime sidecar lifecycle, runtime reset and step calls, scoring, success gates, metrics, artifacts, and cleanup for policy-driven benchmark rollouts.

#### Scenario: Evaluation runner uses robot policy module for actions
- **WHEN** the evaluation runner has a current sidecar observation during an episode
- **THEN** it builds a robot policy observation, sends the observation to the robot policy module through its module-facing inference API, adapts the returned robot policy action to a runtime action frame, and uses that frame for the sidecar step call

#### Scenario: Evaluation runner owns episode lifecycle
- **WHEN** the evaluation runner starts an episode
- **THEN** it resets the runtime sidecar, calls the robot policy module public reset method, steps the runtime with adapted policy actions, records episode metrics, and applies the success gate outside the robot policy module

#### Scenario: Evaluation runner can be launched through DimOS module composition
- **WHEN** a developer launches the LeRobot LIBERO policy evaluation path
- **THEN** benchmark evaluation, policy inference, and runtime sidecar access are represented as DimOS modules or blueprint-compatible module configuration rather than only as directly wired plain Python service objects

### Requirement: Rollout artifacts
The system SHALL write structured rollout artifacts that make policy, runtime, and episode behavior inspectable without storing full videos or image dumps by default.

#### Scenario: Required metadata artifacts are written
- **WHEN** a policy rollout gate starts or completes
- **THEN** it writes `summary.json`, `episodes.jsonl`, `runtime_description.json`, and `checkpoint_metadata.json` in the artifact directory

#### Scenario: Episode records include rollout diagnostics
- **WHEN** an episode record is written
- **THEN** it includes task index, init state index, episode id, success, step count, reward sum, done state, failure reason when present, action shape, action min/max, and observed stream names

#### Scenario: Videos are opt-in
- **WHEN** the rollout is run without a video-saving option
- **THEN** it does not write full rollout videos or complete image dumps by default

### Requirement: Benchmark sample builder
The system SHALL provide an explicit LIBERO benchmark sample-building seam that converts runtime sidecar observations and payloads into robot policy observations.

#### Scenario: Evaluation builds observation from runtime observations
- **WHEN** benchmark evaluation receives LIBERO runtime observations and payload references for an episode tick
- **THEN** it resolves the required payloads and produces a robot policy observation with policy-facing observation roles, timestamps, and metadata while keeping benchmark episode identifiers in evaluation-layer records and requests

#### Scenario: Sample builder remains outside policy module
- **WHEN** LIBERO runtime observations must be converted to robot policy observations
- **THEN** the conversion is owned by benchmark evaluation or its sample-builder seam rather than by the robot policy module
