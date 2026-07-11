## ADDED Requirements

### Requirement: Benchmark policy evaluation runner
The system SHALL provide a benchmark policy evaluation runner that owns episode matrix selection, runtime sidecar lifecycle, runtime reset and step calls, scoring, success gates, metrics, artifacts, and cleanup for policy-driven benchmark rollouts.

#### Scenario: Evaluation runner uses robot policy module for actions
- **WHEN** the evaluation runner has a current sidecar observation during an episode
- **THEN** it passes the corresponding robot learning sample to the robot policy module and uses the returned runtime action frame for the sidecar step call

#### Scenario: Evaluation runner owns episode lifecycle
- **WHEN** the evaluation runner starts an episode
- **THEN** it resets the runtime sidecar, calls the robot policy module public reset method, steps the runtime with emitted actions, records episode metrics, and applies the success gate outside the robot policy module

### Requirement: LIBERO policy rollout gate
The system SHALL provide a policy-driven LIBERO gate using `lerobot/VLA-JEPA-LIBERO` over `libero_object`, all 10 task indices, and init states `[0, 1, 2, 3, 4]`, for 50 total episodes with pass condition `success_rate > 0.50`.

#### Scenario: Gate evaluates the configured episode matrix
- **WHEN** the policy rollout gate is run with prepared LIBERO assets and compatible LeRobot dependencies
- **THEN** it evaluates 50 episodes formed from all 10 `libero_object` task indices crossed with init states `[0, 1, 2, 3, 4]`

#### Scenario: Gate records pass/fail summary
- **WHEN** all scheduled episodes complete without setup or contract aborts
- **THEN** the gate writes a summary containing episode count, success count, success rate, threshold, and pass/fail result

#### Scenario: Policy failures continue the run
- **WHEN** an individual episode times out, reaches done without success, or reports unsuccessful completion
- **THEN** the episode is recorded as a failed policy episode and the rollout continues with the next scheduled episode

#### Scenario: Setup and contract errors abort the run
- **WHEN** checkpoint loading, sidecar compatibility, action spec validation, observation mapping, action conversion, or protocol validation fails
- **THEN** the rollout aborts the run and records the failure as an integration/setup error rather than counting it as a policy episode failure

### Requirement: Rollout artifacts
The system SHALL write structured rollout artifacts that make policy, contract, runtime, and episode behavior inspectable without storing full videos or image dumps by default.

#### Scenario: Required metadata artifacts are written
- **WHEN** a policy rollout gate starts or completes
- **THEN** it writes `summary.json`, `episodes.jsonl`, `runtime_description.json`, `contract_description.json`, and `checkpoint_metadata.json` in the artifact directory

#### Scenario: Episode records include rollout diagnostics
- **WHEN** an episode record is written
- **THEN** it includes task index, init state index, episode id, success, step count, reward sum, done state, failure reason when present, action shape, action min/max, and observed stream names

#### Scenario: Videos are opt-in
- **WHEN** the rollout is run without a video-saving option
- **THEN** it does not write full rollout videos or complete image dumps by default
