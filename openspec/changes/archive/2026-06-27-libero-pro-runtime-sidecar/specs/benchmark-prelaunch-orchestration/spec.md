## MODIFIED Requirements

### Requirement: Benchmark episode config
The system SHALL support a benchmark episode config that declares benchmark intent, including backend selection, task identity, robot profile, control constraints, observation needs, evaluator expectations, artifact destination, and backend-specific options before a DimOS blueprint is launched.

#### Scenario: Robosuite task is declared as benchmark intent
- **WHEN** an episode config names backend `robosuite`, env name `Lift`, robot `Panda`, controller profile, control frequency, horizon, and seed
- **THEN** the config is treated as portable benchmark intent rather than as a precomputed DimOS hardware component

#### Scenario: LIBERO-PRO task is declared with backend options
- **WHEN** an episode config names backend `libero-pro` with common runtime fields and backend options for benchmark name, task order index, task index, init-state index, controller, cameras, horizon, and asset paths
- **THEN** the config is treated as portable benchmark intent without adding LIBERO-PRO-specific fields to the common top-level config surface

## ADDED Requirements

### Requirement: Backend-specific option validation
The system SHALL validate backend-specific episode options before deriving a resolved runtime plan, using typed validation for LIBERO-PRO options without requiring shared runtime protocol models to expose LIBERO-PRO task types.

#### Scenario: Valid LIBERO-PRO options are accepted
- **WHEN** a `libero-pro` episode config provides required registered-suite task selection and runtime options
- **THEN** validation produces typed backend options that can be passed to sidecar launch, reset, and demo orchestration

#### Scenario: Missing LIBERO-PRO options fail before launch
- **WHEN** a `libero-pro` episode config omits required suite, task, init-state, or asset validation options
- **THEN** prelaunch fails before starting the DimOS blueprint and reports the invalid backend option

### Requirement: LIBERO-PRO resolved runtime plan validation
The system SHALL derive LIBERO-PRO resolved runtime plans from live sidecar metadata and SHALL reject mismatches between episode config and the sidecar-described motor surface, backend, protocol version, or robot identity before starting the DimOS blueprint.

#### Scenario: LIBERO-PRO hardware component is derived from sidecar metadata
- **WHEN** the LIBERO-PRO sidecar reports an ordered Panda whole-body motor surface compatible with the requested robot id and degree of freedom count
- **THEN** the resolved runtime plan includes a matching benchmark runtime hardware component for the ControlCoordinator-facing adapter

#### Scenario: LIBERO-PRO motor surface mismatch fails
- **WHEN** the LIBERO-PRO sidecar reports a backend, robot id, motor count, or supported command mode that is incompatible with the episode config
- **THEN** prelaunch fails before starting the DimOS blueprint and records the mismatch
