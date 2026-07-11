## ADDED Requirements

### Requirement: Synchronous benchmark path preservation
The system SHALL preserve the synchronous benchmark policy evaluation path as the fast and deterministic simulator evaluation path even after adding the live policy stream path.

#### Scenario: Benchmark runner keeps lockstep runtime stepping
- **WHEN** benchmark policy evaluation runs the existing LIBERO policy gate
- **THEN** benchmark evaluation owns runtime reset, snapshot collection, policy inference call, action adaptation, runtime step, scoring, artifacts, and cleanup in a lockstep loop without requiring ControlCoordinator policy chunk execution

#### Scenario: Live path does not replace fast benchmark path
- **WHEN** both the synchronous benchmark gate and live policy stream parity gate exist
- **THEN** the synchronous benchmark gate remains available for faster-than-realtime or parallel simulator evaluation, while the live gate validates real-world rollout topology
