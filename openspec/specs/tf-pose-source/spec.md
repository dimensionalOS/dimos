## Purpose
Define a reusable source for pose-only odometry derived from the TF tree.

## Requirements

### Requirement: TF-derived pose-only odometry
The system SHALL provide a reusable module that publishes pose-only `Odometry` from a configured TF lookup between a target frame and a source frame.

#### Scenario: Publishes odometry from available TF
- **WHEN** the configured TF lookup from `target_frame` to `source_frame` is available within tolerance
- **THEN** the module MUST publish an `Odometry` message whose `frame_id` is `target_frame`, whose `child_frame_id` is `source_frame`, and whose pose matches the TF transform

#### Scenario: Omits output when TF is unavailable
- **WHEN** the configured TF lookup is unavailable or older than the configured tolerance
- **THEN** the module MUST NOT publish a stale pose sample for that cycle

### Requirement: Fixed-rate odometry publishing
The TF pose source SHALL support fixed-rate publishing for v1 camera-pose odometry use cases.

#### Scenario: Publishes at configured rate
- **WHEN** the module is configured with a positive publish rate and TF is available
- **THEN** it MUST repeatedly publish pose-only odometry at approximately the configured rate until stopped
