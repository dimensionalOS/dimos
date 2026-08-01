## Purpose
Define reusable pointcloud self-filtering behavior for removing robot-owned primitive regions before mapping.

## Requirements

### Requirement: TF-anchored primitive pointcloud self-filtering
The system SHALL provide a reusable pointcloud filtering module that removes points inside configured robot-owned primitive regions anchored to TF frames.

#### Scenario: Removes points inside exclusion region
- **WHEN** an input `PointCloud2` contains points that fall inside a configured sphere or box exclusion region after TF transformation into the pointcloud frame
- **THEN** the output pointcloud MUST omit those points

#### Scenario: Preserves points outside exclusion regions
- **WHEN** an input `PointCloud2` contains points outside all configured exclusion regions
- **THEN** the output pointcloud MUST preserve those points subject only to normal pointcloud serialization behavior

### Requirement: Preserve pointcloud frame semantics
The pointcloud self-filter SHALL preserve the input pointcloud frame and timestamp in the filtered output.

#### Scenario: Output remains in sensor frame
- **WHEN** the input pointcloud frame is `wrist_camera_color_optical_frame`
- **THEN** the output pointcloud MUST also use `wrist_camera_color_optical_frame` rather than transforming points into the world frame

### Requirement: Configurable fallback behavior for missing TF
The pointcloud self-filter SHALL make missing exclusion-region TF behavior explicit and configurable.

#### Scenario: Region TF unavailable
- **WHEN** a configured exclusion region cannot be transformed into the pointcloud frame
- **THEN** the module MUST either skip that region or drop the cloud according to configuration, and MUST surface a diagnostic warning
