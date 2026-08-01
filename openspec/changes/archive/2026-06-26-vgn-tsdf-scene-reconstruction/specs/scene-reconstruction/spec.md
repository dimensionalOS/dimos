## ADDED Requirements

### Requirement: Depth-based scene reconstruction
The system SHALL provide a scene reconstruction capability that consumes depth images, depth camera calibration, and camera pose information to maintain a reconstructed scene.

#### Scenario: Integrate depth observation
- **WHEN** a depth image, matching depth camera info, and a TF transform for the depth image frame at the image timestamp are available
- **THEN** the reconstruction capability integrates the observation into the current scene reconstruction

#### Scenario: Missing transform
- **WHEN** a depth image is available but the camera pose cannot be resolved within the configured tolerance
- **THEN** the reconstruction capability does not integrate that image and records the dropped frame in reconstruction status

### Requirement: Publish scene products
The system SHALL publish reconstructed scene products as streams, including a pointcloud, a TSDF grid, and reconstruction status.

#### Scenario: Fixed output rate
- **WHEN** reconstruction is active and scene data is available
- **THEN** the system publishes scene products at the configured reconstruction output rate rather than by RPC request

#### Scenario: Pointcloud and TSDF share source observations
- **WHEN** the system publishes both pointcloud and TSDF products
- **THEN** both products are derived from the same integrated depth observations rather than converting TSDF from an unrelated pointcloud-only source

### Requirement: Reconstruction lifecycle control
The system SHALL expose RPC controls for scene reconstruction lifecycle and workspace configuration while keeping large scene payloads on streams.

#### Scenario: Reset scene
- **WHEN** a caller invokes reset scene control
- **THEN** the current integrated scene is cleared and subsequent scene products reflect only observations integrated after the reset

#### Scenario: Pause and resume integration
- **WHEN** a caller pauses integration
- **THEN** incoming depth observations are not integrated until integration is resumed

#### Scenario: Set workspace
- **WHEN** a caller sets a workspace frame, center, and size
- **THEN** the reconstruction capability reconstructs subsequent TSDF data in that workspace and converts the center specification into the internal grid-origin representation

### Requirement: TSDF grid message
The system SHALL represent TSDF output with a streamable `TSDFGrid` data type using VGN/Open3D-aligned grid semantics.

#### Scenario: VGN-compatible grid shape
- **WHEN** a TSDF grid is published for VGN consumption
- **THEN** its distances array is a float32 array shaped `(1, X, Y, Z)` and includes resolution metadata matching `X`, `Y`, and `Z`

#### Scenario: Grid origin semantics
- **WHEN** a consumer maps voxel index `[i, j, k]` to metric space
- **THEN** the voxel position is computed from the grid min-corner origin plus `[i, j, k] * voxel_size` in the TSDF grid frame

#### Scenario: Optional weights
- **WHEN** integration weights or observation counts are available
- **THEN** the TSDF grid may include weights aligned with the distances grid; otherwise weights are absent without invalidating the TSDF grid
