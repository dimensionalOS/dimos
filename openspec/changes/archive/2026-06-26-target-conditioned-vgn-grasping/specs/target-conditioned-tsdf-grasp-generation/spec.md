## ADDED Requirements

### Requirement: Target-bounds TSDF grasp generation
The TSDF grasp generation capability SHALL support generating grasp candidates for a target region described by world-frame Target bounds.

#### Scenario: Generate from latest TSDF and target bounds
- **WHEN** the TSDF grasp generator has a latest TSDF and receives target center, target size, and cushion distance
- **THEN** it generates a `GraspCandidateArray` for geometry inside the cushioned target bounds or returns a clear no-result outcome

#### Scenario: No latest TSDF available
- **WHEN** target-bounds grasp generation is requested before any TSDF has been received
- **THEN** the generator does not publish stale candidates and reports that no TSDF is available

### Requirement: Grasp generation decoupled from object identity
The low-level TSDF grasp generation spec SHALL accept Target bounds and SHALL NOT require object ids or object database access.

#### Scenario: Caller passes bounds instead of object id
- **WHEN** a caller requests target-conditioned TSDF grasp generation
- **THEN** the request contains target center, target size, and cushion rather than an object id or `RegisteredObject`

### Requirement: Target-masked TSDF preprocessing
The VGN grasp generator SHALL construct a Target-masked TSDF internally before target-conditioned inference.

#### Scenario: Suppress outside target bounds
- **WHEN** a Target-masked TSDF is constructed from a full scene TSDF
- **THEN** voxels outside the target bounds expanded by the cushion are suppressed so VGN does not select grasps on outside clutter

#### Scenario: Preserve target cushion
- **WHEN** target bounds are expanded by the configured cushion
- **THEN** voxels inside the cushioned bounds remain available for VGN inference

#### Scenario: Original TSDF remains unchanged
- **WHEN** target-conditioned generation masks a TSDF for inference
- **THEN** the module does not mutate the stored full-scene latest TSDF

### Requirement: Target frame handling
The target-conditioned TSDF grasp generator SHALL transform Target bounds into the TSDF grid frame before masking when frames differ.

#### Scenario: Target transform available
- **WHEN** target bounds are provided in a frame that can be transformed to the TSDF frame
- **THEN** the generator masks the TSDF using the transformed bounds and publishes candidates in the configured output frame

#### Scenario: Target transform unavailable
- **WHEN** target bounds cannot be transformed into the TSDF frame
- **THEN** the generator reports a clear transform failure and does not publish misleading candidates

### Requirement: Workspace-level generation preserved
The system SHALL preserve target-agnostic workspace TSDF grasp generation.

#### Scenario: Workspace generation still available
- **WHEN** a caller invokes workspace-level `generate_grasps_from_tsdf`
- **THEN** VGN inference runs on the provided TSDF without applying object target bounds
