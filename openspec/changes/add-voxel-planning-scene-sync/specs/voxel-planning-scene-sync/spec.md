## ADDED Requirements

### Requirement: Pointcloud map planning input
The manipulation planning stack SHALL accept a latest accumulated pointcloud map for planning-scene collision projection.

#### Scenario: Receives accumulated global map
- **WHEN** the planning stack receives a `PointCloud2` representing the latest accumulated voxel mapper `global_map`
- **THEN** it MUST cache that map as the latest planning voxel-map input without requiring Scene Registry integration in v1

### Requirement: RoboPlan octree planning projection
The planning stack SHALL project the latest accumulated pointcloud map into RoboPlan as octree collision geometry.

#### Scenario: Builds octree obstacle before planning
- **WHEN** a planning request runs after a planning voxel map has been received
- **THEN** the RoboPlan planning world MUST contain an octree obstacle built from the latest map points before collision checking the planned path

#### Scenario: Replaces prior octree obstacle
- **WHEN** a newer planning voxel map is synced into RoboPlan
- **THEN** the prior voxel-map octree obstacle MUST be replaced as one logical obstacle rather than appended as independent stale obstacles

### Requirement: Configurable octree resolution
The RoboPlan octree projection SHALL use configurable octree resolution.

#### Scenario: Uses configured resolution
- **WHEN** RoboPlan world configuration provides an explicit octree resolution
- **THEN** the octree projection MUST use that configured resolution when creating the collision geometry

#### Scenario: Defaults to mapper resolution when provided
- **WHEN** the demo wiring provides a voxel mapper resolution and no explicit RoboPlan octree override
- **THEN** the octree projection SHOULD use the mapper resolution as the octree resolution
