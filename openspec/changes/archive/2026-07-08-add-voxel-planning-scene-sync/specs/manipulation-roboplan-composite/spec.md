## ADDED Requirements

### Requirement: Dynamic octree obstacles remain scene state
Dynamic octree obstacles SHALL remain Planning world and RoboPlan scene state, not baked into generated robot URDF or SRDF assets.

#### Scenario: Octree obstacle changes after finalization
- **WHEN** a planning voxel-map octree obstacle changes after RoboPlan world finalization
- **THEN** the change MUST be represented through RoboPlan scene/world state updates rather than by regenerating robot model files

### Requirement: RoboPlan octree collision geometry support
`RoboPlanWorld` SHALL support adding, replacing, and removing octree collision geometry as a dynamic obstacle representation.

#### Scenario: Add octree obstacle from pointcloud map
- **WHEN** the planning stack projects a `PointCloud2` map into a RoboPlan octree obstacle
- **THEN** `RoboPlanWorld` MUST add that octree to the RoboPlan scene with the configured obstacle id and resolution

#### Scenario: Replace octree obstacle by id
- **WHEN** a new pointcloud map is projected using the same obstacle id as an existing octree obstacle
- **THEN** `RoboPlanWorld` MUST replace the old octree collision geometry rather than retaining both old and new geometry
