## ADDED Requirements

### Requirement: Viser planning map visualization
The system SHALL display the planning pointcloud or voxel-map layer directly in Viser for the voxel-backed manipulation demo.

#### Scenario: Visualizes planning map layer
- **WHEN** the voxel-backed demo is running and a planning voxel map has been published
- **THEN** Viser MUST render the map layer used for planning collision checks alongside the robot visualization

#### Scenario: Updates map visualization
- **WHEN** a new planning voxel map replaces the prior map
- **THEN** Viser MUST update the rendered map layer to reflect the latest planning map

### Requirement: Integrated sim manipulation Viser demo
The system SHALL provide a final demo entrypoint that launches MuJoCo simulation, manipulation planning, voxel-map planning sync, and Viser visualization together.

#### Scenario: Launches integrated demo stack
- **WHEN** the final voxel-backed demo blueprint is launched
- **THEN** it MUST include MuJoCo simulation, the manipulation planning module, TF-pose odometry, pointcloud self-filtering, Rust ray-tracing voxel mapping, RoboPlan octree projection, and Viser visualization

#### Scenario: User inspects and plans around map
- **WHEN** the integrated demo is running with obstacles visible to the wrist RGBD camera
- **THEN** the user MUST be able to see the pointcloud/voxel-map layer in Viser and manually verify manipulation planning that collision-checks around the corresponding RoboPlan octree obstacle
