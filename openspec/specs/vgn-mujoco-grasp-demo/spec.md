## ADDED Requirements

### Requirement: Simulated VGN grasp demo
The system SHALL provide an opt-in MuJoCo demo that composes xArm simulation, wrist depth camera output, scene reconstruction, VGN grasp generation, and visualization.

#### Scenario: Demo composition starts
- **WHEN** the demo blueprint or script is launched with required optional dependencies available
- **THEN** it starts the simulated xArm scene, depth camera streams, scene reconstruction, VGN grasp generation, and Rerun visualization path

#### Scenario: Existing simulation remains unchanged
- **WHEN** existing xArm simulation blueprints are launched
- **THEN** their behavior is not changed by the VGN demo unless the new demo is selected explicitly

### Requirement: Minimal graspable scene
The demo SHALL include a repeatable simulated scene with xArm7, gripper, table, and one simple stable graspable object.

#### Scenario: Scene contains object and support
- **WHEN** the demo scene loads
- **THEN** the robot, gripper, table, and graspable object are present in consistent world-frame positions

#### Scenario: Object suitable for first validation
- **WHEN** VGN reconstruction and inference run against the demo object
- **THEN** the object scale and placement are compatible with parallel-jaw grasp proposal visualization

### Requirement: Wrist depth camera validation
The demo SHALL use a wrist-mounted simulated depth camera attached through the existing xArm/MuJoCo camera mechanism.

#### Scenario: Depth streams available
- **WHEN** the demo starts
- **THEN** depth image and depth camera info streams are available for scene reconstruction

#### Scenario: Camera transform resolvable
- **WHEN** the reconstruction module receives a depth image from the wrist camera
- **THEN** the camera frame can be resolved to `world` through TF or the frame failure is reported clearly

### Requirement: Rerun scene visualization
The demo SHALL visualize enough scene context in Rerun to judge whether generated grasps align with the object.

#### Scenario: Scene products visible
- **WHEN** reconstruction products are available
- **THEN** Rerun shows the object/table scene pointcloud and a TSDF-derived surface or voxel visualization in world coordinates

#### Scenario: Fixed Rerun entity layout
- **WHEN** the demo logs visualization data
- **THEN** it uses stable entity paths under `world/`, including `world/scene_pointcloud`, `world/tsdf_surface`, and `world/grasp_candidates`

### Requirement: Simplified gripper candidate visualization
The demo SHALL visualize grasp candidates as simplified gripper wireframes rather than pose arrows only.

#### Scenario: Candidate wireframes rendered
- **WHEN** a `GraspCandidateArray` is available
- **THEN** Rerun displays up to the configured maximum number of candidates as line-strip gripper wireframes transformed by each candidate pose

#### Scenario: Width and score visible
- **WHEN** candidates include jaw width and score
- **THEN** jaw width controls finger separation and score controls ordering or color so top candidates are visually distinct

#### Scenario: No candidates
- **WHEN** VGN returns no candidates
- **THEN** the demo reports the no-candidate outcome clearly instead of showing stale grippers
