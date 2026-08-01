## Why

The new agentic grasp simulation can plan trajectories that collide with simulated obstacles because motion planning does not consume the same accumulated scene belief that perception and grasp generation observe. We need a direct v1 path from wrist RGBD observations to RoboPlan collision geometry so the manipulation planner avoids obstacles in the xArm grasp demo.

## What Changes

- Add a reusable TF-to-pose source that publishes pose-only camera-pose odometry from an existing TF chain.
- Add a reusable pointcloud self-filter that removes TF-anchored robot/gripper regions before pointclouds enter mapping.
- Wire an xArm MuJoCo manipulation/Viser demo through self-filtering, Rust ray-tracing voxel mapping, and RoboPlan octree projection.
- Add a final integrated demo that launches MuJoCo simulation, manipulation planning, and Viser together.
- Render the planning pointcloud/voxel-map layer directly in Viser so users can inspect the map the planner uses.
- Extend RoboPlan planning-world support to consume a pointcloud/voxel-map stream as one replace-on-update octree collision obstacle.
- Add backend/world configuration for octree resolution, defaulting to the voxel mapper resolution when possible.
- Keep Scene Registry integration out of v1; preserve a migration path where the same global map becomes a `SceneEntity(payload=PointCloud2)` later.

## Capabilities

### New Capabilities
- `tf-pose-source`: Provides a generic TF-derived pose-only odometry stream for sensors whose pose is represented in the TF tree.
- `pointcloud-self-filtering`: Removes robot-owned geometry from pointcloud streams using TF-anchored primitive exclusion regions.
- `voxel-planning-scene-sync`: Projects an accumulated voxel/pointcloud belief into the manipulation planning world as a RoboPlan octree obstacle.
- `voxel-planning-visualization`: Displays the planning pointcloud/voxel-map layer in Viser for inspection alongside the robot and planned motion.

### Modified Capabilities
- `manipulation-roboplan-composite`: RoboPlan world integration shall support octree collision geometry as a planning-world obstacle representation.

## Impact

- Affected modules: `MujocoSimModule`, `ManipulationModule`, RoboPlan world backend, Viser visualization, Rust `RayTracingVoxelMap` demo wiring, and new TF/pointcloud utility modules.
- Affected blueprints: xArm MuJoCo manipulation/Viser demo blueprint or a derived demo blueprint.
- Affected specs/tests: TF-derived odometry, pointcloud self-filtering, RoboPlan octree obstacle projection, and demo wiring tests.
- No breaking changes are intended; v1 direct wiring should coexist with existing perception, grasp generation, and PickAndPlace-based flows.
