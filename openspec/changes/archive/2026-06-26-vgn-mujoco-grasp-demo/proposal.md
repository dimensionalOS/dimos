## Why

VGN grasp integration is hard to validate from unit tests alone because correctness depends on camera geometry, reconstruction frames, object placement, and visualization. A MuJoCo end-to-end demo gives a repeatable scene where generated grasps can be inspected against the robot, table, object, pointcloud, and TSDF-derived scene output.

## What Changes

- Add a simulation demo around xArm7 with a wrist depth camera, table, and one simple graspable object.
- Compose MuJoCo depth output, scene reconstruction, VGN grasp generation, and Rerun visualization into an opt-in demo blueprint/script.
- Visualize object/table scene pointcloud, optional TSDF surface/voxel points, and simplified gripper grasp candidates in a fixed Rerun layout.
- Add reusable Rerun visualization for `GraspCandidateArray` using simplified gripper wireframes.
- Keep v1 focused on visual/algorithm verification, not autonomous pick execution.

## Capabilities

### New Capabilities
- `vgn-mujoco-grasp-demo`: End-to-end simulated VGN grasp proposal demo with wrist depth camera and Rerun grasp visualization.

### Modified Capabilities
- None.

## Impact

- Affected code areas: xArm simulation blueprints/demos, MuJoCo scene assets, Rerun visualization helpers, and grasp candidate visualization.
- Depends on the core `vgn-tsdf-scene-reconstruction` change for `TSDFGrid`, scene reconstruction, and VGN grasp candidate output.
- Uses existing MuJoCo camera stream support and existing Rerun bridge infrastructure where possible.
- Does not change real robot behavior or existing pick-and-place heuristics.
