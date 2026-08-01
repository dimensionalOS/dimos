## 1. Demo scene and simulation wiring

- [x] 1.1 Identify the existing xArm MuJoCo scene asset and wrist camera configuration used by `xarm_perception_sim`.
- [x] 1.2 Add or select a minimal demo scene with xArm7, gripper, table, and one stable graspable object.
- [x] 1.3 Ensure the simulated wrist camera publishes depth image and depth camera info streams for reconstruction.
- [x] 1.4 Verify the wrist camera frame can be resolved to `world` through TF during the demo.

## 2. Demo blueprint/script

- [x] 2.1 Create an opt-in VGN grasp demo blueprint or `demo_*` script that does not modify existing xArm simulation defaults.
- [x] 2.2 Compose MuJoCo simulation, scene reconstruction, VGN grasp generation, and Rerun visualization modules.
- [x] 2.3 Configure reconstruction workspace and output rate for the demo scene.
- [x] 2.4 Add a simple demo flow that resets reconstruction, allows/causes depth integration, triggers grasp generation, and keeps visualization live.

## 3. Grasp visualization

- [x] 3.1 Implement `GraspVisConfig` with the fixed v1 parameters from the design.
- [x] 3.2 Add `GraspCandidateArray` Rerun visualization using simplified gripper `LineStrips3D` wireframes.
- [x] 3.3 Map candidate jaw width to finger separation and score/rank to color or highlight style.
- [x] 3.4 Clear or update visualization explicitly when no candidates are available to avoid stale grippers.
- [x] 3.5 Add optional TSDF surface or voxel-point visualization under a stable Rerun entity path.

## 4. Validation

- [x] 4.1 Add a lightweight smoke test for demo imports and blueprint/script construction.
- [x] 4.2 Add a visualization helper test for gripper line geometry shape/count using synthetic grasp candidates.
- [x] 4.3 Run the demo locally and confirm Rerun shows scene pointcloud, TSDF-derived surface/voxels, and grasp wireframes in world alignment.
- [x] 4.4 Document the demo command and expected visual acceptance criteria.
- [x] 4.5 Run `openspec status --change "vgn-mujoco-grasp-demo"` and verify artifacts remain apply-ready.
