---
title: "xArm Voxel Planning with Viser"
---

# xArm Voxel Planning with Viser

This simulation exercises the complete wrist-RGBD-to-collision-planning path:

```text
MuJoCo RGB-D capture
  ├─ pointcloud ──────────────> capture-triggered TF pose ─> Odometry ─┐
  └─ pointcloud ─> robot-model self-filter ─> filtered cloud ──────────┤
                  └─ metric voxel-clear mask ──────────────────────────┤
                                                                     v
                                                   RayTracingVoxelMap.global_map
                                                                     |
                                                                     v
                                                   GlobalMapObstacleBridge
                                                                     |
                                     add/update/remove Obstacle RPC (stable ID)
                                                                     |
                                                                     v
                                                        RoboPlan + Viser
```

The camera pose estimate and cloud use the capture timestamp. The simulator
publishes the real mounted `link7 -> wrist_camera_color_optical_frame` edge,
while `RobotTfPublisher` publishes the complete robot tree from each measured
joint state. The pose adapter emits exactly one `world <- camera` Odometry for
each cloud and never falls back to the latest unrelated pose.

`PointCloudSelfFilter` removes points inside padded URDF collision geometry,
including the base, arm, and articulated gripper. Missing capture-time TF for
any required link drops the whole capture. Its independent clear mask contains
the previous and current robot volumes as metric world-frame positions. The
native mapper applies masks monotonically as authoritative free-space evidence;
cloud processing never requires a mask, so navigation and manipulation use the
same mapper behavior. Cross-topic delivery is eventually consistent: a late
mask may temporarily remove a newly exposed surface until the next cloud.

The accumulated world map is reconciled as the single RoboPlan OCTREE obstacle
`mapping/global-voxel-map`. Non-empty maps update first and add if the obstacle
does not exist; empty maps remove it. The async bridge allows one RPC at a time
and coalesces backlog to the newest map. Mapper silence retains the last
accepted obstacle. OCTREE registration is intentionally RoboPlan-only.

Viser renders the backend-accepted OCTREE with a display-only point cap. The
cap does not change collision geometry.

## Run the manual test

Install the required dependencies and start the blueprint:

```bash
uv sync --extra manipulation --extra sim --inexact
dimos run xarm-voxel-planning-viser-demo
```

Open the Viser URL printed in the log, then check the following in order:

1. Before moving the arm, compare the simulated desk to its blue OCTREE. The
   desk must have the same world position and scale; it must not appear closer
   to the camera than the rendered desk.
2. Inspect the base, links, wrist, fingers, and knuckles. No persistent blue
   voxels should overlap the robot. Nearby desk voxels must remain.
3. Move the end-effector gizmo through several poses while watching the map.
   Static desk edges must stay fixed in world coordinates instead of smearing
   or drifting with the wrist.
4. Return the arm toward a previous pose. Vacated arm and gripper volumes must
   not leave a collision trail.
5. Request a collision-aware plan. The gizmo should report IK/collision status,
   the plan should finish without timing out, and execution should transition
   out of `EXECUTING` automatically when the coordinator completes.
6. Stop point-cloud production briefly, if convenient. The last accepted
   obstacle must remain; the planning world must not silently become empty.

During motion, occasional throttled missing-TF or dropped-capture warnings may
appear at startup. Repeated warnings, pending-capacity evictions, cloud expiry,
or a continuously missing `link_base` transform indicate a failed test.

## Limits

- The voxel size is 0.05 m across the filter, mapper, and OCTREE bridge.
- Grasped payloads are not part of self-exclusion in this change.
- Sensor-confidence or soft-collision costs are not implemented; OCTREE voxels
  remain hard collision geometry.
