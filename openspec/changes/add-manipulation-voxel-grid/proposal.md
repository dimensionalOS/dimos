# Proposal: capture-aligned manipulation voxel mapping

## Problem

The xArm wrist camera produced point clouds and poses on independent timers, so
motion registered static objects at the wrong world positions. The camera also
sees the robot base, arm, and gripper; those returns became hard planning
obstacles and historical robot voxels could remain after motion.

## Change

- Establish a shared mapper contract for navigation and manipulation:
  sensor-frame `PointCloud2` plus fixed-frame sensor `Odometry`.
- Generate wrist-camera Odometry once per cloud by querying TF at the cloud's
  capture timestamp.
- Buffer clouds in the native mapper until the corresponding pose arrives,
  with bounded capacity, timestamp tolerance, frame validation, and expiry.
- Remove modeled robot collision geometry before mapping and provide optional,
  independent metric voxel-clear masks to remove current and vacated volumes.
- Publish the complete robot TF tree from measured joint state in a reusable
  module.
- Reconcile every complete global map through typed `Obstacle` add/update/remove
  RPCs under one stable RoboPlan OCTREE ID.
- Provide a MuJoCo + Viser blueprint and manual verification guide.

## Non-goals

- Soft collision costs or sensor-confidence weighting.
- Grasped-payload self exclusion.
- OCTREE support in Drake.
- TF interpolation; capture-triggered closest-sample lookup uses tight bounded
  tolerances.
