# Manipulation Planning

This context describes the environment representation used to plan collision-free robot motion.

## Language

**Planning Collision Snapshot**:
The complete current environment occupancy supplied to motion planning for collision checks. Each valid input supersedes older staged input; an empty accepted snapshot clears it, and robot-body occupancy must already have been removed. Non-empty snapshots occupy one stable backend obstacle ID and replace its complete geometry atomically from DimOS queries. It is neither semantic nor persistent.
_Avoid_: Planning voxel map, persistent world model, semantic map

After a replacement failure, `committed()` remains the last backend-confirmed snapshot.

**Fresh Planning Collision Snapshot**:
A valid Planning Collision Snapshot whose Capture Time remains within the planning safety horizon. Only a fresh snapshot may start a new plan; stale geometry remains registered until a fresh snapshot replaces it.
_Avoid_: Latest map, cached map, usable stale snapshot

**Planning World Frame**:
The canonical coordinate frame in which planning obstacles are expressed. A Planning Collision Snapshot is accepted only when its declared frame matches this frame.
_Avoid_: Implicit frame, assumed frame

**Snapshot Resolution**:
The configured edge length represented by each occupied point in a Planning Collision Snapshot. The initial snapshot source uses 0.05 metres and does not infer resolution from point spacing.
_Avoid_: Point spacing, inferred resolution

**Capture Time**:
The instant at which a sensor observation was acquired. It is the authoritative time for resolving the observation's pose and robot state, independent of when the observation or transforms are published.
_Avoid_: Publication time, processing time, current time

**Robot Self Geometry**:
The complete collision geometry belonging to the robot, including its base, articulated links, gripper, sensor mounts, and registered attachments. It is derived from the robot model and excluded from environment occupancy.
_Avoid_: Self-collision obstacles, camera blind mask, hand-authored exclusion zones

**Self-Exclusion Padding**:
The uniform distance around Robot Self Geometry within which sensor returns are treated as robot returns. It covers aggregate sensing, calibration, timing, and model error and is independent of Snapshot Resolution.
_Avoid_: Collision padding, voxel size, per-link margin

**Occupancy Evidence**:
Sensor-derived belief that a region may be occupied, before planning policy promotes it to a collision constraint. Uncertain evidence may influence path preference but does not permit penetration of confirmed obstacles.
_Avoid_: Soft obstacle, small collision, allowed collision

**Confirmed Occupancy**:
Occupancy evidence strong enough to become a hard planning collision constraint. Confirmed occupancy is not geometrically penetrable, regardless of how small an intersection appears.
_Avoid_: High-cost obstacle, mostly hard obstacle
