# Design: capture-aligned manipulation voxel mapping

## Data flow

```text
camera capture
  ├─ cloud ─> PointCloudTfPoseSource ─> capture-stamped Odometry ─┐
  └─ cloud ─> PointCloudSelfFilter ─> filtered cloud ─────────────┤
               └─ metric voxel_clear_mask ────────────────────────┤
                                                                v
                                                RayTracingVoxelMap
                                                  └─ global_map
                                                        v
                                            GlobalMapObstacleBridge
                                      typed Obstacle mutation RPCs
                                                        v
                                                   RoboPlan/Viser

measured JointState ─> RobotTfPublisher ─> complete robot TF tree
MuJoCo camera capture ───────────────────> mounted camera TF edges
```

## Mapper contract

`RayTracingVoxelMap` remains application-neutral. Its required inputs are a
sensor-frame cloud and an Odometry whose child is the cloud frame and whose
parent equals configured `map_frame`. It buffers clouds until the odometry
watermark reaches the cloud stamp, then chooses the closest frame-compatible
pose within `pose_match_tolerance_s`. Capacity and watermark expiry bound
memory and latency. This improves navigation too, because PointLIO/FastLIO may
publish a same-stamp cloud before its Odometry.

The optional `voxel_clear_mask` contains metric positions in `map_frame`. Rust
quantizes them using its own voxel size and clears them immediately. Masks are
monotonic by timestamp and never gate cloud processing. This preserves one
mapper behavior across navigation and manipulation. Two independent topics
cannot provide atomic clear-and-integrate ordering, so cleanup is eventually
consistent; the next cloud restores a real surface erased by a late mask.

## Capture-time TF

`PointCloudTfPoseSource` is cloud-triggered. For each cloud it queries
`fixed_frame <- cloud.frame_id` at `cloud.ts` using explicit backward and
bounded-forward tolerances. A successful estimate is published once as
Odometry stamped exactly `cloud.ts`. Failure emits no pose; there is no latest
fallback or periodic resampling.

`RobotTfPublisher` owns robot-model FK publication. Each complete measured
joint state yields one fixed-frame base transform and every URDF parent-child
edge at the state timestamp. The simulator publishes the physical
`link7 -> camera` mounting chain, matching the real-camera topology.

## Self exclusion

`PointCloudSelfFilter` loads all URDF collision geometry, including gripper
links. Analytic primitives use exact padded containment; meshes use trimesh
signed distance with `rtree`. All required link transforms are queried at the
capture timestamp. Any missing transform drops the complete capture.

The filter removes robot returns and emits metric samples covering the union
of previous and current padded robot volumes. Previous-volume cleanup prevents
ghost trails that ordinary ray clearing cannot guarantee after the camera or
robot moves. Attachments must be modeled in the authoritative robot model; the
demo has no hand-written link box.

## Planning-world bridge

Manipulation exposes complete typed `Obstacle` add/update/remove RPCs. The
bridge consumes complete global maps with latest-wins async dispatch, allowing
one blocking RPC at a time. A non-empty map becomes a world-frame OCTREE named
`mapping/global-voxel-map`; reconciliation calls update first, then add if it
does not exist. Empty maps remove the ID idempotently. Mapper silence retains
the previous obstacle. RoboPlan scene replacement remains atomic under its
existing obstacle lifecycle and Viser renders only accepted backend state.

The bridge rejects non-world frames, non-finite points, and non-RoboPlan
configuration at startup. Full-map RPC is deliberately supported over the
configured LCM/Zenoh pickle transport; default SHM RPC slots are not suitable
for these payloads.
