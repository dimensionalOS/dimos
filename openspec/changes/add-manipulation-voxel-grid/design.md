## Context

Manipulation planning currently receives robot state through
`ManipulationModule.coordinator_joint_state` and registers modeled obstacles
through `WorldSpec`.  The xArm simulation already has a complete world-frame
`PointCloud2` stream from `MujocoSimModule.pointcloud`, but there is no
planning collision-snapshot path.  This design adds that path without making
the point cloud a persistent semantic world model or changing trajectory
execution.

A planning collision snapshot is complete, pre-filtered occupancy in the
planning world frame.  The source declares the configured snapshot resolution
of 0.05 m; resolution is not inferred from point spacing.  Robot-body points
are removed before the snapshot reaches manipulation.

## Goals / Non-Goals

### Goals

- Accept complete `PointCloud2` snapshots whose frame is the configured
  planning world frame.
- Stage the latest accepted snapshot and synchronize it at the beginning of
  each planning operation.
- Replace accepted collision geometry under one stable native ID through
  `WorldSpec.add_obstacle`, `update_obstacle`, and `remove_obstacle`.
- Represent snapshots natively in RoboPlan as octree geometry and expose the
  accepted backend obstacle set in Viser.
- Keep the xArm MuJoCo simulation blueprint runnable and observable through
  its existing perception, coordinator, and visualization modules.
- Test lifecycle ordering, failure behavior, conversion, wiring, and
  generated blueprint discovery.

### Non-Goals

- No freshness policy for committed snapshots beyond the mapper's bounded
  cloud/pose synchronization contract.
- No collision-point cap; every occupied voxel is registered for collision.
- No changes to trajectory generation, reservation, or execution.
- No new CLI syntax, skill, MCP contract, or persistent/semantic map model.
- No Frontier timeout changes, GUI changes, or IK debug changes.

## DimOS Architecture

Snapshot policy lives in a thin helper outside the `ManipulationModule` class.
`ManipulationModule.planning_voxel_map: In[PointCloud2]` delegates receipt to
that helper, which stages only the latest message.  At planning start the
module asks the helper to validate and convert the staged snapshot and commit
it through `WorldMonitor`.  The helper owns source-specific frame, resolution,
generation, and empty-snapshot policy; it does not bypass the planning world's
existing obstacle interface.

The concrete simulation path is:

`MujocoSimModule.pointcloud: Out[PointCloud2]` → point-cloud self-filter →
`RayTracingVoxelMap.global_map: Out[PointCloud2]` →
`ManipulationModule.planning_voxel_map: In[PointCloud2]`.  A TF pose source
supplies the wrist-camera pose needed by the mapping path.  `MujocoSimModule`
also supplies simulated robot state to the existing coordinator path;
`coordinator_joint_state: In[JointState]` remains the robot-state input.

Camera clouds and odometry are asynchronous. `RayTracingVoxelMap` SHALL keep a
small, fixed-capacity pending-cloud queue. A cloud remains pending while the
odometry watermark is less than its timestamp. Once the watermark is at or
past the cloud timestamp, the mapper compares the preceding buffered pose and
the first pose at or after the cloud timestamp, selecting the nearest pose
within `pose_match_tolerance_s`; it MUST NOT consume the preceding pose early.
If no valid pose exists, the cloud expires only when the watermark reaches or
passes `cloud_stamp + pose_match_tolerance_s`. Every received cloud MUST be
processed at most once: a matched/ready cloud MUST be processed exactly once,
while an expired or capacity-evicted cloud MUST be processed zero times. Each
processed cloud retains its source timestamp. Throttled warnings or counters
MUST distinguish watermark expiry from capacity eviction and include cloud
timestamp, watermark, tolerance/capacity, and pose-gap context. The default
tolerance is 0.1 s and does not
widen the demo's existing synchronization behavior.

The runnable example is a dedicated xArm voxel-planning Viser blueprint in
`dimos.robot.manipulators.xarm.blueprints.simulation`.  It composes
`ManipulationModule`, `MujocoSimModule`, point-cloud self-filtering,
`RayTracingVoxelMap`, the TF pose source, the xArm coordinator, RoboPlan, and
Viser.  It does not modify the existing pick-and-place example or use
`PickAndPlaceModule`, because collision-snapshot synchronization belongs to
the base manipulation planning path.

At planning start, `ManipulationModule` asks the helper to synchronize the most
recently staged snapshot into `WorldMonitor`/`WorldSpec`. The first non-empty
snapshot adds one obstacle; later non-empty snapshots replace its complete
geometry under the same native ID. Backend mutation happens first;
visualization is notified only after the backend reports the corresponding
mutation. An empty accepted snapshot is meaningful and removes that obstacle.

## Decisions

1. **Snapshot contract.** The input is a complete, pre-filtered `PointCloud2`
   in the planning world frame.  A frame mismatch is rejected.  The configured
   resolution is 0.05 m and is carried explicitly; no point-spacing inference
   is performed.

2. **Staging and synchronization.** The adapter/module stages only the latest
   snapshot.  There is no initial freshness policy.  Synchronization occurs at
   planning start, not on every incoming point cloud, so collision-world
   mutation is bounded to the planning lifecycle.

3. **Asynchronous voxel-map inputs.** Cloud and odometry callbacks are not
   assumed to arrive in lockstep. The mapper uses a fixed small implementation
   bound for pending clouds; capacity is not configurable. Before watermark
   readiness it retains the cloud. At readiness it compares the preceding and
   first-at/after poses and chooses the nearest valid sample. It expires only
   at the tolerance boundary when no valid pose exists, retries pending clouds
   on odometry arrival, and processes each cloud once with its source
   timestamp. Every received cloud is handled at most once; ready clouds are
   handled exactly once, and expired or evicted clouds are handled zero times.
   Diagnostics distinguish watermark expiry from capacity eviction and report
   cloud timestamp, watermark, tolerance/capacity, and pose-gap context. The default
   `pose_match_tolerance_s` is 0.1 s. This generic synchronization correction
   is necessary to make the demo deterministic; worker-lifecycle failures are
   out of scope.

4. **Unified obstacle lifecycle.** PR #3108 owns the generic accepted-obstacle
   lifecycle. `add_obstacle()` returns a nonempty native ID only for a newly
   inserted obstacle, `remove_obstacle()` returns true only for actual removal,
   and `update_obstacle(native_id, obstacle)` replaces complete geometry while
   preserving the ID. Backend mutation happens first. A successful full update
   reuses visualization `add_vis_obstacle(native_id, obstacle)` as an upsert;
   it does not emit a remove event. Visualization failures are logged
   synchronization faults and never redefine backend success. The namespace is
   `/manipulation/obstacles/<encoded-native-id>`.

5. **Atomic stable-ID replacement.** The first non-empty snapshot creates the
   stable planning-collision obstacle. Later snapshots call
   `update_obstacle()` with the same native ID. RoboPlan has no native geometry
   replacement call, so it validates first, builds a complete replacement
   scene without holding the active-scene query lock, and atomically publishes
   the scene and obstacle registry. In-flight queries finish against the old
   scene; later queries use the new one. Construction or insertion failure
   leaves the old scene published. Empty snapshots remove the stable obstacle.

6. **RoboPlan adapter.** For RoboPlan, convert each occupied point to native
   OcTree boxes using `(x, y, z, resolution, 1.0, 0.5)` and register the
   resulting geometry with `addOcTreeGeometry`.  The placement uses the valid
   parent planning frame, and the point/transform conversion follows the
   required Fortran-order transform convention.  Unsupported planning
   backends raise an explicit unsupported-operation error before any world
   mutation.  There is no collision registration cap.

7. **Visualization.** Viser projects ordinary accepted backend obstacles under
   their encoded native-ID paths. The reserved planning-collision obstacle uses
   one prompt, capped visualization of the latest valid staged snapshot at
   `/planning/collision_snapshot`; its overlapping accepted OCTREE projection
   is suppressed. Rejected snapshots remain invisible, and render caps never
   change backend data.

8. **Execution boundary.** Plans and trajectories continue through the
   existing planning and coordinator execution paths unchanged.  Snapshot
   synchronization changes collision inputs only.

## Safety / Simulation / Replay

The source contract makes frame correctness and robot self-filtering explicit:
the adapter accepts only planning-world-frame data and assumes the complete
input has already had robot-body occupancy removed. Stable-ID replacement
validates before construction and publishes only a complete replacement scene.
Construction or insertion failure leaves the previous scene active.

The xArm planning self-filter is all-or-nothing for required robot-region TFs:
if any required transform is unavailable, it drops the whole cloud rather than
publishing a partially filtered snapshot. This generic synchronization
correction is required for deterministic demo input; worker-lifecycle failures
remain out of scope.

The runnable xArm demo also exposed a RoboPlan Jacobian detail: RoboPlan returns
the Jacobian in the full Pinocchio velocity space, including gripper, passive,
and possible free-flyer columns.  Planning groups must therefore project by the
authoritative `JointGroupInfo.v_indices` metadata, in DimOS group-local order,
rather than treating positions in `joint_names` as Jacobian columns.

RoboPlan collision queries can be substantially slower than FK and Jacobian
queries. The adapter therefore owns a separate immutable robot-only kinematics
scene and lock. Collision queries and scene publication remain serialized on
the active collision scene, while pose-target IK can continue using the
kinematics scene. Viser uses independent latest-wins workers for those phases:
an IK result updates the target ghost immediately, but collision feasibility
remains pending and Plan stays disabled until validation completes.

The generic Jacobian solver is still available for diagnostics, but the xArm
demo selects Pink for pose-target IK. A real-model comparison showed the
Jacobian iteration diverging for a 5 cm displacement from the demo observation
pose while Pink converged for 1–10 cm displacements. RoboPlan remains the world
and planning backend. MuJoCo publishes the wrist-camera optical transform
directly relative to `world`; mapping no longer waits for a separately derived
`world -> link7` edge.

The xArm MuJoCo example is the primary end-to-end validation path.  It uses
the simulated `MujocoSimModule.pointcloud` and existing xArm coordinator,
planning, and visualization modules; it does not introduce hardware behavior.
Replay behavior and CLI syntax remain unchanged.  In replay or simulation,
an empty accepted cloud clears obstacles, while absence of a new cloud does
not trigger a timeout or automatic clear.

## Risks / Trade-offs

- Stable-ID replacement rebuilds the complete RoboPlan collision scene, which
  consumes CPU and memory proportional to the robot model and registered
  obstacles. Collision-scene publication remains serialized; robot-only
  kinematics use an independent scene and lock.
- RoboPlan registration cost scales with all occupied voxels because collision
  geometry is not capped.  Viser can remain responsive with an independent
  render-only cap, at the cost of a less complete display than the collision
  world.
- A cloud may wait briefly for odometry, and missing odometry may drop it after
  the watermark advances. The bounded queue and explicit tolerance make this
  behavior deterministic without widening the demo.
- Backends without octree support cannot consume this feature and fail before
  mutation rather than approximating the snapshot with another obstacle type.
- A replacement construction or insertion failure leaves the previous scene
  and obstacle registry published and aborts the pending planning operation.
- Drake pose updates must be truthful; if the backend cannot update collision
  state, pose updates are unsupported rather than falsely reported successful.

## Migration / Rollout

The change is additive. Existing modeled obstacles, robot-state synchronization,
planning APIs, trajectory execution, skills, MCP tools, and CLI commands keep
their current contracts. Existing blueprints without the snapshot adapter
continue to plan as before.

Roll out the xArm simulation blueprint first, with the adapter and RoboPlan
octree support enabled where selected. Keep the snapshot input optional so
empty or absent occupancy does not alter existing startup behavior. Regenerate
the built-in blueprint registry after the blueprint wiring is complete with:

`pytest dimos/robot/test_all_blueprints_generation.py`

## Open Questions

- What later freshness policy, if any, should gate planning on snapshot age?
- Should a future backend advertise octree support through a capability on
  `WorldSpec`, rather than rejecting unsupported geometry during validation?
- What default Viser render cap is appropriate for typical xArm scenes?
- Should later versions expose committed snapshot generation/resolution as
  operator status, or remain visualization-only?
