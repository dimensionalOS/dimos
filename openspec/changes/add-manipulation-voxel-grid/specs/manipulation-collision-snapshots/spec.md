## ADDED Requirements

### Requirement: Accept complete planning collision snapshots
The manipulation planning world SHALL accept a planning collision snapshot as the complete, pre-filtered occupancy for the current environment. The snapshot MUST already have robot-body occupancy removed, and each occupied voxel MUST be represented in the planning world frame. The snapshot source SHALL use a configured resolution of 0.05 m and MUST NOT infer snapshot resolution from point spacing.

#### Scenario: A complete filtered snapshot is accepted without resolution inference
- GIVEN a snapshot containing all currently occupied environment voxels, with robot-body occupancy removed
- AND the snapshot declares the planning world frame
- AND the snapshot source is configured with a resolution of 0.05 m
- WHEN the snapshot is submitted to manipulation planning
- THEN the complete snapshot is accepted as the collision input
- AND its resolution is treated as 0.05 m regardless of spacing between occupied points

### Requirement: Validate the planning world frame
The planning system MUST validate the declared frame of every submitted snapshot against the configured planning world frame. A snapshot with a different frame SHALL be rejected and MUST NOT become staged or committed collision state.

#### Scenario: A snapshot in the wrong frame is rejected
- GIVEN the planning world frame is `world`
- AND a submitted snapshot declares frame `camera`
- WHEN the snapshot is submitted
- THEN submission fails with a frame-validation error
- AND the previously staged and committed collision states remain unchanged

### Requirement: Synchronize asynchronous camera clouds and odometry
`RayTracingVoxelMap` MUST support asynchronous camera-cloud and odometry
streams. A cloud MUST remain pending while the odometry watermark is less than
the cloud timestamp. Once the watermark is at or past that timestamp, the
mapper MUST compare the preceding buffered pose and the first pose at or after
the cloud timestamp, selecting the nearest pose within configurable
`pose_match_tolerance_s`; it MUST NOT consume the preceding pose early. If no
valid pose exists, the cloud MUST expire only when the watermark reaches or
passes `cloud_stamp + pose_match_tolerance_s`. The pending-cloud capacity MUST
be a fixed small implementation bound, not configurable. Odometry arrival MUST
retry pending clouds, and every received cloud MUST be processed at most once: matched/ready clouds
MUST be processed exactly once, while expired or capacity-evicted clouds MUST
be processed zero times. Processed clouds MUST retain their source timestamp.
Throttled warnings or counters MUST distinguish watermark expiry from capacity
eviction and include cloud timestamp, watermark, tolerance/capacity, and
pose-gap context. The default tolerance MUST be
0.1 s, with valid non-default values plumbed through and invalid values
rejected. This correction MUST NOT widen the demo. It is necessary to make the
demo deterministic; worker-lifecycle failures are out of scope.

#### Scenario: A cloud waits for a matching pose and is processed once
- GIVEN a camera cloud has timestamp `t`
- AND the odometry watermark is less than `t`
- AND buffered poses exist at `t-50ms` and `t+3ms`
- WHEN the watermark reaches `t`
- THEN the preceding and first-at/after poses are compared
- AND the `t+3ms` pose is selected rather than consuming `t-50ms` early
- AND the cloud is processed once with timestamp `t`

#### Scenario: Missing odometry expires and bounds pending clouds
- GIVEN pending clouds have no matching odometry
- WHEN the odometry watermark reaches or passes each cloud timestamp plus the tolerance
- THEN those clouds expire without processing
- AND the fixed small pending queue remains bounded

#### Scenario: Capacity eviction never processes an evicted cloud
- GIVEN the fixed pending-cloud capacity is full
- WHEN another unmatched cloud arrives
- THEN one pending cloud is evicted according to the implementation policy
- AND the evicted cloud is never processed or emitted
- AND diagnostics identify capacity eviction with timestamp and gap context

#### Scenario: Synchronization diagnostics identify watermark expiry
- GIVEN a cloud expires because the watermark reaches its tolerance boundary
- WHEN expiry is recorded
- THEN diagnostics identify watermark expiry rather than capacity eviction
- AND include the cloud timestamp, watermark, and relevant pose gap

#### Scenario: Tolerance configuration is validated and propagated
- GIVEN the default tolerance is 0.1 s
- WHEN a valid non-default tolerance is configured
- THEN the configured value is used for matching and expiry
- WHEN an invalid tolerance is configured
- THEN configuration is rejected before synchronization starts

### Requirement: Drop clouds when required self-filter TF is unavailable
The xArm planning self-filter MUST drop the whole cloud when any required
robot-region TF is unavailable. It MUST NOT publish a partially filtered cloud.

#### Scenario: A missing robot-region transform drops the complete cloud
- GIVEN a wrist-camera cloud requires multiple robot-region transforms
- AND one required transform exists
- AND another required transform is unavailable
- WHEN the self-filter handles the cloud
- THEN no filtered cloud is published for that input

### Requirement: Stage the latest snapshot and commit at plan start
The system SHALL use latest-wins staging: each accepted snapshot replaces any previously staged snapshot that has not yet been committed. The currently staged snapshot MUST be committed when planning starts, using the obstacle lifecycle, before collision-aware planning proceeds. The system MUST NOT reject a staged or committed snapshot based on age or freshness.

#### Scenario: The latest accepted snapshot is committed when planning starts
- GIVEN snapshot A has been accepted and staged
- AND snapshot B is subsequently accepted before planning starts
- WHEN planning starts
- THEN snapshot B is committed
- AND snapshot A is not committed
- AND planning uses snapshot B for collision checks

### Requirement: Empty snapshots clear committed collision state
An accepted empty planning collision snapshot SHALL mean that no occupancy is present. When an empty snapshot is committed, the previously committed collision snapshot MUST be removed so that the planning world is clear of snapshot-derived collision geometry.

#### Scenario: An empty snapshot clears the previous snapshot
- GIVEN a non-empty snapshot is committed in the planning world
- AND an accepted empty snapshot is staged
- WHEN planning starts and commits the empty snapshot
- THEN the previous snapshot-derived collision geometry is removed
- AND the committed planning collision state is empty

### Requirement: Use the unified obstacle lifecycle
`WorldSpec.add_obstacle()` MUST return a nonempty native ID iff an obstacle was newly inserted; duplicate insertion returns `None`, empty names are rejected, and validation/backend errors raise. `remove_obstacle()` MUST return true iff removal occurred. `clear()` MUST remove all obstacles or raise. `update_pose(native_id, pose)` MUST return true only when collision state was updated. `update_obstacle(native_id, obstacle)` MUST replace complete supported geometry while preserving the native ID, return false for an unknown ID, validate before mutation, and raise on backend failure.

Backend mutation MUST occur before visualization notification. Visualization MUST expose PR #3108's add-or-replace `add_vis_obstacle(native_id, obstacle)`, `remove_vis_obstacle(native_id)`, and `clear_vis_obstacles()` operations. A successful full update MUST upsert the existing visualization handle through `add_vis_obstacle` and MUST NOT emit a remove event. Visualization failures are logged synchronization faults and MUST NOT redefine backend success. Scene paths use `/manipulation/obstacles/<encoded-native-id>`; initialization occurs before the first mutation.

#### Scenario: A newly inserted obstacle emits one native-ID add event
- GIVEN a valid nonempty obstacle name that is not present
- WHEN `add_obstacle()` inserts the obstacle
- THEN it returns a nonempty native ID
- AND visualization receives exactly one `add(native_id, obstacle)` event after insertion

### Requirement: Update snapshots atomically under one stable native ID
The first non-empty snapshot MUST add one planning-collision obstacle. Every later non-empty snapshot MUST replace its complete geometry through `update_obstacle()` while preserving that native ID. RoboPlan MUST build the complete replacement scene without holding the active-scene query lock, then publish the replacement scene and obstacle registry in one critical section. In-flight queries MUST finish against the previously published scene, and later queries MUST use the replacement scene. No DimOS query may observe an empty or partially constructed scene.

Replacement MUST validate before construction. If replacement-scene construction or obstacle insertion fails, RoboPlan MUST report the failure, MUST leave the previous scene and registry published, MUST retain the previous committed snapshot, and MUST emit no visualization update.

#### Scenario: A successful update preserves one obstacle identity
- GIVEN snapshot A is committed under native ID `planning-collision`
- AND snapshot B is staged
- WHEN planning synchronizes snapshot B
- THEN complete geometry B replaces A under native ID `planning-collision`
- AND no collision query observes an empty or partial scene
- AND Viser replaces the handle under the same encoded path

#### Scenario: Failed insertion preserves the previous snapshot
- GIVEN snapshot A is committed
- WHEN insertion of replacement B fails
- THEN the scene containing A remains published
- AND A remains the committed snapshot
- AND no visualization update for B is emitted

#### Scenario: Collision queries continue during replacement construction
- GIVEN snapshot A is committed
- AND replacement scene B is still being constructed
- WHEN a collision query starts
- THEN it completes against scene A without waiting for B's construction
- AND B becomes visible only after complete scene publication

### Requirement: Register voxel collision geometry by backend capability
The RoboPlan backend SHALL support registration of an octree snapshot as collision geometry. A planning backend without octree support MUST fail explicitly when asked to register an octree snapshot, rather than silently ignoring, approximating, or treating it as a supported obstacle type.

#### Scenario: RoboPlan registers an octree snapshot
- GIVEN a complete planning collision snapshot with occupied voxels
- AND the selected planning backend is RoboPlan
- WHEN the snapshot is registered for collision checking
- THEN RoboPlan accepts the snapshot as octree collision geometry
- AND every occupied voxel is retained for collision registration

#### Scenario: An unsupported backend fails explicitly
- GIVEN a complete planning collision snapshot with occupied voxels
- AND the selected planning backend does not support octree collision geometry
- WHEN the snapshot is registered
- THEN registration fails with an explicit unsupported-operation error
- AND the occupied voxels are not silently discarded or replaced by an inferred representation

### Requirement: Retain all occupied voxels for collision checking
Collision registration MUST retain every occupied voxel in an accepted snapshot. The system SHALL NOT apply a collision-registration point cap, downsample occupied voxels, or discard voxels based on display limits.

#### Scenario: Dense occupancy remains complete in collision registration
- GIVEN an accepted snapshot contains more occupied voxels than any configured visualization cap
- WHEN the snapshot is committed for collision checking
- THEN all occupied voxels are registered for collision checking
- AND no voxel is removed solely because the snapshot is large

### Requirement: Render accepted backend obstacle lifecycle in Viser
Viser SHALL project ordinary backend-accepted obstacles under encoded native-ID paths. The reserved `planning-collision` OCTREE SHALL instead use one unified visualization of the latest valid staged snapshot under `/planning/collision_snapshot` for prompt perception observability. Viser MUST suppress its overlapping accepted OCTREE projection without changing backend registration. The unified projection MUST remain advisory and MUST NOT imply backend acceptance; rejected snapshots MUST remain invisible. Viser MAY apply independent display caps, and those caps MUST NOT affect collision registration or backend data. Drake pose updates MUST be truthful or explicitly unsupported.

#### Scenario: Visualization reflects accepted state with an independent cap
- GIVEN snapshot A is committed and snapshot B is staged under the same native ID
- AND the Viser display cap is smaller than the number of occupied voxels in B
- WHEN Viser renders the planning scene
- THEN it renders B through the unified advisory planning snapshot layer
- AND it does not render a second accepted OCTREE projection
- AND it may display at most the configured capped subset
- AND rejected input remains absent
- AND the display cap does not remove any voxels from collision registration

#### Scenario: Latest staged snapshot refreshes without waiting for planning
- GIVEN a valid non-empty planning collision snapshot is staged
- WHEN Viser refreshes before a planning operation commits it
- THEN Viser shows the snapshot at `/planning/collision_snapshot`
- AND no overlapping accepted planning-collision layer is created after commit
- AND an accepted empty staged snapshot clears the staged layer promptly

#### Scenario: Duplicate insertion emits no visualization event
- GIVEN an obstacle with native ID `native-a` is already accepted
- WHEN the same obstacle is added again
- THEN `add_obstacle()` returns `None`
- AND no visualization add event is emitted

#### Scenario: Visualization failure does not change backend success
- GIVEN backend insertion succeeds
- AND the visualization add event fails
- WHEN the mutation completes
- THEN the backend operation remains successful
- AND the visualization failure is logged as a synchronization fault

#### Scenario: Full geometry update is visualized as an upsert
- GIVEN an accepted obstacle has native ID `native-a`
- WHEN its complete geometry is updated successfully
- THEN visualization receives `add_vis_obstacle(native-a, replacement)`
- AND visualization receives no remove event
- AND the existing native-ID scene path is replaced

#### Scenario: Pose update follows the backend result
- GIVEN an accepted obstacle has native ID `native-a`
- WHEN its pose is updated
- THEN a visualization pose event is emitted only if collision state was updated
- AND a backend without truthful pose updates reports unsupported behavior

### Requirement: Provide runnable xArm simulation observability
The xArm simulation example SHALL be runnable through the normal DimOS blueprint discovery and run flow, and SHALL expose observable output for the perception-to-planning path: wrist-camera depth occupancy, the filtered planning collision snapshot, its accepted backend planning-world state, and the Viser representation of that accepted state. This capability SHALL be simulation-only and MUST NOT change trajectory execution behavior.

#### Scenario: The xArm simulation demonstrates the complete collision path
- GIVEN the xArm simulation blueprint is discovered by the normal blueprint listing
- WHEN the blueprint is run with simulation enabled
- THEN wrist-camera depth produces observable occupancy input
- AND the observable input is filtered into a planning collision snapshot
- AND the snapshot is observable in the planning world after commit
- AND Viser shows the accepted backend obstacle state
- AND trajectory execution behavior is unchanged

### Requirement: Keep RoboPlan kinematics independent from collision queries
RoboPlan forward-kinematics and Jacobian queries SHALL use a robot-only kinematics scene whose lock is independent from the active collision scene. Viser pose-target IK and collision validation SHALL run as independent latest-wins work. A long-running collision query or collision-scene publication MUST NOT prevent a newer pose-target IK request from making kinematics progress or updating the target ghost. Planning actions MUST remain disabled until collision validation succeeds.

#### Scenario: IK kinematics continues during a collision query
- GIVEN a collision query is blocked while holding the active collision-scene lock
- WHEN pose-target IK requests a link pose or group Jacobian
- THEN the kinematics query completes through the robot-only scene
- AND it does not wait for the collision query to release its lock
- AND its solved target ghost may update while feasibility remains checking

### Requirement: Use convergent demo IK and a direct simulated camera pose
The xArm voxel-planning demo SHALL use Pink for pose-target IK while retaining RoboPlan as its collision-world and motion-planning backend. The simulated camera SHALL publish its optical transform directly relative to `world`, so mapping pose availability does not depend on the manipulation module's derived link transforms.

#### Scenario: Demo pose and camera paths remain independent
- GIVEN the xArm voxel-planning demo is running
- WHEN a Cartesian target is moved and a wrist-camera frame is produced
- THEN pose-target IK is evaluated by Pink
- AND motion planning remains backed by RoboPlan
- AND `world -> wrist_camera_color_optical_frame` is available directly from simulation
