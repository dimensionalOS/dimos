# Grasp Planning

This context defines the language at the boundary between upstream perception, grasp generation, and robot motion planning.

## Language

**Segmented Object Cloud**:
A target-only 3D point cloud supplied by upstream perception in the manipulation planning frame.
_Avoid_: Scene cloud, detection cloud, raw camera cloud

**Grasp Provider**:
The single configured source of ranked grasp candidates for a Pick/Place Transaction. Provider order is authoritative, provider failure ends proposal generation, and a transaction never switches providers automatically.
_Avoid_: Grasp fallback, grasp strategy

**Feasible Grasp Sequence**:
A connected trajectory from the robot's current state through any required safety lift, pre-grasp, grasp, and retreat. Preparation and pre-grasp approach are collision-checked; intentional contact legs use Unchecked Contact Motion. Each segment begins at the preceding segment's endpoint, validation is a no-motion dry run, and execution replans from fresh measured state.
_Avoid_: Reachable grasp, feasible pose, independent IK success

**Unchecked Contact Motion**:
A short, straight TCP motion that intentionally bypasses all planning-scene collision checks while retaining sequential kinematic feasibility, joint limits, tracking, timing, and execution-result validation.
_Avoid_: Collision-free contact, target-only collision allowance, unvalidated motion

**Safety Lift**:
An optional shared preparation segment planned before evaluating grasp candidates. If required and unplannable, the pick aborts once in `PREPARE`; the failure is not attributed to every candidate.
_Avoid_: Pre-grasp failure, candidate rejection

**Retreat Feasibility (MVP)**:
A connected grasp-to-retreat motion validated through sequential collision-disabled IK. Planning-scene obstacles, attached-object geometry, and held-object clearance are not checked during this contact leg.
_Avoid_: Payload-safe retreat, attached-object validation

**Live-Scene Validation (MVP)**:
Each collision-checked segment of a Feasible Grasp Sequence uses the latest available planning scene. The MVP does not snapshot the scene or freeze obstacle updates across the sequence; Unchecked Contact Motion does not query the planning scene.
_Avoid_: Atomic scene validation, frozen-scene guarantee

**Gripper Geometry During Validation (MVP)**:
Arm-path collision checks use the gripper configuration currently represented in the planning scene. Dry-run validation does not model the open-to-closed gripper transition or claim separate clearance guarantees for each finger configuration.
_Avoid_: Coordinated arm-gripper plan, validated finger sweep

**Candidate Rejection Reason (MVP)**:
Candidate rejection is reported by failed sequence stage: `pre_grasp_infeasible`, `grasp_infeasible`, or `retreat_infeasible`. Detailed IK and planner outcomes remain diagnostic logs rather than public skill-result categories.
_Avoid_: Backend-specific public failure codes

**Pipeline Demo**:
A no-hardware contributor command that runs a recorded Segmented Object Cloud through real grasp proposal and connected motion validation, then saves candidate outcomes and all planned segments. It stops before trajectory execution.
_Avoid_: Grasp-only demo, hardware pick demo

**Visualization Layer**:
A display-only, named collection of visual elements owned by exactly one producer. Publishing replaces its contents, while clearing leaves the layer registered and preserves viewer-owned visibility; the layer cannot affect collision checking or other planning behavior.
_Avoid_: Collision layer, shared scene state, visualization object

**Visual Element**:
A backend-neutral drawable contained in a Visualization Layer, initially a point cloud or line set. It carries no collision or planning authority.
_Avoid_: Grasp visualization command, Viser handle, collision object

**Visualization Layer Group**:
A viewer-only grouping of independently replaceable and toggleable Visualization Layers that share a name prefix, such as `grasp/object-cloud` and `grasp/proposals`.
_Avoid_: Compound layer, element-level visibility

**Accepted Collision Projection**:
A display-only representation published after the planning world accepts a collision-object change. Its presence, absence, or rendering failure never changes collision checking.
_Avoid_: Collision authority, visualization obstacle

## Manipulation Tasks

**Detection Snapshot**:
The immutable, numbered set of objects produced by one completed scene scan.
_Avoid_: Live detections, object list

**Object Obstacle Proxy**:
Planning geometry derived for one detected object in a Detection Snapshot. It represents the best collision evidence available from its source and is not assumed to be exact physical geometry.
_Avoid_: Ground-truth geometry, object mesh, detection

**Object Number**:
A human-facing selection handle scoped to one Detection Snapshot. It has no meaning outside that snapshot and is never a persistent object identity.
_Avoid_: Object ID, stable ID

**Selected Object**:
The object record pinned from a Detection Snapshot before a Pick/Place Transaction begins. Later detection ordering cannot change which physical object the selection denotes.
_Avoid_: Object number, current detection

**Prepared Pick**:
A Selected Object together with its pinned Segmented Object Cloud and ranked grasp candidates. It is inspection input for a later Pick/Place Transaction and carries no reservation or continuing feasibility guarantee; a later scan, preparation timeout, or planning-frame mismatch makes it stale.
_Avoid_: Planned pick, reserved pick, feasible grasp

**Verified Pick**:
A completed pick whose configured gripper feedback indicates that an object prevented empty closure. Pick completion without that evidence is a failure, not an unverified success.
_Avoid_: Assumed hold, commanded pick, unverified pick

**Object Placement Target**:
The desired world position of the held object's reference point, initially its detected center. It is not a robot TCP target; the Pick/Place Transaction derives the required TCP pose from the retained grasp relationship.
_Avoid_: TCP target, gripper position, drop pose

**Manipulation World Frame**:
The canonical `world` frame in which Detection Snapshots, Segmented Object Clouds, grasp candidates, Object Obstacle Proxies, and Object Placement Targets are expressed.
_Avoid_: Planning-frame option, robot base coordinates, camera coordinates

**Pick/Place Transaction**:
A single physical operation that picks one selected object and either places it at a target pose or into a selected container. One owner has exclusive control of the selected robot from validation through completion and owns the physical success or failure outcome, but not the policy for choosing a sequence of objects.
_Avoid_: Box-filling workflow, manipulation task

**Box-Filling Task**:
A task policy that selects which objects belong in a container and invokes one Pick/Place Transaction for each selected object. Its container measurement and drop behavior are application-specific and are not part of the generic Pick/Place Transaction interface.
_Avoid_: Pick/place transaction, grasp pipeline

# Manipulation Planning

This context describes requests for planning robot motion through joint and Cartesian spaces.

## Language

**Cartesian Waypoint**:
One absolute TCP pose or relative rigid displacement within a Cartesian target.

**Cartesian Target**:
An ordered, homogeneous sequence of Cartesian waypoints for one planning group, including its starting waypoint. An absolute target contains only `PoseStamped` waypoints and starts at the current TCP pose. A relative target contains only `Transform` waypoints, starts with the identity transform, and measures every waypoint from the planning-start TCP pose.
_Avoid_: Cartesian track

**Cartesian Path Configuration**:
Per-planning-call policy that selects how Cartesian waypoints are connected and constrains that operation. It is independent of the startup configuration that selects and constructs a planner backend.

**Standard Cartesian Planning**:
Cartesian waypoint planning through a backend's supported serializable options. For RoboPlan, this includes multi-waypoint and simultaneous multi-end-effector paths, bounded and time-optimal speed modes, tracking tolerances, and solver tuning.

**Bounded Speed Mode**:
A Cartesian timing policy that treats configured tool speeds and accelerations as maxima and slows the motion further when required by tracking or joint limits.

**Time-Optimal Speed Mode**:
A Cartesian timing policy that resolves the requested path into joint space and retimes it against joint limits, optionally blending intermediate corners.

**Custom Planner Components**:
Backend-native solver tasks, constraints, and barriers injected as live objects. These are outside standard Cartesian planning and require a separate constrained-IK interface.
