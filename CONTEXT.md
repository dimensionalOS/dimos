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

# Grasp Planning

This context defines the language at the boundary between upstream perception, grasp generation, and robot motion planning.

## Language

**Segmented Object Cloud**:
A target-only 3D point cloud supplied by upstream perception in the manipulation planning frame.
_Avoid_: Scene cloud, detection cloud, raw camera cloud

**Feasible Grasp Sequence**:
A connected, collision-free trajectory from the robot's current state through any required safety lift, pre-grasp, grasp, and retreat. Each segment begins at the preceding segment's endpoint. Validation is a no-motion dry run; execution replans each segment from fresh measured state.
_Avoid_: Reachable grasp, feasible pose, independent IK success

**Safety Lift**:
An optional shared preparation segment planned before evaluating grasp candidates. If required and unplannable, the pick aborts once in `PREPARE`; the failure is not attributed to every candidate.
_Avoid_: Pre-grasp failure, candidate rejection

**Retreat Feasibility (MVP)**:
A connected grasp-to-retreat plan that collision-checks the robot and gripper against the non-target scene. The selected target remains excluded; attached-object geometry and held-object clearance are not modeled.
_Avoid_: Payload-safe retreat, attached-object validation

**Live-Scene Validation (MVP)**:
Each segment of a feasible grasp sequence is checked against the latest available planning scene. The MVP does not snapshot the scene or freeze non-target obstacle updates across the sequence. Execution replans against freshly measured state and scene data.
_Avoid_: Atomic scene validation, frozen-scene guarantee

**Gripper Geometry During Validation (MVP)**:
Arm-path collision checks use the gripper configuration currently represented in the planning scene. Dry-run validation does not model the open-to-closed gripper transition or claim separate clearance guarantees for each finger configuration.
_Avoid_: Coordinated arm-gripper plan, validated finger sweep

**Candidate Rejection Reason (MVP)**:
Candidate rejection is reported by failed sequence stage: `pre_grasp_infeasible`, `grasp_infeasible`, or `retreat_infeasible`. Detailed IK and planner outcomes remain diagnostic logs rather than public skill-result categories.
_Avoid_: Backend-specific public failure codes
