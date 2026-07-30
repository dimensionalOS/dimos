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
