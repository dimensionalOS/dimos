# Manipulation Planning

This context defines the language used by DimOS to configure and invoke motion
planning independently of any one planning implementation.

## Language

**Planner backend**:
The motion-planning implementation selected exclusively by `planner.backend`.
_Avoid_: Planner name, planner type

**World backend**:
The implementation that represents the robot, environment, and collision state
used during planning. Its compatibility with the planner backend is validated
separately from backend selection.
_Avoid_: Planner world

**Planning specification**:
The internal interface through which manipulation modules invoke a planner
backend.
_Avoid_: Public planning API, planner API

**Cartesian target**:
An end-effector goal represented by a `PoseStamped` for an absolute pose or a
`Transform` for a relative rigid displacement.
_Avoid_: Twist, Cartesian delta

**Linear Cartesian plan**:
A joint trajectory whose selected end effectors follow straight Cartesian paths
to their Cartesian targets.
_Avoid_: Constrained plan
