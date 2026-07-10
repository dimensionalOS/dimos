# DimOS Robotics

DimOS composes robot capabilities into plans that can be previewed and executed across one or more robots.

## Language

**Planning**:
The complete process of finding a geometric motion and parameterizing it in time for preview and execution.
_Avoid_: Using “planning” to mean geometric path finding alone

**Path**:
A geometric motion through configuration space without assigned timing, represented by planner-produced waypoints.
_Avoid_: Trajectory

**Waypoint**:
A discrete robot configuration emitted by a planner as part of a path and consumed by trajectory parameterization.
_Avoid_: Trajectory point, path point

**Planned Joint**:
A joint explicitly included in a generated plan’s waypoints and synchronized trajectory. A generated plan does not command joints outside this set.
_Avoid_: Every joint belonging to an affected robot

**Generated Plan**:
The completed outcome of planning, containing both its geometric waypoints and one synchronized parameterized motion. An incomplete plan may exist transiently during construction but is not externally observable.
_Avoid_: Path, planner result

**Synchronized Trajectory**:
A parameterized motion on one shared relative clock for every planned joint.
_Avoid_: Per-robot trajectory collection

**Trajectory Parameterization**:
The assignment of timing and motion derivatives to a geometric waypoint path, producing the motion used by preview and execution.
_Avoid_: Preview sampling, path projection
