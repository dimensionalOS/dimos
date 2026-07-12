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

**Plan Freshness**:
The condition that current planned-joint positions still match a generated plan’s synchronized trajectory start within the accepted tolerance.
_Avoid_: UI snapshot freshness, telemetry freshness

**Synchronized Trajectory**:
A parameterized motion on one shared relative clock for every planned joint.
_Avoid_: Per-robot trajectory collection

**Trajectory Parameterization**:
The assignment of timing and motion derivatives to a geometric waypoint path, producing the motion used by preview and execution.
_Avoid_: Preview sampling, path projection

**Manipulation Operator**:
The human-facing control boundary attached to a manipulation visualization session, through which a client inspects manipulation state and requests target evaluation, planning, preview, execution, cancellation, or reset. It is shared by visualization clients rather than belonging to a particular renderer.
_Avoid_: Visualization backend, renderer

**Target Draft**:
A frontend-owned, uncommitted set of desired joint or pose targets. A target draft is not a generated plan and has no execution authority.
_Avoid_: Plan, planned path

**Target Evaluation**:
An advisory assessment of a target draft’s feasibility and resulting joint or pose state. It is feedback for an operator and is not planning or execution authority.
_Avoid_: Generated plan, validated plan

**Preview**:
A transient visual playback of a generated plan’s synchronized trajectory. A preview has no execution authority and disappears when playback finishes or is cancelled.
_Avoid_: Target ghost, generated plan
