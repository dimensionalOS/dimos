## Context

On `main`, the DimOS `PlannerSpec` Protocol exposes only robot-scoped and
planning-group-scoped joint-path operations. `RoboPlanWorld` implements both
`WorldSpec` and `PlannerSpec`, owns the native RoboPlan scene, and currently uses
RoboPlan RRT for joint paths. `RRTConnectPlanner` is the other planner implementation.
`PlanningResult` already supports globally named `JointState` waypoints, optional
velocities, and optional timestamps.

RoboPlan's official `CartesianPathPlanner` accepts one or more end-effector paths and a
start configuration, then returns a timed joint trajectory. Its bounded-speed mode
checks Cartesian tracking during servo integration, while its time-optimal mode blends
a resampled joint trace without revalidating the final blend against the Cartesian
path. Neither mode performs the DimOS planning-world collision validation required
before a result is accepted.

This change concerns the internal planner Protocol and backend configuration. A
caller-facing `ManipulationModule`/RPC API will be designed separately.

## Goals / Non-Goals

**Goals:**

- Add one concise, backend-neutral internal operation for linear Cartesian paths.
- Preserve the established target-plus-auxiliary-group convention.
- Support absolute and relative world-frame targets in the same request.
- Support synchronized multi-group planning and return RoboPlan's timing.
- Keep trajectory acceptance under DimOS collision-safety rules.
- Configure official RoboPlan Cartesian planning through a typed RoboPlan planner
  subconfiguration.

**Non-Goals:**

- A generic path-constraint model or arbitrary Cartesian constraints.
- Public `ManipulationModule`, RPC, CLI, skill, or MCP convenience APIs.
- Tool-local or arbitrary-frame target resolution.
- Free-space fallback, timeout/cancellation emulation, or time-optimal Cartesian mode.
- Custom OInK tasks, position barriers, or collision barriers.
- A separate trajectory-parametrization abstraction or acceleration field on
  `JointState`.

## DimOS Architecture

Add these internal planning models:

```python
@dataclass(frozen=True)
class CartesianDelta:
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frame_id: str = "world"

CartesianTarget: TypeAlias = PoseStamped | CartesianDelta
```

Extend the DimOS `PlannerSpec` Protocol with:

```python
def plan_linear_cartesian_path(
    self,
    world: WorldSpec,
    selection: PlanningGroupSelection,
    start: JointState,
    targets: Mapping[PlanningGroupID, CartesianTarget],
    *,
    auxiliary_groups: Sequence[PlanningGroupID] = (),
) -> PlanningResult:
    ...
```

`RoboPlanWorld` implements the operation against its native scene.
`RRTConnectPlanner` implements the same Protocol method and returns
`PlanningStatus.UNSUPPORTED`. No adapter Protocol, module reference, stream,
transport, blueprint, RPC, skill/MCP, or CLI surface is added. No generated blueprint
registry input changes.

Add planner configuration alongside the existing kinematics configuration pattern:

```python
class RRTConnectPlannerConfig(BaseConfig):
    backend: Literal["rrt_connect"] = "rrt_connect"

class RoboPlanLinearCartesianConfig(BaseConfig):
    dt: float = 0.01
    max_linear_speed: float = 0.1
    max_angular_speed: float = 0.5
    max_linear_acceleration: float = 0.5
    max_angular_acceleration: float = 2.5
    max_position_error: float = 0.005
    max_orientation_error: float = 0.01
    velocity_scale: float = 1.0
    acceleration_scale: float = 1.0
    limit_ratio_tolerance: float = 1.05
    position_limit_gain: float = 1.0
    max_attempts_per_step: int = 16

class RoboPlanPlannerConfig(BaseConfig):
    backend: Literal["roboplan"] = "roboplan"
    linear_cartesian: RoboPlanLinearCartesianConfig = Field(
        default_factory=RoboPlanLinearCartesianConfig
    )
```

`ManipulationPlannerConfig` is a discriminated union over the two backend configs.
Factories receive the typed config and pass the RoboPlan subconfiguration into the
scene-backed planner. `ManipulationModuleConfig.planner` defaults to
`RoboPlanPlannerConfig`; deprecated `planner_name`, when explicitly set, is converted
to a default typed configuration and takes precedence during the compatibility window,
matching the existing `kinematics_name` migration convention.

## Decisions

### One linear operation with a target union

Absolute and relative requests are the same planning problem after endpoint
resolution, so they share one operation. Separate public convenience methods may
eventually delegate to it, but the internal surface remains concise. A generic
`path_constraints` field was rejected because v1 supports one concrete behavior and
does not yet have a coherent generic constraint model.

### Target and auxiliary group invariants

The target keys and `auxiliary_groups` MUST be disjoint, and their union MUST equal
`selection.group_ids`. At least one target is required. Targeted groups require a TCP
tip; auxiliary groups participate in the combined configuration but do not receive an
end-effector track. Invalid coverage or target types return `INVALID_GOAL`.

The explicit `start` is normalized to the selection's globally named joint order and
must match the authoritative RoboPlan scene state, consistent with existing native
planning. A mismatch returns `INVALID_START`.

### World-frame target semantics

An absolute `PoseStamped` MUST use `frame_id="world"`. A `CartesianDelta` is resolved
from the TCP pose at the explicit start state:

- `p_target = p_start + translation`
- `R_target = R_delta @ R_start`

`rotation_rpy` uses XYZ roll, pitch, and yaw in radians. Zero delta rotation therefore
preserves the starting TCP orientation. Unsupported frames return `UNSUPPORTED` rather
than being silently reinterpreted. Target resolution happens independently per group,
so one request may mix absolute poses and relative deltas.

### Official RoboPlan bounded Cartesian planning

The adapter builds one official Cartesian path per targeted group. Position follows a
line segment and orientation follows spherical interpolation between the start and
target rotations. Matching start and target rotations expresses a fixed-orientation
path.

All tracks use RoboPlan's shared timeline. When track lengths differ, a shorter track
reaches its target and holds while the longer track completes. The adapter forces
`CartesianSpeedMode.Bounded`; `speed_mode` is not configurable in v1. This retains the
official planner's per-step Cartesian error checks and avoids the unvalidated final
blend used by its time-optimal mode.

The RoboPlan configuration maps directly to official planner options. No artificial
timeout parameter is exposed because the official API provides no timeout or
cancellation mechanism.

RoboPlan 0.5.1 requires `q_start` to contain the full model configuration even when
`options.group_name` selects a smaller planning group. The adapter therefore passes the
scene's full native joint-name order and full scratch-context positions, allowing
passive and unselected state to remain coherent. It projects the returned full-model
trajectory down to the selected global joints.

### Universal result conversion

The returned `PlanningResult` contains:

- combined `JointState` waypoints in the selection's global joint-name order;
- positions and velocities supplied by RoboPlan;
- RoboPlan waypoint times in `timestamps`;
- normal DimOS planning time and joint-space path length metadata.

RoboPlan accelerations are omitted because `JointState` has no acceleration field;
they MUST NOT be placed in `effort`.

### Atomic collision post-validation

Success is conditional on DimOS post-validation. For each combined waypoint, the
adapter partitions global joints by robot, installs all robot states into one scratch
world context, and checks the resulting scene. For every adjacent waypoint pair, it
also samples the combined joint-space edge at synchronized interpolation fractions and
checks each combined sample. This preserves inter-robot collision correctness rather
than validating each arm against stale positions of the others.

Any waypoint or edge collision rejects the entire result as `NO_SOLUTION`. DimOS does
not fall back to a free-space joint planner because that would violate the requested
linear TCP behavior.

## Safety / Simulation / Replay

Planning alone does not command hardware. Existing execution code remains responsible
for synchronized dispatch of the returned combined trajectory. The same result and
post-validation rules apply whether a later caller uses hardware or simulation.

Replay does not gain a planning entry point in this change. Manual QA should use a
RoboPlan-backed test scene or simulation to verify a fixed-orientation move, a mixed
absolute/relative multi-arm move, and rejection when a synchronized waypoint or edge
collides. Hardware QA is not required for the internal API addition.

## Risks / Trade-offs

- Official bounded Cartesian planning may be slower or fail where time-optimal blending
  would succeed. This is accepted to retain path-error guarantees.
- Collision checking after planning may reject an otherwise valid Cartesian result and
  does not guide RoboPlan around obstacles. Linear geometry leaves no free-space
  alternative; callers receive `NO_SOLUTION`.
- World-only relative deltas are less ergonomic than tool-local commands. Frame
  resolution belongs in the future public facade and can be added without changing the
  internal operation.
- Making the new Protocol method mandatory affects third-party planner implementers.
  A small `UNSUPPORTED` implementation is the intentional compatibility behavior.
- Upgrading RoboPlan may expose dependency or binding differences. Pin a verified
  release, refresh the lockfile, and exercise real-binding integration coverage.

## Migration / Rollout

1. Upgrade and lock a RoboPlan version with the official Cartesian API.
2. Add the models, typed planner configs, factory migration, and Protocol operation.
3. Implement RoboPlan planning/conversion/post-validation and RRT unsupported behavior.
4. Retain `planner_name` as deprecated input during rollout; do not remove it in this
   change.
5. Add focused unit tests with fake bindings plus a real-RoboPlan integration test.
6. Update manipulation configuration documentation and validate the OpenSpec change.

No blueprint name or module registry input changes, so
`dimos/robot/all_blueprints.py` regeneration is unnecessary. Rollback consists of
reverting the dependency/config/API addition; existing joint planning remains
unchanged.

## Open Questions

None. Public facade naming and frame ergonomics are intentionally deferred to a later
design.
