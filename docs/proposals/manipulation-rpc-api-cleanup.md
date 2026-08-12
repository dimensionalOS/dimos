# Simplify the manipulation RPC API

`ManipulationModule` exposes about 30 manipulation RPCs across planning, execution,
visualization, robot discovery, gripper control, and skills. Replace them with 10
group-based primitives.

This is a breaking cleanup. Remove old RPCs and update callers. Do not add aliases.

Constraints:

- RPC first. Agents will call these methods from Python in another module.
- Use opaque planning-group IDs. Do not expose robot names.
- One pending plan and one active execution at a time.
- Keep obstacle RPCs unchanged, but outside `ManipulationSpec`.
- Move current skills to a separate deprecated `ManipulationSkills` adapter. Do not
  polish them.

## 1. Current RPCs

`@skill` implies `@rpc`, so the current skill methods are also remote calls.

```text
State/config
  get_state                    get_error
  get_current_joints           get_ee_pose
  list_robots                  list_planning_groups
  get_robot_info               get/set_init_joints
  set_init_joints_to_current   get/set_motion_speed

Planning
  inverse_kinematics           inverse_kinematics_single
  solve_ik                     is_collision_free
  plan_to_pose                 plan_to_pose_targets
  plan_to_joints               plan_to_joint_targets
  plan_cartesian_targets

Plan/visualization state
  preview_path                 preview_plan
  has_planned_path             clear_planned_path
  get_visualization_url

Execution
  execute                      execute_plan(plan)
  cancel                       reset

Gripper/skills
  get_gripper                  set_gripper
  open_gripper                 close_gripper
  get_robot_state              move_to_pose
  move_to_joints               go_home / go_init
```

| Current surface | Replacement |
|---|---|
| Robot discovery and per-robot state | `list_planning_groups()`, `get_state()` |
| Three IK calls | Internal planner API only |
| Pose planner variants | `plan_to_poses()` |
| Joint planner variants | `plan_to_joints()` |
| Cartesian planner RPC | `move_linear()` |
| Preview and visualization RPCs | Private in-process visualization API |
| Plan query/clear RPCs | Fixed pending-plan lifecycle |
| Persistent speed RPCs | Per-call `speed_scale` |
| `execute()` and `execute_plan(plan)` | `execute()` with no plan argument |
| Gripper getter | Included in `get_state()` |
| Gripper skill variants | Deprecated `ManipulationSkills` adapter |

Leave these obstacle RPCs as-is:

```python
add_obstacle(...)
update_obstacle(...)
update_obstacle_pose(...)
remove_obstacle(...)
```

They do not belong to the new `ManipulationSpec`.

Move the retained skills out of `ManipulationModule`:

```text
ManipulationSkills                 ManipulationModule
  deprecated @skill methods  ────>  ManipulationSpec @rpc methods
                                  planning and execution state
```

```python
class ManipulationSkills(Module):
    manipulation: ManipulationSpec

    @skill
    def move_to_pose(...) -> SkillResult:
        plan = self.manipulation.plan_to_poses(...)
        if not plan.succeeded:
            return plan_to_skill_result(plan)
        return execution_to_skill_result(
            self.manipulation.execute(blocking=True)
        )
```

Do not subclass `ManipulationModule`. The adapter must use only `ManipulationSpec`,
so skills cannot depend on private planner state. Agentic blueprints include both
modules; RPC-only blueprints omit `ManipulationSkills`.

## 2. Refactored API

```python
class ManipulationSpec(Spec, Protocol):
    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]: ...

    def get_state(self) -> ManipulationSnapshot: ...

    def plan_to_joints(
        self,
        targets: Mapping[PlanningGroupID, JointState],
        speed_scale: float | None = None,
    ) -> PlanResult: ...

    def plan_to_poses(
        self,
        targets: Mapping[PlanningGroupID, PoseStamped],
        speed_scale: float | None = None,
    ) -> PlanResult: ...

    def execute(
        self,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> ExecutionResult: ...

    def wait_for_execution(
        self,
        timeout: float | None = None,
    ) -> ExecutionResult: ...

    def move_linear(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        planning_group: PlanningGroupID | None = None,
        check_collision: bool = False,
        speed_scale: float | None = None,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> MoveResult: ...

    def set_gripper_position(
        self,
        position: float,
        planning_group: PlanningGroupID | None = None,
    ) -> CommandResult: ...

    def cancel(self) -> ExecutionResult: ...

    def reset(self) -> CommandResult: ...
```

Public group and state types:

```python
@dataclass(frozen=True)
class PlanningGroupInfo:
    id: PlanningGroupID               # opaque
    joint_names: tuple[str, ...]
    base_frame: str
    tip_frame: str | None
    has_gripper: bool


@dataclass(frozen=True)
class PlanningGroupState:
    joints: JointState
    end_effector_pose: PoseStamped | None
    gripper_position: float | None


@dataclass(frozen=True)
class ManipulationSnapshot:
    timestamp: float
    operation_status: OperationStatus
    error: str | None
    has_pending_plan: bool
    execution_status: ExecutionStatus
    groups: Mapping[PlanningGroupID, PlanningGroupState]
```

`get_state()` always returns all groups. High-rate users should read the Memory2 state
stream instead.

### Planning and execution

```text
plan_* success ──> PENDING
plan_* failure ──> EMPTY             # stale plan is cleared

PENDING ── execute() ──> DISPATCHING # plan is consumed here
                         ├─ rejected ──> REJECTED
                         └─ accepted ──> EXECUTING
                                         ├─ COMPLETED
                                         ├─ ABORTED
                                         └─ FAULT
```

Rules:

- `plan_*` accepts one or more group targets and never moves hardware.
- A returned `GeneratedPlan` is an inspectable snapshot, not an execution token.
- `execute()` consumes the pending plan on every dispatch attempt, including rejection.
- `blocking=False` waits for coordinator acceptance or rejection, then returns.
- `blocking=True` also waits for terminal JTT status.
- `wait_for_execution()` needs no execution ID because only one execution can run.
- A timeout returns `TIMED_OUT` without cancelling motion.
- `speed_scale` is per call. `None` uses module configuration.

### `move_linear()`

```text
current end-effector pose
          + world-frame (dx, dy, dz)
          + unchanged orientation
                     |
                     v
      RoboPlan Cartesian plan
      collision check: off by default
                     |
                     v
        dispatch + optional wait
```

- This is an atomic plan-and-move call. It leaves no pending plan.
- It clears any older pending plan before planning, even if planning fails.
- `planning_group=None` works only when one pose-capable group exists.
- Zero displacement returns `NO_MOTION` without dispatch.
- `check_collision=False` skips RoboPlan collision checking and DimOS post-validation.
  IK, reachability, joint limits, timing, arbitration, and execution checks still apply.
- `speed_scale` scales planning, not physical metres per second. Use `0.5` as the
  configured default for this method.
- `timeout` covers the execution wait, not planning.

## 3. Other contracts

### Result types and `repr`

```python
class PlanStatus(Enum):
    SUCCEEDED = auto()
    FAILED = auto()
    INVALID_TARGET = auto()
    AMBIGUOUS_GROUP = auto()
    NO_MOTION = auto()


class ExecutionStatus(Enum):
    ACCEPTED = auto()
    REJECTED = auto()
    COMPLETED = auto()
    ABORTED = auto()
    FAULT = auto()
    TIMED_OUT = auto()
    NO_PLAN = auto()
    NO_EXECUTION = auto()


@dataclass(frozen=True)
class MoveResult:
    plan: PlanResult
    execution: ExecutionResult | None
```

All results have typed status and message fields. `PlanResult` also exposes the full
plan and statistics. Its `repr` stays short:

```text
PlanResult(SUCCEEDED, groups=('left_arm',), waypoints=18, duration=2.4s)
ExecutionResult(REJECTED, message='another trajectory is active')
MoveResult(plan=SUCCEEDED, execution=ACCEPTED, delta=(0.02, 0, -0.01), collision_check=False)
```

Add a manipulation result contract test:

```python
@pytest.mark.parametrize("result", public_result_examples())
def test_result_is_agent_readable(result: Any) -> None:
    assert has_deliberate_repr(type(result))  # reject object/default dataclass repr
    text = repr(result)
    assert type(result).__name__ in text
    assert len(text) <= MAX_AGENT_REPR_LENGTH
    if status := getattr(result, "status", None):
        assert status.name in text
    if message := getattr(result, "message", None):
        assert message in text
    assert repr(rpc_round_trip(result)) == text
```

The guard applies to manipulation results only. It must not hide typed fields or full
trajectory data.

### Execution status source

```text
JointTrajectoryTask              ManipulationModule
  TrajectoryStatus stream  ────> cache latest status
  EXECUTING every tick            execute(blocking=True)
  terminal status once            wait_for_execution()

ControlCoordinator
  accepts/rejects dispatch only
  no execution-status RPC
```

JTT owns `COMPLETED`, `ABORTED`, and `FAULT`. Preserve its terminal value. Also check
the RPC timeout so a valid blocking call is not cut off by the transport default.

### Implementation order

| Layer | End-to-end result |
|---|---|
| 1 | Spec, group discovery, state snapshot, result types, `repr` guard |
| 2 | Joint/pose planning and one pending plan |
| 3 | JTT status stream, execute, wait, cancel, timeout |
| 4 | Linear move and gripper position |
| 5 | Move deprecated skills to their adapter; remove old RPCs and update callers |

Done when the protocol above is the whole `ManipulationModule` RPC surface, apart
from module lifecycle and obstacle RPCs. Cover plan ownership, dispatch outcomes,
blocking modes, timeouts, group ambiguity, collision modes, JTT terminal states, RPC
round trips, and bounded `repr` output in tests.

Out of scope: obstacle cleanup, skill behavior changes, concurrent execution,
execution IDs, pick/place behavior, Memory2 changes, and visualization UI changes.
