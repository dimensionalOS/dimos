# Planner Eval

A small tool for scoring motion-planning behaviour on a fixed list of cases.
Runs the IK + RRT planning stack against an arm and reports, per case: did
it plan, how long it took, and how close the planner got. Pure planner — no
Module boot, no LCM, no hardware.

The point: today there is no way to know whether a change to the IK solver,
the RRT planner, or a Drake bump made things better or worse. This closes
that gap for the planner layer.

## What an eval for a robot-arm trajectory looks like

Software unit tests work because functions are deterministic — `f(x) → y,
assert y == expected`. Robot trajectories are not:

- The output is a **path through space and time**, not a single value.
- The planner uses **random sampling** (RRT-Connect), so the same input
  may produce a different path each run.
- "Success" is **multi-dimensional**: did IK find a config? did the
  planner find a collision-free path? is the path accurate enough? can
  the arm actually execute it within its velocity limits?
- "Failure" has **multiple flavours**: IK couldn't converge, planner
  timed out, goal config in collision, joint limit violated.

So an eval here is built around four ideas:

1. **A case is a task spec.** "Reach this pose, given these obstacles,
   starting here." Pure data. You can write 10 cases or 10,000 — they
   don't touch the framework.

2. **A run produces a structured score, not a boolean.** Pass/fail is
   one field; *why* is a human-readable string; *how well* is a
   `dict[str, float]` of named metrics. Adding a new metric is one
   dict key; nothing else changes.

3. **Failure is reported, not thrown.** A case that can't be planned
   returns `passed=False` with a reason like `"ik failed: NO_SOLUTION"`.
   The eval never raises — one bad case can't kill the suite.

4. **The eval is honest about its scope.** This scores the *planning
   stack* (IK + RRT). It tells you whether the planner did its job. It
   does **not** tell you whether the trajectory executes on hardware,
   whether perception saw the right object, or whether async timing in
   the Module stack is correct. Those are different evals at higher
   layers (see `What this does and doesn't catch` below).

If you've used MoveIt's `moveit_ros_benchmarks` or OMPL's benchmarking
suite, this follows the same pattern with the same metric vocabulary —
path length, clearance, smoothness — plus the TOPP-RA-style feasibility
pair (`max_joint_velocity_ratio`, `max_joint_acceleration_ratio`).

## How to use it, end to end

**1. Confirm the baseline works on your machine.**

```bash
.venv/bin/python -m dimos.manipulation.eval --arm xarm6
```

Should print a 7-row table ending in `7/7 passed`. If anything fails
before you've made changes, the planner stack itself is broken — fix
that first.

**2. Capture a baseline before making changes.**

```bash
.venv/bin/python -m dimos.manipulation.eval --arm xarm6 --json baseline.json
```

Now make whatever IK / planner / catalog / Drake change you're
investigating. Re-run against `your-branch.json`.

**3. Diff the two runs.**

```bash
diff <(jq '.[].metrics' baseline.json) <(jq '.[].metrics' your-branch.json)
```

Look for shifts in `planning_time_s`, `position_error_m`, `path_length_cartesian_m`,
or the velocity/acceleration ratios. A regression shows up as a numeric
shift; a hard break shows up as `passed: false` with a reason.

**4. When a metric surprises you, look at the arm.**

```bash
.venv/bin/python -m dimos.manipulation.eval --arm xarm6 --viz
```

Opens a Meshcat URL; watch the arm animate each case. You usually see
the issue in a few seconds — wrong elbow branch, path clipping an
obstacle, wrong EE orientation.

**5. Add cases for your own work.** A customer cell, a specific task
pose, an arm with a new fixture — write your own `reach_case(...)`
calls in your own script. You never edit the framework.

```python
from dimos.manipulation.eval import reach_case, evaluate, print_scores
from dimos.robot.catalog.ufactory import xarm6

cases = [reach_case("my_pose", 0.40, 0.10, 0.30)]
print_scores(evaluate(xarm6(adapter_type="mock"), cases))
```

**6. Wire it into your tests.** `pytest dimos` already discovers
`test_planner_eval.py`, which asserts the default suite passes on xArm6.
Nothing extra to configure for CI.

## Quick start

```bash
.venv/bin/python -m dimos.manipulation.eval --arm xarm6
.venv/bin/python -m dimos.manipulation.eval --arm xarm6 --json out.json
.venv/bin/python -m dimos.manipulation.eval --arm xarm6 --viz
```

The `--viz` flag opens a Meshcat viewer (URL printed at startup) and
animates each planned path through the case.

Sample output:

```
────────────────────────────────────────────────────────────
  Planner Eval · 7 cases
────────────────────────────────────────────────────────────
  ✓  reach_center              590ms  err=0.6mm
  ✓  reach_left                528ms  err=0.6mm
  ✓  reach_right               570ms  err=0.6mm
  ✓  reach_high                690ms  err=0.7mm
  ✓  reach_low                 377ms  err=0.6mm
  ✓  reach_extended            479ms  err=0.8mm
  ✓  reach_obstacle_box       4028ms  err=0.7mm
────────────────────────────────────────────────────────────
  7/7 passed
```

Exit code is `0` if all cases pass, `1` otherwise — drop into CI as-is.

CLI-exposed arms: `xarm6`, `xarm7`, `piper`. Each has a tuned case set.

### Per-arm status

| Arm | Result | Notes |
|---|---|---|
| `xarm6` | 7/7 pass | 6 reaches + 1 obstacle case. The canonical "default" suite. |
| `xarm7` | 6/6 pass | Same six reaches as xArm6, no obstacle case — the extra DOF puts the goal config in collision when the obstacle is present. |
| `piper` | 6/6 pass | Smaller reach envelope (~half of xArm), and a different gripper orientation (pitch=90° rather than roll=180°) because Piper's `gripper_base` link convention differs. |

Panda is not CLI-exposed because its URDF tarball isn't in default Git
LFS pulls. The framework supports it — construct the `RobotConfig` and
pass to `evaluate()` directly.

## How it's structured

Three pieces, each independently scalable:

| Piece | What it is | How it scales |
|-------|------------|---------------|
| `Case` | What the robot should do (target pose, optional obstacles, tolerances) | Write more cases |
| `evaluate()` | How to run a Case against an arm | Add a sibling function for a new runner (e.g. against a live `ManipulationModule`) |
| `Score` | What came back. Includes `metrics: dict[str, float]` | Add a key to the dict |

The `metrics` dict is the deliberate extensibility point. CI watches the
keys it cares about; new metrics never force a type change.

Standard metric keys today:

**Always present (IK + planning):**
- `planning_time_s` — wall-clock IK + RRT time
- `position_error_m` — IK Cartesian error at goal
- `orientation_error_rad` — IK orientation error at goal

**Present when a plan is found:**
- `path_length_rad` — joint-space path length
- `n_waypoints` — number of waypoints in the path
- `path_length_cartesian_m` — 3D EE travel distance along the path. Large
  values vs direct-reach baseline indicate inefficient routing (e.g. the
  obstacle case here travels ~5× the direct distance).
- `joint_limit_margin_rad` — minimum distance from any joint to its hard
  limit anywhere on the path. Lower → less slack for execution-time drift.
  Negative would mean the planner produced an out-of-limits state.
- `max_joint_velocity_ratio` — `max(|v_i| / v_limit_i)` after time
  parameterization via `JointTrajectoryGenerator`. ≤ 1 is feasible; > 1
  means the trajectory cannot execute within configured limits. Trapezoidal
  profiles typically saturate at 1.0 during cruise — that's correct.
- `max_joint_acceleration_ratio` — same for acceleration. Numerically
  differentiated from velocity, so sensitive to time-step granularity.

These follow MoveIt's benchmark suite (`moveit_ros_benchmarks`) and OMPL's
benchmarking conventions for path length, clearance, and smoothness, plus
the TOPP-RA-style feasibility pair (velocity/acceleration ratios).

## Programmatic use

```python
from dimos.manipulation.eval import evaluate, default_cases
from dimos.robot.catalog.ufactory import xarm6

scores = evaluate(xarm6(adapter_type="mock"), default_cases())
for s in scores:
    print(s.name, s.passed, s.reason, s.metrics)
```

## Writing your own case

For simple reaches, use the `reach_case` helper:

```python
from dimos.manipulation.eval import reach_case, evaluate
from dimos.robot.catalog.ufactory import xarm6

cases = [
    reach_case("above_fixture", 0.40, 0.10, 0.30),
    reach_case("tight_tol",      0.35, 0.00, 0.25, position_tolerance_m=0.005),
]
scores = evaluate(xarm6(adapter_type="mock"), cases)
```

For cases with obstacles, build the obstacle from dimOS types and pass it:

```python
from dimos.manipulation.eval import reach_case, evaluate
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.catalog.ufactory import xarm6

fixture = Obstacle(
    name="fixture",
    obstacle_type=ObstacleType.BOX,
    pose=PoseStamped(
        frame_id="world",
        position=Vector3(0.40, 0.10, 0.10),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    ),
    dimensions=(0.10, 0.10, 0.15),
)
case = reach_case("above_fixture", 0.40, 0.10, 0.30, obstacles=[fixture])
scores = evaluate(xarm6(adapter_type="mock"), [case])
```

Customers with a specific cell can ship a Python file like this in their
own repo and run it before pulling a dimOS update.

## Tests

Two test files, run by their usual `pytest dimos` discovery:

- **`dimos/manipulation/test_eval_unit.py`** — 27 unit tests for the data
  types, `reach_case`, case registry, and reporting. No Drake, runs in
  under a second. Use for fast feedback on framework changes.
- **`dimos/manipulation/test_planner_eval.py`** — one integration test
  that runs the default suite on xArm6 against a real Drake world.
  Skips automatically when Drake or the catalog entry is unavailable.

```bash
.venv/bin/python -m pytest dimos/manipulation/test_eval_unit.py -v
.venv/bin/python -m pytest dimos/manipulation/test_planner_eval.py -v
```

The repo's root `conftest.py` calls LCM `autoconf`, which on macOS may
prompt for sudo to add a multicast route. If that blocks pytest collection,
run `sudo -v` first or pre-add the route manually:

```bash
sudo route add -net 224.0.0.0/4 -interface lo0
```

Alternatively, pass `--confcutdir=dimos/manipulation` to bypass the root
conftest entirely for the unit tests (they don't need it):

```bash
.venv/bin/python -m pytest dimos/manipulation/test_eval_unit.py \
    --confcutdir=dimos/manipulation -v
```

## Default cases

Today the default suite has seven cases tuned for tabletop arms with the
gripper pointing down:

| Name | Target (x, y, z) | Notes |
|------|------------------|-------|
| `reach_center` | 0.35, 0.0, 0.25 | Dead-centre workspace |
| `reach_left` | 0.30, 0.25, 0.25 | Left of centre |
| `reach_right` | 0.30, -0.25, 0.25 | Right of centre |
| `reach_high` | 0.25, 0.0, 0.42 | High overhead reach |
| `reach_low` | 0.35, 0.0, 0.13 | Low near the table surface |
| `reach_extended` | 0.48, 0.0, 0.20 | Near max reach |
| `reach_obstacle_box` | 0.42, 0.0, 0.28 | Box overhead, planner routes under |

The obstacle case is the canary — it consistently runs ~7× slower than the
free-space cases, so any planner regression that hurts obstacle handling
shows up immediately as a TIMEOUT failure or a large `planning_time_s`
shift.

## What this does not do (yet)

Each of these is a deliberate cut, not an oversight. They get added when
they earn their keep.

- **No `ManipulationModule` boot.** Pure planner only. Won't catch
  regressions that show up only when the full Module + Coordinator stack
  is wired together.
- **No hardware.** Won't catch tracking error, actuator delay, or noise.
- **No closed-loop execution metrics.** Today's metrics describe the
  planned trajectory, not the executed one.
- **No multi-arm coordination.** One arm at a time.
- **No constraint enforcement.** `ManipulationTask` constraints are not
  yet checked against the planned path.

## Adding a new arm

The framework is in `eval.py`; the per-arm case data lives separately in
`eval_cases.py`. **You add a new arm by editing data, not framework.**

```python
# in dimos/manipulation/eval_cases.py

def _my_robot_cases() -> list[Case]:
    q = (0.0, 0.7071, 0.0, 0.7071)   # whatever orientation suits the EE link
    return [
        _case("reach_a", 0.20, 0.0, 0.15, orientation=q),
        _case("reach_b", 0.25, 0.10, 0.20, orientation=q),
        # ...
    ]

CASES_BY_ARM = {
    # ...existing entries...
    "my_robot": _my_robot_cases,
}
```

Then add the catalog factory in `_ARM_FACTORIES` in `eval.py` so the CLI
recognises `--arm my_robot`. That's the only thing to change in `eval.py`.

For workflows where the cases are not generic (e.g. a customer's specific
cell), skip the registry and pass cases directly:

```python
from dimos.manipulation.eval import Case, evaluate
my_cases = [Case(name=..., target=...), ...]
evaluate(my_arm, my_cases)
```

## File layout

```
dimos/manipulation/eval.py                 # framework — Case, Score, evaluate, CLI
dimos/manipulation/eval_cases.py           # data — per-arm case lists, gripper conventions
dimos/manipulation/test_eval_unit.py       # 27 unit tests (no Drake)
dimos/manipulation/test_planner_eval.py    # integration test (requires Drake)
docs/capabilities/manipulation/eval.md     # this file
```

The split is deliberate: framework and data are edited by different
people for different reasons. Adding a case or tuning an existing one
should never require touching `eval.py`.
