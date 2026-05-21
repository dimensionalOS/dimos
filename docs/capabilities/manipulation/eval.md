# Planner Eval

A small tool for checking that the motion planner still works after you
change something. Give it an arm, it runs a list of motion-planning
problems, and tells you which ones pass.

## Run it

```bash
dimos eval --arm xarm6                          # run the suite
dimos eval --arm xarm6 --viz                    # watch in Meshcat
dimos eval --arm xarm6 --json out.json          # save results
```

Exit code is 0 if everything passes, 1 if anything fails.

Supported arms: `xarm6`, `xarm7`, `piper`.

## What the output looks like

```
  ✓  reach_center              553ms  err=0.6mm  v=100%
  ✓  reach_obstacle_box       4868ms  err=0.7mm  v=100%
  ✓  pp_left_to_right_pick    9334ms  err=0.8mm  v=100%
  ✗  some_failing_case         150ms             ik failed: NO_SOLUTION
  15/16 passed
```

Each row is one motion-planning problem:

- `ms` — how long planning took
- `err` — how close the end-effector got to the target
- `v%` — how close to the joint velocity limit the trajectory runs (≤ 100% means feasible)
- Failure reason if it didn't pass (`ik failed`, `plan failed: TIMEOUT`, `COLLISION_AT_GOAL`, etc.)

## Write your own test scenario

```python
from dimos.manipulation.eval import reach_scenario, evaluate, print_scores
from dimos.robot.catalog.ufactory import xarm6

scenarios = [reach_scenario("my_pose", 0.40, 0.10, 0.30)]
print_scores(evaluate(xarm6(adapter_type="mock"), scenarios))
```

For a pick-and-place sequence:

```python
from dimos.manipulation.eval_cases import pick_place_scenarios

scenarios = pick_place_scenarios(
    "my_pick_place",
    pick=(0.30, 0.20, 0.10),
    place=(0.30, -0.20, 0.10),
)
```

For an obstacle, build it with their existing `Obstacle` type and pass
it via the `obstacles=[...]` argument on `reach_scenario`.

## Add a new arm

1. Open `dimos/manipulation/eval_cases.py`.
2. Write a function that returns a list of `Scenario` objects with poses
   inside that arm's workspace.
3. Add an entry to `SCENARIOS_BY_ARM`.
4. Add the catalog factory path to `_ARM_FACTORIES` in `eval.py`.

That's it. The CLI, the JSON output, and the metrics all work
automatically. You never edit `eval.py` to add a scenario or an arm.

## Tests

```bash
# unit tests, no Drake, ~0.2s
pytest dimos/manipulation/test_eval_unit.py --confcutdir=dimos/manipulation

# integration test, needs Drake
pytest dimos/manipulation/test_planner_eval.py
```

## What it tests

For each scenario, IK + RRT, then:

- did IK find a joint config that reaches the target
- did the planner find a collision-free path to that config
- is the end-effector position close enough to the target
- does the path stay within joint limits and within velocity / acceleration limits

## What it doesn't test

- Real hardware. This is simulation only.
- Whether the executed trajectory tracks the planned one. Planning only.
- Perception — obstacles are given as data, we don't check if perception sees them right.
- Multi-arm coordination. One arm at a time.

## Per-arm status

| Arm | Scenarios | Notes |
|---|---|---|
| xArm6 | 16 | full suite: basic reaches, obstacles, tight clearance, pick-and-place over a work surface |
| xArm7 | 14 | xArm6 minus two scenarios that COLLISION_AT_GOAL on 7-DOF |
| Piper | 6 | smaller workspace, different gripper orientation |

## Files

```
dimos/manipulation/eval.py              framework: Scenario, Score, evaluate, CLI
dimos/manipulation/eval_cases.py        data: per-arm scenarios, gripper orientations
dimos/manipulation/test_eval_unit.py    unit tests
dimos/manipulation/test_planner_eval.py integration test
```
