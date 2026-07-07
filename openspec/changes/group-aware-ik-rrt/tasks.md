## 1. Extract algorithm changes

- [x] 1.1 Start from PR 2 and use `cc/spec/movegroup` as reference.
- [x] 1.2 Extract PinkIK group target-frame and base-link validation changes.
- [x] 1.3 Extract Jacobian IK group selection changes.
- [x] 1.4 Extract Drake optimization IK group target-frame changes.
- [x] 1.5 Extract RRT planner group-local target and projection changes.
- [x] 1.6 Address review feedback: Pink multi-target pose-target sets, shared IK helpers, and explicit unsupported statuses/protocols.

## 2. Tests

- [x] 2.1 Bring over PinkIK tests relevant to group target frames and base-link validation.
- [x] 2.2 Bring over Jacobian IK selection tests.
- [x] 2.3 Bring over RRT planner selection/group tests.
- [x] 2.4 Add focused tests for Pink multi-target/auxiliary IK, unsupported IK/planner capabilities, and shared helper behavior.

## 3. Validation

- [x] 3.1 Run `uv run pytest dimos/manipulation/planning/kinematics/test_pink_ik.py dimos/manipulation/planning/kinematics/test_jacobian_ik_selection.py dimos/manipulation/planning/planners/test_rrt_planner_selection.py -q`.
- [x] 3.2 Run targeted mypy on changed algorithm files.
- [x] 3.3 Optional manual smoke: solve IK and plan a small reachable group pose; print status and path length.
- [x] 3.4 Re-run focused pytest, targeted mypy, ruff, and diff checks after review-response changes.
