## 1. Extract algorithm changes

- [ ] 1.1 Start from PR 2 and use `cc/spec/movegroup` as reference.
- [ ] 1.2 Extract PinkIK group target-frame and base-link validation changes.
- [ ] 1.3 Extract Jacobian IK group selection changes.
- [ ] 1.4 Extract Drake optimization IK group target-frame changes.
- [ ] 1.5 Extract RRT planner group-local target and projection changes.

## 2. Tests

- [ ] 2.1 Bring over PinkIK tests relevant to group target frames and base-link validation.
- [ ] 2.2 Bring over Jacobian IK selection tests.
- [ ] 2.3 Bring over RRT planner selection/group tests.

## 3. Validation

- [ ] 3.1 Run `uv run pytest dimos/manipulation/planning/kinematics/test_pink_ik.py dimos/manipulation/planning/kinematics/test_jacobian_ik_selection.py dimos/manipulation/planning/planners/test_rrt_planner_selection.py -q`.
- [ ] 3.2 Run targeted mypy on changed algorithm files.
- [ ] 3.3 Optional manual smoke: solve IK and plan a small reachable group pose; print status and path length.
