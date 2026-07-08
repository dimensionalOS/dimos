## 1. Extract module API changes

- [x] 1.1 Start from PR 3 and use `cc/spec/movegroup` as reference.
- [x] 1.2 Extract group-aware target, IK, preview, robot-info, and execution behavior from `ManipulationModule`.
- [x] 1.3 Extract coordinator client and example client updates only as needed for the public API.
- [x] 1.4 Include Greptile follow-up behavior for `get_ee_pose` and `plan_to_pose`.

## 2. Tests

- [x] 2.1 Bring over focused `test_manipulation_unit.py` coverage for group APIs and compatibility wrappers.
- [x] 2.2 Bring over relevant `test_manipulation_module.py` integration coverage.
- [x] 2.3 Bring over `dimos/e2e_tests/test_manipulation_planning_groups.py` if it can run against this stacked base.

## 3. Validation

- [x] 3.1 Run `uv run pytest dimos/manipulation/test_manipulation_unit.py dimos/manipulation/test_manipulation_module.py dimos/e2e_tests/test_manipulation_planning_groups.py -q`.
- [x] 3.2 Run `uv run mypy dimos/manipulation/manipulation_module.py`.
- [x] 3.3 Verify no Viser implementation files are included.
