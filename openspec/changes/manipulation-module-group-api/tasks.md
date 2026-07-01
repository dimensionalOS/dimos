## 1. Extract module API changes

- [ ] 1.1 Start from PR 3 and use `cc/spec/movegroup` as reference.
- [ ] 1.2 Extract group-aware target, IK, preview, robot-info, and execution behavior from `ManipulationModule`.
- [ ] 1.3 Extract coordinator client and example client updates only as needed for the public API.
- [ ] 1.4 Include Greptile follow-up behavior for `get_ee_pose` and `plan_to_pose`.

## 2. Tests

- [ ] 2.1 Bring over focused `test_manipulation_unit.py` coverage for group APIs and compatibility wrappers.
- [ ] 2.2 Bring over relevant `test_manipulation_module.py` integration coverage.
- [ ] 2.3 Bring over `dimos/e2e_tests/test_manipulation_planning_groups.py` if it can run against this stacked base.

## 3. Validation

- [ ] 3.1 Run `uv run pytest dimos/manipulation/test_manipulation_unit.py dimos/manipulation/test_manipulation_module.py dimos/e2e_tests/test_manipulation_planning_groups.py -q`.
- [ ] 3.2 Run `uv run mypy dimos/manipulation/manipulation_module.py`.
- [ ] 3.3 Verify no Viser implementation files are included.
