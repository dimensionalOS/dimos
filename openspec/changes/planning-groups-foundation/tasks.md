## 1. Extract foundation files

- [x] 1.1 Start from `main` and use `cc/spec/movegroup` as the reference implementation.
- [x] 1.2 Bring over `dimos/manipulation/planning/groups/*`.
- [x] 1.3 Bring over required planning spec/config changes only.
- [x] 1.4 Bring over `dimos/robot/config.py` and manipulator config group declarations.

## 2. Tests and docs

- [x] 2.1 Bring over `dimos/manipulation/planning/test_planning_groups.py` and `test_planning_group_utils.py`.
- [x] 2.2 Bring over `dimos/robot/test_config.py`.
- [x] 2.3 Bring over only foundation docs needed to explain planning groups and custom-arm config.

## 3. Validation

- [x] 3.1 Run `uv run pytest dimos/manipulation/planning/test_planning_groups.py dimos/manipulation/planning/test_planning_group_utils.py dimos/robot/test_config.py -q`.
- [x] 3.2 Run targeted mypy on changed production files.
- [x] 3.3 Confirm no files from world/IK/RRT/module/Viser/control were pulled in accidentally.
