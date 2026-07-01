## 1. Extract foundation files

- [x] 1.1 Start from `main` and use `cc/spec/movegroup` as the reference implementation.
- [x] 1.2 Bring over `dimos/manipulation/planning/groups/*`.
- [x] 1.3 Bring over required planning spec/config changes only.
- [x] 1.4 Bring over manipulator config group declarations without `dimos/robot/config.py`.

## 2. Tests and docs

- [x] 2.1 Bring over `dimos/manipulation/planning/groups/test_planning_groups.py`.
- [x] 2.2 Remove stale `dimos/robot/test_config.py` scope from this PR.
- [x] 2.3 Bring over only foundation docs needed to explain planning groups and custom-arm config.

## 3. Validation

- [x] 3.1 Run targeted planning-group tests under `dimos/manipulation/planning/groups/test_planning_groups.py`.
- [x] 3.2 Run targeted mypy on changed production files.
- [x] 3.3 Confirm no files from world/IK/RRT/module/Viser/control were pulled in accidentally.
