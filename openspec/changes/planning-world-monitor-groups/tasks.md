## 1. Extract backend group support

- [x] 1.1 Start from PR 1 and use `cc/spec/movegroup` as reference.
- [x] 1.2 Extract Drake world group FK/Jacobian and base-link handling changes.
- [x] 1.3 Extract RoboPlan world group FK/Jacobian, joint-name normalization, and URDF strip handling changes.
- [x] 1.4 Extract world monitor and robot state monitor group-aware query changes.

## 2. Tests

- [x] 2.1 Bring over Drake group world tests.
- [x] 2.2 Bring over RoboPlan world tests, including planning split file if needed.
- [x] 2.3 Bring over WorldMonitor tests for group state and ambiguity behavior.

## 3. Validation

- [x] 3.1 Run `uv run pytest dimos/manipulation/planning/world/test_drake_world_planning_groups.py dimos/manipulation/test_roboplan_world.py dimos/manipulation/test_roboplan_world_planning.py dimos/manipulation/planning/monitor/test_world_monitor.py -q`.
  - Note: `dimos/manipulation/test_roboplan_world_planning.py` does not exist in this branch; ran the same targeted command without that optional split file.
- [x] 3.2 Run targeted mypy on changed backend/monitor files.
- [x] 3.3 Optional manual smoke: load a robot config, print group list, FK pose, Jacobian shape, and collision status.
