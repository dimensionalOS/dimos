## 1. Extract backend group support

- [ ] 1.1 Start from PR 1 and use `cc/spec/movegroup` as reference.
- [ ] 1.2 Extract Drake world group FK/Jacobian and base-link handling changes.
- [ ] 1.3 Extract RoboPlan world group FK/Jacobian, joint-name normalization, and URDF strip handling changes.
- [ ] 1.4 Extract world monitor and robot state monitor group-aware query changes.

## 2. Tests

- [ ] 2.1 Bring over Drake group world tests.
- [ ] 2.2 Bring over RoboPlan world tests, including planning split file if needed.
- [ ] 2.3 Bring over WorldMonitor tests for group state and ambiguity behavior.

## 3. Validation

- [ ] 3.1 Run `uv run pytest dimos/manipulation/planning/world/test_drake_world_planning_groups.py dimos/manipulation/test_roboplan_world.py dimos/manipulation/test_roboplan_world_planning.py dimos/manipulation/planning/monitor/test_world_monitor.py -q`.
- [ ] 3.2 Run targeted mypy on changed backend/monitor files.
- [ ] 3.3 Optional manual smoke: load a robot config, print group list, FK pose, Jacobian shape, and collision status.
