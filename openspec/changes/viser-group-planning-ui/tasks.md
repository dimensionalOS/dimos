## 1. Extract Viser UI changes

- [x] 1.1 Start from PR 4 and use only `cc/spec/movegroup@0edb8d3dd` as the normative UI reference; do not rely on the upstream extraction commit.
- [x] 1.2 Extract Viser panel/backend/scene/state/visualizer group-aware changes and the minimal group-native shared-clock preview seam.
- [x] 1.3 Remove or replace the old adapter code only as required by this UI slice.
- [x] 1.4 Extract visualization type/factory updates needed by Viser.

## 2. Tests and manual check

- [x] 2.1 Bring over Viser unit/lifecycle tests and add protocol, monitor forwarding, module routing, Meshcat projection, shared-tick, all-robot freshness, and preview cancellation race coverage.
- [x] 2.2 Run the manual checklist: group selector, joint sliders, pose gizmo, target ghost, infeasible target color, path preview, execute gate, clear path.

## 3. Validation

- [x] 3.1 Run `uv run pytest dimos/manipulation/test_manipulation_unit.py dimos/manipulation/planning/monitor/test_world_monitor.py dimos/manipulation/planning/world/test_drake_world_planning_groups.py dimos/manipulation/visualization/test_factory.py dimos/manipulation/visualization/viser/test_*.py -q` and `openspec validate viser-group-planning-ui --strict`.
- [x] 3.2 Run mandatory targeted mypy on changed Viser production files plus `manipulation_module.py`, `protocols.py`, `world_monitor.py`, and `drake_world.py`.
- [x] 3.3 Verify changes outside visualization are limited to the approved group-native preview protocol/forwarding/implementations, with no planner/backend algorithm or control changes.
