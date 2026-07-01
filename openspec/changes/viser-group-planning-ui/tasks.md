## 1. Extract Viser UI changes

- [ ] 1.1 Start from PR 4 and use `cc/spec/movegroup` as reference.
- [ ] 1.2 Extract Viser panel/backend/scene/state/visualizer group-aware changes.
- [ ] 1.3 Remove or replace the old adapter code only as required by this UI slice.
- [ ] 1.4 Extract visualization type/factory updates needed by Viser.

## 2. Tests and manual check

- [ ] 2.1 Bring over Viser unit tests and lifecycle tests.
- [ ] 2.2 Run the manual checklist: group selector, joint sliders, pose gizmo, target ghost, infeasible target color, path preview, execute gate, clear path.

## 3. Validation

- [ ] 3.1 Run `uv run pytest dimos/manipulation/visualization/test_factory.py dimos/manipulation/visualization/viser/test_*.py -q`.
- [ ] 3.2 Run targeted mypy on changed Viser production files if practical.
- [ ] 3.3 Verify no planner/backend implementation changes are included beyond compile fixes against PR 4.
