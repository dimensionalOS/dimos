## Why

The Viser changes are large and have different review criteria from planning algorithms. They should be reviewed as a dedicated UI/backend PR after the public manipulation group APIs exist.

## What Changes

- Make the Viser panel group-aware.
- Split panel/backend behavior out of the old adapter shape.
- Update scene previews, target ghosts, group selection, feasibility state, and safe execution checks.
- Update Viser tests and manual review checklist.

## Capabilities

### New Capabilities
- `viser-group-planning-ui`: Viser visualization supports group-aware planning, preview, target evaluation, and execution state.

### Modified Capabilities
- `manipulation-visualization`: Visualization targets explicit planning groups rather than a robot-scoped end-effector field.

## Impact

- Base branch: PR 4 `manipulation-module-group-api`.
- Reference implementation: `cc/spec/movegroup`.
- Primary files: `dimos/manipulation/visualization/viser/*`, `dimos/manipulation/visualization/types.py`, `dimos/manipulation/visualization/test_factory.py`.
- Out of scope: planning model/backend/algorithm changes except as already supplied by earlier PRs.
