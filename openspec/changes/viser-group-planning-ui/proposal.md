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
- Normative UI reference: `cc/spec/movegroup@0edb8d3dd`. User-visible deviations require explicit approval.
- API base: PR 4 at `cc/planning_group/main@05c25787a`; its explicit planning-group API scope remains unchanged.
- The upstream extraction commit `origin/cc/planning_group/viser@737fb3381` is not an implementation source.
- Primary files: `dimos/manipulation/visualization/viser/*`, `dimos/manipulation/visualization/types.py`, `dimos/manipulation/visualization/test_factory.py`.
- Supporting preview seam: group-native visualization protocol, world-monitor forwarding, manipulation-module preview routing, and the existing Meshcat implementation.
- Out of scope: all other manipulation-module, planning model/backend/algorithm, and control changes except as already supplied by earlier PRs.
