## Why

The planning-group refactor is currently bundled in one large branch. Reviewers need a small first PR that establishes the core planning-group abstraction and robot configuration plumbing without also reviewing planner algorithms, module APIs, visualization, or control integration.

This change extracts PR 1 from the reference branch/worktree `cc/spec/movegroup`. Treat the existing branch as a working reference implementation; do not redesign the feature unless required to keep this slice independently valid.

## What Changes

- Add planning-group identifiers, definitions, runtime models, registry helpers, and joint-name conversion utilities.
- Update planning spec/config models so robot-scoped `end_effector_link` is removed and pose target frames are represented by planning-group `tip_link` values.
- Add planning-group conversion/discovery support through manipulation planning config models.
- Update manipulator robot configs to declare or discover explicit planning groups.
- Add foundation-level unit tests and minimal concept docs.

## Capabilities

### New Capabilities
- `planning-group-foundation`: Core planning-group data model, discovery, robot config generation, and validation contracts.

### Modified Capabilities
- `manipulation-planning-config`: Robot model configuration uses planning groups as the source of chain and pose-target metadata.

## Impact

- Base branch: `main`.
- Reference implementation: branch/worktree `cc/spec/movegroup`; stabilize to a commit before assigning agents.
- Primary files: `dimos/manipulation/planning/groups/*`, `dimos/manipulation/planning/spec/*`, `dimos/robot/manipulators/*/config.py`, `dimos/manipulation/planning/groups/test_planning_groups.py`, foundation tests, and planning-group docs.
- Out of scope: world backends, IK/RRT algorithms, `ManipulationModule`, Viser UI, and control changes.
