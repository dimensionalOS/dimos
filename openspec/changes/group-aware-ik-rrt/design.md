## Context

IK and RRT previously operated mostly through robot-scoped assumptions. With groups, each target must map to an explicit group, local joint order, and target frame.

## Goals / Non-Goals

**Goals:**
- Make IK target frames come from the requested planning group.
- Make RRT accept and return group-local joint targets and paths.
- Keep collision checks and global robot-state projection correct.
- Fail clearly when a requested group lacks required pose target metadata.

**Non-Goals:**
- Do not change `ManipulationModule` public APIs in this PR.
- Do not include visualization changes.
- Do not change control tasks.

## Decisions

- Prefer explicit group IDs for all new solver/planner paths.
- Robot-scoped compatibility may resolve through a unique pose-targetable group, but ambiguous robots must fail clearly.
- PinkIK and Viser-style base-pose transforms must validate that robot-scoped `base_link` is compatible with model-root assumptions.

## Risks / Trade-offs

- Algorithm tests can become broad quickly. Keep this PR focused on solver/planner contracts and leave module-level behavior to PR 4.
- Some optional dependencies may be unavailable locally; fake dependency tests should stay hermetic.
