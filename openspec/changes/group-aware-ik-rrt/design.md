## Context

IK and RRT previously operated mostly through robot-scoped assumptions. With groups, each target must map to an explicit group, local joint order, and target frame.

## Goals / Non-Goals

**Goals:**
- Make IK target frames come from the requested planning group.
- Treat `solve_pose_targets` as a pose-target set IK API while allowing backend capability differences.
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
- PinkIK supports multi-target pose-target set IK and auxiliary planning groups following the reference branch behavior: targets are grouped per robot, each robot is solved independently, auxiliary-only robots retain seed/current state, and selected joints are returned in `PlanningGroupSelection` order.
- Drake optimization IK and Jacobian IK currently support exactly one pose target and no auxiliary groups. They return `IKStatus.UNSUPPORTED` for broader target-set shapes instead of reporting a search failure.
- Selected joint planning is part of `PlannerSpec`. `RRTConnectPlanner` implements it; planner backends without selected/group planning support return `PlanningStatus.UNSUPPORTED`.
- Shared IK request helpers live near the IK backends in `dimos/manipulation/planning/kinematics/utils.py`; planner code must not import from `kinematics`.

## Risks / Trade-offs

- Algorithm tests can become broad quickly. Keep this PR focused on solver/planner contracts and leave module-level behavior to PR 4.
- Some optional dependencies may be unavailable locally; fake dependency tests should stay hermetic.
- Backend capability differences are explicit: unsupported target-set shapes are caller-visible `UNSUPPORTED` results, not hidden best-effort sequential solves.
