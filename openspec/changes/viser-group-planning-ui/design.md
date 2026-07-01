## Context

Viser must let users select planning groups, move joint and pose targets, see feasibility feedback, preview paths, and execute only when the preview still matches the current robot state.

## Goals / Non-Goals

**Goals:**
- Expose group selection and group-aware robot controls.
- Keep target ghost and preview animation tied to selected groups.
- Preserve safe execution behavior and clear recoverable errors.
- Keep UI review isolated from planner correctness review.

**Non-Goals:**
- Do not change core planning algorithms in this PR.
- Do not alter the planning-group data model.
- Do not include control/coordinator changes.

## Decisions

- Use a panel backend boundary so UI code does not directly own all manipulation-module orchestration.
- Treat planning-group IDs as the UI's stable selection keys.
- Validate base-link/root assumptions before rendering URDFs under base poses.
- Keep manual demo/checklist coverage for UI feel in addition to unit tests.

## Risks / Trade-offs

- UI tests can pass while interaction feels wrong; require a small manual checklist before review.
- This PR is still large, but isolating it keeps visual review focused.
