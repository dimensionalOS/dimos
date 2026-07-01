## Context

The current branch introduces planning groups across the stack, but the reviewable foundation is the abstraction itself: stable group IDs, local/global joint mapping, group definitions, RobotConfig discovery/generation, and removal of robot-scoped end-effector metadata.

## Goals / Non-Goals

**Goals:**
- Define planning groups as the only source for chain base links and pose target tip links.
- Preserve robot-scoped `base_link` as placement/weld/strip metadata, not a planning-chain selector.
- Let robot configs create explicit or discovered planning groups.
- Provide small, direct tests that prove group IDs, joint-name mapping, discovery, and strict config validation.

**Non-Goals:**
- Do not migrate world backends or planners in this PR.
- Do not migrate `ManipulationModule` public APIs in this PR.
- Do not include visualization or control changes.

## Decisions

- Use `PlanningGroupDefinition` for config-time group declarations and `PlanningGroup` for resolved runtime group IDs.
- Use globally scoped group IDs such as `<robot>/<group>` at API boundaries.
- Keep `RobotModelConfig.base_link` for robot placement semantics only.
- Reject legacy robot-scoped `end_effector_link`; pose target frames come from group `tip_link` or SRDF discovery.
- Keep this PR compatible enough for later stacked PRs; if a later change is needed for tests, prefer a small compatibility wrapper over pulling later files forward.

## Risks / Trade-offs

- Earlier PRs may need temporary compatibility paths because the full branch removes legacy behavior across several layers.
- Discovery behavior must be clear: high-level `RobotConfig.to_robot_model_config()` can discover/fallback groups, while direct `RobotModelConfig(...)` requires explicit groups.
