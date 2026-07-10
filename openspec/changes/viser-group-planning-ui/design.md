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
- Treat `cc/spec/movegroup@0edb8d3dd` as the sole source for visible controls and interaction behavior; do not derive behavior from the upstream extraction commit.
- Adapt the reference UI to PR 4's existing explicit group APIs rather than importing manipulation or planner changes from the reference branch.
- Keep execution trajectory parameterization in the manipulation/module layer. Group-native preview renderers may map canonical `GeneratedPlan` waypoints onto the requested display duration, but must not reinterpret them as execution timing.
- Use the reference group-native visualization contract: `show_preview(Sequence[PlanningGroupID])`, `hide_preview(Sequence[PlanningGroupID])`, and `animate_plan(GeneratedPlan, duration: float = 3.0)`. `ManipulationModule.preview_plan` supplies `1.0` when duration is omitted, matching the normative reference. Update Viser and the existing Meshcat implementation together; do not import unrelated source-branch planning changes.
- Treat preview as one validated transaction. Resolve and deduplicate affected robots, snapshot each robot's non-selected joints once, project selected global joints into complete local frames from that fixed baseline, reject malformed/unknown/incomplete input before showing ghosts, then update every ghost once per shared tick before sleeping once. Unequal renderer frame counts map onto the same normalized clock.
- Use an animation generation/cancellation token. Cancel, clear, close, or a replacement preview invalidates the old generation so it cannot mutate removed or replacement scene handles. Serialize preview-scene mutation against periodic visualization publication.
- Group selection owns a monotonically increasing epoch. Target/plan callbacks commit only if both their operation sequence and selection epoch still match. Execute snapshots every affected robot immediately before dispatch, compares every selected group's ordered joints, and calls `ManipulationModule.execute()` without a robot filter.
- Execute snapshots every affected robot immediately before dispatch and delegates the stored plan without a robot filter. An external caller can unavoidably replace that stored plan between the panel freshness check and dispatch.
- Do not copy absent reference APIs such as `ManipulationModule.forward_kinematics` or `check_collision`. Use PR4 `WorldMonitor.get_group_ee_pose` for group FK and `WorldMonitor.is_state_valid` on composed full per-robot target states. Multi-robot validity is a conjunction of per-robot checks, not proof of simultaneous target-target collision freedom; pose IK retains PR4's coordinated collision check.

## Risks / Trade-offs

- UI tests can pass while interaction feels wrong; require a small manual checklist before review.
- This PR is still large, but isolating it keeps visual review focused.
- A whole-commit cherry-pick would leak unrelated planner and module changes; extraction must be semantic and visualization-scoped.
- The preview contract is the sole approved scope expansion outside visualization. Keep its module/monitor/protocol/world changes minimal and independently testable.
- A compatibility `robot_name` argument may validate that the named robot is affected, but MUST NOT trim a multi-robot generated plan before preview.
