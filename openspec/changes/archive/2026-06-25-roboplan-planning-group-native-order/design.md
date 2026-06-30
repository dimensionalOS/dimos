## Context

The manipulation stack has moved toward planning-group-first APIs. `ManipulationModule` plans through `PlannerSpec.plan_selected_joint_path(...)`, and `WorldSpec` exposes `get_group_ee_pose(...)` and `get_group_jacobian(...)`. Drake already implements those group-oriented world queries.

`RoboPlanWorld` is still mostly robot-scoped. It stores full robot state in `RoboPlanContext.q_by_robot`, generates an SRDF group, converts `JointState` inputs to `config.joint_names`, and its native planning entry point predates the up-to-date selected-group planner API. RoboPlan itself supports SRDF groups through `Scene.getJointGroupInfo(...)`, `Scene.toFullJointPositions(...)`, and `RRTOptions.group_name`, but it does not expose a dedicated persistent planning-group joint-state API. DimOS must continue owning belief state and pass explicit vectors to RoboPlan queries.

The user-facing interface should stay lean: use the existing public `PlanningGroup` model and `PlanningGroupRegistry`; avoid adding a custom group dataclass unless it encodes backend-only information not already available publicly.

## Goals / Non-Goals

**Goals:**

- Make RoboPlan-native planning implement `plan_selected_joint_path(...)` for a single selected planning group.
- Keep full robot-local state as the canonical world belief in `RoboPlanContext.q_by_robot`.
- Convert between public group order, robot-local order, and RoboPlan native group order by joint name.
- Use existing `PlanningGroup` objects for public group metadata.
- Store only minimal RoboPlan-specific metadata, primarily native joint order per planning group.
- Keep generated SRDF support simple and deterministic.
- Keep the RoboPlan adapter interface minimal by implementing only the up-to-date `PlannerSpec` and `WorldSpec` group-oriented methods.

**Non-Goals:**

- Do not add multi-group or multi-robot RoboPlan-native coupled planning in this change.
- Do not build a full SRDF authoring or parsing system in `RoboPlanWorld`.
- Do not replace the public planning-group models or create a parallel custom interface.
- Do not change the high-level `ManipulationModule` planning API.
- Do not add new RoboPlan dependencies.
- Do not preserve RoboPlan legacy wrapper APIs such as `plan_joint_path(...)`, `get_ee_pose(...)`, or `get_jacobian(...)`.
- If protocol definitions still expose deprecated robot-scoped planner/world methods, update the protocol/factory contract so RoboPlan is validated through the group-oriented APIs only.

## Decisions

### Decision: Keep belief state in full robot-local order

`RoboPlanContext.q_by_robot[robot_id]` remains the canonical belief state, ordered by `robot.config.joint_names`.

Rationale: live joint-state synchronization, compatibility collision checks, and whole-robot wrappers already operate on full robot state. RoboPlan does not provide a persistent per-group state API, so DimOS must own state and provide explicit query vectors.

Alternative considered: store group state directly in the context. Rejected because group selections can overlap or omit joints, and collision/FK calls still need a full-scene vector.

### Decision: Use public `PlanningGroup` plus minimal backend metadata

`RoboPlanWorld` should add a `PlanningGroupRegistry` and a robot-name lookup, then store only backend-specific extras:

```python
self._planning_groups: PlanningGroupRegistry
self._robot_ids_by_name: dict[RobotName, WorldRobotID]
self._roboplan_native_joint_names: dict[PlanningGroupID, tuple[str, ...]]
```

Rationale: `PlanningGroup` already contains group ID, robot name, public/global joint names, local joint names, base link, and tip link. A new `_RoboPlanGroupData` class would duplicate public fields without adding meaning.

Alternative considered: create a RoboPlan-specific group dataclass. Rejected unless implementation later reveals multiple backend-only fields that are always passed together.

### Decision: Keep generated SRDF narrow; require SRDF for complex cases

When `RobotModelConfig.srdf_path` is absent, generated SRDF supports exactly one configured planning group and writes that group using `PlanningGroupDefinition.name`. If more than one group is configured, `RoboPlanWorld.add_robot(...)` fails early with a message instructing the caller to provide `RobotModelConfig.srdf_path`.

When `RobotModelConfig.srdf_path` is present, pass it directly to `roboplan_core.Scene(...)` and validate each configured DimOS group against `Scene.getJointGroupInfo(group.group_name)`.

Rationale: RoboPlan already consumes SRDF. Complex group semantics should live in user-provided SRDF instead of expanding DimOS into a partial SRDF generator.

Alternative considered: generate all configured SRDF groups. Rejected because it duplicates SRDF semantics and risks generating incomplete or misleading SRDF for complex robots.

### Decision: Centralize conversions by name

All order changes should go through helpers. Suggested helper responsibilities:

- convert named or unnamed local `JointState` to full robot-local `q`
- project full robot-local `q` to RoboPlan native group `q`
- convert public selected global `JointState` to RoboPlan native group `q`
- convert RoboPlan native group path waypoints back to public selected global `JointState`
- overlay group-native positions onto a full robot-local `q` only if a current group-oriented query requires non-group joints to be preserved

Rationale: this refactor is about avoiding hidden order assumptions. Conversion code should be small, explicit, and test-covered.

Alternative considered: inline conversions in planning/FK methods. Rejected because duplicated order logic is error-prone.

### Decision: Support one selected group first

`plan_selected_joint_path(...)` initially supports only selections containing exactly one planning group on one robot where the selected joints exactly match that group. Unsupported selections return `PlanningStatus.UNSUPPORTED`.

Rationale: RoboPlan RRT accepts one `RRTOptions.group_name`. Supporting coupled multi-group planning requires additional semantics not needed for the immediate integration.

Alternative considered: silently fall back to full robot planning for multi-group selections. Rejected because it would obscure unsupported behavior and can plan joints the caller did not select.

### Decision: Implement only current protocol APIs

RoboPlan should implement the up-to-date protocol surface used by the current codebase: `PlannerSpec.plan_selected_joint_path(...)` plus `WorldSpec.get_group_ee_pose(...)` and `WorldSpec.get_group_jacobian(...)`. Legacy RoboPlan-specific robot-scoped planning/FK/Jacobian wrapper behavior should be removed from the refactor plan rather than maintained. If the protocol file still includes deprecated robot-scoped methods, this change should narrow the RoboPlan planner/world validation path to the group-oriented contract.

Rationale: a compatibility layer would enlarge the adapter surface and keep order-conversion behavior alive in paths the current stack should no longer call. The goal is to make RoboPlan follow the same minimal group-native interface as the rest of the up-to-date planning stack.

Alternative considered: keep wrappers for old callers. Rejected because the user explicitly wants the RoboPlan interface to stay bare-minimal and aligned only with current `PlannerSpec`/`WorldSpec`.

## Risks / Trade-offs

- Generated SRDF becomes stricter for multi-group configurations → Mitigation: fail early with a clear message and support user-provided `srdf_path`.
- RoboPlan native group order may differ from DimOS public order → Mitigation: store and test `Scene.getJointGroupInfo(...).joint_names`, and reorder by name at every boundary.
- Returned `JointPath` may not expose names in every binding variant → Mitigation: if names are present, validate/reorder by them; otherwise assume the configured native order only after validating waypoint lengths.
- Removing RoboPlan legacy wrappers may break stale callers that still invoke robot-scoped methods directly → Mitigation: update tests/factory usage to the current protocol and let stale callers fail loudly instead of preserving duplicate paths.
- Existing fake RoboPlan tests may not model all binding details → Mitigation: extend fakes only around documented APIs used by the adapter and keep failure modes explicit.

## Migration Plan

1. Add registry and minimal native-order metadata during `RoboPlanWorld.add_robot(...)`.
2. Update SRDF selection/generation logic with clear validation.
3. Add conversion helpers and tests for order projections.
4. Implement group FK/Jacobian only through the current `WorldSpec` group methods.
5. Implement `plan_selected_joint_path(...)` as the RoboPlan native planning entry point.
6. Update protocol/factory validation as needed so RoboPlan is checked against the group-oriented current contract, not deprecated robot-scoped methods.
7. Run targeted RoboPlan world/planning factory tests.

Rollback is straightforward because changes are isolated to the RoboPlan adapter and tests; revert the adapter and factory validation if native group planning causes regressions.

## Open Questions

- Should `RoboPlanWorld` accept one generated subset group without SRDF, or should generated SRDF require the group to cover all controllable joints for maximum predictability?
