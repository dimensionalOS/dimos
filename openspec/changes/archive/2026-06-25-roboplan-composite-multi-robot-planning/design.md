## Context

DimOS manipulation planning uses a Planning world as the authoritative belief state for robot and scene state. The public planning surface is group-oriented: callers select one or more Planning groups and expect returned paths in global joint-name order.

RoboPlan, however, constructs a `Scene` from a URDF/SRDF pair and selects an existing SRDF group through `RRTOptions.group_name`. Current `RoboPlanWorld` supports only one robot per `Scene`, creates the scene during `add_robot`, and cannot represent inter-robot collisions or coupled bimanual motion. RoboPlan does not expose a dynamic API for adding planning groups after scene construction, so any Composite planning groups must exist in the generated SRDF before the RoboPlan `Scene` is created.

The accepted architectural direction is captured in `docs/adr/0001-generated-composite-roboplan-models.md`: multi-robot RoboPlan uses one generated Composite RoboPlan model, not one RoboPlan scene per robot.

## Goals / Non-Goals

**Goals:**

- Support coupled RoboPlan-native planning across multiple selected Planning groups.
- Build one Composite RoboPlan model from registered robot models, applying each `RobotModelConfig.base_pose` exactly once.
- Preserve DimOS public names and selection order at API boundaries while using RoboPlan-native prefixed names internally.
- Generate deterministic Composite planning groups eagerly at world finalization.
- Keep inter-robot collisions enabled by default and preserve per-robot collision exclusions where they can be rewritten safely.
- Keep non-selected joints fixed at the Planning world's current full state during RoboPlan group RRT.

**Non-Goals:**

- Runtime mutation of RoboPlan planning groups after scene construction.
- A user-facing configuration language for declaring named Composite planning groups.
- Splitting returned bimanual paths into per-robot plans for the coordinator.
- Baking dynamic obstacles into the generated URDF/SRDF.
- Supporting arbitrary model formats beyond the robot model inputs already accepted by `RobotModelConfig`.

## Decisions

### Generate one Composite RoboPlan model for multi-robot worlds

For two or more robots, `RoboPlanWorld` will defer RoboPlan `Scene` construction until finalization and generate one URDF/SRDF pair containing all registered robots under a synthetic world root. Each robot is attached by a fixed joint using `RobotModelConfig.base_pose`.

**Rationale:** One RoboPlan `Scene` is required for inter-robot collision checks and coupled planning. Separate scenes are simpler but cannot reason about cross-robot collisions or coordinated motion.

**Alternative considered:** One RoboPlan scene per robot. Rejected for true bimanual planning because it only supports independent per-arm motion.

### Prefix RoboPlan-native names, preserve DimOS public names

All robot-local URDF link, joint, and frame names will be rewritten with a RoboPlan-native-safe prefix, such as `left_arm__joint1`. Public DimOS names remain `left_arm/joint1`, and `PlanningResult.path` remains in global selection order.

**Rationale:** Multiple robots often share local joint and link names. Prefixing prevents collisions in the composite URDF/SRDF while preserving the existing DimOS API contract.

**Alternative considered:** Use global DimOS names directly in RoboPlan. Rejected because `/` may not be safe across all URDF/SRDF consumers and because RoboPlan-facing names should remain backend-private.

### Generate Composite planning groups eagerly with a safety cap

At finalization, RoboPlan will generate one SRDF group for each non-overlapping Planning-group combination of size at least two, plus each individual configured Planning group. Composite group identity uses canonical registry order, not caller selection order. If the number of generated Composite planning groups exceeds `max_generated_composite_groups`, finalization fails clearly.

**Rationale:** RoboPlan groups are selected by name from the SRDF at planning time and cannot be added dynamically. Eager generation makes supported combinations deterministic and avoids hidden scene rebuilds.

**Alternative considered:** Lazy generation per request. Rejected because it would require rebuilding the RoboPlan `Scene` and invalidating backend state.

### Multi-robot SRDF is always generated

Single-robot RoboPlan may continue to pass a provided `RobotModelConfig.srdf_path` directly to RoboPlan. Multi-robot RoboPlan always generates a composite SRDF. Per-robot SRDFs may be parsed as source material for group and collision-disable extraction, but they are not passed through as final RoboPlan input.

**Rationale:** A multi-robot SRDF must reference the prefixed names in the generated composite URDF. Passing through per-robot SRDFs would reference stale names and omit Composite planning groups.

### Set RoboPlan full current state before group RRT

Before invoking RoboPlan-native RRT, DimOS will assemble the full composite scene joint vector from the Planning world and call RoboPlan's full current joint-state setter. Start and goal are still passed as `JointConfiguration` values for the selected RoboPlan group, and options such as `collision_check_use_bisection` are set explicitly.

**Rationale:** RoboPlan's group expansion holds non-selected joints at `Scene` current state. That current state must match the Planning world for fixed joints and non-selected robots to be meaningful.

**Alternative considered:** Trust only the `start` `JointState`. Rejected because the start is a selected projection, not the authoritative full-world state.

### Reject selected-start disagreement with the Planning world

`plan_selected_joint_path` will compare the selected `start` against the Planning world's current selected state. If they disagree beyond a tight configurable tolerance, it returns `PlanningStatus.INVALID_START` before invoking RoboPlan.

**Rationale:** The Planning world is the source of truth. Allowing an independent start state would make non-selected state and selected state internally inconsistent.

### Return caller-order global paths

RoboPlan plans in canonical native group order. DimOS will convert each returned waypoint back to a global `JointState` in `PlanningGroupSelection.joint_names` order.

**Rationale:** This keeps RoboPlan backend details private and preserves the current selected-planning contract.

## Risks / Trade-offs

- **Composite URDF/SRDF rewriting is complex** → Keep prefixing/mapping helpers small and covered by focused unit tests for links, joints, frames, limits, groups, and collision pairs.
- **Base placement can be applied twice** → Treat `RobotModelConfig.base_pose` as the only composite placement source and continue honoring `strip_model_world_joint` when removing model-authored world joints.
- **Composite group count can grow combinatorially** → Enforce `max_generated_composite_groups` and fail finalization with a message that lists how many groups would be generated.
- **Per-robot SRDF collision disables may not always rewrite safely** → Preserve explicit `collision_exclusion_pairs`; rewrite SRDF disables only when both referenced links can be mapped unambiguously.
- **RoboPlan Python defaults may diverge from C++ defaults** → Set planner options explicitly in DimOS instead of relying on binding defaults.
- **Generated model bugs can be hard to diagnose** → Persist or expose generated URDF/SRDF paths in errors/logs so users can inspect the exact RoboPlan input.

## Migration Plan

1. Introduce composite model data structures and name-mapping helpers without changing single-robot behavior.
2. Move RoboPlan `Scene` construction from `add_robot` to world finalization and keep single-robot pass-through behavior intact.
3. Add generated composite URDF/SRDF construction for multi-robot registrations.
4. Add Composite planning group lookup and selected-planning conversion.
5. Update collision/FK/Jacobian helpers to assemble full composite q vectors from the Planning world.
6. Add dual-arm tests and run existing single-robot RoboPlan tests to guard compatibility.

Rollback is straightforward before adoption: disable the multi-robot path and retain the existing single-robot RoboPlan backend. After callers depend on coupled multi-robot planning, rollback requires changing those blueprints back to Drake or another supported backend.

## Open Questions

- Should future user-facing configuration allow named Composite planning groups instead of generating every non-overlapping combination?
- What is the first supported path for extracting collision disables from arbitrary per-robot SRDF files when references are ambiguous?
- Should `max_generated_composite_groups` be exposed through blueprint config immediately or remain a constructor-only backend option initially?
