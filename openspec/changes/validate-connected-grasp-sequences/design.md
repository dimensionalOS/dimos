## Context

`PickAndPlaceModule` currently gates ranked grasp candidates by solving collision-aware IK independently for pre-grasp, grasp, and retreat poses. This proves that each pose is individually reachable, but not that collision-free paths connect the robot's current state to those poses in order.

The generic planner contract already accepts explicit `start` and `goal` joint states. The RoboPlan adapter is the exception: it rejects a valid explicit start when it differs from the authoritative live state, even though RoboPlan's native RRT accepts arbitrary start and goal configurations. The pick pipeline also performs an optional low-height safety lift immediately before approach, so candidate validation must account for that shared segment.

Upstream perception remains responsible for supplying a target-only segmented cloud in the manipulation planning frame. This change starts at grasp proposals and does not add detection or segmentation behavior.

## Goals / Non-Goals

**Goals:**

- Require a connected, collision-free sequence before accepting a grasp candidate.
- Align RoboPlan with the explicit-start planner contract without mutating authoritative live state.
- Include a required safety lift as one shared prerequisite rather than repeating it for every candidate.
- Keep validation motion-free and retain execution-time replanning from measured state.
- Validate the behavior with compact, deterministic fixtures.

**Non-Goals:**

- Snapshotting or freezing the planning scene across validation calls.
- Attaching generated object geometry to the gripper for retreat planning.
- Coordinated arm-and-gripper planning or validation of finger sweeps.
- Reusing dry-run paths for physical execution.
- Adding perception, point-cloud segmentation, a new timeout, or backend-specific public error codes.

## Decisions

### 1. RoboPlan will honor an explicit planning start

Both robot-scoped and selected-group RoboPlan planning entry points will pass the supplied normalized start and goal to native RRT. The live planning context remains authoritative for unselected joints, other robots, and the rest of the scene. Planning from a hypothetical selected-joint start MUST NOT update the live context.

Alternative: clone and overwrite the whole live scene for every request. Rejected because RoboPlan already represents the selected planning group's start explicitly, while the live scene is still needed for unselected state.

### 2. Connected validation will be a side-effect-free planning helper

A manipulation planning helper will accept an ordered pose sequence and an optional explicit selected-joint start. For each pose it will:

1. solve collision-aware IK seeded by the current sequence endpoint;
2. plan from that endpoint to the IK solution;
3. use the returned path endpoint as the next segment's IK seed and planning start.

It will return the first failed pose index and the final endpoint on success. It will not store a preview/execution plan, change manipulation state, or dispatch motion.

Alternative: call the existing stateful `plan_to_pose` API three times. Rejected because failed screening would fault manipulation state, overwrite the execution plan, and prevent clean evaluation of lower-ranked candidates.

### 3. The safety lift is planned once as shared preparation

The current low-height check will be factored so the required lift target can be computed without moving. If no lift is required, candidate validation begins at authoritative current state. If a lift is required, it is dry-run planned once:

- failure aborts the pick in `PREPARE` with `PLANNING_FAILED`;
- success supplies a common hypothetical endpoint for every candidate.

The lift failure is not a candidate rejection because changing candidates cannot make the shared preparation segment feasible.

### 4. Candidate validation is connected and ordered

For each candidate in generator-score order, the pipeline derives pre-grasp and retreat poses and validates:

```text
shared start -> pre-grasp -> grasp -> retreat
```

The first failed segment increments the existing stage-level rejection counter. A candidate is selected only after all three paths succeed. No motion or gripper command occurs while candidates are screened.

Alternative: retain independent collision-aware IK checks. Rejected because disconnected feasible configurations do not establish an executable grasp sequence.

### 5. Execution replans rather than reusing validation paths

The selected candidate retains its poses, not the dry-run paths. Physical execution continues to plan each phase from fresh measured state immediately before dispatch. If an execution-time plan fails after motion begins, the transaction stops and does not try a different candidate.

Alternative: execute the exact dry-run paths. Rejected because controller error, settling, preparation motion, and live scene changes can make stored paths stale.

### 6. MVP validation uses the latest live scene and bounded collision fidelity

Every segment uses the latest available non-target scene. The target remains suppressed during validation and execution, but non-target updates are not frozen. Retreat collision checking covers the robot and gripper state represented in the planning scene; it does not include attached-object geometry. The dry run does not model separate open and closed finger configurations.

These limits are explicit so a successful result is not described as payload-safe or as a validated finger sweep.

### 7. Reporting remains stable and stage-oriented

Public rejection counts remain `pre_grasp_infeasible`, `grasp_infeasible`, and `retreat_infeasible`. IK status, planner status, timeout, and no-path details are logged for diagnosis rather than added to the skill-result contract.

### 8. Tests use a small reusable fixture set

Tests will be layered:

- orchestration unit tests with deterministic IK and planner results;
- RoboPlan contract tests proving arbitrary starts and live-state preservation;
- an open scene plus one parameterized blocker for segment failures;

This avoids maintaining a separate fixture file for every rejection case.

## Risks / Trade-offs

- [A scene update can occur between connected segment checks] → Use the latest scene for every segment and replan again before execution.
- [Execution-time IK can choose a different configuration than validation] → Seed from fresh measured state and require a complete collision-free execution plan before each dispatch.
- [Retreat can be safe for the robot but unsafe for the held object's volume] → State the MVP limitation explicitly and defer attached-object planning.
- [Current gripper geometry may differ from approach or retreat geometry] → Do not claim finger-sweep validation; add coordinated gripper planning only as a later capability.
- [Ranked screening multiplies planner calls] → Retain the configured candidate cap and existing planner timeout; planning is expected to be cheap for the MVP.

## Migration Plan

1. Land explicit-start RoboPlan behavior with backend contract tests.
2. Add the side-effect-free connected-sequence planning helper.
3. Integrate shared safety-lift and candidate-sequence validation.
4. Add deterministic integration coverage.
5. Enable the behavior through the existing `pick` pipeline with no caller migration.

Rollback restores independent IK screening and the RoboPlan live-start guard. The public `pick` signature and configuration remain unchanged.

## Open Questions

No material MVP design questions remain. Payload attachment, coordinated gripper geometry, and physical gripper-threshold tuning are explicitly deferred follow-up work.
