## 1. Contracts and Configuration

- [x] 1.1 Add phase-specific grasp pipeline error codes to `ManipulationSkillError` and unit-test their `SkillResult` serialization/logging behavior.
- [x] 1.2 Add validated `PickAndPlaceModuleConfig` fields for provider fallback, planning frame, input age, candidate limit, pre-grasp/retreat offsets, and gripper-feedback verification.
- [x] 1.3 Declare optional injected `ObjectSceneRegistrationSpec` and `GraspGenSpec` dependencies on `PickAndPlaceModule`, and add blueprint build tests for present, absent, and ambiguous providers.
- [x] 1.4 Define private typed transaction, phase, candidate, rejection, and verification-result models so state that changes together is not stored in parallel fields.

## 2. Target-Aware Planning Scene

- [x] 2.1 Add a scoped target-object suppression API to `WorldObstacleMonitor` and its `WorldMonitor` facade using the existing monitor lock.
- [x] 2.2 Ensure live perception updates cannot re-add a suppressed target while other object obstacles continue to add/update normally.
- [x] 2.3 Ensure suppression exit refreshes or restores the target on normal return, exception, cancellation, and partial setup failure.
- [x] 2.4 Add deterministic unit tests covering nested/duplicate suppression requests, concurrent perception updates, failed obstacle mutations, and cleanup.

## 3. Object Resolution and Candidate Selection

- [x] 3.1 Implement unique object resolution by stable ID or unambiguous current name, returning actionable failures without motion.
- [x] 3.2 Retrieve and validate the selected object's point cloud through `ObjectSceneRegistrationSpec`, including non-empty data, timestamp age, and planning-frame checks.
- [x] 3.3 Call `GraspGenSpec.propose_grasps`, validate the candidate-array header and poses, and preserve stable descending generator-score order.
- [x] 3.4 Derive pre-grasp and retreat targets from the configured TCP approach axis and candidate pose.
- [x] 3.5 Implement no-motion feasibility gating for pre-grasp, grasp, and retreat targets, capped by configuration and reporting rejection counts by reason.
- [x] 3.6 Retain the heuristic generator only behind explicit fallback configuration and identify the selected proposal source in results.
- [x] 3.7 Add unit tests for duplicate names, ID prefixes, missing/stale/wrong-frame clouds, provider failures, malformed candidates, stable score ties, candidate limits, and lower-ranked feasible selection.

## 4. Pick Transaction and Verification

- [x] 4.1 Add a single-active-pick guard that rejects concurrent transactions without mutating robot, gripper, proposal, or obstacle state.
- [x] 4.2 Implement the `PREPARE`, `APPROACH`, `GRASP`, `CLOSE`, `VERIFY`, and `RETREAT` phase runner behind the existing `pick` signature.
- [x] 4.3 Check every planning, execution, wait, and gripper-command result; regenerate motion plans from live state at each phase and terminate after the first post-motion failure.
- [x] 4.4 Replace fixed grasp sleeps with timeout-bounded gripper feedback polling and robot-specific held/empty threshold evaluation.
- [x] 4.5 Preserve a closed gripper on every post-closure failure, include “object may be held” context, and store `_last_pick_pose` only after verified closure and successful retreat.
- [x] 4.6 Guarantee transaction and target-suppression cleanup through one exit path while retaining the primary failure if cleanup also fails.
- [x] 4.7 Add phase-by-phase unit tests for success, command rejection, planning/execution failure, timeout, empty close, retreat failure, cleanup failure, and concurrent calls.

## 5. Blueprint and Agent Integration

- [x] 5.1 Define reviewed xArm sweep-volume, grasp-frame-to-TCP, approach-axis, and closure-verification configuration without performing model or hardware work at import time.
- [x] 5.2 Add a distinct GraspGenX-enabled xArm perception blueprint and agentic blueprint that compose exactly one perception provider, proposal provider, manipulation module, MCP server, and MCP client.
- [x] 5.3 Keep existing xArm perception blueprints free of the `graspgenx` runtime requirement and explicitly configure their intended heuristic fallback behavior.
- [x] 5.4 Update the manipulation agent prompt to keep `pick` as the sole high-level pick tool and document exact-name/object-ID disambiguation plus safe recovery.
- [x] 5.5 Regenerate `dimos/robot/all_blueprints.py` through `test_all_blueprints_generation.py` and verify the new names appear in `dimos list`.

## 6. End-to-End Validation and Documentation

- [x] 6.1 Add integration tests with fake perception, proposal, planner/coordinator, and gripper feedback providers that exercise the full RPC/Spec-wired pipeline.
- [x] 6.2 Add a replay or fixture-based test proving object/proposal/planning frame consistency with real `PointCloud2` and `GraspCandidateArray` messages.
- [ ] 6.3 Validate the GraspGenX-enabled blueprint startup and one successful/one infeasible candidate flow with the `graspgenx` extra on a GPU-capable environment.
- [ ] 6.4 Calibrate and record the xArm empty-close versus held-object threshold across representative object widths before enabling verification on hardware.
- [ ] 6.5 Run guarded real-xArm tests for success, empty grasp, unreachable proposals, execution interruption, and retreat failure; verify the gripper never auto-opens after closure.
- [x] 6.6 Update manipulation capability documentation with architecture, configuration, failure semantics, and the distinction between closure-proxy verification and force/slip verification.
- [x] 6.7 Run focused pytest suites, blueprint-generation validation, formatting/lint checks, and mypy on touched modules.
