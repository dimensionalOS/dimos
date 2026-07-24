## 1. Point-cloud preprocessing and pose sourcing

- [x] 1.1 Add the reusable point-cloud self-filter module for removing robot-body occupancy from the simulated wrist-camera cloud while preserving the planning-world frame and point-cloud metadata.
- [x] 1.2 Add the TF pose-source module needed by the voxel-mapping path to provide the wrist-camera pose from the configured transform tree.
- [x] 1.3 Add unit tests for self-filter robot-point removal, empty/non-robot clouds, frame and metadata propagation, and TF pose lookup success/failure.
- [x] 1.4 Add focused stream/type wiring tests showing the self-filter and TF pose source can feed the existing voxel-map inputs.

## 2. Octree obstacle model and backend adapters

- [x] 2.1 Add the octree obstacle model/type for complete 0.05 m planning collision snapshots, including explicit planning-frame and resolution data.
- [x] 2.2 Add explicit unsupported-operation handling for planning backends that cannot register octree collision geometry, before any world mutation.
- [x] 2.3 Implement the native RoboPlan OcTree adapter using one native OcTree box per occupied voxel with the required transform/order convention and `addOcTreeGeometry` registration.
- [x] 2.4 Add unit tests for octree construction, empty occupancy, all-voxel retention, native RoboPlan conversion, and planning-frame placement.
- [x] 2.5 Add backend tests proving unsupported octree registration raises the explicit error without silently discarding or approximating occupancy.

## 3. Planning collision snapshot lifecycle

- [x] 3.1 Add a thin planning-collision-snapshot helper outside `ManipulationModule` that validates the planning-world frame, carries the configured 0.05 m resolution, and owns staged/committed generation state.
- [x] 3.2 Integrate `ManipulationModule.planning_voxel_map` receipt with latest-wins staging, rejecting invalid frames without changing staged or committed state.
- [x] 3.3 Integrate plan-start synchronization so the first non-empty staged snapshot is added, later snapshots update complete geometry under the same native ID, and empty snapshots remove it before collision-aware planning proceeds.
- [x] 3.4 Add `WorldSpec.update_obstacle`, implement stable-ID synchronization, and advance committed state only after backend-confirmed add, update, or remove.
- [x] 3.5 Add unit tests for frame rejection, latest-wins staging, no-freshness-policy behavior, empty snapshots, stable native IDs, and unchanged committed state when replacement fails.
- [x] 3.6 Add unit tests for unknown active IDs, failed replacement construction/insertion, planning-start integration, and atomic query visibility.

## 4. Committed collision visualization

- [x] 4.1 Integrate Viser rendering with the backend-accepted obstacle lifecycle, keeping any display cap independent from collision registration.
- [x] 4.2 Add lifecycle tests proving staged and failed snapshots are not rendered, successful stable-ID updates replace the accepted handle, and empty clears remove it.
- [x] 4.3 Add visualization lifecycle coverage for startup, update, clear, and shutdown without changing existing modeled-obstacle rendering behavior.

## 5. xArm voxel-planning simulation blueprint

- [x] 5.1 Add the dedicated xArm MuJoCo/Viser voxel-planning simulation blueprint under `dimos.robot.manipulators.xarm.blueprints.simulation` using `ManipulationModule` rather than `PickAndPlaceModule`.
- [x] 5.2 Compose the blueprint’s wrist-camera point cloud, self-filter, `RayTracingVoxelMap`, TF pose source, xArm coordinator, RoboPlan, manipulation planning, and Viser streams with a 0.05 m snapshot resolution.
- [x] 5.3 Add blueprint tests for module composition, stream connections, simulation-only configuration, planning snapshot delivery, and committed-state observability.
- [x] 5.4 Regenerate the built-in blueprint registry with `pytest dimos/robot/test_all_blueprints_generation.py` and verify the new blueprint is discoverable by `dimos list`.

## 6. User documentation

- [x] 6.1 Add `docs/usage/xarm_voxel_planning_viser_demo.md` covering installation, `dimos list`, `dimos run`, Viser connection, and the complete perception-to-planning flow using the defined Planning Collision Snapshot, Planning World Frame, and 0.05 m Snapshot Resolution terms.
- [x] 6.2 Document robot self-filtering, latest-wins staging, copy-on-write stable-ID updates, failure preservation, empty snapshots, frame mismatches, RoboPlan OcTree support, unsupported backends, and the independent Viser display cap.
- [x] 6.3 Link the new guide from `docs/usage/index.md` and `docs/capabilities/manipulation/index.md`, without adding Frontier README links.

## 7. Focused verification and manual QA

- [x] 7.1 Run focused pytest targets for self-filter/TF helpers, octree models and adapters, snapshot lifecycle, Viser lifecycle, and the xArm blueprint tests.
- [x] 7.2 Run production-Python type checks and lint/format checks for the changed implementation and test modules using the repository’s configured commands.
- [x] 7.3 Run `python -m dimos.utils.docs.doclinks docs/` and `bin/run-doc-codeblocks --ci docs/usage/xarm_voxel_planning_viser_demo.md` as required by `docs.md`.
- [x] 7.4 Run `openspec validate add-manipulation-voxel-grid` and resolve any change validation failures.
- [ ] 7.5 Run `dimos list` and manually launch the dedicated xArm voxel-planning Viser simulation blueprint to verify wrist-camera occupancy, filtered snapshots, accepted planning state, and accepted-backend rendering.
- [x] 7.6 Confirm the change does not include Frontier timeout/GUI/IK debug work, new CLI or skill behavior; the only new world API is full-geometry `update_obstacle`.

## 8. Asynchronous voxel-map synchronization correction

- [x] 8.1 Update `RayTracingVoxelMap` to accept asynchronous camera-cloud and odometry callbacks, plumb `pose_match_tolerance_s` with a 0.1 s default, accept valid non-default values, reject invalid values, and enforce a fixed small pending-cloud capacity that is not configurable.
- [x] 8.2 Implement readiness at watermark >= cloud timestamp, compare the preceding and first-at/after poses, select the nearest valid pose within tolerance, retain clouds before readiness, expire only at watermark >= cloud+tolerance when no valid pose exists, and retry pending clouds on odometry arrival.
- [x] 8.3 Preserve each cloud's source timestamp and enforce exactly-once handler/output behavior at the synchronization seam; add throttled warnings or counters distinguishing watermark expiry from capacity eviction with cloud, watermark, and pose-gap context.
- [x] 8.4 Change the xArm planning self-filter to drop the entire cloud when any required robot-region TF is unavailable; do not publish a partially filtered cloud.
- [x] 8.5 Add unit tests for the `T-50ms`/`T+3ms` nearest-pose case, pre-readiness retention, tolerance-boundary expiry, capacity eviction never processing, diagnostics classification/context, and default/non-default/invalid tolerance plumbing.
- [x] 8.6 Add synchronization-seam tests proving exactly-once handler and output behavior with source timestamp preservation, plus a partial self-filter case where one required TF exists and another is missing and no output is produced.
- [x] 8.7 Add an integration/blueprint test proving asynchronous camera-cloud and odometry delivery produces deterministic voxel snapshots without widening the demo; worker-lifecycle failures remain out of scope.
- [ ] 8.8 Repeat the live QA run across starts and observe previous-pose, next-pose, and exact-alignment gaps, recording matching, expiry, capacity, and diagnostic behavior; keep manual QA task 7.5 pending.
- [x] 8.9 Run focused tests, type/lint/format checks, `openspec validate add-manipulation-voxel-grid`, and `git diff --check`; resolve validation failures without changing completed checklist history or closing task 7.5.

## 9. RoboPlan full-model Jacobian projection correction

- [x] 9.1 Project full RoboPlan Jacobians through validated `JointGroupInfo.v_indices` metadata while preserving planning-group order and rejecting inconsistent/non-independent metadata.
- [x] 9.2 Add focused regressions for full xArm-style, reordered/noncontiguous, free-flyer-style, malformed metadata, mimic/multi-DOF, and non-finite Jacobian cases.
- [x] 9.3 Run focused RoboPlan/Jacobian verification, scoped lint/type checks, OpenSpec validation, and `git diff --check`.
- [ ] 9.4 Repeat the live xArm+gripper Viser evaluation and confirm the corrected Jacobian projection in the runnable demo; keep live tasks 7.5 and 8.8 pending.

## 10. Unified accepted-obstacle lifecycle correction

- [x] 10.1 Rebase the lifecycle design on PR #3108 and add the authoritative full-geometry `WorldSpec.update_obstacle` contract with stable native IDs.
- [x] 10.2 Serialize RoboPlan queries on the published scene while constructing replacement scenes outside the active-scene lock.
- [x] 10.3 Validate replacements before construction and preserve the published scene when replacement construction or insertion fails.
- [x] 10.4 Update `PlanningCollisionSnapshot` to add once, update one stable backend object, remove for empty snapshots, and commit only backend-confirmed state.
- [x] 10.5 Reuse PR #3108's visualization add operation as a same-ID upsert after successful backend updates, with no intermediate remove event.
- [x] 10.6 Update the OpenSpec artifacts, xArm guide, domain glossary, and ADR for atomic copy-on-write stable-ID updates.
- [x] 10.7 Run strict OpenSpec validation, focused lifecycle/spec tests, documentation checks, scoped type/lint checks, and `git diff --check`.
- [ ] 10.8 Perform live QA across startup, repeated same-ID OCTREE updates, empty removal, replacement-failure logging, and stable Viser scene replacement; keep existing live tasks 7.5, 8.8, and 9.4 pending.

## 11. Collision-query latency correction

- [x] 11.1 Solve Viser pose-target IK without collision checks, then validate the complete robot state exactly once.
- [x] 11.2 Build RoboPlan replacement scenes outside the active-scene lock and publish the complete scene and registry atomically.
- [x] 11.3 Add deterministic regressions for single collision validation, non-blocking collision queries during replacement construction, and failure preservation.
- [x] 11.4 Run focused tests, scoped lint/type checks, OpenSpec validation, and `git diff --check`.

## 12. Live snapshot and kinematics responsiveness correction

- [x] 12.1 Restore a capped, throttled Viser layer for the latest valid staged planning collision snapshot without changing authoritative accepted-obstacle rendering.
- [x] 12.2 Decouple RoboPlan FK and Jacobian queries from the active collision scene with a separate immutable robot-only scene and lock, and split Viser pose evaluation into independent latest-wins IK and collision workers.
- [x] 12.3 Add regressions for staged snapshot create/update/clear, visualizer forwarding, ghost-pose publication before collision completion, and kinematics progress while a collision query is blocked.
- [x] 12.4 Run focused manipulation and Viser regression tests plus scoped lint/type checks and `git diff --check`.

## 13. xArm IK convergence and camera TF correction

- [x] 13.1 Reproduce the demo's Jacobian IK behavior against the real RoboPlan xArm model and compare it with a supported convergent IK backend.
- [x] 13.2 Select Pink for demo pose-target IK while retaining RoboPlan as the collision world and default motion planner.
- [x] 13.3 Publish the simulated wrist-camera optical transform directly in `world`, removing the mapper's dependency on the derived manipulation link-TF chain.
- [x] 13.4 Update blueprint regressions and documentation and run focused verification.

## 14. Unified planning snapshot visualization

- [x] 14.1 Use the prompt blue staged snapshot as the single visualization for the reserved planning-collision obstacle.
- [x] 14.2 Suppress only its duplicate accepted OCTREE projection while preserving backend registration and ordinary accepted obstacle rendering.
- [x] 14.3 Add focused lifecycle coverage and update the visualization contract documentation.
