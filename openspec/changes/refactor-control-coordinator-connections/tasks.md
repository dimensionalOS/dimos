# Stacked Implementation Plan

The numbered sections are review and merge units. PR1 and PR2 have merged as #3420 and #3431. PR2a reconciles their implementation with the final reviewed design and current `main` before new control contracts land. PR4a–PR4c may proceed in parallel after PR3; PR5 depends on all connection families needed by registered production blueprints. PR #3381 is a prerequisite. Every cutover removes its obsolete path in the same PR; there is no compatibility layer.

```text
#3381 -> PR1 (merged) -> PR2 (merged) -> PR2a cleanup -> PR3
                                                      |
                                                      v
                                      [PR4a, PR4b, PR4c] -> PR5 -> PR6 -> PR7
```

## 0. Prerequisite and Inventory

- [ ] 0.1 Base the stack on merged gripper API PR #3381 and record the final gripper task API used by PR6.
- [ ] 0.2 Inventory every registered blueprint that constructs a control coordinator, manipulation module, hardware adapter, or migrated connection; assign each to PR4a, PR4b, PR4c, or an explicit removal decision.
- [ ] 0.3 Inventory public coordinator, manipulation, world, planning, visualization, skill/MCP, and configuration surfaces containing `robot_id`, robot names, hardware IDs, adapter registries, or hardware-specific gripper plumbing.
- [ ] 0.4 Record focused test targets and available hardware owner for xArm/gripper, G1, Piper/OpenArm, and mobile-base families before changing code.

## 1. PR1 — Static Single-Robot Model (merged as #3420)

- [x] 1.1 Add a parser compatibility test for canonical `/` names across every supported manipulation backend and record which backend, if any, needs private reversible encoding.
- [x] 1.2 Define the one-model configuration containing prepared URDF, optional SRDF, logical root, optional whole-model pose, and planning-group declarations.
- [x] 1.3 Define planning groups with group name, canonical joint names, base link, and tip link; remove the singular model-wide end-effector assumption from the new shape.
- [x] 1.4 Prepare and check in or generate through the existing asset workflow one static dual-xArm mock URDF/SRDF fixture with `left/...` and `right/...` canonical names.
- [x] 1.5 Update Drake, RoboPlan, Pink, and visualization model loaders to consume one prepared model configuration and isolate any necessary backend-native name encoding.
- [x] 1.6 Add model-validation errors for missing joints, duplicate canonical names, invalid groups, invalid base/tip links, and malformed prepared model assets.
- [x] 1.7 Convert focused model/backend fixtures and tests to one-model inputs and canonical planning groups.
- [x] 1.8 Remove superseded multi-model configuration properties within the model-loading boundary; do not add aliases.
- [x] 1.9 Run focused model loader, FK, collision, IK, and dual-arm fixture tests for every installed backend.

## 2. PR2 — Single-Robot Manipulation Cutover (merged as #3431)

- [x] 2.1 Change manipulation configuration from a robot list/registry to exactly one prepared model configuration.
- [x] 2.2 Remove `RobotName`, `WorldRobotID`, public robot selectors, and manipulation robot lookup/list APIs.
- [x] 2.3 Remove local/global joint-name types, robot-prefix parse/make helpers, and mapping inversion from planning and execution.
- [x] 2.4 Change world state, collision, FK, and model access APIs to operate on the one logical model without a robot-ID parameter.
- [x] 2.5 Change planning-group selection to select canonical joint subsets and group-specific frames without robot membership.
- [x] 2.6 Simplify Pink and other multi-target solvers to solve several groups within one model rather than grouping by robot.
- [x] 2.7 Replace per-robot initialization state, trajectory splitting, and execution targets with one canonical joint-state and trajectory flow.
- [x] 2.8 Replace visualization and world-monitor maps keyed by robot ID with one logical model state.
- [x] 2.9 Remove robot selectors from semantic manipulation RPCs, DimOS `Spec` Protocols, skills, MCP schemas, and callers.
- [x] 2.10 Convert the dual-xArm mock and authored bimanual blueprints to the static prepared model and planning groups.
- [ ] 2.11 Delete obsolete multi-robot code paths and compatibility properties exposed by the migrated manipulation surfaces; remaining post-merge work is itemized in PR2a.
- [x] 2.12 Run focused manipulation, planning, collision, FK, execution, visualization, skill-schema, and blueprint-build tests.

## 2a. PR2a — Post-Merge Model and Manipulation Cleanup

- [ ] 2a.1 Rebase the implementation stack on current `main` after #3420 and #3431, and audit their actual public surface against the final single-robot and canonical-name specifications rather than their original PR descriptions.
- [ ] 2a.2 Remove the singular `end_effector_link` compatibility projection from manipulation model-info and visualization types; callers select a planning group and use its `tip_link`.
- [ ] 2a.3 Rename remaining local/global, robot-prefixed, or multi-robot terminology in one-model implementation helpers, variables, test names, documentation, and errors when it describes canonical joint ordering rather than a real backend-private mapping.
- [ ] 2a.4 Delete transition-only aliases, adapters, fixtures, and redundant regression coverage left by the two merged cutovers; do not preserve their obsolete APIs for compatibility.
- [ ] 2a.5 Re-run repository searches for `RobotName`, `WorldRobotID`, manipulation-facing `robot_id`/`robot_name`, robot registries, prefix parsers, singular model-wide end effectors, and per-robot trajectory splitting, and either remove each result or document why it belongs to an unrelated domain.
- [ ] 2a.6 Keep `gripper_hardware_id`, coordinator task-name construction, and model-cached gripper limits explicitly assigned to PR6, where the connection-described replacement lands; do not delete a working path before its end-to-end replacement exists.
- [ ] 2a.7 Run the complete manipulation, planning-backend, visualization, dual-arm, blueprint, skill-schema, mypy, and ruff gates on current `main` before starting PR3.

## 3. PR3 — Scalar Control and Lifecycle Contracts

- [ ] 3.1 Add universal live module readiness, distinct from worker liveness: successful initialization marks ready, modules can withdraw readiness with a diagnostic reason, and readiness changes are observable by other modules.
- [ ] 3.2 Add framework tests for startup failure, initial readiness, live readiness loss and recovery, process death, and readiness diagnostics.
- [ ] 3.3 Define an encodable, self-describing `ControlValues` message with source, source timestamp, producer/control epoch, sequence, fully qualified interface names, and parallel `float64` values; names remain explicit in every frame rather than becoming description-defined indices.
- [ ] 3.4 Define encodable `ConnectionDescription`, `ConnectionStatus`, and `ConnectionControl` contracts with description epochs, control epochs, correlated lifecycle operation IDs, `PREPARE_ARM`, `COMMIT_ARM`, `ABORT_ARM`, `SAFE_STOP`, `CLEAR_SAFE_STOP`, `ESTOP`, and `CLEAR_ESTOP`; include `PREPARING`, `PREPARED`, and `COMMITTING`, use `STANDBY` rather than a generic disarmed state, and do not duplicate universal readiness or expose native command-interface transition state.
- [ ] 3.5 Define typed description structures for joint resources, state/command interfaces, canonical units, limits, static combination constraints, expected rates, stale/watchdog timeouts, omission and armed-idle policies, device-specific safe stop, `DIRECT` versus `OPERATOR_CONFIRMED` activation, and a required process-loss classification: native watchdog, external supervisor, intrinsically safe, simulation, or unprotected.
- [ ] 3.6 Implement reusable validators for length mismatch, duplicate/unknown/missing keys, finite values, supported unit/interface combinations, hard limits, and immutable descriptions.
- [ ] 3.7 Define exact canonical key syntax and helpers without robot IDs or semantic remapping; include joints, gains, grippers, bases, and control-relevant IMU scalars.
- [ ] 3.8 Implement sequence/epoch comparison and document producer timestamp versus receiver-monotonic freshness semantics.
- [ ] 3.9 Build a deterministic fake connection that publishes descriptions, status, complete state, and live module readiness; supports direct and operator-confirmed preparation, commit, abort, and idempotent lifecycle acknowledgement; records commands; and injects invalid, stale, not-ready, lost-stop, delayed-command, and fault cases.
- [ ] 3.10 Add contract codec round-trip and boundary tests, including maximum expected G1 payload size.
- [ ] 3.11 Add validator tests for every malformed-frame and rejected-command scenario, including missing, unverified, simulation-on-physical, and unprotected process-loss classifications.
- [ ] 3.12 Add transport-assignment validation or blueprint tests proving multiwriter control planes use LCM/Zenoh and the command direction may use a safe single-writer transport.
- [ ] 3.13 Remove superseded experimental or ambiguous coordinator command message types rather than retaining conversion adapters.
- [ ] 3.14 Run focused message, transport, module-port, and fake-connection tests plus mypy/ruff for the new public contracts.

## 4. PR4a — Manipulator, Gripper, and Simulation Connections

- [ ] 4.1 Move xArm SDK connection, activation, native ordering, unit conversion, hard limits, and safe stop behind the xArm connection module.
- [ ] 4.2 Expose supported xArm position and velocity command interfaces and implement autonomous safe transitions when winning command semantics change.
- [ ] 4.3 Publish xArm immutable description, complete state snapshots, status, and lifecycle acknowledgements over the shared contracts.
- [ ] 4.4 Validate xArm command batches atomically and implement its powered safe-stop policy, independent heartbeat watchdog, control-epoch gate, omission behavior, and declared lower-level process-loss protection.
- [ ] 4.5 Integrate arm-attached gripper joints into the same connection description, state, limits, units, command validation, and watchdog path.
- [ ] 4.6 Migrate standalone gripper connection ownership where it remains after PR #3381, without creating a coordinator-facing gripper adapter registry.
- [ ] 4.7 Update manipulator mock/simulation connections to match hardware descriptions, command-interface transitions, names, lifecycle, and safety behavior.
- [ ] 4.8 Migrate Piper and OpenArm connection packages to the contract or explicitly split them into a follow-up dependency before PR5 if their registered blueprints cannot yet cut over.
- [ ] 4.9 Remove coordinator-facing adapter construction and migrated hardware-specific global configuration from these packages.
- [ ] 4.10 Run fake/simulation contract tests and a single-xArm hardware smoke test including arm, gripper, limits, powered safe stop, device-specific shutdown, watchdog/process-loss behavior, stale command rejection, and emergency stop.

## 5. PR4b — Mobile-Base Connections

- [ ] 5.1 Inventory mobile-base command/state fields currently routed through coordinator hardware and declare the supported command-interface combinations for each connection family.
- [ ] 5.2 Move native lifecycle, conversion, limits, watchdog, braking/safe-zero behavior, control-epoch gate, and lower-level process-loss protection into each migrated mobile-base connection.
- [ ] 5.3 Publish descriptions, complete state, status, and lifecycle acknowledgements on shared control streams.
- [ ] 5.4 Implement atomic sparse-command validation with zero velocity/effort omission behavior appropriate to each base.
- [ ] 5.5 Keep odometry, rich IMU, lidar, image, and other semantic sensor streams independent from scalar coordinator state.
- [ ] 5.6 Remove migrated coordinator-facing adapter plumbing and hardware-specific global configuration.
- [ ] 5.7 Run fake/simulation tests for command routing, heartbeat loss, safe stop/braking, stale commands and state, device-specific shutdown, emergency stop, and blueprint construction.
- [ ] 5.8 Complete the available platform-owner mobile-base smoke checklist and record unavailable hardware without blocking unrelated families.

## 6. PR4c — G1 Whole-Body Connection

- [ ] 6.1 Define the supported G1 impedance command-interface combination over canonical position, velocity, `kp`, `kd`, and effort keys.
- [ ] 6.2 Move G1 native DDS/SDK lifecycle, driver ordering, unit conversion, hard limits, control-epoch gate, watchdog, supported safe-stop behavior, and lower-level process-loss protection into the connection.
- [ ] 6.3 Publish the immutable G1 description, complete whole-body state, status, and correlated lifecycle acknowledgements.
- [ ] 6.4 Implement atomic G1 command validation and driver dispatch for the complete compatible field set.
- [ ] 6.5 Route full IMU and other rich sensor messages to their existing consumers while declaring only control-required scalars to the coordinator.
- [ ] 6.6 Move the current measured-pose-to-policy-default activation ramp and stable prepared-state hold out of `G1GrootWBCTask` into the G1 connection's bounded `PREPARE_ARM` behavior; allow GR00T shadow evaluation while preventing its output from reaching hardware before commit.
- [ ] 6.7 Migrate `dimos hardware g1 arm/enable/activate/disable/status` to the robot-level preparation, confirmation, task-cancellation, safe-stop, and status APIs; keep the optional bimanual ready pose as an ordinary post-arm manipulation trajectory.
- [ ] 6.8 Remove migrated G1-specific coordinator hardware plumbing, task-level physical activation ownership, and global configuration.
- [ ] 6.9 Benchmark expected state/command payload and rate over LCM/Zenoh with margin; record evidence for or against a later multiwriter-SHM project.
- [ ] 6.10 Run G1 simulation tests for preparation motion, prepared hold, shadow policy output blocking, abort, confirmation, impedance commands, omission, balance/damping safe stop, watchdog and process loss, stale commands and state, device-specific shutdown, lifecycle rollback, and emergency stop.
- [ ] 6.11 Complete the real-G1 platform-owner checklist when hardware is available; do not weaken simulation gates if it is unavailable.

## 7. PR5 — Atomic Coordinator and Blueprint Cutover

- [ ] 7.1 Replace coordinator hardware configuration with an allow-list of required connection source names and fixed class-level control ports.
- [ ] 7.2 Assemble and validate runtime descriptions, including missing sources, duplicate ownership, inconsistent metadata, changed epochs, and initial complete-state requirements.
- [ ] 7.3 Replace adapter polling with atomic per-source state snapshots and receiver-monotonic staleness tracking; project validated position, velocity, and effort interfaces into one named `coordinator_joint_state` output.
- [ ] 7.4 Replace `ResourceClaim` with a precise whole-joint claim type while keeping priority and preemption at joint granularity; remove compatibility aliases.
- [ ] 7.5 Implement per-joint priority arbitration so one winning task supplies every command-interface value for each joint.
- [ ] 7.6 Route each winning task's command interfaces and values without adding coordinator-owned native mode or transition state.
- [ ] 7.7 Publish current-control-epoch command heartbeat frames while healthy and armed, including valid empty frames when no task owns a joint; withdraw ordinary heartbeats after closing the safety gate.
- [ ] 7.8 Derive robot readiness from each required connection's universal live module readiness plus valid description and fresh state; fail production arming closed on unsafe process-loss classifications; implement standby, two-phase prepare/commit arming, direct and operator-confirmed public APIs, abort, robot-wide repeated safe stop, transactional all-connection safe-stop clearing with rollback, emergency stop, clear-estop, and explicit fault recovery.
- [ ] 7.9 Expose coordinator introspection for joints, interfaces, current winning command interfaces, state, tasks, lifecycle status, module readiness, descriptions, and faults through RPC and DimOS `Spec` Protocols.
- [ ] 7.10 Remove coordinator hardware objects, adapter registries, construction/activation/polling/dispatch, hardware add/remove/list RPCs, and adapter-specific gripper RPCs.
- [ ] 7.11 Remove dynamic task add/remove APIs if the inventory confirms no shipped caller; otherwise record the concrete caller and retain only the necessary existing behavior.
- [ ] 7.12 Migrate every registered control blueprint to connection modules, shared stream transports, required-source configuration, and per-instance CLI/environment config.
- [ ] 7.13 Remove superseded hardware-specific `GlobalConfig` fields and update configuration tests without fallback or migration aliases.
- [ ] 7.14 Add coordinator tests for joint arbitration, task cancellation versus latched safe stop, malformed state, duplicate ownership, initial and live readiness loss, command-gate ordering, direct arm, operator-confirmed prepare/commit, abort, stale confirmation, preparation/commit rollback, repeated/idempotent safe stop, lost stop, heartbeat withdrawal, stale epochs, all-connection clear success and rollback, delayed acknowledgement, stale state, aggregated-state suppression, estop latch, and non-resuming recovery; keep activation motion, native-transition, and physical safe-policy tests in connection packages.
- [ ] 7.15 Build every affected production, simulation, replay, teleoperation, and agentic blueprint and verify exact stream type/name wiring.
- [ ] 7.16 Regenerate the built-in blueprint registry with `pytest dimos/robot/test_all_blueprints_generation.py` and commit the generated change.

## 8. PR6 — Manipulation, Trajectory, and Gripper Integration

- [ ] 8.1 Connect manipulation execution to coordinator canonical interfaces without hardware IDs, semantic name conversion, or per-robot trajectory splitting.
- [ ] 8.2 Update trajectory tasks to claim planning-group joints and emit position, velocity, or effort command interfaces only for joints they win.
- [ ] 8.3 Refactor `GripperControlTask` to obtain canonical gripper joints, limits, and state from the normal control contracts while preserving PR #3381 vector, normalized, sweep, hold, and reference-pose semantics.
- [ ] 8.4 Remove manipulation gripper-range caching, special `/gripper` parsing, hardware-derived task names, adapter access, and pick/place leakage into task invocation internals.
- [ ] 8.5 Make integrated and standalone grippers differ only by description ownership and connection configuration.
- [ ] 8.6 Update teleoperation, pick/place, keyboard, Quest, skill/MCP, and demo callers to use public semantic manipulation/gripper APIs and canonical names.
- [ ] 8.7 Add end-to-end tests from prepared model and planning group through task claim, arbitration, connection command, state feedback, ordinary cancellation/armed-idle behavior, latched safe stop, and visualization.
- [ ] 8.8 Add multi-DOF gripper tests for limits, normalized reads/writes, sweep reference requirements, preemption, omission, and watchdog behavior.
- [ ] 8.9 Delete all remaining manipulation-facing robot-ID, hardware-ID, range-mapping, task-name, and old command-path remnants discovered by repository search.
- [ ] 8.10 Run focused manipulation/control integration tests, skill schema tests, mypy, and ruff.

## 9. PR7 — Cross-Platform Verification and Documentation

- [ ] 9.1 Run the full fast test suite and all affected slow/self-hosted test subsets; triage failures by owning stack PR rather than masking them in PR7.
- [ ] 9.2 Run representative xArm, dual-arm mock, G1 simulation, mobile-base simulation, replay, teleoperation, and agentic blueprints through their documented CLI surface.
- [ ] 9.3 Verify through `dimos shell` or equivalent RPC surface that interface descriptions, source health, task claims, lifecycle status, faults, and state are inspectable.
- [ ] 9.4 Verify replay cannot arm or command a physical connection and that unsupported replay/hardware composition fails closed.
- [ ] 9.5 Complete and attach the xArm/gripper hardware checklist and collect platform-owner checklists for available Piper/OpenArm, mobile-base, and G1 hardware; each checklist must exercise and record the declared process/link-loss behavior.
- [ ] 9.6 Update user module, blueprint, configuration, manipulation, control capability, platform, replay, and skill/MCP documentation listed in `docs.md`.
- [ ] 9.7 Update contributor guidance for whole-joint arbitration, connection interface transitions, scalar contracts, naming, units, transports, layered safety tests, and intentional deletion policy.
- [ ] 9.8 Update coding-agent docs and scoped `AGENTS.md` guidance to prohibit the removed architecture and point to the new extension path.
- [ ] 9.9 Run `uv run doclinks` and repair every changed documentation link.
- [ ] 9.10 Run `uv run md-babel-py run <changed-executable-doc>` for each executable document and exercise both one- and two-connection examples.
- [ ] 9.11 Run `bin/gen-diagrams` for generated architecture diagrams and verify no generated file was edited manually.
- [ ] 9.12 Run the repository-supported `uv run sphinx-build -W docs docs/_build/html` documentation build or its current documented equivalent.
- [ ] 9.13 Run `pytest dimos/robot/test_all_blueprints_generation.py` once more and verify the generated registry is clean.
- [ ] 9.14 Run repository-wide mypy, ruff/pre-commit, and the final supported test matrix.
- [ ] 9.15 Run `openspec validate refactor-control-coordinator-connections` and resolve every proposal, design, spec, and task consistency error.
- [ ] 9.16 Search for deleted terminology and APIs, review each remaining occurrence, and remove obsolete `multi-robot`, `robot_id`, coordinator hardware, adapter registry, and compatibility references within scope.
- [ ] 9.17 Present the final design decisions, benchmark results, hardware evidence, deferred follow-up register, and any deviations from this stack for approval before archive.
