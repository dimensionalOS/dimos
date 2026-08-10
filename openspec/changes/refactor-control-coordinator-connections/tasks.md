# Stacked Implementation Plan

The numbered sections are review and merge units. PR4a–PR4c may proceed in parallel after PR3; PR5 depends on all connection families needed by registered production blueprints. PR #3381 is a prerequisite. Every cutover removes its obsolete path in the same PR; there is no compatibility layer.

```text
#3381 -> PR1 -> PR2 -> PR3 -> [PR4a, PR4b, PR4c] -> PR5 -> PR6 -> PR7
```

## 0. Prerequisite and Inventory

- [ ] 0.1 Wait for gripper API PR #3381 to merge, rebase the stack onto its merge commit, and record the final gripper task API used by PR6.
- [ ] 0.2 Inventory every registered blueprint that constructs a control coordinator, manipulation module, hardware adapter, or migrated connection; assign each to PR4a, PR4b, PR4c, or an explicit removal decision.
- [ ] 0.3 Inventory public coordinator, manipulation, world, planning, visualization, skill/MCP, and configuration surfaces containing `robot_id`, robot names, hardware IDs, adapter registries, or hardware-specific gripper plumbing.
- [ ] 0.4 Record focused test targets and available hardware owner for xArm/gripper, G1, Piper/OpenArm, and mobile-base families before changing code.

## 1. PR1 — Static Single-Robot Model

- [x] 1.1 Add a parser compatibility test for canonical `/` names across every supported manipulation backend and record which backend, if any, needs private reversible encoding.
- [x] 1.2 Define the one-model configuration containing prepared URDF, optional SRDF, logical root, optional whole-model pose, and planning-group declarations.
- [x] 1.3 Define planning groups with group name, canonical joint names, base link, and tip link; remove the singular model-wide end-effector assumption from the new shape.
- [x] 1.4 Prepare and check in or generate through the existing asset workflow one static dual-xArm mock URDF/SRDF fixture with `left/...` and `right/...` canonical names.
- [x] 1.5 Update Drake, RoboPlan, Pink, and visualization model loaders to consume one prepared model configuration and isolate any necessary backend-native name encoding.
- [x] 1.6 Add model-validation errors for missing joints, duplicate canonical names, invalid groups, invalid base/tip links, and malformed prepared model assets.
- [x] 1.7 Convert focused model/backend fixtures and tests to one-model inputs and canonical planning groups.
- [x] 1.8 Remove superseded multi-model configuration properties within the model-loading boundary; do not add aliases.
- [x] 1.9 Run focused model loader, FK, collision, IK, and dual-arm fixture tests for every installed backend.

## 2. PR2 — Single-Robot Manipulation Cutover

- [ ] 2.1 Change manipulation configuration from a robot list/registry to exactly one prepared model configuration.
- [ ] 2.2 Remove `RobotName`, `WorldRobotID`, public robot selectors, and manipulation robot lookup/list APIs.
- [ ] 2.3 Remove local/global joint-name types, robot-prefix parse/make helpers, and mapping inversion from planning and execution.
- [ ] 2.4 Change world state, collision, FK, and model access APIs to operate on the one logical model without a robot-ID parameter.
- [ ] 2.5 Change planning-group selection to select canonical joint subsets and group-specific frames without robot membership.
- [ ] 2.6 Simplify Pink and other multi-target solvers to solve several groups within one model rather than grouping by robot.
- [ ] 2.7 Replace per-robot initialization state, trajectory splitting, and execution targets with one canonical joint-state and trajectory flow.
- [ ] 2.8 Replace visualization and world-monitor maps keyed by robot ID with one logical model state.
- [ ] 2.9 Remove robot selectors from semantic manipulation RPCs, DimOS `Spec` Protocols, skills, MCP schemas, and callers.
- [ ] 2.10 Convert the dual-xArm mock and authored bimanual blueprints to the static prepared model and planning groups.
- [ ] 2.11 Delete obsolete multi-robot code paths and compatibility properties exposed by the migrated manipulation surfaces.
- [ ] 2.12 Run focused manipulation, planning, collision, FK, execution, visualization, skill-schema, and blueprint-build tests.

## 3. PR3 — Scalar Control and Lifecycle Contracts

- [ ] 3.1 Define an encodable `ControlValues` message with source, source timestamp, sequence/epoch, interface names, and parallel `float64` values.
- [ ] 3.2 Define encodable `ConnectionDescription`, `ConnectionStatus`, and `ConnectionControl` contracts with description epochs and correlated lifecycle operation IDs.
- [ ] 3.3 Define typed description structures for state/command interfaces, canonical units, limits, command profiles, expected rates, stale/watchdog timeouts, omission policies, and safe stop.
- [ ] 3.4 Implement reusable validators for length mismatch, duplicate/unknown/missing keys, finite values, supported unit/profile combinations, hard limits, and immutable descriptions.
- [ ] 3.5 Define exact canonical key syntax and helpers without robot IDs or semantic remapping; include joints, gains, grippers, bases, and control-relevant IMU scalars.
- [ ] 3.6 Implement sequence/epoch comparison and document producer timestamp versus receiver-monotonic freshness semantics.
- [ ] 3.7 Build a deterministic fake connection that publishes descriptions, status, and complete state; acknowledges lifecycle; records commands; and injects invalid, stale, and fault cases.
- [ ] 3.8 Add contract codec round-trip and boundary tests, including maximum expected G1 payload size.
- [ ] 3.9 Add validator tests for every malformed-frame and rejected-command scenario in the capability specs.
- [ ] 3.10 Add transport-assignment validation or blueprint tests proving multiwriter control planes use LCM/Zenoh and the command direction may use a safe single-writer transport.
- [ ] 3.11 Remove superseded experimental or ambiguous coordinator command message types rather than retaining conversion adapters.
- [ ] 3.12 Run focused message, transport, module-port, and fake-connection tests plus mypy/ruff for the new public contracts.

## 4. PR4a — Manipulator, Gripper, and Simulation Connections

- [ ] 4.1 Move xArm SDK connection, activation, native ordering, unit conversion, hard limits, and safe stop behind the xArm connection module.
- [ ] 4.2 Add xArm position and velocity command-profile configuration and expose only the fields compatible with the selected static profile.
- [ ] 4.3 Publish xArm immutable description, complete state snapshots, status, and lifecycle acknowledgements over the shared contracts.
- [ ] 4.4 Validate xArm command batches atomically and implement its independent heartbeat watchdog and omission behavior.
- [ ] 4.5 Integrate arm-attached gripper joints into the same connection description, state, limits, units, command validation, and watchdog path.
- [ ] 4.6 Migrate standalone gripper connection ownership where it remains after PR #3381, without creating a coordinator-facing gripper adapter registry.
- [ ] 4.7 Update manipulator mock/simulation connections to match hardware descriptions, profiles, names, lifecycle, and safety behavior.
- [ ] 4.8 Migrate Piper and OpenArm connection packages to the contract or explicitly split them into a follow-up dependency before PR5 if their registered blueprints cannot yet cut over.
- [ ] 4.9 Remove coordinator-facing adapter construction and migrated hardware-specific global configuration from these packages.
- [ ] 4.10 Run fake/simulation contract tests and a single-xArm hardware smoke test including arm, gripper, limits, disarm, watchdog, and emergency stop.

## 5. PR4b — Mobile-Base Connections

- [ ] 5.1 Inventory mobile-base command/state fields currently routed through coordinator hardware and select one static compatible profile per connection family.
- [ ] 5.2 Move native lifecycle, conversion, limits, watchdog, and safe-zero behavior into each migrated mobile-base connection.
- [ ] 5.3 Publish descriptions, complete state, status, and lifecycle acknowledgements on shared control streams.
- [ ] 5.4 Implement atomic sparse-command validation with zero velocity/effort omission behavior appropriate to each base.
- [ ] 5.5 Keep odometry, rich IMU, lidar, image, and other semantic sensor streams independent from scalar coordinator state.
- [ ] 5.6 Remove migrated coordinator-facing adapter plumbing and hardware-specific global configuration.
- [ ] 5.7 Run fake/simulation tests for command routing, heartbeat loss, stale state, disarm, emergency stop, and blueprint construction.
- [ ] 5.8 Complete the available platform-owner mobile-base smoke checklist and record unavailable hardware without blocking unrelated families.

## 6. PR4c — G1 Whole-Body Connection

- [ ] 6.1 Define the G1 static impedance command profile over canonical position, velocity, `kp`, `kd`, and effort interface keys.
- [ ] 6.2 Move G1 native DDS/SDK lifecycle, driver ordering, unit conversion, hard limits, watchdog, and safe stop into the connection.
- [ ] 6.3 Publish the immutable G1 description, complete whole-body state, status, and correlated lifecycle acknowledgements.
- [ ] 6.4 Implement atomic G1 command validation and driver dispatch for the complete compatible field set.
- [ ] 6.5 Route full IMU and other rich sensor messages to their existing consumers while declaring only control-required scalars to the coordinator.
- [ ] 6.6 Remove migrated G1-specific coordinator hardware plumbing and global configuration.
- [ ] 6.7 Benchmark expected state/command payload and rate over LCM/Zenoh with margin; record evidence for or against a later multiwriter-SHM project.
- [ ] 6.8 Run G1 simulation tests for impedance commands, omission, watchdog, stale state, lifecycle rollback, and emergency stop.
- [ ] 6.9 Complete the real-G1 platform-owner checklist when hardware is available; do not weaken simulation gates if it is unavailable.

## 7. PR5 — Atomic Coordinator and Blueprint Cutover

- [ ] 7.1 Replace coordinator hardware configuration with an allow-list of required connection source names and fixed class-level control ports.
- [ ] 7.2 Assemble and validate runtime descriptions, including missing sources, duplicate ownership, inconsistent metadata, changed epochs, and initial complete-state requirements.
- [ ] 7.3 Replace adapter polling with atomic per-source state snapshots and receiver-monotonic staleness tracking.
- [ ] 7.4 Rename `ResourceClaim` and joint-specialized command/arbitration concepts to exact interface claims and commands without compatibility aliases.
- [ ] 7.5 Implement exact-key priority arbitration and sparse winner output independent of joint, hardware, mode, gripper, or atomic-device semantics.
- [ ] 7.6 Publish command heartbeat frames at the configured rate, including valid empty frames when no task owns an interface.
- [ ] 7.7 Implement robot-level readiness, correlated transactional arm/rollback, disarm, emergency stop, clear-estop, and explicit fault recovery.
- [ ] 7.8 Expose coordinator introspection for interfaces, state, tasks, lifecycle status, source health, descriptions, and current faults through RPC and DimOS `Spec` Protocols.
- [ ] 7.9 Remove coordinator hardware objects, adapter registries, construction/activation/polling/dispatch, hardware add/remove/list RPCs, and adapter-specific gripper RPCs.
- [ ] 7.10 Remove dynamic task add/remove APIs if the inventory confirms no shipped caller; otherwise record the concrete caller and retain only the necessary existing behavior.
- [ ] 7.11 Migrate every registered control blueprint to connection modules, shared stream transports, required-source configuration, and per-instance CLI/environment config.
- [ ] 7.12 Remove superseded hardware-specific `GlobalConfig` fields and update configuration tests without fallback or migration aliases.
- [ ] 7.13 Add coordinator tests for arbitration, malformed state, duplicate ownership, readiness timeout, arm rollback, delayed acknowledgement, heartbeat, stale fault, estop latch, and non-resuming recovery.
- [ ] 7.14 Build every affected production, simulation, replay, teleoperation, and agentic blueprint and verify exact stream type/name wiring.
- [ ] 7.15 Regenerate the built-in blueprint registry with `pytest dimos/robot/test_all_blueprints_generation.py` and commit the generated change.

## 8. PR6 — Manipulation, Trajectory, and Gripper Integration

- [ ] 8.1 Connect manipulation execution to coordinator canonical interfaces without hardware IDs, semantic name conversion, or per-robot trajectory splitting.
- [ ] 8.2 Update trajectory tasks to claim exact position/velocity/effort interfaces selected from planning groups and published descriptions.
- [ ] 8.3 Refactor `GripperControlTask` to obtain canonical gripper joints, limits, and state from the normal control contracts while preserving PR #3381 vector, normalized, sweep, hold, and reference-pose semantics.
- [ ] 8.4 Remove manipulation gripper-range caching, special `/gripper` parsing, hardware-derived task names, adapter access, and pick/place leakage into task invocation internals.
- [ ] 8.5 Make integrated and standalone grippers differ only by description ownership and connection configuration.
- [ ] 8.6 Update teleoperation, pick/place, keyboard, Quest, skill/MCP, and demo callers to use public semantic manipulation/gripper APIs and canonical names.
- [ ] 8.7 Add end-to-end tests from prepared model and planning group through task claim, arbitration, connection command, state feedback, cancellation, and visualization.
- [ ] 8.8 Add multi-DOF gripper tests for limits, normalized reads/writes, sweep reference requirements, preemption, omission, and watchdog behavior.
- [ ] 8.9 Delete all remaining manipulation-facing robot-ID, hardware-ID, range-mapping, task-name, and old command-path remnants discovered by repository search.
- [ ] 8.10 Run focused manipulation/control integration tests, skill schema tests, mypy, and ruff.

## 9. PR7 — Cross-Platform Verification and Documentation

- [ ] 9.1 Run the full fast test suite and all affected slow/self-hosted test subsets; triage failures by owning stack PR rather than masking them in PR7.
- [ ] 9.2 Run representative xArm, dual-arm mock, G1 simulation, mobile-base simulation, replay, teleoperation, and agentic blueprints through their documented CLI surface.
- [ ] 9.3 Verify through `dimos shell` or equivalent RPC surface that interface descriptions, source health, task claims, lifecycle status, faults, and state are inspectable.
- [ ] 9.4 Verify replay cannot arm or command a physical connection and that unsupported replay/hardware composition fails closed.
- [ ] 9.5 Complete and attach the xArm/gripper hardware checklist and collect platform-owner checklists for available Piper/OpenArm, mobile-base, and G1 hardware.
- [ ] 9.6 Update user module, blueprint, configuration, manipulation, control capability, platform, replay, and skill/MCP documentation listed in `docs.md`.
- [ ] 9.7 Update contributor guidance for connection profiles, scalar contracts, naming, units, transports, safety tests, and intentional deletion policy.
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
