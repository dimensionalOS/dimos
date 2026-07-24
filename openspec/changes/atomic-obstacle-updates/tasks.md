## 1. Protocol and Lifecycle Contracts

- [x] 1.1 Add complete and pose-only obstacle update methods to `WorldSpec` and matching update commands to `VisualizationSpec`
- [x] 1.2 Update all production adapters, test fakes, and runtime protocol-conformance fixtures for the new method signatures
- [x] 1.3 Enforce finalization before add, remove, complete update, pose update, clear, and obstacle retrieval in both world backends
- [x] 1.4 Update pre-finalization obstacle tests and direct callers to follow the finalized-world lifecycle

## 2. Backend Atomicity and State Ownership

- [x] 2.1 Add a backend-owned `RLock`, usability state, and fail-closed native-operation guard to `RoboPlanWorld`
- [x] 2.2 Extend Drake's existing `RLock` with usability state and fail-closed guards for all mutable native scene operations
- [x] 2.3 Protect collision, distance, context/state, scene-backed FK/Jacobian, and other native scene queries with each backend's shared reentrant lock
- [x] 2.4 Hold the RoboPlan scene lock for the full opaque native planning call while preserving per-call locking for generic planners
- [x] 2.5 Deep-copy accepted obstacle values, replace stored values instead of mutating them, and return defensive copies from both backends

## 3. World Obstacle Updates

- [x] 3.1 Implement atomic complete obstacle replacement in `RoboPlanWorld`, including validation before mutation and invalidation after unexpected native failure
- [x] 3.2 Implement optimized atomic RoboPlan pose updates that preserve all non-pose properties
- [x] 3.3 Implement atomic complete obstacle replacement in `DrakeWorld` by replacing post-finalization dynamic collision geometry and stored snapshots
- [x] 3.4 Replace Drake's visualization-only pose mutation with atomic collision-geometry replacement that preserves non-pose properties
- [x] 3.5 Verify missing identities return `False`, invalid inputs raise before mutation, and operations on invalid worlds raise `RuntimeError`

## 4. Monitoring and Visualization

- [x] 4.1 Add `WorldMonitor` complete and pose-only update paths that mutate the authoritative world first and forward only successful updates
- [x] 4.2 Route pose-only collision messages to pose update, complete messages to full replacement, reject incomplete replacements, and prevent pose-only update-as-add
- [x] 4.3 Implement atomic complete and pose-only Viser obstacle updates using the existing renderer scene lock
- [x] 4.4 Handle Viser renderer failures internally with obstacle-aware logs and a persistent frontend warning, without retry or replay
- [x] 4.5 Add matching Drake embedded-visualization methods that rely on authoritative world mutation without duplicating geometry changes
- [x] 4.6 Leave batch updates, stored-plan invalidation, execution-time obstacle behavior, and intentional callback redesign unchanged
- [x] 4.7 Expose complete and pose-only obstacle update RPCs through `ManipulationModule` and its interactive client

## 5. Deterministic Verification

- [x] 5.1 Add lifecycle, result-contract, complete-replacement, and pose-preservation tests for both world backends
- [x] 5.2 Add defensive-copy tests proving accepted and retrieved obstacle mutation cannot alter stored or native world state
- [x] 5.3 Add event/barrier-based concurrency tests proving collision queries block during replacement and replacement blocks during native queries
- [x] 5.4 Add RoboPlan-native planning exclusion tests and generic-planner interleaving coverage
- [x] 5.5 Add native failure-injection tests proving the original exception propagates and later scene operations fail closed
- [x] 5.6 Add Drake regression tests proving pose and complete updates change collision geometry, not only Meshcat state
- [x] 5.7 Add monitor-routing and visualization tests for successful forwarding, unknown obstacles, incomplete messages, internal renderer warnings, and no automatic recovery
- [x] 5.8 Add module RPC and interactive-client forwarding tests for complete and pose-only updates

## 6. Validation and Review

- [ ] 6.1 Run focused manipulation world, monitor, and visualization test suites for environments with and without optional Drake/RoboPlan dependencies
- [ ] 6.2 Run formatting, lint, type checking, and the repository fast test suite; resolve all regressions
- [x] 6.3 Document the breaking finalized-world lifecycle and atomic-operation boundary in relevant manipulation planning documentation
- [x] 6.4 Call out any incidental `WorldObstacleMonitor` callback behavior change for repository-owner discussion in the PR
