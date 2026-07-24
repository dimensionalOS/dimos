## Context

`WorldSpec` currently exposes add, remove, pose update, clear, and retrieval operations, but only Drake protects obstacle dictionaries with a lock and neither backend consistently excludes collision or other native scene queries while an obstacle changes. `WorldMonitor` serializes its own mutation calls, yet planners and kinematics components receive `WorldSpec` directly and therefore bypass the monitor lock.

The current Drake pose update changes stored metadata and Meshcat while leaving collision geometry at its registered pose. RoboPlan can update placement natively, but its scene and native RRT calls are unsynchronized. `VisualizationSpec` supports obstacle add/remove/clear but has no explicit full or pose-only update operations.

The world can receive obstacle observations faster than a complete planning run. The design therefore protects individual native scene operations rather than pinning one obstacle snapshot for the lifetime of a multi-step planner.

## Goals / Non-Goals

**Goals:**

- Provide complete replacement and efficient pose-only obstacle update APIs in both planning and visualization protocols.
- Ensure a native scene query observes either the state before one obstacle update or the state after it, never a remove/add intermediate state.
- Apply one consistent finalized-world lifecycle and failure contract across Drake and RoboPlan.
- Make the planning world own defensive obstacle snapshots.
- Make Drake collision geometry reflect accepted pose and full-obstacle updates.
- Preserve planning-world availability when a best-effort visualization update fails.

**Non-Goals:**

- A stable obstacle snapshot for an entire generic planning or IK run.
- Atomic batches spanning multiple obstacles or a complete perception frame.
- Retry, rollback, or reconstruction after a native backend update failure.
- Automatic visualization resynchronization after a renderer failure.
- Stored-plan invalidation or execution-time dynamic-obstacle safety.
- Redesigning the currently unused `WorldObstacleMonitor` callback extension point.

## Decisions

### Complete replacement and pose-only update are separate operations

`WorldSpec` will expose:

```python
def update_obstacle(self, obstacle: Obstacle) -> bool: ...
def update_obstacle_pose(self, obstacle_name: str, pose: PoseStamped) -> bool: ...
```

`update_obstacle()` replaces the complete obstacle value and never merges omitted information. `Obstacle.name` is the immutable identity and selects the existing obstacle. Renaming remains remove followed by add.

`update_obstacle_pose()` changes only pose and preserves all other properties. It remains first-class because moving obstacles are common and RoboPlan can optimize this operation through native placement update.

`VisualizationSpec` receives matching `update_vis_obstacle(obstacle)` and `update_vis_obstacle_pose(obstacle_name, pose)` commands.

Alternative considered: a generic partial patch. It was rejected because optional values cannot distinguish preservation from clearing, and merging would create ambiguous ownership between perception, monitors, and the world.

### All planning-world obstacle operations require finalization

Add, remove, complete update, pose update, clear, and retrieval will raise `RuntimeError` before `finalize()`. Initialization establishes robot topology first; runtime obstacle state begins only after the native world is query-ready. Production initialization already finalizes before adding its configured floor obstacle.

Alternative considered: backend-specific pre-finalization support. It was rejected because Drake cannot cleanly remove or replace welded pre-finalization obstacle bodies and divergent lifecycle behavior would weaken `WorldSpec`.

### Each backend owns one native scene lock

Drake will extend its existing `RLock`; RoboPlan will add one. Every operation touching mutable native scene state will use that same reentrant lock, including:

- obstacle add, remove, complete update, pose update, clear, and retrieval;
- collision and distance queries;
- native scene context/state operations where concurrent mutation would be unsafe;
- scene-backed FK and Jacobian calls;
- RoboPlan-native planning.

Validation that cannot mutate backend state occurs before replacement begins. A complete replacement removes and recreates native geometry while holding the lock. A pose update uses native placement update where correct; Drake must replace registered collision geometry because its dynamic world-frame geometry pose is fixed after registration.

Generic planners acquire and release the lock through their individual `WorldSpec` calls, so an obstacle update may commit between two collision checks. RoboPlan-native RRT is opaque, so the complete native planning call is one atomic scene operation and blocks obstacle mutation for its duration.

Alternative considered: a read/write lock. It was rejected for version one because Python has no standard implementation, the native backends have broader thread-safety constraints, and fairness/reentrancy would become custom infrastructure.

### Native update failure invalidates the world

Missing obstacle identity returns `False`. Invalid obstacle values raise `ValueError` before mutation. Lifecycle violations raise `RuntimeError`.

If native mutation fails after update begins, the original exception propagates and an internal usability flag marks the world invalid. All later native scene operations fail with `RuntimeError` until the world is reconstructed. Version one does not attempt rollback because such failures are considered invariant violations rather than expected API outcomes.

### The world owns obstacle snapshots

World backends deep-copy accepted obstacle inputs and return copies from obstacle retrieval. Stored obstacles are replaced rather than mutated in place. This prevents callers from changing metadata without acquiring the native scene lock or updating collision geometry.

### The planning world is authoritative and visualization is best-effort

`WorldMonitor` updates the planning world first and forwards only successful updates to visualization. Each visualization implementation owns its renderer failure handling: it catches renderer failures, logs the obstacle identity and exception, and exposes a persistent warning if it has a frontend. It returns normally so visualization availability cannot change the planning-world result.

Viser uses its existing scene lock to make handle replacement or transform update atomic inside the renderer. Drake's embedded visualization remains synchronized through the authoritative world mutation and may implement the visualization update commands as no-ops.

Version one does not retry or perform full visualization replay after failure.

### Incoming messages choose an explicit operation

A collision-object update carrying only pose uses `update_obstacle_pose()`. If it carries any structural or appearance field, it must contain a complete obstacle description and uses `update_obstacle()`. Incomplete replacement messages are rejected rather than merged.

An update for an unknown name may become an add only when it supplies a complete obstacle. A pose-only update cannot create an obstacle.

### Atomicity is per obstacle

Updates to different obstacles from one perception frame are separate atomic scene operations. Collision queries may run between them. A batch transaction is deferred until a consumer demonstrates a need for coherent frame-wide commits.

## Risks / Trade-offs

- **[Serialized scene queries reduce concurrency]** → Start with the auditable `RLock` boundary and measure contention before considering a backend-safe read/write scheme.
- **[Opaque RoboPlan planning delays dynamic updates]** → Document that high-frequency dynamic worlds should use generic planners with exposed collision-query boundaries.
- **[A generic planner can span multiple obstacle revisions]** → Accept this explicitly; the design prevents partial native mutations, not stale plans.
- **[Unexpected native failure makes the world unrecoverable]** → Fail closed and require reconstruction instead of allowing planning against an unknown scene.
- **[Visualization can remain stale after a failure]** → Display a persistent warning and keep the planning world authoritative; recovery is deferred beyond version one.
- **[Finalization requirement breaks direct pre-finalization callers]** → Update tests and callers to finalize before obstacle operations and document the lifecycle contract.
- **[Defensive deep copies add small overhead]** → Obstacles are small relative to native geometry work, and ownership safety is more valuable than aliasing.

## Migration Plan

1. Extend both protocols and update all production/test conformers.
2. Enforce finalized-world guards and update tests that currently add obstacles before finalization.
3. Add backend locks, snapshot ownership, invalid-world state, and full/pose update implementations.
4. Migrate monitor routing and visualization forwarding.
5. Add deterministic concurrency and failure tests before enabling callers to use full replacement.

Rollback consists of reverting the protocol and backend changes together. Because this change is not data-persistent, no stored-data migration is required.

## Open Questions

- During PR review, confirm with repository owners whether the unused `WorldObstacleMonitor` callback registration methods are supported external extension surface. Callback redesign does not gate this change.
