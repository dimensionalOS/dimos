## Why

Obstacle mutation and collision queries currently access the same native planning scenes without a shared synchronization boundary, allowing a query to observe a partially replaced obstacle. The existing pose update also leaves Drake collision geometry at its original pose, and visualization has no explicit update contract.

## What Changes

- Add complete obstacle replacement to `WorldSpec`, keyed solely by immutable `Obstacle.name`.
- Retain a distinct pose-only update for high-frequency obstacle motion while preserving every non-pose property.
- Add matching complete and pose-only obstacle update operations to `VisualizationSpec`.
- Make each obstacle mutation or native scene query atomic with respect to the others by using one backend-owned reentrant scene lock.
- Make RoboPlan-native planning one opaque atomic scene operation, excluding obstacle mutation for the duration of the native call.
- Store and return defensive obstacle snapshots so callers cannot bypass atomic mutation.
- Treat unexpected native update failure as an invariant violation that invalidates the planning world.
- Keep visualization best-effort: renderer failures are handled internally with log and frontend warnings and do not invalidate the planning world.
- Route pose-only collision messages to pose update and require complete obstacle information for structural or appearance replacement.
- **BREAKING** Require world finalization before every planning-world obstacle operation, including add, remove, update, clear, and retrieval.

## Capabilities

### New Capabilities

- `atomic-obstacle-updates`: Complete and pose-only obstacle updates, lifecycle rules, atomic scene-operation semantics, backend failure behavior, and best-effort visualization synchronization.

### Modified Capabilities

None.

## Impact

- Changes the `WorldSpec` and `VisualizationSpec` protocols and all conforming production and test implementations.
- Changes `WorldMonitor` and `WorldObstacleMonitor` mutation routing and visualization forwarding.
- Adds synchronization and invalid-world state to the Drake and RoboPlan world backends.
- Changes Drake obstacle pose updates from visualization-only movement to actual collision-geometry replacement.
- Requires deterministic concurrency, lifecycle, failure, snapshot-ownership, monitor, and visualization tests.
- Does not add batch/perception-frame transactions, visualization recovery, stored-plan invalidation, or execution-time obstacle avoidance.
