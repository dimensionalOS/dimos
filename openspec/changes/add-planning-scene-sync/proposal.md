## Why

External manipulation collaborators need the same initialized planning-scene metadata that the world already receives. The current Viser integration uses an ad-hoc `register_robot` hook from `WorldMonitor.add_robot()`, which solves bootstrap for Viser but leaves the core monitor aware of backend-specific capabilities and does not generalize to future external visualization, IK, or planner implementations.

## What Changes

- Add a backend-neutral planning-scene snapshot contract that describes initialized robots and can later grow to include static world metadata.
- Add a lifecycle sync method for visualization backends so an external visualizer can receive the full initialized scene at startup instead of incremental `register_robot` calls during robot addition.
- Move Viser robot bootstrap from `WorldMonitor.add_robot()` into startup scene synchronization.
- Preserve Meshcat behavior: embedded Meshcat through `DrakeWorld` may no-op the scene sync because it already observes robots through the Drake world.
- Document the extension path for future external IK/planner implementations without implementing those follow-on sync methods yet.

## Capabilities

### New Capabilities
- `planning-scene-sync`: Defines the startup planning-scene synchronization lifecycle for external manipulation collaborators, initially visualization backends.

### Modified Capabilities
- None.

## Impact

- Affects manipulation planning protocols, `WorldMonitor` lifecycle, manipulation module startup sequencing, Viser visualization bootstrap, and focused tests.
- Removes the dynamic `register_robot` capability check from `WorldMonitor.add_robot()`.
- No new third-party dependencies.
- No intended change to planning, IK, execution safety, or Meshcat behavior.
