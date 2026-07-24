# Update planning collision snapshots under one stable obstacle ID

Planning collision snapshots extend PR #3108's unified `WorldSpec` obstacle
lifecycle with full-geometry `update_obstacle`. The first non-empty snapshot
adds one obstacle, later snapshots replace its geometry under the same native
ID, and an empty snapshot removes it. Backend mutation is authoritative;
successful updates reuse visualization add as an add-or-replace event.

## Considered Options

- Stable-ID update: chosen because it exposes one coherent snapshot object and
  lets visualization replace one handle.
- Add then remove with generation IDs: rejected because planners can observe
  two generations unless all queries participate in an enclosing transaction.
- Unlocked remove then add: rejected because a query can observe no collision
  snapshot and a failed insertion can lose the previous geometry.

## Consequences

RoboPlan has no native geometry-replacement call. Its adapter validates the
replacement, builds a complete replacement scene without locking the active
scene, and publishes the new scene and obstacle registry in one short critical
section. In-flight collision, kinematics, and native-planning queries finish
against the old scene. Later queries use the new scene. A construction or
insertion failure leaves the old scene published, so replacement requires no
destructive rollback.

Native IDs remain encoded in
`/manipulation/obstacles/<encoded-native-id>`. Collision registration retains
every occupied voxel; rendering may apply an independent display cap. Backends
without OCTREE support reject the replacement before mutation.
