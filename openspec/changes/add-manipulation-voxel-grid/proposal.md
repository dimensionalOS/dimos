## Why

Manipulation planning can register modeled obstacles, but it cannot use live occupancy produced by the perception stack. Plans may therefore ignore objects seen by the wrist camera even when the voxel mapper has an up-to-date view of the workspace.

This change connects complete planning collision snapshots to the unified obstacle lifecycle, exposes the accepted backend obstacle set in Viser, and provides an xArm simulation example that exercises the full perception-to-planning path.

## What Changes

- Accept complete, pre-filtered occupancy snapshots in the planning world frame as manipulation collision input.
- Stage the latest snapshot and commit it at planning time through stable-ID obstacle add/update/remove operations.
- Register octree collision geometry in RoboPlan and raise an explicit unsupported-operation error on planning backends without octree support.
- Project backend-accepted obstacles in Viser through PR #3108's add-or-replace/remove/clear lifecycle.
- Add reusable point-cloud self-filtering and TF pose-source modules needed by the xArm simulation flow.
- Add an xArm MuJoCo/Viser example that maps wrist-camera depth data into manipulation collision geometry.
- Add focused unit, integration, blueprint, and user-documentation coverage.

## Affected DimOS Surfaces

- Modules/streams: manipulation planning, world monitoring, obstacle models, RoboPlan world adaptation, Viser visualization, point-cloud self-filtering, TF pose sourcing, and voxel-map streams.
- Blueprints/CLI: a runnable xArm simulation blueprint discoverable through `dimos list` and `dimos run`; no CLI syntax changes.
- Skills/MCP: none.
- Hardware/simulation/replay: simulation example only; no direct hardware behavior or replay contract changes.
- Docs/generated registries: xArm voxel-planning usage documentation and regenerated built-in blueprint registry.

## Capabilities

### New Capabilities

- `manipulation-collision-snapshots`: Complete occupancy snapshots, planning-world registration, unified obstacle lifecycle behavior, accepted-backend visualization, and xArm simulation observability.

### Modified Capabilities

None.

## Impact

The change is additive and does not alter public CLI syntax, skills, MCP tools, or trajectory execution. RoboPlan gains octree collision registration; other planning backends reject that obstacle type explicitly. Planning cost grows with occupied-voxel count because collision registration does not discard points, while Viser may cap display points independently. Verification covers snapshot lifecycle and failure behavior, native RoboPlan conversion, accepted-backend visualization, self-filter and TF helpers, blueprint wiring, and the generated registry.
