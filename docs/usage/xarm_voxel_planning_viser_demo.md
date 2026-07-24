---
title: "xArm Voxel Planning with Viser"
---

# xArm Voxel Planning with Viser

This simulation demonstrates the complete wrist-camera perception-to-planning
path for an xArm. It uses MuJoCo, RoboPlan, the manipulation planner, and Viser
to show the collision state used for planning.

## Install

Install the manipulation dependencies:

```bash
uv sync --extra manipulation --inexact
```

The blueprint is included in the runnable blueprint list. Confirm its name
before starting the simulation:

```bash
dimos list
```

Start the dedicated simulation with:

```bash
dimos run xarm-voxel-planning-viser-demo
```

The command starts simulation only; it does not change CLI syntax or add a
hardware execution path. Open the Viser address printed by the process. The
scene shows the simulated arm, the planning scene, and the obstacles actually
accepted by the planning backend. The planning collision snapshot has one
unified blue visualization at `/planning/collision_snapshot`, showing the
latest valid staged perception before the next plan commits it. Viser
suppresses the overlapping accepted OCTREE projection for this reserved
obstacle only; backend collision state and other accepted obstacle visuals are
unchanged.

## Perception to planning

The simulated wrist camera produces a point cloud in the planning pipeline:

```text
MujocoSimModule.pointcloud
  -> point-cloud self-filter
  -> RayTracingVoxelMap.global_map
  -> ManipulationModule.planning_voxel_map
```

The self-filter removes points belonging to the robot body before the mapper
produces a **Planning Collision Snapshot**. A snapshot is complete,
pre-filtered occupancy for collision checks, not a semantic or persistent world
model. It must be expressed in the configured **Planning World Frame**. The
initial **Snapshot Resolution** is 0.05 m; it is carried explicitly and is not
inferred from point spacing.

MuJoCo publishes the wrist camera's optical-frame transform directly in
`world`. The mapping pose source therefore does not depend on the manipulation
module first deriving and publishing a `world -> link7` transform.

The snapshot input is latest-wins: incoming snapshots replace the staged
snapshot, with no freshness timeout or automatic clear when no new snapshot
arrives. Viser refreshes this staged view independently of collision-world
registration, caps it at 20,000 displayed points, and throttles replacement to
twice per second to keep the UI responsive. At the beginning of a planning
operation, the latest staged snapshot is synchronized through the existing
obstacle lifecycle. The first non-empty snapshot adds one planning-collision
obstacle. Later non-empty snapshots update its complete OCTREE geometry under
the same native ID. The authoritative obstacle namespace is:

```text
/manipulation/obstacles/<encoded-native-id>
```

The backend is mutated first. Actual mutations drive PR #3108's visualization
lifecycle. A successful full-geometry update uses the existing
`add_vis_obstacle` operation as an add-or-replace event for the same native ID;
it does not emit a remove event or create a second generation. An empty
snapshot removes the stable planning-collision obstacle.

RoboPlan does not expose a native replace-geometry call. DimOS therefore
validates the new OCTREE and builds a complete replacement scene while the
active scene remains available. It then publishes the new scene and obstacle
registry together. In-flight collision and planning queries finish against the
old scene; later queries use the new scene. If construction or insertion
fails, the old scene remains active and no visualization update is emitted.
Forward kinematics and Jacobian queries use a separate immutable robot-only
scene, so a slow collision query or collision-scene swap cannot stall pose
target IK. The Viser panel also runs latest-wins IK and collision validation on
separate workers: the target ghost moves as soon as IK succeeds, remains in the
checking state, and only becomes plannable after the independent collision
result succeeds. The demo uses Pink for pose-target IK because the generic
Jacobian solver can diverge on ordinary xArm Cartesian displacements near the
configured observation pose; RoboPlan remains the collision world and motion
planner.

Visualization failures are logged synchronization faults and do not redefine
backend mutation success. Pose updates are backend-first and are visualized
only after the backend truthfully reports that collision state was updated.
Backends that cannot truthfully update pose report the operation as unsupported.
A frame mismatch is rejected without changing staged or accepted state. Absence
of a new snapshot does not trigger a freshness policy or automatic clear.

## RoboPlan and rendering limits

The initial octree adapter supports RoboPlan. Every occupied voxel is retained
for collision registration, so collision checking is not subject to a point or
voxel cap. Planning backends without octree support reject the snapshot
explicitly before mutating their world; they do not approximate or discard the
occupancy.

Viser uses a render-only display cap for the unified live planning layer and
for ordinary accepted OCTREE obstacles. These caps change only what is drawn,
not the all-voxel collision geometry or the accepted backend obstacle data.

## Planning and execution

Snapshot synchronization changes the collision inputs used by planning. Plan
generation, trajectory reservation, and trajectory execution continue through
their existing manipulation and coordinator paths unchanged.
