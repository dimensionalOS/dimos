## Why

DimOS lacks a reproducible diagnostic corpus for measuring whether an agent can understand an imperfect, static indoor map without requiring simulator rollout or conflating the result with map generation. A small geometry-derived benchmark is needed now to establish reliable short-term spatial evaluations before investing in longer-horizon embodied tasks.

## What Changes

- Add an automatically generated static spatial QA corpus derived from Structured3D authoritative geometry and controlled noisy lidar mapping.
- Store one immutable clean map and two seeded noisy map variants per scene trajectory as native `VoxelGridMapper` `PointCloud2` outputs, while keeping authoritative geometry, topology, and answers private.
- Generate seven deterministic question predicates covering square-footprint pose occupancy, straight translation, in-place rotation, room count, same-room membership, direct room connection, and direct-neighbor count.
- Use physically bounded navigable rooms, neutral query markers, continuous swept-footprint collision checks, and scene-level splits to avoid semantic and planning-task confounds.
- Preserve one physical oracle answer per question across clean and noisy map variants, with variant-specific public coordinates where mapping drift requires them.
- Add automated generation checks for coordinate conventions, geometric margins, room-graph consistency, paired variants, integrity, and leakage.
- Add a minimal read-only Viser inspection tool for viewing maps, poses, footprints, markers, motions, questions, and optional oracle overlays; manual exclusions or corrections are recorded separately from deterministic generator output.
- Scope the pilot to 30 scenes, one fixed trajectory per scene, one clean and two noisy variants, and approximately 1,260 QA instances.
- Explicitly exclude evaluation-run results, scoring, agent-facing map rendering, raw-lidar input to agents, RGB input, voxel/3D questions, simulator rollout, route planning, and task-success evaluation from this change.

## Capabilities

### New Capabilities

- `static-spatial-evaluation-corpus`: Generate, validate, store, and visually inspect a versioned map-grounded spatial QA corpus with paired clean/noisy maps and private geometry-derived answers.

### Modified Capabilities

None.

## Impact

- Adds a new offline benchmark-data generation and validation workflow around Structured3D-derived scenes.
- Reuses `VoxelGridMapper` and native `PointCloud2` serialization for canonical stored maps.
- Adds versioned public and private dataset schemas, generated map/question artifacts, and manual review overrides.
- Reuses the existing Viser runtime and scene primitives for read-only QA inspection.
- Introduces a gated external dataset dependency whose access and derivative-data terms must be verified before distribution.
- Does not change existing runtime, mapping, navigation, agent, or benchmark-prelaunch requirements.
