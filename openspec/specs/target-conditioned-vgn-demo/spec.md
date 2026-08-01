## ADDED Requirements

### Requirement: Target-conditioned VGN demo flow
The opt-in VGN MuJoCo demo SHALL exercise object-id-targeted grasp generation rather than only workspace-level grasp generation.

#### Scenario: Demo observation pose sees object
- **WHEN** the target-conditioned demo starts reconstruction
- **THEN** the wrist camera has been moved or initialized to a pose where the configured table object is visible to the depth camera

#### Scenario: Demo resolves a registered object
- **WHEN** the target-conditioned demo flow runs after object perception has registered an object
- **THEN** it resolves a registered object id into Target bounds before requesting VGN grasps

#### Scenario: Demo auto-selects configured target only
- **WHEN** object registration creates runtime registered objects after the demo starts
- **THEN** the demo selects only a registered object matching the configured demo target prompt or name rather than an arbitrary workspace surface

#### Scenario: Demo uses generated runtime object id
- **WHEN** the demo selects a registered object matching the configured target prompt or name
- **THEN** it uses that runtime-generated object id when calling the user-facing object grasp API

#### Scenario: Demo triggers target-conditioned generation
- **WHEN** reconstruction has produced a latest TSDF and the demo has Target bounds
- **THEN** the demo calls the target-conditioned TSDF grasp generation path

### Requirement: Demo remains opt-in
The target-conditioned VGN demo SHALL remain separate from existing xArm simulation defaults.

#### Scenario: Existing simulation unchanged
- **WHEN** existing xArm simulation blueprints are launched
- **THEN** they do not start target-conditioned VGN grasping unless the opt-in demo is selected

### Requirement: Demo target failure reporting
The demo SHALL report clear no-target outcomes when object selection or target-conditioned generation cannot proceed.

#### Scenario: No registered object available
- **WHEN** the demo cannot resolve a registered object for grasping
- **THEN** it reports a clear no-target outcome instead of running workspace-level VGN as if it were object-targeted

#### Scenario: Configured target not visible
- **WHEN** the configured demo target prompt or name does not produce a registered object after the observation window
- **THEN** the demo reports the missing target and does not publish target-conditioned candidates

#### Scenario: Target-conditioned generation returns no candidates
- **WHEN** VGN returns no candidates for the Target-masked TSDF
- **THEN** the demo clears stale grasp visualization and reports the no-candidate outcome

### Requirement: Target-conditioned visualization context
The demo SHALL make it possible to visually confirm that generated grasps are associated with the selected target region.

#### Scenario: Target bounds visible or inspectable
- **WHEN** the demo produces target-conditioned grasp candidates
- **THEN** the selected Target bounds are visible in Rerun under a stable entity path alongside grasp candidates

#### Scenario: Stable target-conditioned entity layout
- **WHEN** the demo logs target-conditioned visualization data
- **THEN** it uses stable entity paths under `world/`, including `world/scene_pointcloud`, `world/tsdf_surface`, `world/grasp_target_bounds`, and `world/grasp_candidates`

#### Scenario: Selected target metadata visible
- **WHEN** the demo selects a registered object for grasping
- **THEN** the selected object id and name are visible in logs or visualization metadata for debugging
