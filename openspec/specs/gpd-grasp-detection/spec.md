## Purpose

Define the GPD-based grasp detection workflow that consumes existing DimOS pointcloud perception outputs, produces compatible grasp pose outputs, and supports an end-to-end MuJoCo visualization demo without robot execution.

## Requirements

### Requirement: GPD detector consumes existing pointcloud inputs through the grasp generation contract
The system SHALL provide an import-safe GPD grasp detector Module that consumes existing DimOS `PointCloud2` inputs, implements the existing `GraspGenSpec` contract, and can be called by `GraspingModule.generate_grasps(...)`.

#### Scenario: GraspingModule calls GPD through GraspGenSpec
- **WHEN** `GraspingModule.generate_grasps(...)` resolves a registered grasp target pointcloud from existing perception modules and a GPD detector is wired as `_grasp_gen`
- **THEN** the module calls `generate_grasps(pointcloud, scene_pointcloud)` on the GPD detector and publishes the returned `PoseArray` through the existing `grasps` output

#### Scenario: GPD package import is coordinator-safe
- **WHEN** the coordinator imports the GPD grasp generator Module class for blueprint wiring
- **THEN** the import succeeds without importing `gpd.core` or other worker-only native dependencies at module import time

### Requirement: GPD detector preserves compatible and debug outputs
The system SHALL return `PoseArray` outputs for compatibility while exposing richer GPD candidate/debug data when available.

#### Scenario: GPD returns grasp poses
- **WHEN** the GPD backend returns one or more grasp candidates for a pointcloud
- **THEN** the detector returns a `PoseArray` in the configured output frame and publishes grasp pose output for visualization

#### Scenario: GPD returns no grasps
- **WHEN** the GPD backend returns no grasp candidates for a valid pointcloud
- **THEN** the detector returns an empty `PoseArray` or `None` according to the existing `GraspGenSpec` behavior and reports an explicit empty-result message rather than failing silently

#### Scenario: Candidate metadata is available for debugging
- **WHEN** the GPD backend exposes candidate scores, widths, or approach metadata
- **THEN** the detector publishes or records that metadata through debug/candidate outputs without requiring `GraspingModule` to consume a new contract

### Requirement: GPD detector validates pointcloud conversion boundaries
The system SHALL validate the conversion between DimOS `PointCloud2` data produced by existing perception modules and the GPD backend input format before calling the native backend.

#### Scenario: Valid pointcloud converts for GPD
- **WHEN** a `PointCloud2` contains valid XYZ data and frame metadata
- **THEN** the generator converts the pointcloud into the backend input representation and preserves the output frame relationship for returned poses

#### Scenario: Invalid pointcloud is rejected clearly
- **WHEN** a pointcloud is empty, malformed, or lacks usable XYZ data
- **THEN** the generator fails the grasp generation call with a clear error or explicit empty-result response identifying the pointcloud problem

### Requirement: GPD demo workflow reaches candidate generation without robot execution
The system SHALL include a documented end-to-end xArm MuJoCo demo command that starts simulation, uses existing perception modules to produce pointcloud/object data, routes a registered grasp target through `GraspingModule` and the GPD project-runtime worker, and visualizes the resulting grasp detection outputs in Rerun while stopping before robot execution.

#### Scenario: xArm MuJoCo workflow invokes GPD
- **WHEN** the GPD MuJoCo demo runs and a configured grasp target becomes a registered object
- **THEN** the demo invokes `GraspingModule.generate_grasps(...)` for that target and routes the call to the placed GPD generator

#### Scenario: User runs documented demo command
- **WHEN** the user follows the documented GPD grasp demo command after preparing the runtime
- **THEN** the command starts the xArm MuJoCo simulation, enables existing pointcloud/object perception modules, invokes GPD grasp detection for the configured grasp target, and exposes the outputs in Rerun

#### Scenario: Demo does not execute robot motion
- **WHEN** the GPD MuJoCo demo produces grasp poses or an empty-result message
- **THEN** the demo does not command pick/place motion, gripper actuation, or robot execution as part of this change

#### Scenario: Demo outputs are observable
- **WHEN** the GPD MuJoCo demo attempts grasp generation
- **THEN** grasp poses, candidate/debug outputs, or an explicit empty-result message are visible through normal DimOS outputs, logs, or Rerun visualization

### Requirement: GPD candidates can feed manual grasp execution demo
The GPD grasp detection workflow SHALL support an opt-in manual execution demo that consumes generated Grasp candidates through the agent-facing grasp-capable manipulation facade while preserving the existing candidate-generation-only demo behavior.

#### Scenario: Existing GPD visualization demo remains non-executing
- **WHEN** the existing candidate-generation-only GPD MuJoCo demo runs
- **THEN** it MUST continue to stop before robot motion, gripper actuation, pick/place motion, or trajectory execution

#### Scenario: Manual execution demo consumes generated candidates
- **WHEN** the manual execution demo runs and GPD returns one or more Grasp candidates for the configured Grasp target
- **THEN** the agent-facing grasp-capable facade MUST be able to cache those candidates for a subsequent `execute_grasp(candidate_index)` call

#### Scenario: Empty GPD result is reported clearly
- **WHEN** GPD returns no usable Grasp candidates during the manual execution demo
- **THEN** the agent-facing sequence MUST report a clear empty-result failure before attempting `execute_grasp(...)`
