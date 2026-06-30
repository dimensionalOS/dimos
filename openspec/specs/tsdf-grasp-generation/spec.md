## ADDED Requirements

### Requirement: TSDF-native grasp generation
The system SHALL provide a TSDF-native grasp generation capability that accepts a `TSDFGrid` and returns scored grasp candidates.

#### Scenario: Generate grasps from TSDF
- **WHEN** a valid TSDF grid is provided to the TSDF grasp generator
- **THEN** the generator returns a `GraspCandidateArray` or a clear no-result outcome

#### Scenario: No pointcloud required
- **WHEN** VGN-backed grasp generation runs from a TSDF grid
- **THEN** it does not require a pointcloud input for inference

### Requirement: VGN grasp backend
The system SHALL include a VGN-backed TSDF grasp generation module that consumes latest TSDF stream data and can also generate grasps from an explicit TSDF grid through a typed spec.

#### Scenario: Latest TSDF generation
- **WHEN** the VGN module has received a latest TSDF grid and a caller requests grasp generation from latest TSDF
- **THEN** the module runs VGN inference on that TSDF grid and publishes the resulting grasp candidates

#### Scenario: Missing model dependency
- **WHEN** VGN or its model weights are unavailable
- **THEN** the module fails with a clear status or error explaining the missing optional dependency or model path instead of failing at process import time

### Requirement: Grasp candidate output
The system SHALL represent learned grasp output as grasp candidates containing pose, jaw width, score, and optional id.

#### Scenario: Candidate metadata
- **WHEN** VGN produces a grasp proposal
- **THEN** the output candidate includes the proposed gripper pose, jaw width in meters, quality score, and a stable id when available

#### Scenario: PoseArray compatibility
- **WHEN** an existing consumer requires a `PoseArray`
- **THEN** the grasp candidate array can be converted to a `PoseArray` without losing pose ordering

### Requirement: World-frame VGN output
The system SHALL publish VGN grasp candidates in the `world` frame for the first integration.

#### Scenario: Transform successful
- **WHEN** VGN predicts a grasp in TSDF grid coordinates and the transform to `world` is available
- **THEN** the published candidate pose is transformed to `world` and the output header frame id is `world`

#### Scenario: Transform unavailable
- **WHEN** the transform from TSDF grid frame to `world` cannot be resolved
- **THEN** the generator does not publish misleading candidates and reports a clear transform failure

### Requirement: Stream-first data flow
The system SHALL keep TSDF, depth, and other large scene payloads on streams and use RPC only for control or request/response operations that do not carry high-rate sensor payloads.

#### Scenario: Avoid large RPC payloads
- **WHEN** reconstructing a scene or generating grasps from live camera data
- **THEN** depth images and TSDF grids flow through streams rather than being bundled into a single pickled RPC request
