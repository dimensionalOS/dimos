## Purpose

Define the backend-independent boundary for proposing ranked robot TCP grasps from one segmented object point cloud.

## Requirements

### Requirement: Propose ranked TCP grasp candidates from one object cloud

The OpenSpec capability SHALL accept one segmented Object Point Cloud in XYZ metres and return a `GraspCandidateArray` preserving the current DimOS `PointCloud2` `frame_id` and `ts` exactly. Each candidate MUST contain a robot TCP pose expressed in that input frame and a finite generator-local score. The result MUST be sorted by descending score.

#### Scenario: Successful proposal preserves provenance and ordering
- **GIVEN** a finite segmented object cloud with a declared frame and timestamp
- **WHEN** proposal inference returns multiple candidates
- **THEN** the result preserves the input `frame_id` and `ts` exactly
- **AND** every pose is expressed in the input cloud frame
- **AND** scores are finite and candidates are ordered from highest to lowest score

### Requirement: Define empty and failure outcomes

The capability MUST return an empty `GraspCandidateArray` when inference completes successfully without a candidate. Invalid cloud data, invalid deployment configuration, unavailable model assets, model-load failures, and inference/runtime failures MUST raise an exception rather than return fabricated or partially valid candidates.

#### Scenario: No candidate is a successful empty result
- **GIVEN** a valid object cloud and valid loaded deployment
- **WHEN** the backend finds no grasp candidate
- **THEN** the result contains the preserved header and an empty candidate list

#### Scenario: Invalid input fails explicitly
- **GIVEN** an object cloud containing non-finite XYZ values or no usable points
- **WHEN** proposal inference is requested
- **THEN** the request raises an input validation exception
- **AND** no candidate array is emitted as a substitute

### Requirement: Keep proposal scope separate from execution

The capability MUST operate on one already-segmented target object and MUST NOT require or perform raw depth segmentation, scene collision checking, IK, approach validation, motion planning, actuation, execution, batching, or runtime gripper selection. Returned candidates MUST be treated as proposals requiring downstream robot-specific validation.

#### Scenario: Scene and execution concerns remain downstream
- **GIVEN** a valid target object cloud and a deployment-fixed gripper configuration
- **WHEN** candidates are returned
- **THEN** the result contains only ranked TCP proposals
- **AND** no scene, collision, IK, planning, or actuation result is implied

### Requirement: Apply deployment TCP calibration

The capability MUST return TCP poses after applying the configured transform from the GraspGenX grasp frame to the robot TCP frame. The adapter MUST use identity when no calibration is supplied, and the active gripper model and checkpoint MUST be deployment-fixed for the running stack.

#### Scenario: Non-identity TCP transform is applied
- **GIVEN** a backend grasp pose and a configured non-identity `grasp_frame_to_tcp` transform
- **WHEN** a candidate is produced
- **THEN** its pose equals `T_input_graspgenx @ T_graspgenx_tcp` in the input frame
- **AND** the candidate does not expose the uncalibrated backend pose

#### Scenario: Runtime tool switching is rejected
- **GIVEN** a running proposal stack with one configured sweep-volume gripper model
- **WHEN** a request attempts to select a different gripper
- **THEN** the request is rejected because gripper selection is deployment configuration, not request data

### Requirement: Bound candidate count

The capability MUST apply a deployment-configured maximum candidate count, defaulting to 100, after ranking candidates. The returned list MUST never exceed that limit.

#### Scenario: Candidate limit is enforced
- **GIVEN** inference produces more candidates than the configured maximum
- **WHEN** the result is assembled
- **THEN** only the highest-ranked candidates up to the maximum are returned
