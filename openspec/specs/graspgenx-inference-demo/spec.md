## Purpose

Define the deterministic, non-executing GraspGenX CUDA demonstration used to validate grasp proposal inference and visualization.

## Requirements

### Requirement: Provide a deterministic YCB banana proposal demo

The demo SHALL use a deterministic recorded YCB banana scene fixture containing a table, banana, and distractor in the `world` frame. It MUST load the canonical inclusive ROI `[0.18, 0.08, 0.10]`–`[0.42, 0.32, 0.24]` by default, permit deployment-time runner configuration overrides, reject request-time ROI controls, run the configured GraspGenX CUDA adapter, and write ranked candidates to YAML.

#### Scenario: Fixture produces a usable object crop
- **GIVEN** the checked-in or repository-convention fixture and its configured banana ROI
- **WHEN** demo smoke mode runs
- **THEN** the scene and object point counts are reported
- **AND** the crop contains sufficient finite banana points
- **AND** table and distractor points are excluded

### Requirement: Report and validate real inference

The demo MUST report checkpoint, CUDA/device, scene point count, object point count, candidate count, best score, result frame, and output path. Smoke mode MUST assert at least one candidate, descending scores, finite poses in `world`, application of the TCP transform, and successful YAML output. The generic coordinator-owned `OneShotModule` lifecycle MUST run the demo once and exit 0 after cleanup on success.

#### Scenario: CUDA smoke run succeeds
- **GIVEN** a compatible checkpoint, fixed sweep-volume gripper configuration, and available CUDA device
- **WHEN** the demo is run without visualization
- **THEN** the model loads once and real CUDA inference runs
- **AND** at least one finite candidate is written in the `world` frame
- **AND** the terminal reports the required diagnostics

#### Scenario: Missing runtime configuration fails clearly
- **GIVEN** a missing checkpoint or invalid sweep-volume configuration
- **WHEN** the demo starts
- **THEN** it fails with a diagnostic configuration/model error
- **AND** it does not download assets at runtime or emit candidate output

### Requirement: Visualize proposal inputs and ranked outputs

When the supported `rerun` viewer mode is enabled, the demo MUST show the raw scene in grey, the yellow object crop, the top 20 TCP candidate axes colored by score, and the best candidate's propagated sweep-volume boxes. Visualization MUST not be required for smoke inference or YAML output. The only supported viewer modes are `rerun` and `none`.

#### Scenario: Rerun view matches serialized proposals
- **GIVEN** successful demo inference with visualization enabled
- **WHEN** the viewer is inspected
- **THEN** raw scene and cropped object are visibly distinct
- **AND** candidate axes are ranked/color-coded consistently with the YAML scores
- **AND** optional sweep boxes correspond to the best candidate

### Requirement: Keep the demo outside robot execution

The demo MUST NOT require a camera, robot, IK, collision checker, planner, actuation, or motion execution. It MUST support a headless smoke path that performs data and output assertions without Rerun.

#### Scenario: Headless demo runs without execution dependencies
- **GIVEN** the shared DimOS environment with the GraspGenX extra and packaged fixture
- **WHEN** the smoke command runs with no viewer
- **THEN** it completes proposal inference and YAML serialization
- **AND** no hardware command or execution-stage dependency is invoked

### Requirement: Protect offline packaged inputs

The fixture NPZ and adjacent provenance JSON MUST be present in both wheel and sdist artifacts. Runtime MUST not clone the upstream repository or download checkpoints, gripper assets, or fixture data.

#### Scenario: Offline artifact retains the fixture
- **GIVEN** a wheel or sdist built from the repository
- **WHEN** the demo package is installed without network access
- **THEN** the fixture NPZ and provenance JSON are present
- **AND** runtime does not invoke a clone or download operation
