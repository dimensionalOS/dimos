## Purpose

Define the deterministic, non-executing GraspGenX CUDA demonstration used to validate grasp proposal inference and visualization.

## Requirements

### Requirement: Provide a deterministic YCB banana proposal demo

The demo SHALL use a deterministic recorded YCB banana scene fixture containing a table, banana, and distractor in the `world` frame. It MUST use the fixed canonical inclusive ROI `[0.18, 0.08, 0.10]`–`[0.42, 0.32, 0.24]`, provide no request-time or runtime ROI option, run the GraspGenX CUDA adapter, and write ranked candidates to YAML.

#### Scenario: Fixture produces a usable object crop
- **GIVEN** the checked-in or repository-convention fixture and its canonical banana ROI
- **WHEN** demo smoke mode runs
- **THEN** the scene and object point counts are reported
- **AND** the crop contains sufficient finite banana points
- **AND** table and distractor points are excluded

### Requirement: Report and validate real inference

The direct contributor tool MUST report checkpoint, CUDA/device, scene point count, object point count, candidate count, best score, result frame, and output path. It MUST validate nonempty finite ranked TCP-frame output with the expected header and successful YAML output. The adapter contract and its nonidentity unit test verify TCP calibration by right-multiplication; the contributor layer does not independently observe raw backend transforms. Contributors invoke it with `uv run --extra graspgenx python -m dimos.manipulation.graspgenx_demo`. The tool MUST run fixture loading, the real adapter, ranked candidate generation, and atomic YAML/RRD publication in one direct process; it MAY launch a detached viewer. Its exit status MUST be zero only after successful validation and publication, and nonzero for configuration, inference, validation, or output failures. This requirement does not provide evidence of worker, RPC, coordinator, or blueprint deployment coverage.

#### Scenario: CUDA smoke run succeeds
- **GIVEN** a compatible checkpoint, fixed sweep-volume gripper configuration, and available CUDA device
- **WHEN** the contributor tool is run without visualization
- **THEN** the model loads once and real CUDA inference runs
- **AND** at least one finite candidate is written in the `world` frame
- **AND** the terminal reports the required diagnostics

#### Scenario: Missing runtime configuration fails clearly
- **GIVEN** a missing checkpoint or invalid sweep-volume configuration
- **WHEN** the demo starts
- **THEN** it fails with a diagnostic configuration/model error
- **AND** it does not download assets at runtime or emit candidate output

### Requirement: Visualize proposal inputs and ranked outputs

When the optional Rerun viewer is enabled, the contributor tool MUST show the raw scene in grey, the yellow object crop, and up to five abstract Grasp Envelope Glyphs when candidates are available. Each glyph MUST be one non-occluding `LineStrips3D` planar fork transformed by a candidate's full 6-DoF TCP pose and colored by score. Rank 1 MUST be visible by default when present; remaining recorded ranks up to 5 MUST remain toggleable in Rerun. Visualization MUST not be required for inference or YAML output.

#### Scenario: Rerun view matches serialized proposals
- **GIVEN** successful demo inference with visualization enabled
- **WHEN** the viewer is inspected
- **THEN** raw scene and cropped object are visibly distinct
- **AND** rank 1 is visible by default when present
- **AND** remaining recorded ranks up to 5 can be toggled without changing YAML output

### Requirement: Keep the demo outside robot execution

The demo MUST NOT require a camera, robot, IK, collision checker, planner, actuation, or motion execution. It MUST support a headless smoke path that performs data and output assertions without Rerun.

#### Scenario: Headless demo runs without execution dependencies
- **GIVEN** the shared DimOS environment with the GraspGenX extra and packaged fixture
- **WHEN** the contributor command runs with no viewer
- **THEN** it completes proposal inference and YAML serialization
- **AND** no hardware command or execution-stage dependency is invoked

### Requirement: Protect offline packaged inputs

The fixture NPZ and adjacent provenance JSON MUST be present in both wheel and sdist artifacts. Runtime MUST not clone the upstream repository or download checkpoints, gripper assets, or fixture data.

#### Scenario: Offline artifact retains the fixture
- **GIVEN** a wheel or sdist built from the repository
- **WHEN** the demo package is installed without network access
- **THEN** the fixture NPZ and provenance JSON are present
- **AND** runtime does not invoke a clone or download operation
