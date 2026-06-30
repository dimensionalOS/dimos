## MODIFIED Requirements

### Requirement: Shared runtime protocol package
The system SHALL provide lightweight backend-neutral runtime protocol models that can be reused by DimOS and simulator runtime packages without requiring HTTP transport, the main DimOS package in schema-only contexts, or any simulator backend SDK.

#### Scenario: Runtime package imports protocol without simulator backend
- **WHEN** a simulator runtime package imports protocol models for validation or compatibility logic
- **THEN** it can import those models without importing Robosuite, LIBERO-PRO, OmniGibson, or backend-specific dependencies

#### Scenario: DimOS module boundary reuses protocol models
- **WHEN** a Simulator Runtime Module exposes RPC request or response payloads based on runtime protocol models
- **THEN** the models describe backend-neutral runtime semantics without implying HTTP JSON transport or payload-fetch endpoints

### Requirement: Protocol compatibility handshake
The runtime protocol SHALL include protocol version and capability metadata that simulator runtimes can report through module-native description RPCs or compatibility handshakes so DimOS can fail fast on incompatible protocol versions or unsupported capabilities.

#### Scenario: Compatible module runtime describes itself
- **WHEN** DimOS calls `describe()` on a Simulator Runtime Module using a compatible protocol version
- **THEN** the runtime description includes protocol version and capability metadata that DimOS records in artifacts

#### Scenario: Incompatible runtime is rejected
- **WHEN** DimOS describes a simulator runtime using an incompatible protocol version or unsupported capabilities
- **THEN** prelaunch or runtime setup fails before benchmark stepping and records the incompatibility reason

### Requirement: Binary-friendly observation transport
The runtime protocol SHALL support image, depth, segmentation, and object/state observation metadata without requiring large image tensors, raw NumPy arrays, or nested JSON lists to be encoded in RPC responses.

#### Scenario: Image observation uses DimOS stream metadata
- **WHEN** a Simulator Runtime Module publishes an RGB image observation on a DimOS stream
- **THEN** associated observation metadata identifies the `Image` stream, encoding, shape, dtype, sequence, or timestamp sufficient for consumers and artifacts to correlate the stream output

#### Scenario: Camera model uses CameraInfo stream
- **WHEN** a Simulator Runtime Module publishes an image stream with camera intrinsics
- **THEN** the camera model is published as `CameraInfo` rather than embedded as ad hoc metadata in the step response

#### Scenario: HTTP payload reference is removed from target path
- **WHEN** a runtime observation is produced by a migrated Simulator Runtime Module
- **THEN** the target path uses stream metadata and DimOS stream outputs rather than HTTP payload references
