## Purpose

Define the lightweight backend-neutral protocol contract shared by DimOS and simulator runtime packages.
## Requirements
### Requirement: Shared runtime protocol package
The system SHALL provide lightweight backend-neutral runtime protocol models that can be reused by DimOS and simulator runtime packages without requiring HTTP transport, the main DimOS package in schema-only contexts, or any simulator backend SDK.

#### Scenario: Runtime package imports protocol without simulator backend
- **WHEN** a simulator runtime package imports protocol models for validation or compatibility logic
- **THEN** it can import those models without importing Robosuite, LIBERO-PRO, OmniGibson, or backend-specific dependencies

#### Scenario: DimOS module boundary reuses protocol models
- **WHEN** a Simulator Runtime Module exposes RPC request or response payloads based on runtime protocol models
- **THEN** the models describe backend-neutral runtime semantics without implying HTTP JSON transport or payload-fetch endpoints

### Requirement: Protocol model validation
The protocol package SHALL define Pydantic models for runtime description, episode reset, step requests, step responses, robot motor surfaces, motor action frames, motor state frames, observation frames, scores, artifacts, and errors.

#### Scenario: Invalid step request is rejected
- **WHEN** a step request omits required episode identity, tick identity, or action payload fields
- **THEN** protocol validation rejects the message before backend-specific step logic runs

#### Scenario: Runtime description reports motor surface
- **WHEN** a sidecar describes a robot runtime
- **THEN** the response includes robot id, surface type, ordered motors, supported command modes, and available state fields

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

### Requirement: Backend-neutral protocol types
Runtime protocol models MUST NOT expose Robosuite, LIBERO-PRO, OmniGibson, DimOS hardware adapter, or simulator object types in public fields.

#### Scenario: Robosuite observation is translated
- **WHEN** Robosuite produces an `OrderedDict` observation
- **THEN** the Robosuite sidecar translates it into runtime protocol observation and motor state frames before sending it to DimOS

### Requirement: Native runtime action frames
The runtime protocol SHALL define a native runtime action frame for benchmark action surfaces that are not DimOS motor or joint command surfaces.

#### Scenario: Runtime action frame identifies semantic action surface
- **WHEN** a runtime action frame is serialized
- **THEN** it includes a discriminator, semantic action surface identifier, numeric action values, and sequence or tick identity without requiring motor names, motor command modes, gains, or joint position fields

#### Scenario: Runtime action frame validates numeric action values
- **WHEN** a runtime action frame contains non-finite values or values that cannot be parsed as a numeric vector
- **THEN** protocol validation rejects the frame before backend-specific step logic runs

### Requirement: Step request action frame union
The runtime protocol SHALL allow a step request to carry either a motor action frame or a native runtime action frame while preserving explicit frame discrimination.

#### Scenario: Motor step request remains valid
- **WHEN** an existing client sends a step request with a valid motor action frame
- **THEN** protocol validation accepts the request as a motor-frame step request

#### Scenario: Native runtime step request is valid
- **WHEN** a client sends a step request with a valid native runtime action frame
- **THEN** protocol validation accepts the request as a runtime-action step request

#### Scenario: Ambiguous action frame is rejected
- **WHEN** a step request action lacks a supported discriminator or mixes incompatible motor-frame and runtime-action-frame fields
- **THEN** protocol validation rejects the request before it reaches sidecar step logic
