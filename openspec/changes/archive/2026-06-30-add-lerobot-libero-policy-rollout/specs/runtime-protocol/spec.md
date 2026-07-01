## ADDED Requirements

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
