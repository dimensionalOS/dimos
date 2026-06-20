## ADDED Requirements

### Requirement: External hardware adapter registration
DimOS SHALL provide a public API that lets external packages register a hardware adapter name for each supported hardware type before a blueprint using that adapter is built.

#### Scenario: External base adapter resolves through coordinator configuration
- **GIVEN** an external package has registered a base hardware adapter with adapter type `mydog`
- **WHEN** a blueprint configures a hardware component with hardware type `BASE` and adapter type `mydog`
- **THEN** DimOS MUST resolve that adapter through the normal ControlCoordinator hardware path
- **AND** the blueprint MUST NOT require the adapter implementation to live under the DimOS source tree.

#### Scenario: All current hardware types are supported
- **GIVEN** an external package registers adapters for manipulator, base, and whole-body hardware types
- **WHEN** blueprints configure matching hardware components for each registered adapter type
- **THEN** DimOS MUST route each adapter registration to the matching hardware category
- **AND** each registered adapter type MUST be resolvable by the normal ControlCoordinator path for that category.

### Requirement: External control task registration
DimOS SHALL provide a public API that lets external packages register a control task type using a lazy factory path before a blueprint using that task is built.

#### Scenario: External task resolves lazily
- **GIVEN** an external package has registered control task type `mydog_gait` with a factory path
- **WHEN** a coordinator configuration requests task type `mydog_gait`
- **THEN** DimOS MUST resolve the task through the normal control task registry path
- **AND** DimOS MUST NOT import the task factory module at registration time.

#### Scenario: Invalid task registration is rejected early
- **GIVEN** an external package attempts to register a control task with an empty name or malformed factory path
- **WHEN** registration is attempted
- **THEN** DimOS MUST reject the registration before coordinator construction
- **AND** the error MUST identify the invalid registration input.

### Requirement: Duplicate extension registration is deterministic
DimOS SHALL make duplicate hardware adapter and control task registrations deterministic by allowing idempotent same-mapping registration and rejecting conflicting mappings.

#### Scenario: Same hardware registration is idempotent
- **GIVEN** a hardware adapter type is registered for a hardware category with a stable factory object
- **WHEN** the same hardware category, adapter type, and factory object are registered again
- **THEN** DimOS MUST treat the second registration as idempotent
- **AND** the registered adapter mapping MUST remain unchanged.

#### Scenario: Conflicting hardware registration is rejected
- **GIVEN** a hardware adapter type is already registered for a hardware category
- **WHEN** a different factory object is registered for the same hardware category and adapter type
- **THEN** DimOS MUST reject the registration
- **AND** the original adapter mapping MUST remain unchanged.

#### Scenario: Same task path registration is idempotent
- **GIVEN** a control task type is registered with a factory path
- **WHEN** the same control task type and same factory path are registered again
- **THEN** DimOS MUST treat the second registration as idempotent
- **AND** the registered task mapping MUST remain unchanged.

#### Scenario: Conflicting task path registration is rejected
- **GIVEN** a control task type is already registered with a factory path
- **WHEN** a different factory path is registered for the same control task type
- **THEN** DimOS MUST reject the registration
- **AND** the original task mapping MUST remain unchanged.

### Requirement: External package registration pattern
DimOS SHALL document a supported external package pattern where extension registration is grouped in a package helper and run before blueprint construction.

#### Scenario: External blueprint module performs registration before use
- **GIVEN** an external robot package defines a blueprint module with a `register_extensions()` helper
- **WHEN** that blueprint module is imported
- **THEN** the helper MUST be able to register required hardware adapters and control tasks before the blueprint is built
- **AND** the blueprint MUST be able to reference those registered names through unchanged `HardwareComponent` and `TaskConfig` usage.
