## ADDED Requirements

### Requirement: Default annotation IO remains supported
The system SHALL preserve existing annotation-derived module IO behavior for modules that do not override `Module.io_contract(config)`.

#### Scenario: Static annotated module resolves IO
- **WHEN** a module declares `In` or `Out` streams with class annotations and does not override `io_contract`
- **THEN** the resolved module IO contract contains the same streams that current annotation discovery would expose

### Requirement: Modules can return configuration-resolved IO contracts
The system SHALL allow a module to override `Module.io_contract(config)` and return a complete `ModuleIOContract` derived from final validated module config.

#### Scenario: Config changes IO shape
- **WHEN** a module's final validated config selects one IO mode instead of another
- **THEN** blueprint wiring uses the streams returned by `io_contract(config)` for that selected mode

#### Scenario: Custom contract replaces annotations
- **WHEN** a module overrides `io_contract(config)` and also has annotated `In` or `Out` fields
- **THEN** blueprint wiring uses only the streams returned by the override

### Requirement: IO resolves after final config merge
The system SHALL resolve module IO after blueprint kwargs, CLI/config-file module overrides, and global config have been merged into validated module config.

#### Scenario: CLI override changes resolved streams
- **WHEN** a module's blueprint kwargs select one IO mode and a build/load override selects another IO mode
- **THEN** the resolved IO contract reflects the build/load override

### Requirement: Dynamic streams use input and output registries
The system SHALL expose resolved streams through `self.inputs` and `self.outputs` for both annotated and configuration-resolved IO.

#### Scenario: Configuration-resolved input is accessible
- **WHEN** a module resolves an input stream from `io_contract(config)` without an annotated attribute
- **THEN** module code can access that stream through `self.inputs[stream_name]`

#### Scenario: Annotated stream attributes remain compatible
- **WHEN** a module declares annotated stream attributes
- **THEN** those attributes remain available while the streams are also present in `self.inputs` or `self.outputs`

### Requirement: Stream names are unique within each module IO contract
The system SHALL reject a `ModuleIOContract` that contains duplicate stream names, even when duplicate names have different directions.

#### Scenario: Duplicate input and output names are rejected
- **WHEN** a module IO contract declares an input named `joint_state` and an output named `joint_state`
- **THEN** validation fails before blueprint wiring

### Requirement: Remappings validate against resolved streams
The system SHALL apply stream remappings to resolved stream names and reject remappings whose local stream name is absent from the resolved module IO contract.

#### Scenario: Valid remapping keeps module-local key
- **WHEN** a resolved input named `primary_camera` is remapped to graph name `zed_front_rgb`
- **THEN** the graph connects using `zed_front_rgb` and module code still accesses `self.inputs["primary_camera"]`

#### Scenario: Stale remapping fails
- **WHEN** a blueprint remapping references a stream name not present in the resolved IO contract
- **THEN** blueprint resolution fails before deployment

### Requirement: Existing graph conflict semantics remain unchanged
The system SHALL preserve current graph conflict and autoconnect semantics after applying remappings to resolved stream names.

#### Scenario: Same graph name and same type autoconnect
- **WHEN** multiple resolved streams share the same graph name and message type after remapping
- **THEN** the coordinator uses the existing shared-transport autoconnect behavior

#### Scenario: Same graph name and different type fails
- **WHEN** multiple resolved streams share the same graph name with different message types after remapping
- **THEN** the coordinator rejects the blueprint using the existing conflict behavior
