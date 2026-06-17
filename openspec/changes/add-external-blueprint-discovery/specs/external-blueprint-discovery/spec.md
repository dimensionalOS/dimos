## ADDED Requirements

### Requirement: External blueprint names SHALL be discovered from installed package metadata
DimOS SHALL discover external blueprint names from installed Python distribution entry points in the `dimos.blueprints` group.

#### Scenario: Installed package exposes external blueprints
- **GIVEN** an installed distribution named `my-robot-stack`
- **AND** the distribution declares `dimos.blueprints` entry points named `go2` and `keyboard-teleop`
- **WHEN** DimOS discovers external blueprints
- **THEN** the discovered names SHALL include `my-robot-stack.go2` and `my-robot-stack.keyboard-teleop`
- **AND** discovery SHALL use package metadata without importing the entry point target modules.

#### Scenario: Distribution name is canonicalized
- **GIVEN** an installed distribution whose name contains uppercase letters, underscores, dots, or repeated separators
- **WHEN** DimOS derives the external blueprint namespace
- **THEN** it SHALL lowercase the name and collapse runs of `-`, `_`, and `.` into a single `-`
- **AND** the canonical distribution namespace SHALL be used as the prefix of every external blueprint name from that distribution.

### Requirement: External blueprint names SHALL be explicitly namespaced
DimOS SHALL require external blueprints to be referenced as `<canonical-distribution-namespace>.<external-local-blueprint-name>`.

#### Scenario: Bare built-in name remains built-in only
- **GIVEN** a user requests `dimos run unitree-go2`
- **WHEN** DimOS resolves the bare blueprint name
- **THEN** it SHALL resolve only against DimOS built-in blueprint and module registries
- **AND** it SHALL NOT search external packages for a matching bare name.

#### Scenario: Namespaced external name is requested
- **GIVEN** an installed distribution with canonical namespace `my-robot-stack`
- **AND** it declares an external local blueprint name `go2`
- **WHEN** a user requests `dimos run my-robot-stack.go2`
- **THEN** DimOS SHALL resolve the external blueprint identified by namespace `my-robot-stack` and local name `go2`.

### Requirement: External local blueprint names SHALL use DimOS-style kebab-case
DimOS SHALL accept only lowercase kebab-case external local blueprint names matching `^[a-z0-9]+(-[a-z0-9]+)*$`.

#### Scenario: Valid local blueprint name
- **GIVEN** an external entry point named `keyboard-teleop`
- **WHEN** DimOS lists or resolves external blueprint metadata
- **THEN** the local blueprint name SHALL be considered valid.

#### Scenario: Invalid local blueprint name
- **GIVEN** an external entry point named `Go2`, `go2_sim`, `go2.sim`, or `go2/real`
- **WHEN** DimOS encounters that entry point as an external blueprint
- **THEN** DimOS SHALL treat the entry point name as invalid external blueprint metadata
- **AND** it SHALL NOT silently normalize the invalid local blueprint name into a different runnable name.

### Requirement: External blueprint resolution SHALL load only the requested entry point
DimOS SHALL defer importing external target modules until a specific external blueprint name is resolved for execution or API use.

#### Scenario: Listing external blueprints does not import targets
- **GIVEN** an installed external package with `dimos.blueprints` metadata
- **WHEN** a user runs `dimos list`
- **THEN** DimOS SHALL display the external blueprint names from metadata
- **AND** it SHALL NOT import the entry point target modules merely to list them.

#### Scenario: Running external blueprint loads target
- **GIVEN** an installed external package exposing `my-robot-stack.go2`
- **WHEN** a user runs `dimos run my-robot-stack.go2`
- **THEN** DimOS SHALL load the target for that specific entry point
- **AND** it SHALL resolve it to a runnable blueprint before building the stack.

### Requirement: External targets SHALL resolve to blueprints or DimOS Module classes
DimOS SHALL accept external entry point targets that are Blueprint objects or DimOS Module classes, and SHALL reject unsupported target types.

#### Scenario: Entry point targets a Blueprint object
- **GIVEN** an external entry point target resolves to a Blueprint object
- **WHEN** DimOS resolves the namespaced blueprint name
- **THEN** DimOS SHALL return that Blueprint for normal composition and execution.

#### Scenario: Entry point targets a DimOS Module class
- **GIVEN** an external entry point target resolves to a DimOS Module class
- **WHEN** DimOS resolves the namespaced blueprint name
- **THEN** DimOS SHALL convert the class using `.blueprint()`
- **AND** it SHALL return the resulting Blueprint for normal composition and execution.

#### Scenario: Entry point targets an unsupported object
- **GIVEN** an external entry point target resolves to a factory function, non-DimOS class, primitive value, or other unsupported object
- **WHEN** DimOS resolves the namespaced blueprint name
- **THEN** DimOS SHALL fail with an error that identifies the target as an invalid external blueprint target.

### Requirement: Blueprint composition commands SHALL support mixed built-in and external names
DimOS SHALL resolve each argument to `dimos run` independently so built-in and external blueprint names can be composed in one command.

#### Scenario: Mixed composition command
- **GIVEN** a built-in blueprint named `unitree-go2`
- **AND** an installed external blueprint named `my-robot-stack.keyboard-teleop`
- **WHEN** a user runs `dimos run unitree-go2 my-robot-stack.keyboard-teleop`
- **THEN** DimOS SHALL resolve both names independently
- **AND** it SHALL compose the resulting blueprints into one runnable stack using the existing composition behavior.

### Requirement: List command SHALL group built-in and external blueprints
`dimos list` SHALL include external blueprints by default and SHALL separate built-in blueprints from external blueprints.

#### Scenario: User lists blueprints
- **GIVEN** DimOS has built-in blueprints
- **AND** an installed distribution exposes external blueprints
- **WHEN** a user runs `dimos list`
- **THEN** the output SHALL include a built-in blueprints section
- **AND** the output SHALL include an external blueprints section
- **AND** external names SHALL be displayed fully qualified with namespace and local blueprint name.

#### Scenario: No external blueprints are installed
- **GIVEN** no installed distributions expose `dimos.blueprints` entry points
- **WHEN** a user runs `dimos list`
- **THEN** DimOS SHALL still list built-in blueprints normally
- **AND** it SHALL NOT fail because no external blueprints are present.

### Requirement: External resolution errors SHALL identify namespace-aware failure modes
DimOS SHALL provide distinct errors for unknown namespaces, missing local names, entry point load failures, invalid local names, and invalid target object types.

#### Scenario: Namespace is not discovered
- **GIVEN** no installed distribution has canonical namespace `missing-stack`
- **WHEN** a user requests `dimos run missing-stack.go2`
- **THEN** DimOS SHALL fail with an error indicating that the external blueprint namespace was not discovered.

#### Scenario: Local blueprint name is not declared
- **GIVEN** an installed distribution has canonical namespace `my-robot-stack`
- **AND** it does not declare an external local blueprint name `arm`
- **WHEN** a user requests `dimos run my-robot-stack.arm`
- **THEN** DimOS SHALL fail with an error indicating that the namespace exists but the local blueprint name is not declared.

#### Scenario: Entry point target fails to load
- **GIVEN** an installed distribution declares `my-robot-stack.go2`
- **AND** loading the entry point target raises an import or loading error
- **WHEN** DimOS resolves `my-robot-stack.go2`
- **THEN** DimOS SHALL fail with an error indicating that the external blueprint entry point failed to load
- **AND** the error SHALL preserve enough context to diagnose the underlying load failure.

### Requirement: Shared resolver behavior SHALL apply to CLI, Python API, and coordinator loading
DimOS SHALL use consistent external blueprint name resolution across command-line, local Python API, and coordinator-side loading paths.

#### Scenario: Python API resolves external name
- **GIVEN** an external package exposing `my-robot-stack.go2` is installed in the local Python environment
- **WHEN** user code calls `Dimos.run("my-robot-stack.go2")`
- **THEN** DimOS SHALL resolve the external blueprint using the same rules as `dimos run my-robot-stack.go2`.

#### Scenario: Coordinator resolves external name in its own environment
- **GIVEN** a client asks a coordinator to load `my-robot-stack.go2` by name
- **WHEN** the coordinator performs name resolution
- **THEN** the coordinator SHALL discover external blueprints from packages installed in the coordinator environment
- **AND** packages installed only in the client environment SHALL NOT be assumed available to the coordinator.
