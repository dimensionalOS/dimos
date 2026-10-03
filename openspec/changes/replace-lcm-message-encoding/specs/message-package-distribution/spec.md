## ADDED Requirements

### Requirement: Local and reusable definitions use the same workflow

The tooling SHALL support application-local schema packages and separately distributed schema packages through the same generator and dependency resolver. Adding a message or field MUST NOT require publication, a DimOS source edit, or an external repository PR.

#### Scenario: Add a field in a downstream application
- **WHEN** a separate example application adds a field to its custom `.msg` and reruns generation and builds
- **THEN** its Python, C++, and Rust consumers expose and exchange that field without editing DimOS

### Requirement: Consume artifacts through standard language tooling

The workflow SHALL produce installable Python wheels/sdists, an exported CMake package, and Rust crates, with the definitions and schema metadata needed by downstream generation and recording. Installed Python schema/type providers SHALL be discoverable through `dimos.messages` entry points. Consumers SHALL install dependencies during environment setup or build; runtime MUST NOT download schemas, install packages, or run code generation.

#### Scenario: Fresh downstream consumers
- **WHEN** the stage's built artifacts are installed into a clean ROS-free environment
- **THEN** a Python program imports its message package, a C++ program consumes it through CMake, and a Rust program consumes its crate through Cargo
- **AND** all three exchange the same custom message

#### Scenario: Installed external type discovery
- **WHEN** an installed external message package registers its provider
- **THEN** DimOS can resolve its qualified types and schema metadata without a handwritten built-in registry entry

### Requirement: Reproduce generation locally and in CI

CI SHALL run the same pinned generation workflow available locally, produce official build artifacts, and verify deterministic generated source and schema metadata. Generated source SHALL remain outside version control. CI SHALL exercise the existing supported platform build matrix; package contents SHALL include the schema resources and build inputs appropriate to wheels, source distributions, CMake consumers, and crates.

#### Scenario: Rebuild from packaged inputs
- **WHEN** two clean builds use the same schema inputs and pinned toolchain
- **THEN** their generated sources and schema metadata are identical
- **AND** the resulting packages contain the inputs/outputs required by their documented consumption route

#### Scenario: Generate without network access after setup
- **WHEN** toolchain dependencies and schema packages are already installed and network access is unavailable
- **THEN** local generation and the three-language example build do not attempt to retrieve message definitions at runtime
