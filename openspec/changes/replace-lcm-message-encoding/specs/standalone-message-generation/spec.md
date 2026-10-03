## ADDED Requirements

### Requirement: Generate three languages without a ROS installation

The tooling SHALL provide one documented generation workflow accepting ROS2 `.msg` package roots and emitting Python, C++, and Rust types and encode/decode implementations. Generation and ordinary runtime use MUST NOT require ROS installation, a ROS environment, a DimOS source change, publication, or a `dimos-lcm` PR. Native compilation and standalone build dependencies are permitted.

#### Scenario: Local custom message with a standard dependency
- **WHEN** an application defines a custom message containing `std_msgs/Header`, a string, a numeric sequence, and a nested application message in an environment without ROS
- **THEN** the workflow generates buildable outputs for all three languages using the bundled standard dependency closure
- **AND** all three outputs expose the declared fields and codec operations

### Requirement: Resolve pinned definitions deterministically

The tooling SHALL bundle the required Jazzy standard definitions with immutable upstream revisions and licenses/notices. It SHALL resolve complete transitive dependencies from bundled, local, and installed definitions by qualified name. It SHALL reject conflicting definitions sharing a qualified name, unresolved references, invalid syntax, and recursive value layouts with actionable source diagnostics.

#### Scenario: Transitive standard dependency
- **WHEN** a custom definition references `sensor_msgs/Image`
- **THEN** resolution includes its `std_msgs/Header` and `builtin_interfaces/Time` dependencies without fetching or installing ROS

#### Scenario: Conflicting installed and local definitions
- **WHEN** two inputs provide different definitions of the same qualified type
- **THEN** generation fails identifying the type and both sources rather than choosing an override

#### Scenario: Missing nested type
- **WHEN** a definition references a type absent from the available package roots
- **THEN** generation fails with the referencing definition and missing qualified dependency

### Requirement: Preserve supported message semantics

Generated types SHALL preserve the supported ROS2 `.msg` field names, order, types, constants, defaults, nested types, fixed arrays, sequences, and bounds. The initial supported surface SHALL cover these constructs as used by bundled Jazzy definitions. Unsupported constructs SHALL fail explicitly during generation. Service and action definitions are outside this capability.

#### Scenario: Defaults and bounded fields
- **WHEN** a definition declares defaults, a fixed array, a bounded string, and a bounded sequence
- **THEN** generated types initialize the declared defaults and encode valid values consistently across languages
- **AND** invalid fixed-array lengths or bounds violations are rejected rather than truncated or emitted with a different layout

### Requirement: Use a common ROS2-compatible CDR wire profile

Generated codecs SHALL emit encapsulated plain CDR/XCDR1 with little-endian encoding and decode supported little- and big-endian representations. They SHALL reject unsupported representation headers and malformed/truncated messages. The CDR payload MUST NOT contain an LCM fingerprint or custom schema-hash prefix. Conformance SHALL cover all nine Python/C++/Rust encoder-decoder combinations and an independent Jazzy reference in CI.

#### Scenario: Every language decodes every producer
- **WHEN** each language serializes fixtures containing nested types, empty and nonempty arrays, strings, numeric boundaries, and alignment-sensitive fields
- **THEN** every language recovers the same field values from each producer's bytes
- **AND** emitted bytes match the chosen profile's independent Jazzy reference fixtures

#### Scenario: Unsupported or incomplete payload
- **WHEN** a decoder receives an unsupported encapsulation or truncated field data
- **THEN** decoding reports an error and does not return a partially decoded message as valid

### Requirement: Python bindings have safe documented buffer behavior

Python generated types SHALL use generated native codecs with documented nested-field and sequence mutation behavior. NumPy helpers SHALL provide read-only borrowed views where the layout permits, retaining the storage owner for the view lifetime, and an explicit copy path for mutable processing. Operations that would resize or invalidate borrowed backing storage SHALL be rejected while a view exists. Conversions requiring copies SHALL document that fact. Wire types MUST NOT import geometry or visualization libraries.

#### Scenario: Image view outlives its Python message variable
- **WHEN** a supported image buffer is exposed as a borrowed NumPy view and the caller releases its message variable
- **THEN** the view retains valid storage and remains read-only for its lifetime
- **AND** requesting a mutable copy produces independent writable storage

#### Scenario: Nested and sequence mutation
- **WHEN** a caller edits a nested field or sequence through the documented Python API and serializes the message
- **THEN** C++ and Rust decode the edited values rather than values from an unnoticed temporary copy

#### Scenario: Resize while storage is borrowed
- **WHEN** a caller attempts to resize an exported message buffer while a borrowed view exists
- **THEN** the operation fails explicitly and the existing view remains valid
