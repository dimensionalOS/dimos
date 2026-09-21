## ADDED Requirements

### Requirement: Generated ROS-shaped types replace LCM-backed types

DimOS typed-message APIs SHALL use the generated field shapes and transport-neutral codecs. Consumers SHALL be updated to ROS2 timestamps, nested stamped types, and correct single-transform versus transform-array types. Rich geometry, timestamp, NumPy/Open3D, and visualization behavior SHALL live in separate utilities/adapters. Obsolete LCM codec methods and convenience wrappers SHALL be removed rather than retained as compatibility aliases.

#### Scenario: A stamped pose crosses a module boundary
- **WHEN** a producer publishes a generated `PoseStamped` containing a header and nested pose
- **THEN** the receiver and timestamp/geometry helpers consume its ROS2-shaped fields directly
- **AND** the path does not convert through an LCM message object

#### Scenario: Compressed image publication
- **WHEN** a producer publishes compressed image bytes
- **THEN** it uses the generated `CompressedImage` schema and corresponding viewer adapter rather than disguising compressed bytes as a raw `Image`

### Requirement: LCM and Zenoh transport the same typed CDR payloads

The typed layers for Python, C++, and Rust SHALL serialize through generated CDR codecs while retaining raw LCM and Zenoh transport functionality. Type routing SHALL use explicit topic/type metadata where required, without runtime schema-hash enforcement. Migrated paths MUST NOT retain an LCM message-codec fallback.

#### Scenario: Three-language live relay on each transport
- **WHEN** a hardware-free blueprint exchanges a generated custom message among Python, C++, and Rust using LCM and then Zenoh
- **THEN** the same declared values and language-specific edits are observed on both transports
- **AND** both runs use the generated CDR payload contract

#### Scenario: Large LCM payload
- **WHEN** the blueprint sends an image large enough to require LCM fragmentation between native and Python processes
- **THEN** receivers reconstruct and decode the complete generated message correctly

### Requirement: Existing visualization consumers remain usable

The change SHALL preserve live Rerun through separate generated-type adapters and update browser/WebRTC decoding to CDR with schema metadata. Browser support for a new message SHALL NOT require a handwritten per-message TypeScript decoder. Multiplexed channels SHALL dispatch by explicit channel/type identity rather than probing CDR decoders for success.

#### Scenario: Synthetic data in existing viewers
- **WHEN** the integrated example publishes a changing pose and synthetic image
- **THEN** live Rerun displays their updates and the browser decodes the advertised generated schema correctly

#### Scenario: Two types share a multiplexed connection
- **WHEN** two typed channels with different layouts share a WebRTC connection
- **THEN** each message is delivered to the decoder selected by its channel/type metadata
- **AND** decoder success is not used as the type-identification mechanism

### Requirement: Eliminate the external message repository dependency

The final stack SHALL remove `dimos-lcm` Python/message dependencies, `lcm-msgs`, native header downloads, and the Rust raw transport dependency sourced from the external repository. A replacement or extracted raw transport SHALL preserve required notices and remain separate from schema generation. LCM transport itself SHALL remain available. Old message encodings and compatibility readers SHALL be removed; runtime schema-hash enforcement is not required.

#### Scenario: Clean final build without the old repository
- **WHEN** the supported packages and demo are built and installed in a clean environment where access to the `dimos-lcm` repository is unavailable
- **THEN** the generated-message workflow, both transports, and recording/replay operate successfully
- **AND** manifests, lockfiles, build scripts, and installed dependency inventory contain no active dependency on that repository
