## ADDED Requirements

### Requirement: Record standard CDR and complete embedded schemas

MCAP recordings SHALL use the ROS2 profile, `cdr` message encoding, and `ros2msg` schemas with qualified type names and the complete transitive dependency definitions. Stored payloads SHALL be the generated encapsulated CDR bytes. Compression SHALL use MCAP chunk compression or a standard compressed-message schema, not a private wrapper around the channel's declared CDR encoding.

#### Scenario: Custom type with nested standard definitions
- **WHEN** a generated custom message containing a standard header and a nested custom message is recorded
- **THEN** the MCAP embeds the definitions needed to decode the complete message independently of installed message packages
- **AND** the channel's payload matches its declared CDR schema

### Requirement: Inspect one recording in Foxglove and Rerun

The demonstration recording SHALL be usable in Foxglove and the repository's pinned Rerun version without installing its custom message package in either viewer. Standard image and pose messages SHALL be visually demonstrated. Custom messages SHALL support inspection of their decoded fields; automatic semantic visualization of every custom type is not required.

#### Scenario: Same file opened by both viewers
- **WHEN** a reviewer opens the demonstration MCAP containing an image, moving pose, and custom telemetry in each viewer
- **THEN** the image and pose can be visualized and the custom telemetry fields can be inspected using the embedded definitions

### Requirement: Record arbitrary registered messages without native decoder changes

The runtime SHALL supply stream schemas to the recorder through its existing startup/configuration path. Recording an externally defined message SHALL NOT require adding handwritten recorder decoding logic or rebuilding the recorder for that message. The recorder SHALL use source timestamps for supported stamped types and reception time for unstamped/unknown timestamp layouts; MCAP log time SHALL represent reception time.

#### Scenario: Newly generated external telemetry
- **WHEN** an application installs a custom message package and records its stream with an otherwise unchanged recorder
- **THEN** the recorder stores its payload and full schema and viewers can inspect its fields

#### Scenario: Stamped and unstamped streams
- **WHEN** a recording receives a supported stamped message and an unstamped custom message
- **THEN** their publish times use the source stamp and reception time respectively
- **AND** their log times reflect reception time

### Requirement: Replay new recordings through typed streams

DimOS recording/read/replay paths SHALL support the replacement generated-message representation. Replay into typed application consumers SHALL use installed matching message packages and preserve field values and recorded ordering/timing according to the replay API. The change SHALL remove obsolete LCM-backed recording paths rather than providing a legacy migration or fallback reader.

#### Scenario: Record, stop, and replay the example
- **WHEN** a reviewer records the integrated blueprint, stops its producers, and replays with the matching message packages installed
- **THEN** typed consumers receive the recorded image, pose, and custom telemetry values without an LCM codec
- **AND** the resulting viewer output visibly corresponds to the recorded sequence
