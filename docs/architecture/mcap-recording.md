# Native schema-aware MCAP recording

Status: accepted, 2026-09-20.

## Decision

Add `McapRecordingStore` as a schema-aware Memory2 reader and change the native
MCAP writer to emit standard CDR/ROS message schemas plus explicit custom JSON
schemas. Keep the existing Go2 DDS store's robot-specific conversions separate.

```text
LCM or Zenoh transport (current LCM wire payload)
          |
          v
Rust input decoder -> explicit message mapping -> CDR or JSON
          |                                      |
          +------------------------------> indexed Zstd MCAP
                                                 |
                          +----------------------+----------------+
                          v                                       v
                  generic MCAP tools                  McapRecordingStore
                                                      schema decoder (Python)
                                                              |
                                                      trusted DimOS conversion
                                                              |
                                                     Memory2 queries / rerun
```

The first mappings cover Image, PointCloud2, CameraInfo, Imu, JointState,
PoseStamped, Odometry, Path, TFMessage, and JSON LineSegments3D. A converter
explicitly names its input type, output schema, and codec. Unknown inputs fail
preflight; there is no automatic type inference or opaque byte fallback.
All encoding, JPEG compression, chunk compression, and file writes run in Rust.

The ROS layouts are independent serde wire structs and bundled Jazzy `.msg`
definitions, including dependencies. The `cdr` crate writes encapsulated
little-endian CDR; fixed covariance arrays are serialized without a sequence
length. LCM length fields, fingerprints, and `Header.seq` never enter CDR.
Raw images are the default, with explicit lossy JPEG as standard CompressedImage.

Channels use nonzero, deduplicated schemas, the registered `cdr`/`ros2msg` or
`json`/`jsonschema` encoding pair, and optional DimOS stream metadata. The file
profile is `dimos` because a recording can mix CDR and JSON. This is an MCAP
container contract, not a rosbag directory or ROS runtime dependency.

Memory2 exposes publish time as observation time and keeps payload decoding lazy;
reception time remains the MCAP log time. Unknown schemas in supported encodings
can be inspected without importing Python modules from artifact metadata.
The reader currently requires a finalized summary with statistics.

## Boundaries and consequences

- LCM is an input decoder dependency while current transports emit LCM bytes.
- Recording wire layouts do not depend on LCM code generation or LCM schemas.
- New mappings require an explicit Rust converter, embedded schema, Python conversion when needed, and an independent round-trip test.
- The existing SQLite recorder and Go2 DDS reader retain their separate contracts.
- A Rust reader binding is deferred until profiling shows Python reads are a bottleneck.
- Protobuf, a runtime schema compiler, and legacy format migration are outside this change.

## Terms

| Term | Meaning |
|---|---|
| Transport encoding | Bytes arriving from LCM or Zenoh, currently LCM message bytes |
| Recording encoding | Channel payload encoding, currently CDR or JSON |
| Schema | Named message definition embedded in MCAP, including dependencies |
| Mapping | Explicit conversion from a supported input type to a recording schema |
| Profile | File-level convention (`dimos`); distinct from channel message encoding |
| Source time | Positive input message timestamp used for `publish_time`, with reception fallback |
| Reception time | Recorder arrival timestamp stored as `log_time` |

## Verification

Rust tests cover schema registration, indexing, preflight errors, and native
encoding failures. A test-only Rust entry point records Python-produced transport
bytes, and Python's independent MCAP ROS 2 decoder verifies every supported wire
layout. Memory2 tests cover lazy queries, source-time ordering, unknown schemas,
endianness, padding, covariances, TF timestamps, JPEG, raw depth, JSON, summary,
and Rerun export. Existing transport integration tests exercise SQLite and MCAP
through the native process.

## References

- [MCAP specification](https://mcap.dev/spec) defines schema/channel records and permits schema ID zero only to indicate no schema.
- [MCAP format registry](https://mcap.dev/spec/registry) defines standard encoding identifiers without prohibiting custom encodings.
- [Bundled schemas and provenance](/dimos/experimental/memory/rust/schemas/README.md) identify the pinned upstream message definitions.
- [Recording usage](/docs/usage/recording.md) lists supported mappings and commands.
