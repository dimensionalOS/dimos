## Context

DimOS currently couples message definitions, generated classes, and LCM encoding to `dimos-lcm`. The Python message protocol exposes `lcm_encode`/`lcm_decode`; both LCM and Zenoh typed pub/sub use that codec. C++ defaults to generated LCM codecs. Rust depends on both `lcm-msgs` and a raw `dimos-lcm` transport crate from the same external repository.

Python message classes also contain geometry, timestamp, NumPy, Open3D, and viewer behavior. Their field shapes are not consistently ROS2 shapes: stamped poses can be flattened, timestamps use convenience fields, and a single transform can be exposed as `TFMessage`. Replacing only the serializer would preserve these inconsistencies and the message-authoring bottleneck.

The existing native ping-pong examples already run on LCM or Zenoh. The recorder has an MCAP writer, but its current LCM payload channels do not carry the complete ROS2 schemas needed by viewers. Live Rerun and browser decoding are existing consumers that must remain functional.

## Goals / Non-Goals

**Goals:**

- A user adds a namespaced `.msg` locally, builds all three languages, and uses it without a DimOS or `dimos-lcm` PR.
- Definition resolution, code generation, package installation, and ordinary runtime operation work without ROS installed. Native compilers and standalone codec/parser dependencies are acceptable.
- Generated ROS-shaped types and CDR are the common typed-message representation across Python, C++, Rust, LCM, Zenoh, recordings, and visualization adapters.
- Official artifacts are generated in CI; local users run the same workflow. Generated source is absent from Git.
- Each PR provides a working increment, automated tests, and a reproducible human-facing demo. The final stack removes all dependency on the external repository.

**Non-Goals:**

- ROS CLI compatibility, a DDS/ROS graph, new ROS network interoperation, or `.srv`/`.action` generation. Existing RPC remains separate.
- Reading old LCM-encoded recordings, mixed old/new deployments, compatibility wrappers, fallback decoders, or data migration tools.
- Runtime schema-hash enforcement, transport hash framing, a remote schema service, a custom package manager, or runtime package downloads.
- A new live Foxglove bridge or automatic semantic visualization of every custom message.

## Decisions

### 1. Definitions are the source of truth

Use ROS2 `.msg` as the authoring format. Vendor the standard Jazzy definitions needed by supported DimOS messages with their transitive dependencies, immutable upstream revisions, and required licenses/notices. Standard schemas include dependencies from both `common_interfaces` and `rcl_interfaces`; do not assume all definitions live in one repository.

Resolve names as `package/msg/Type` from bundled definitions, the user's package roots, and installed schema packages. Resolve the full dependency closure before emitting code. Reject unresolved references, recursive value layouts, invalid definitions, and conflicting definitions with the same qualified name. Identical repeated definitions can be deduplicated; there is no override precedence for conflicting definitions.

The initial supported `.msg` surface includes primitive fields, constants, defaults, nested messages, fixed arrays, sequences, and bounded strings/sequences used by Jazzy message definitions. Unsupported syntax must fail at generation with a source location; it must never silently change the wire layout. `.srv` and `.action` are outside the interface.

**Alternative considered:** installing ROS message packages and invoking their build chain. Rejected because it makes ROS installation part of the user's message-development workflow. A ROS-free parser or a narrowly packaged upstream parser is acceptable.

### 2. One generation workflow, native outputs for all three languages

```text
bundled Jazzy .msg + local/installed .msg
                     |
        resolver + normalized definitions
                     |
          +----------+-----------+
          |                      |
  generated C++/Fast CDR    generated Rust/Serde
          |                      |
  generated pybind11        native CDR backend
          |
        Python

        All outputs use the same CDR wire contract.
```

Use Fast CDR-backed generated C++ serialization and generated pybind11 bindings for Python. Python and C++ share the implementation rather than maintaining separate Python wire layouts. Rust uses generated native types with a Serde-compatible CDR backend. The generator also emits qualified type metadata and complete schema text for recording and introspection.

The implementation uses the pinned upstream `rosidl_adapter` parser, a shared normalized definition model, a C++ struct/Fast CDR customization emitter, generated pybind11 bindings, and a Rust struct emitter plus `re_cdr`. Fast CDR owns sizing, alignment, primitive representation, and containers; the emitter only supplies the declared fields, defaults, validation, and library calls. There is no second handwritten wire engine.

The Fast-DDS-Gen 4.3.0 candidate was built successfully without ROS, but its IDL default annotation did not accept a ROS fixed-array default (`@default({1.0, 2.0})`). Using it would require additional default handling and a second schema transformation. Directly emitting the small native value types from the same normalized model preserves scalar and array defaults across all languages and removes Java/Gradle from the user build requirements. The currently validated dependencies are Fast CDR 2.4.0, pybind11 3.0.1, and `re_cdr` 0.1.0. The full 142-type build, independent-codec checks, Jazzy reference, bounds, and buffer tests now pass locally; CI repeats these gates. Packaging is stage 2.

Use encapsulated plain CDR/XCDR1 with little-endian emission as the initial wire profile; decode the supported big- and little-endian representations according to the encapsulation header and reject unsupported representations. Check layout against ROS2 Jazzy serialization in CI. No LCM fingerprint or DimOS schema-hash prefix is part of the CDR payload.

Provide transport-neutral Python `encode()` and class-level `decode()` operations, equivalent generated codec operations in C++ and Rust, and immutable type/schema metadata. Keep generated wire types free of visualization or geometry dependencies. Nested fields and sequences must have documented mutation semantics in Python; do not rely on pybind11's implicit STL copies as if they were live views.

**Alternatives considered:** handwritten codecs, a dynamic Python-only codec system, and a single FFI runtime for all languages. Handwritten layouts recreate the maintenance bottleneck; Python-only generation misses the native-language requirement; sharing the C++ backend with Python while keeping native Rust limits FFI complexity.

### 3. Standard package systems distribute source schemas and generated artifacts

Use one documented local generation entry point for all outputs. Application-local and reusable packages use the same definition layout and resolver; publishing is optional. Output generated sources to ignored build directories.

Distribute Python wheels/sdists, an installable CMake package, and Rust crates through their ordinary tooling. Include source `.msg` resources and schema metadata in the applicable packages. Rust consumers compile crate sources normally; this design does not promise portable precompiled Rust binaries. Source distributions contain the inputs/build dependencies needed for reproducible generation; wheels contain usable compiled bindings.

Use the existing Python packaging/native-build infrastructure and release matrix. CI runs the same generator, checks deterministic output, builds packages, and supplies build artifacts for review before release publication. Adding schemas to an external application must not require editing DimOS's built-in type registry. Discover installed Python message/schema providers through a `dimos.messages` entry-point group; ordinary known-type use continues to use imports.

Install dependencies at environment setup or build time. Runtime never downloads schemas or invokes a generator to discover a type. Generation with all dependencies installed must work without network access.

### 4. Replace message APIs and their consumers together

Use the actual ROS2 field shapes: `Header.stamp.sec/nanosec`, no ROS1 `Header.seq`, nested `PoseStamped.pose`, and separate single `TransformStamped` versus `TFMessage.transforms`. Represent compressed images as `CompressedImage`, not JPEG bytes disguised as a raw `Image`. Preserve arbitrary `PointCloud2` fields, offsets, strides, and endianness when crossing the wire.

Move geometry operations, timestamp extraction, NumPy/Open3D conversion, and Rerun mappings into separate modules. Update callers rather than reintroducing old convenience methods on generated types. Timestamp-dependent buffers and recorders use explicit extractors; unstamped messages use reception time where required.

NumPy helpers return read-only borrowed views when the message layout can be represented without copying, retaining the storage owner for the view lifetime. Reject operations that would resize or invalidate backing storage while a view is borrowed. An explicit copy path supports mutable processing. Layouts that require conversion must document and test the copy; never mislabel an allocating conversion as zero-copy. Stage 1 measures representative image/point-cloud buffers and establishes a reproducible performance baseline, without inventing a throughput target not supplied by the user.

Update typed pub/sub, native SDKs, SHM typed paths, WebRTC/browser paths, tools, existing ROS bridge conversions, robot/perception/navigation consumers, and examples. Raw-byte transport APIs and separate Python object/pickle paths are not redefined by this change. Audit those paths to remove any actual dependency on LCM message generation without broadening the task into a new object-serialization design.

### 5. Keep transport routing separate from the CDR payload

LCM and Zenoh continue to carry opaque payload bytes; typed layers select generated CDR codecs. Preserve topic/type routing using qualified message identity where needed, without adding runtime hash enforcement. Multiplexed browser/WebRTC channels must route by explicit channel/type metadata; attempting multiple CDR decoders until one succeeds is not a valid replacement for LCM fingerprint filtering.

Reuse the browser's existing schema metadata distribution path with ROS2 definitions and a CDR decoder. External message authors do not generate handwritten TypeScript decoders per message. Rerun conversion is handled by explicit adapters outside generated types.

For Rust's raw LCM transport, first inspect the existing external implementation and its license/provenance. Prefer moving the narrowly required, reusable transport implementation into a separate in-tree Rust transport crate, preserving notices, rather than rewriting the protocol. Keep it independent of message schemas and code generation. If that cannot be done under its license or build constraints, resolve a maintained raw transport replacement during stage 1. Test native/Python communication and fragmented large payloads before depending on the replacement. This is an early feasibility gate, not a final-stage surprise.

### 6. MCAP carries the schemas needed by viewers

Write standard MCAP ROS2-profile channels with message encoding `cdr`, schema encoding `ros2msg`, qualified schema names, and the complete concatenated dependency definitions. Keep the stored payload exactly the generated CDR message. Use MCAP chunk compression rather than wrapping channel payloads in a custom compression codec. Compressed image data remains a standard `CompressedImage` message.

Feed schema metadata to the recorder through its existing stream startup configuration. Recording an arbitrary custom type requires no recorder-specific generated decoder or rebuild. Existing recognized-type optimizations must preserve the declared schema and message semantics. Use source stamps for supported stamped messages and reception time for otherwise unstamped messages, with recording log time always recording reception.

First prove viewer interoperability using a standalone producer and recording, then connect the actual recorder and replay paths. Both Foxglove and the repository's pinned Rerun version must inspect custom fields using only embedded definitions. Standard image and pose messages must have semantic visualization; arbitrary custom messages need only field inspection unless an explicit visualization adapter exists. Preserve existing live Rerun behavior; do not add a live Foxglove server in this stack.

### 7. A cumulative demo is part of every PR

Use one small hardware-free example project throughout: a custom telemetry message referencing a standard type, synthetic image data, and a moving pose. Early terminal demos show actual fields and mutations; later stages show the same data through packages, transports, viewers, and recording/replay. Name Python demo scripts `demo_*.py` to avoid pytest collection.

Every PR description includes setup, exact commands, expected visible results, teardown, limitations, test results, and a terminal capture or screen recording. Persist useful outputs as CI artifacts. Automated tests are mandatory and never substitute for the human demo. Avoid building a new demo framework; scripts and a concise README suffice.

## Risks / Trade-offs

- **No verified turnkey three-language generator** → Stage 1 is a real production-capable generation/codec gate. Do not cut over the application until the full dependency and conformance checks pass.
- **Python binding copies or unsafe lifetimes** → Exercise nested mutations, sequence edits, read-only buffers, owner lifetime, and realistic image/point-cloud sizes in stage 1, then preserve those checks through packaging and runtime integration.
- **A large consumer refactor** → Inventory consumers early. Keep the public type switch and affected callers in a working change; split stage 4 only along independently runnable boundaries with their own demos, not into broken intermediate commits.
- **MCAP parses but is not useful in a viewer** → Human acceptance requires visible standard-message visualization and custom-field inspection in both viewers, using the same file.
- **Runtime schema mismatches remain possible** → Build-time definition conflicts fail. Runtime schema-hash enforcement is explicitly deferred; do not claim decoding or routing detects all same-name schema conflicts. RIHS01 is the agreed future direction, outside this acceptance gate.
- **Rust transport source remains tied to the old repository** → Inspect and prove the removal route in stage 1 and verify a clean final build with external repository access unavailable.
- **Existing recordings and clients break** → This is an intentional clean replacement. Document the break; do not add fallback readers or migration tooling.

## Migration Plan

Deliver a linear PR stack. The bottom branch targets `main`; each later PR is reviewed against its predecessor and ultimately lands on `main` in dependency order. No branches, PRs, releases, or pushes are created by this proposal task.

| Stage | Outcome | Required human demo |
| --- | --- | --- |
| 1. Generate and exchange | All three languages generate from one definition and use compatible CDR; prove the Rust raw transport exit path early | File-based Python → C++ → Rust → Python relay, printing and changing actual nested/custom fields; demonstrate realistic buffer conversion behavior |
| 2. Package and consume | CI and local builds produce usable standard packages | A separate application adds a field, regenerates, builds all three consumers, and shows the new field in the relay without editing DimOS |
| 3. Inspect recordings | Standalone generated messages produce self-contained MCAP | Open one image/pose/custom-telemetry MCAP in Foxglove and Rerun, with no installed custom message package in either viewer |
| 4. Run DimOS | Generated types and CDR replace runtime message APIs and affected consumers | Hardware-free Python/C++/Rust blueprint over LCM and then Zenoh, with live Rerun and browser decoding |
| 5. Record and replay | Actual recorders/readers use the new representation | Record the blueprint including a custom type, stop it, inspect in both viewers, then replay through DimOS |
| 6. Remove old dependency | No build/runtime dependency on `dimos-lcm` or its message encodings | Clean install/build with external repository access unavailable, dependency inventory, and the full custom-message-to-recording workflow |

Stages 1–3 establish the new generation workflow in standalone examples before the runtime cutover. This temporary development sequence is not a shipped compatibility layer: migrated runtime paths have a single CDR implementation. Update or replace repository fixtures and their consumers as the corresponding path changes. Stage 4 must include the minimum new-format recorder/reader and replay adaptations required by its type switch; it must not leave existing consumers broken until stage 5. Stage 5 completes generic schema propagation and the integrated recording/viewer/replay workflow, reusing stage 3's MCAP support. Move prerequisite work earlier if the consumer inventory reveals another dependency. All previous demos remain runnable at each accepted stage.

Rollback is source/release rollback as a unit; old and new processes or recordings are not promised interoperability. Retain demo artifacts as review evidence, not as a promise to decode the obsolete format.

## Engineering gates

No outstanding product decisions require another user interview. The stage-1 checks below are resolved by the implementation evidence that follows; CI repeats the executable checks:

1. Exact pinned parser, C++ generator/Fast CDR, pybind11, and Rust codec versions that pass the complete `.msg` and CDR conformance suite without ROS installed.
2. Measured Python buffer-copy and owner-lifetime behavior, with any unavoidable copies documented.
3. The license/provenance and build viability of extracting Rust's existing raw LCM transport, or the concrete maintained replacement if extraction is unsuitable.
4. The consumer inventory and any necessary subdivision of stage 4 into independently runnable PRs, each retaining the demo requirement.

## Implementation inventory and evidence

The initial source inventory found 185 files under `dimos` with direct LCM-codec/generated-type references, 8 native SDK files, 2 native examples, and the browser SDK decoder. A broader scan finds 690 Python files importing `dimos.msgs`; this includes tests and indirect convenience-API users, so the direct codec count is not the size of the full cutover.

| Consumer group | Required cutover |
| --- | --- |
| Message classes and helpers | Replace generated/wrapped types, dynamic lookup, timestamp inheritance, flattened poses, and rich geometry/image/cloud methods |
| Python transports and workers | CDR codec selection on LCM/Zenoh and typed SHM; worker serialization and explicit WebRTC type routing |
| C++ and Rust modules | Generated native types, codec adapters, transform streams, hardware/perception producers, and examples |
| Navigation/perception/robot code | Constructors, geometry operators, image/cloud conversion, timestamp access, detector messages, and standard primitive imports |
| Recorders and readers | Python and Rust MCAP/SQLite paths, image codecs, timestamps, transform collections, replay, and fixtures |
| Live viewers and browser | Rerun adapters, cockpit schema export, browser CDR decoder, relay codecs, and WebXR/phone/hosted teleoperation |

Bundled upstream packages now cover builtin, standard, geometry, sensor, navigation, trajectory, visualization, shape, TF2, and vision definitions. Existing nonstandard messages also require in-tree definitions: JointCommand, RobotState, MotorCommandArray, ImuInfo, TrajectoryStatus, grasp candidates, bounding-box arrays, entity markers, and weighted line segments. Use a DimOS-owned schema namespace for nonstandard messages rather than redefining standard ROS types. Replace the current custom `TrajectoryPoint`/`JointTrajectory` layouts with the actual standard joint trajectory definitions and explicit time conversion at callers. The current line-segment decoder's overloaded Path/orientation representation requires an explicit generated line-segment message and matching native producer change.

Keep stage 4 as a coordinated public type/caller cutover unless a dependency-closed working subdivision is proven. Separate helper extraction can precede the switch only if it has its own runnable demo and does not introduce compatibility methods on the generated classes.

Initial executable evidence is the `examples/message-codegen` relay: sequence 40 and hops `[1]` enter C++, C++ adds hop 2, Rust adds hop 3, and Python receives sequence 42 with the nested reading/position/arrays intact. A generated PoseStamped also matched an independent Jazzy codec byte-for-byte. See the example README and `scripts/test_message_codegen.sh` for reproducible commands. Local stage-1 verification built 139 bundled types and three demo types, passed 22 Python tests plus nine Rust transport tests and its doctest, exercised all nine language combinations in both byte orders, and checked every generated type against the independent Jazzy codec. An isolated Jazzy container independently generated the demo types and decoded all three native implementations' payloads. ROS alignment padding can be unspecified, so ROS comparisons assert every decoded field; the rosbags oracle checks deterministic native bytes. Jazzy's Python generator emitted invalid code for an explicit empty sequence default, so the demo uses the equivalent implicit empty default; our parser still tests explicit empty defaults.

The raw Rust LCM implementation is extracted into `native/rust/dimos-lcm-transport` from external revision `04d78e8622500244123ba9cefa4c51b4cb454549`, preserving its Apache-2.0 provenance. It has no generated-message dependency. The Python/native duplex demo verified 128-byte, 64 KiB, and fragmented 1 MiB payloads exactly; receive buffers are 4 MiB to accommodate the large burst. Integration into the native SDK remains stage 4.

Primitive sequences and nested message fields are live. Nested sequence elements are returned by value and must be assigned back after edits, avoiding references invalidated by vector resizing. Numeric `.view()` returns read-only NumPy storage retaining the root owner; borrowed views block resizing or replacement of that owner's nested storage. `.copy()` explicitly produces independent writable storage. Local 30-iteration medians were 4.2/31.6/71.4/74.2 microseconds for borrowing/copying/encoding/decoding a 921,600-byte RGB image and 5.7/47.0/119.3/128.2 microseconds for a 1,200,000-byte cloud. These are local measurements, not portable performance promises. Lifetime, multiple-borrow, mutation rejection, fixed-array edits, and worker pickle behavior have regression tests.

The generation entry point is `python -m dimos.message_codegen.generate`. It reads bundled/local sources only; setup downloads native dependencies separately. Generated sources stay under ignored `build/`. The host has no ROS installation, the generated Rust crate builds with Cargo's offline flag after setup, and `.github/workflows/message-codegen.yml` separates standalone builds from the independent ROS reference container. Release packaging and installed external providers remain stage 2.

## Reference material

These references informed exploration; pinned versions and executable conformance evidence, not current web documentation alone, determine acceptance.

- [ROS2 interface definition format](https://design.ros2.org/articles/legacy_interface_definition.html)
- [Jazzy common interfaces](https://github.com/ros2/common_interfaces/tree/jazzy) and [Jazzy runtime interfaces](https://github.com/ros2/rcl_interfaces/tree/jazzy)
- [Fast DDS generator](https://fast-dds.docs.eprosima.com/en/3.x/fastdds/dds_layer/topic/fastddsgen/fastddsgen.html) and [Fast CDR](https://github.com/eProsima/Fast-CDR)
- [pybind11 STL conversion semantics](https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html)
- [Rust re_cdr](https://docs.rs/re_cdr/latest/re_cdr/)
- [MCAP schema and encoding registry](https://mcap.dev/spec/registry)
- [Foxglove custom schema encodings](https://docs.foxglove.dev/docs/getting-started/custom/custom-schema-encodings)
- [Rerun 0.32 MCAP message formats](https://github.com/rerun-io/rerun/blob/0.32.0/docs/content/concepts/logging-and-ingestion/mcap/message-formats.md)

### Stage 2 packaging evidence

`--package` emits a standard setuptools project with native bindings, schema
closure, notices, and a `dimos.messages` provider. Its sdist contains the pinned
generator and inputs; the wheel contains compiled code and schema resources.
CMake exports a relocatable target and headers/schema installation; Cargo packages
the native source crate and schemas. Per-definition C++ guards allow two packages
to include the same standard types without duplicate definitions; conflicting
layouts still fail rather than silently overriding one another.

The external application adds `application_note` in its own copied definition,
builds a wheel from the sdist, installs it into an isolated environment, installs
CMake packages, and builds a Rust consumer from the `.crate` archive. The field
returns as `added-locally/cpp/rust`. A second clean environment installed the full
DimOS wheel and external wheel together: 141 types were discovered, the external
schema resolved through installed providers, and a standard Point from the built-in
package was accepted by the external native Telemetry type. The full DimOS wheel
includes its existing web assets; the sdist content check passed at 10.7 MB.

The official setuptools build invokes the same generator and native build helper.
Release CI retains its existing platform/Python matrix, builds the pinned Fast
CDR dependency before native extensions, and adds CMake/Cargo source artifacts to
GitHub releases. PyPI jobs download only Python distribution artifacts. Ordinary
wheel users do not run setup downloads, message generation, or a ROS build.
Source developers install Fast CDR during explicit environment setup, including
the existing developer installer. Release publishing itself remains tied to the
normal release workflow; this implementation has not published a release.
