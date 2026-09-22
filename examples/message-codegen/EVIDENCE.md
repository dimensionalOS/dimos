# Stage 1 local verification

Captured on 2026-09-21 on Linux x86_64 with Python 3.12, GCC 16, and Rust 1.98.
These are terminal demos intended for human inspection, executed automatically.
The ROS reference runs independently in a Jazzy container.

The full build generated 142 message types. Focused verification after adding
the all-types oracle passed **22 tests**. The raw transport passed nine unit tests
and one doctest. `cargo build --offline` also passed after dependency setup.

## relay

```text
Python sends: sequence=40 label=start hops=[1]
C++ received: frame=map sequence=40 label=start temperature=21.5 x=4.25 axes=[1,2,3]
Rust received: frame=map sequence=41 label=start/cpp temperature=21.5 x=4.25 axes=[1.0, 2.0, 3.0]
Python receives: sequence=42 label=start/cpp/rust hops=[1, 2, 3]
Nested fields, fixed arrays, bytes, defaults, and edits survived. Evidence: build/message-codegen/demo/evidence
```

## conformance

```text
LE Python -> Python: fields and bytes match
LE Python -> C++   : fields and bytes match
LE Python -> Rust  : fields and bytes match
LE C++    -> Python: fields and bytes match
LE C++    -> C++   : fields and bytes match
LE C++    -> Rust  : fields and bytes match
LE Rust   -> Python: fields and bytes match
LE Rust   -> C++   : fields and bytes match
LE Rust   -> Rust  : fields and bytes match
BE Python -> Python: fields and bytes match
BE Python -> C++   : fields and bytes match
BE Python -> Rust  : fields and bytes match
BE C++    -> Python: fields and bytes match
BE C++    -> C++   : fields and bytes match
BE C++    -> Rust  : fields and bytes match
BE Rust   -> Python: fields and bytes match
BE Rust   -> C++   : fields and bytes match
BE Rust   -> Rust  : fields and bytes match
Python/C++/Rust reject truncated
Python/C++/Rust reject unsupported representation
Python/C++/Rust reject trailing bytes
Python/C++/Rust reject missing header
All nine encoder/decoder combinations and both input byte orders match the independent ROS2 codec.
```

## buffers

```text
640x480 RGB image: 921,600 bytes, shared read-only view; resize rejected
  local median (microseconds): borrow=4.2, copy=31.6, encode=71.4, decode=74.2
100,000-point buffer: 1,200,000 bytes, shared read-only view; resize rejected
  local median (microseconds): borrow=5.7, copy=47.0, encode=119.3, decode=128.2
Borrowed image remains readable after its message variable is released.
```

## transport

```text
Python -> native Rust -> Python: 128 raw bytes match
Python -> native Rust -> Python: 65536 raw bytes match
Python -> native Rust -> Python: 1048576 raw bytes match
```

## jazzy-reference

```text
ROS2 Jazzy decoded python-le.cdr: every field matches
ROS2 Jazzy decoded python-be.cdr: every field matches
ROS2 Jazzy decoded C++.cdr: every field matches
ROS2 Jazzy decoded Rust.cdr: every field matches
ROS2 Jazzy confirms declared defaults, nested fields, bounds, arrays, Unicode, and both byte orders.
```

Reproduce with the commands in [README.md](README.md). CI retains fresh output
and binary payloads as artifacts; generated sources and build products are ignored.

## Stage 2 installed application

Built a Python wheel from its sdist, installed CMake packages, and compiled a
separate consumer from the Cargo archive. The Python consumer ran with `-I` in a
fresh environment containing only the external message wheel.

```text
Installed Python package sends new field: added-locally
Installed C++ package received: added-locally
Packaged Rust crate received: added-locally/cpp
Installed Python package receives: added-locally/cpp/rust
A locally added field crossed three installed native packages; no DimOS source change or upstream PR.
```

The full DimOS wheel also built with web assets. Its sdist passed the repository
content check (1,928 entries; 10.7 MB). A fresh environment with both wheels
discovered 141 types and accepted a built-in Point inside external Telemetry.
Two separately installed CMake packages can include the same standard Point
without duplicate definitions. Focused tests now pass 34 cases.

## Stage 3 automated recording checks

`bash scripts/test_message_mcap.sh` passed on 2026-09-21 with Rerun 0.32.0,
MCAP 1.4.0, and the locked Foxglove JavaScript packages. The artifact contains
150 messages and is 3,123,201 bytes, including all schema dependencies.

```text
4 passed
Foxglove libraries decoded 30 /camera/image messages from embedded definitions.
Foxglove libraries decoded 30 /camera/compressed messages from embedded definitions.
Foxglove libraries decoded 30 /robot/pose messages from embedded definitions.
Foxglove libraries decoded 30 /telemetry messages from embedded definitions.
Foxglove libraries decoded 30 /tf messages from embedded definitions.
Rerun imported 30 /telemetry rows as demo_msgs.msg.Telemetry:message.
Rerun imported 30 /camera/image rows as Image:buffer.
Rerun imported 30 /camera/compressed rows as EncodedImage:blob.
Rerun imported 30 /robot/pose rows as InstancePoses3D:translations.
Rerun imported 30 /tf rows as Transform3D:translation.
Custom telemetry is a structured Arrow value with nested fields.
```

The stage-1 conformance/buffer/raw-transport demos and stage-2 installed-package
demo were rerun after correcting dependency section names to `package/Type`.
The Rerun browser viewer displayed both images and exposed custom Telemetry's
`sequence: 29`, `reading.temperature: 22.9`, and `hops: [1, 2, 3]` from the MCAP.
Local screenshots are in `build/message-codegen/viewers/`.

Full viewer acceptance is still pending. Foxglove's application redirected to
sign-in in the isolated browser; no credentials were entered. Its decoding
libraries pass independently, but that does not establish application UI
acceptance. Rerun's direct-file screen capture also needs final confirmation
of pose visualization and complete semantic playback before task 3.5 is closed.

## Runtime transports and browser cutover (2026-09-21)

The runtime work is still in progress. This evidence covers typed transports,
web schema decoding, and the migrated built-in web adapters; it does not establish
that all DimOS consumers or native SDKs have completed the cutover.

- `demo_pubsub.py`: generated `LineSegments3D` and 921,600-byte `Image` exchanged
  over raw LCM, separate loopback TCP Zenoh sessions, and CPU shared memory.
  Decoded bytes and integer nanosecond timestamps matched on every path.
- Typed pub/sub pattern, core CDR transport, and CLI checks: 67 passed. A glob
  subscription regression was fixed: `/sensor/*` must match the logical topic
  before the slash-separated `#package/msg/Type` suffix.
- WebRTC plus core transport checks: 34 passed. Explicit data-frame channel/type
  routing distinguishes `Point` and `Vector3` despite their identical CDR layouts.
  Truncated, trailing, raw/unframed, foreign-type, foreign-channel, and wrong-codec
  frames are dropped; the next valid frame is delivered.
- Python web codecs: 48 passed. Cockpit authoring plus generated array/helper
  checks: 94 passed. Costmap and map codec checks: 17 passed.
- Browser SDK: 322 passed. Cockpit: 154 passed. Both TypeScript checks passed.
  The CDR decoder uses pinned Foxglove parser/serialization libraries. Fixtures
  cover standard and custom messages in both byte orders, raw images, fixed and
  dynamic arrays, empty messages, Unicode, and integers beyond JavaScript's safe
  number range. Former LCM codecs and their browser fixtures were removed.
- Chromium opened `demo_cdr.html` and displayed the generated custom message's
  endpoints, weight 4, and `stamp.nanosec = 500000123`. Switching to the image
  and big-endian CDR displayed both pixels. Reading the rendered canvas yielded
  RGBA `[0, 1, 2, 255, 253, 254, 255, 255]` as expected.
  Screenshots: ignored `build/message-codegen/viewers/cdr-browser-custom.png`
  and `cdr-browser-image.png`. Browser recording:
  `/tmp/dimos-cdr-browser-harness/agent-workspace/recordings/session-20260921-192504`
  (6 frames). The task-owned browser and Vite server were stopped afterward.

The combined Python runtime/web subset passed 318 tests before adding the explicit
WebRTC sequence regression; that WebRTC file then passed all 29 tests. The final
SDK and cockpit runs remained 322 and 154 tests, with both type checks passing.
Mypy passed on 12 changed runtime/helper/codec modules. The external packaging
relay and installed typing check also passed after making the bundled generator
a regular package so an installed DimOS cannot shadow it.

The subsequent relay consumer migration passed the entire `dimos/web` suite
plus the generated image/grid helper tests: **677 passed in 35.59 s**. This
includes forkserver blueprint deployment, real relay session/transport tests,
image and map delivery, cached-map replay after unsubscribing, relay respawn,
teleoperation stop/watchdog behavior, and lease handover. The watchdog's old
`Twist.zero()` call was replaced with the generated zero-valued `Twist()`.
The new array-to-image helper also passed strided input, independent copy,
explicit encoding, and big-endian depth tests.

### Native SDK CDR relay

The C++ SDK suite passed all **76 tests** using real generated messages in its
default-codec tests. The Rust SDK passed **143 tests**, including TF, both CDR
byte orders, malformed input, and propagation of publish encoding errors without
enqueuing a payload. C++ and Rust ping/pong examples and the custom/image relays
built successfully. The generated Rust crate also passed Cargo source-package
verification: a build from the packaged archive, with only Python's standard
library used for generation and no preexisting generated source.

The native demo used the real SDK processes, generated message packages, stdin
launch protocol, and Python typed transports:

```text
lcm: Python weight=4 → C++ weight=5 → Rust weight=6 → Python verified
lcm: 921,600 image bytes and source nanoseconds=1700000000123456789 match
zenoh: Python weight=4 → C++ weight=5 → Rust weight=6 → Python verified
zenoh: 921,600 image bytes and source nanoseconds=1700000000123456789 match
```

Full output and process logs are under `build/message-codegen/demo/evidence`.
This does not complete the native consumer, Nix build matrix, coordinator
blueprint, or live Rerun migration; stage 4 remains open.
