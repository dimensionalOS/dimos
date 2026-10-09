# Native message acceptance examples

The current proposal uses native rosbags values, upstream ROSIDL/FastRTPS source
builds and ros2msg/re_cdr. See the self-contained
[external-project walkthrough](../../docs/development/messages-external-project.md)
for message definitions, `dimos build`, Python installs and a real three-language
coordinator. Ordinary built-in Python message users install matching wheels;
C++/Rust preparation is explicit and source-only.

The [limitation register](../../docs/development/message-limitations.md) records
accepted malformed-input behavior and unsupported bounded schemas. It links the
original assertions to exact strict xfails. These are provisional non-blockers,
not proof that the decoder validates hostile input.

## Maintainer checks

Prepare matching backend and built-in packages and the documented language tools.
The scripts accept `DIMOS_CODEGEN_PYTHON` to select the build interpreter;
`DIMOS_RUNTIME_PYTHON` selects the full runtime environment for module/viewer demos.

```sh
bash scripts/test_message_codegen.sh
```

This runs native catalog and strict decoder cases and compiles/runs a separate
Python → C++ → Rust → Python ownership exchange. The external type imports the
same dependency-owned Header. MCAP checks reconstruct it from embedded schemas.
The evidence directory is `build/message-codegen/demo/evidence`; CI's independent
ROS2 Jazzy job decodes the three supported ownership payloads from that directory.
This is a file exchange; it is not native transport pub/sub.

With a wheelhouse containing the matching proposal distributions and their
ordinary Python build dependencies:

```sh
export DIMOS_MESSAGE_WHEELHOUSE="$PWD/build/message-codegen/ux-wheelhouse"
bash scripts/test_message_packages.sh
```

This verifies isolated source/wheel/editable installation and repository docs,
then builds a Rust crate and a relocatable C++ **source** archive and consumes it.
It does not build a precompiled SDK product or publish to a package registry.

The small module/file relay and the actual native transport coordinator are
separate checks:

```sh
bash scripts/test_message_user_story.sh
bash scripts/test_external_native_messages.sh
```

The first sends 20.5 through Python, C++, Rust and back as 23.5, retaining Header
and sequence fields. The second builds and runs the supported project from the
walkthrough with loopback Zenoh; it requires the full runtime and native SDK
source prerequisites. It does not operate hardware.

For a self-contained five-channel recording (Image, CompressedImage, PoseStamped,
TFMessage and the supported custom DeviceReading):

```sh
bash scripts/test_message_mcap.sh
```

The pinned Foxglove libraries decode all records and the native Rerun CLI imports
semantic and reflected fields. This is automated decoder acceptance, not a claim
of human visual review or Foxglove sign-in.

## Preserved historical fixtures

`demo_msgs/msg/Telemetry.msg`, the original relay/conformance sources and the
`external-app` / `external-native` fixtures preserve the earlier bounded/default
contract. They are not the supported native-library walkthrough and are not
silently changed to unbounded definitions. Native Telemetry generation remains
two explicit strict xfails (CDR-L09); its full legacy relay/viewer/default claims
must not be reported as passing. The old pybind container methods are not native
NumPy/value APIs. Historical evidence documents describe their recorded revision,
not the current implementation. Benchmark scripts are preserved and are not part
of these acceptance runs.
