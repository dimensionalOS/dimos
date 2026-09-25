## Why

Adding a DimOS message currently requires generated bindings from the external `dimos-lcm` repository. That makes an upstream PR part of normal application development. Users need to define, build, exchange, and inspect their own messages locally, without installing ROS or changing DimOS.

## What Changes

- Introduce one standalone generation workflow for ROS2 `.msg` definitions, producing Python, C++, and Rust types and CDR encode/decode implementations. Bundle pinned ROS2 Jazzy standard definitions and their dependency closure.
- Support application-local definitions and reusable message packages through normal Python, CMake, and Cargo tooling. Generate sources during local builds and CI; do not commit generated sources.
- **BREAKING** Replace handwritten/rich LCM-backed message classes with generated ROS-shaped types. Move geometry, timestamp, NumPy, and visualization behavior into separate utilities and adapters; update all consumers.
- **BREAKING** Replace LCM message encoding with ROS2-compatible CDR throughout typed messaging and recordings. Keep LCM and Zenoh as transports. Remove old wire decoders, compatibility aliases, and obsolete recording paths instead of supporting legacy data.
- Embed complete `ros2msg` schemas in CDR MCAP recordings so Foxglove and Rerun can inspect standard and custom messages without installed message packages. Preserve live Rerun and browser visualization.
- Eliminate all dependencies on the `dimos-lcm` repository, including the Rust raw transport dependency, while retaining LCM transport functionality.
- Deliver six dependent PR stages, each with a runnable human-facing demo, automated tests, and review evidence. Keep all earlier demos working.
- Defer runtime schema-hash enforcement, ROS CLI/network integration, service/action generation, and a new live Foxglove bridge.

## Capabilities

### New Capabilities

- `standalone-message-generation`: ROS-free definition resolution and generated Python/C++/Rust types and compatible CDR codecs.
- `message-package-distribution`: Local and reusable schema packages, installed artifacts, and reproducible CI generation.
- `cdr-typed-messaging`: Generated ROS-shaped types throughout DimOS, transport-independent CDR, separate helpers, and removal of the external repository dependency.
- `self-describing-message-recordings`: Embedded-schema CDR MCAP capture, viewer inspection, and DimOS replay.
- `demonstrable-message-delivery`: Dependent PR stages with cumulative human-facing demos and automated acceptance coverage.

### Modified Capabilities

None. This checkout has no existing OpenSpec capability specifications; the new specifications describe the replacement behavior.

## Impact

Affected areas include `dimos/msgs`, typed pub/sub and transports, Python module consumers, C++ and Rust SDKs/examples, timestamp and geometry helpers, live Rerun, browser decoding, memory recorders/readers, build manifests, CI, and documentation. Existing ROS bridge consumers must be updated to the generated field shapes; this does not add a new ROS interoperability objective.

The intended implementation uses Fast CDR-backed C++ codecs and generated pybind11 bindings, plus generated native Rust types with a CDR backend. Stage 1 must validate and pin the standalone generation backends before the application cutover. Native compilation is acceptable for local custom Python message builds. ROS is allowed only in optional integration environments and the CI conformance job, not as a requirement for generation, installation, or ordinary runtime use.
