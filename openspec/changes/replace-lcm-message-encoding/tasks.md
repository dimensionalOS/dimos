## 1. PR 1 — Generate and exchange messages without ROS

- [x] 1.1 Inventory message definitions and consumers across Python, C++, Rust, recordings, SHM, WebRTC/browser, live Rerun, and the existing ROS bridge; record required standard/custom types and any working subdivisions of the runtime cutover in design.md.
- [x] 1.2 Inspect the external Rust raw LCM implementation and license; prove an extraction or maintained-replacement route with native/Python raw-byte and fragmented-payload checks, and record the choice before advancing the stack.
- [x] 1.3 Pin and vendor the required Jazzy `.msg` dependency closure with immutable upstream revisions, licenses, and notices.
- [x] 1.4 Implement standalone package resolution and parsing, including qualified names, transitive dependencies, defaults/bounds, conflict rejection, and source diagnostics for unsupported or invalid definitions.
- [x] 1.5 Validate and pin the shared-definition C++ emitter/Fast CDR pipeline without ROS or DDS runtime requirements; generate buildable types and encapsulated XCDR1 codecs rather than handwritten message layouts.
- [x] 1.6 Generate pybind11 bindings sharing the C++ codecs, with transport-neutral encode/decode, nested-field/sequence mutation semantics, and type/schema metadata.
- [x] 1.7 Validate and pin the native Rust generation/CDR backend, emit equivalent types/metadata/codecs, and document any rejected candidates with evidence in design.md.
- [x] 1.8 Implement and verify read-only image/point-cloud buffer views, explicit mutable copies, owner lifetimes, and rejection of storage-invalidating mutations while views exist; capture representative buffer performance and copying behavior.
- [x] 1.9 Add all nine language encoder/decoder conformance combinations, endian/alignment cases, defaults, bounds, fixed/dynamic arrays, nested standard/custom messages, malformed inputs, and independent ROS2 Jazzy reference checks in a separate CI job.
- [x] 1.10 Provide one documented generation entry point and an ignored output directory; demonstrate generation/build/runtime with ROS absent and preinstalled dependencies available offline.
- [x] 1.11 Create the cumulative example definitions and a runnable `demo_` file relay: Python → C++ → Rust → Python, printing decoded fields and each process's edits; include realistic buffer-view behavior in its terminal demo.
- [x] 1.12 Add exact demo setup/run/teardown instructions, capture visible output and automated results for PR review, and resolve the stage-1 engineering gates in design.md before accepting PR 1.

## 2. PR 2 — Package and consume messages outside DimOS

- [ ] 2.1 Add Python wheel/sdist generation using the existing native-build infrastructure; include schema resources and the inputs needed to reproduce source builds.
- [ ] 2.2 Add an installable/exported CMake message package and a consumable generated Rust crate with schema metadata and dependency declarations.
- [ ] 2.3 Register installed Python schema/type providers through `dimos.messages`; support external packages without editing the built-in message registry.
- [ ] 2.4 Wire the same pinned local generation into CI and release packaging, preserving the supported build matrix; upload review artifacts and publish official packages through the normal release workflow.
- [ ] 2.5 Verify deterministic generated sources/metadata, packaged schema closure, clean wheel/sdist/CMake/Cargo consumers, and absence of generated source from version control.
- [ ] 2.6 Extend the demo into a separate application: install built packages, add a field to a local custom `.msg`, regenerate/build all three consumers, and visibly exchange the new field without changing DimOS or publishing a package.
- [ ] 2.7 Document exact external-project commands and setup-time dependencies, verify no runtime downloading/generation, capture demo evidence, and rerun PR 1's demo and automated checks.

## 3. PR 3 — Inspect the same MCAP in Foxglove and Rerun

- [ ] 3.1 Generate complete concatenated `ros2msg` definitions and qualified schema metadata for every demo type, including transitive standard and custom dependencies.
- [ ] 3.2 Add reusable MCAP writing support for the ROS2 profile, `cdr` channels, embedded `ros2msg` schemas, and MCAP chunk compression without private payload wrappers.
- [ ] 3.3 Extend the example with a deterministic synthetic image, moving pose, and custom telemetry producer writing generated CDR directly to one MCAP artifact.
- [ ] 3.4 Add automated checks for schema dependency closure, channel encodings, independently decodable payloads, source/log timestamps, and standard compressed-image representation.
- [ ] 3.5 Open the same artifact in Foxglove and pinned Rerun with the custom message package absent from the viewer environment; demonstrate image/pose visualization and custom-field inspection, recording exact UI steps and screen evidence.
- [ ] 3.6 Publish the demo MCAP as a review artifact, document reproduction and cleanup, and rerun all preceding demos and automated checks.

## 4. PR 4 — Run DimOS on generated messages and CDR

- [ ] 4.1 Generate the complete inventory of standard and DimOS custom messages; replace the message protocol and generated-type discovery with the transport-neutral codec and schema contract.
- [ ] 4.2 Extract geometry, timestamp, NumPy/Open3D, and Rerun helpers; update callers to nested ROS2 fields, sec/nanosec timestamps, and distinct TransformStamped/TFMessage representations without compatibility wrappers.
- [ ] 4.3 Update Python typed LCM/Zenoh pub/sub and typed SHM paths to CDR, preserving raw transport and separate Python-object serialization behavior.
- [ ] 4.4 Update C++ and Rust SDK codecs, native examples, and typed worker/module configuration to the generated types; integrate the proven Rust raw transport replacement with preserved provenance.
- [ ] 4.5 Update image and point-cloud producers/consumers to generated fields, proper CompressedImage use, arbitrary PointCloud2 layouts, and explicit conversion helpers.
- [ ] 4.6 Update remaining robot/perception/navigation/tool consumers and existing optional ROS bridge conversions; replace affected repository fixtures rather than retaining LCM fallback decoding.
- [ ] 4.7 Adapt existing recorder/reader and replay consumers minimally to the new message APIs/codecs in this PR so the type cutover does not leave them broken pending PR 5.
- [ ] 4.8 Move live Rerun mappings outside generated types; update browser schema advertisement, CDR decoding, and explicit multiplexed channel/type dispatch without fingerprint probing or runtime hash enforcement.
- [ ] 4.9 Add integration checks for three-language typed streams on both transports, large fragmented LCM messages, helper semantics, browser multiplexing, existing consumer behavior, and live viewer adapters.
- [ ] 4.10 Extend the human demo to a hardware-free Python/C++/Rust blueprint over LCM and then Zenoh; show changing custom fields, live Rerun image/pose updates, and browser decoding; capture terminal and screen evidence.
- [ ] 4.11 Run affected automated checks and all earlier demos; keep any inventory-driven subdivision independently runnable with its own human demo and tests.

## 5. PR 5 — Record and replay the running system

- [ ] 5.1 Pass complete generated stream schemas to recorder startup configuration and register MCAP schemas/channels using PR 3's reusable support.
- [ ] 5.2 Support arbitrary installed custom messages without recorder-specific generated decoders or rebuilds; preserve declared schema semantics in recognized-type image/transform processing.
- [ ] 5.3 Complete Python/Rust recorder and reader changes, including new-format SQLite codec paths where applicable, stamped/unstamped timing behavior, and removal of obsolete LCM storage wrappers.
- [ ] 5.4 Complete typed replay with matching installed message packages and verify recorded fields, ordering, and existing replay timing semantics.
- [ ] 5.5 Add end-to-end capture/read/replay checks for standard and external custom messages, schema closure, timestamp behavior, compressed/raw images, and both transport inputs.
- [ ] 5.6 Demonstrate recording the cumulative blueprint, stopping producers, opening its MCAP in both viewers, and replaying through DimOS with visibly matching image/pose/telemetry output.
- [ ] 5.7 Capture the recording and screen/terminal evidence, document exact steps and cleanup, and rerun all preceding demos and automated checks.

## 6. PR 6 — Remove the external repository dependency completely

- [ ] 6.1 Remove remaining `dimos-lcm`/`lcm-msgs` dependencies, native header fetches, build/lockfile references, obsolete LCM message codecs, old convenience APIs, and unused generation assets; retain raw LCM transport.
- [ ] 6.2 Remove remaining legacy-format fixtures/readers and update all message-authoring, native-module, transport, viewer, recording, and release documentation to the supported workflow.
- [ ] 6.3 Add a dependency audit and clean build/install check with old repository access unavailable, ensuring no preexisting generated artifacts or dependency caches mask an active dependency.
- [ ] 6.4 Run the supported packaging/build matrix, full affected regression checks, and all cumulative demos against the final installed artifacts.
- [ ] 6.5 Demonstrate the full clean-environment workflow: add a local message, generate/install all three outputs, exchange it on both transports, record/replay it, and inspect the MCAP in both viewers; show the dependency inventory.
- [ ] 6.6 Attach final human demo evidence and automated results, document the intentional API/wire break and deferred features, and verify every PR's acceptance requirements before marking the change implemented.
