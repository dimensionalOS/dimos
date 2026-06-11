## Why

DimOS image streams currently use full `Image` objects over typed transports and memory2 stores images as independent JPEG payloads. That is simple and compatible, but inefficient for long-running camera streams and remote subscribers because each frame is compressed independently and no shared video codec state is reused.

DimOS needs an opt-in H.264 image-stream path that preserves the public `Image` stream contract while allowing live transports and memory2 storage to carry compact video frame packets. The design should make H.264 reusable across carriers such as LCM first, and DDS/WebRTC later, while keeping memory2 queries, pose/tag alignment, and replay frame semantics intact.

## What Changes

- Add a carrier-neutral H.264 image packet behavior for RGB/BGR-style `Image` streams, with one encoded video access unit per source frame.
- Add stateful H.264 encode/decode behavior that produces periodic self-contained keyframes, rejects unsupported image formats clearly, detects sequence gaps, and resumes delivery only after a valid keyframe.
- Add an opt-in live transport path for H.264 image streams, starting with LCM, that exposes decoded `Image` objects to subscribers rather than video packets.
- Add memory2 H.264 image storage that preserves one observation row per frame, stores per-frame video packet payloads, indexes GOP/keyframe relationships, and lazily reconstructs `obs.data` as an `Image` on demand.
- Preserve the existing JPEG image codec and JPEG-backed memory2 storage as the default behavior.
- No hardware-safety behavior changes are intended.
- No public robot-control, skill, or MCP breaking changes are intended.

## Affected DimOS Surfaces

- Modules/streams: typed `Image` streams, image-specific transport adapters, memory2 Recorder ingestion, memory2 Stream/Observation lazy payload access, and replay output of decoded images.
- Blueprints/CLI: blueprints may opt image streams into H.264-capable transports or memory2 H.264 storage; existing blueprint behavior remains unchanged unless configured.
- Skills/MCP: no direct skill or MCP behavior changes expected.
- Hardware/simulation/replay: camera-heavy hardware and simulation streams may benefit from reduced bandwidth/storage; replay must continue to emit normal decoded `Image` frames on the same schedule.
- Docs/generated registries: memory2 and transport docs need updates; generated blueprint registries are not expected to change unless new demo blueprints are added.

## Capabilities

### New Capabilities

- `h264-image-streams`: Covers carrier-neutral H.264 image packets, live image-stream encode/decode behavior, keyframe/GOP handling, sequence-gap behavior, and transport compatibility expectations.
- `memory2-h264-storage`: Covers opt-in H.264-backed memory2 image observation storage, per-frame packet persistence, GOP indexing, lazy `Image` reconstruction, and replay compatibility.

### Modified Capabilities

- None.

## Impact

Users and developers gain a more bandwidth- and storage-efficient option for camera streams while keeping existing `Image` stream consumers and memory2 query/replay behavior familiar. Existing JPEG-backed recordings, default transports, and non-image streams remain compatible.

Compatibility risk centers on adding optional video codec dependencies, preserving lazy-load lifetimes, making GOP recovery deterministic after packet loss or missing storage rows, and avoiding silent corruption when frames cannot be decoded. Documentation and QA should cover opt-in configuration, supported image formats, dependency installation, LCM live-stream behavior, memory2 append/query/lazy-decode/replay behavior, packet-loss recovery, and a small synthetic image-stream demo.
