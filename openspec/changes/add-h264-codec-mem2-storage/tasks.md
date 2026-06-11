## 1. Core video packet and codec behavior

- [x] 1.1 Add the carrier-neutral encoded video frame message for one complete H.264 Annex B access unit per source `Image` frame, including sequence, timestamp, frame identifier, dimensions, format, codec, bitstream, keyframe, keyframe-reference, presentation timestamp, and payload fields.
- [x] 1.2 Add H.264 configuration covering bitrate, target FPS, keyframe interval, profile, preset/tune, maximum GOP length, and supported pixel format settings.
- [x] 1.3 Add the aiortc-backed H.264 adapter that converts `Image` frames to H.264 output and converts H.264 input back to `Image` while keeping aiortc/RTP internals out of public DimOS APIs.
- [x] 1.4 Add access-unit assembly so all NAL units emitted for one encoder input frame are stored or transmitted as one Annex B packet, not as individual RTP fragments.
- [x] 1.5 Add GOP/keyframe state tracking that detects sequence gaps, marks decoder state invalid, suppresses corrupted output, and resumes only after a usable keyframe.
- [x] 1.6 Add explicit errors for unsupported image formats, missing video dependencies, and unusable GOP/decode state.
- [x] 1.7 Add focused codec tests for per-frame Annex B packet shape, keyframe metadata, SPS/PPS bootstrap behavior, sequence-gap handling, dependency errors, and unsupported image formats.

## 2. Image lazy data support

- [x] 2.1 Extend `Image` construction/access so metadata fields remain available without forcing pixel decode when an image is backed by a lazy pixel loader.
- [x] 2.2 Preserve existing eager `Image` behavior and compatibility for current JPEG, LCM, SHM, memory2, and visualization consumers.
- [x] 2.3 Add tests proving eager images still work, lazy images expose metadata without decode, and `Image.data` materializes pixels exactly when requested.

## 3. Live H.264 image transport

- [x] 3.1 Add the H.264 LCM pubsub adapter that publishes encoded video frame packets on the wire and delivers decoded `Image` values to subscribers.
- [x] 3.2 Add `H264LcmTransport` to the transport layer with worker-safe serialization behavior matching existing transport patterns.
- [x] 3.3 Keep normal image transport behavior unchanged unless a blueprint explicitly opts a stream into H.264 transport.
- [x] 3.4 Add live transport tests for `Out[Image]` to `In[Image]` delivery, keyframe bootstrap, late subscriber behavior, sequence-gap recovery, and default transport compatibility.

## 4. memory2 H.264 image storage

- [x] 4.1 Add per-stream H.264 image storage configuration for direct store creation and recorder configuration.
- [x] 4.2 Route configured memory2 `Image` streams to H.264-backed storage while leaving unconfigured `Image` streams on the existing default storage path.
- [x] 4.3 Store one observation row per source frame and one encoded Annex B frame packet payload per observation.
- [x] 4.4 Add persistent GOP/keyframe index metadata for H.264-backed image streams.
- [x] 4.5 Persist and reload per-stream storage configuration so reopened stores recognize H.264-backed image streams.
- [x] 4.6 Add lazy observation loading that returns metadata without decode and reconstructs `Image` pixels from the nearest prior usable keyframe through the requested observation when `obs.data` is accessed.
- [x] 4.7 Add replay support that emits decoded `Image` values in observation timestamp order for H.264-backed streams.
- [x] 4.8 Add memory2 tests for append/query, metadata access without decode, keyframe and mid-GOP lazy decode, missing-GOP failure, store reopen, replay, default JPEG compatibility, and unsupported formats.

## 5. Synthetic end-to-end blueprint and manual QA surface

- [x] 5.1 Add `dimos/protocol/video/demo_h264_video_e2e.py` with a deterministic synthetic `Image` source, H.264 memory2 recorder, and decoded-frame probe.
- [x] 5.2 Configure the blueprint to exercise both live H.264 LCM transmission and H.264 memory2 storage through normal `Image` stream surfaces.
- [x] 5.3 Add probe status or logs that report received frame counts, dimensions, timestamp monotonicity, validation failures, and drop/recovery observations.
- [x] 5.4 Register the runnable blueprint as `demo-h264-video-e2e` if it is intended to be exposed through `dimos run`.
- [x] 5.5 Regenerate and verify `dimos/robot/all_blueprints.py` if the demo blueprint is registered.

## 6. Documentation

- [x] 6.1 Update user-facing transport docs with H.264 opt-in behavior, `Image` stream preservation, Annex B per-frame packets, keyframe/GOP recovery, unsupported formats, and dependency notes.
- [x] 6.2 Update blueprint docs with an H.264 image transport mapping example.
- [x] 6.3 Update memory2 docs with H.264 image storage configuration, one-observation-per-frame behavior, metadata query without decode, lazy `obs.data` decode, random access from keyframes, and replay behavior.
- [x] 6.4 Add docs for running and inspecting the `demo-h264-video-e2e` synthetic QA blueprint.
- [x] 6.5 Update contributor testing docs with video dependency setup, focused test targets, skip behavior when dependencies are unavailable, and blueprint-registry regeneration guidance.
- [x] 6.6 Update coding-agent docs if maintainers want the H.264/Foxglove packet-shape rule documented for future agent edits.

## 7. Verification

- [x] 7.1 Run `openspec validate add-h264-codec-mem2-storage --strict`.
- [x] 7.2 Run focused unit tests for H.264 codec/access-unit/GOP behavior.
- [x] 7.3 Run focused unit tests for lazy `Image` behavior.
- [x] 7.4 Run focused memory2 storage tests for H.264 append/query/lazy decode/reopen/replay/default compatibility.
- [x] 7.5 Run focused live transport tests for H.264 LCM round-trip and sequence-gap recovery.
- [x] 7.6 Run `uv run pytest dimos/robot/test_all_blueprints_generation.py` if the demo blueprint is registered.
- [x] 7.7 Run relevant docs validation, including `uv run doclinks` if available and `uv run md-babel-py run <changed-doc>` for executable markdown snippets.
- [x] 7.8 Manually run `dimos run demo-h264-video-e2e --daemon`, inspect logs/probe status, query the generated memory2 store without pixel decode, access `obs.data` for keyframe and mid-GOP observations, replay the stream, and verify sequence-gap recovery behavior.
