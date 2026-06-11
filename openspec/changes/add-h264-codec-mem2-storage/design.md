## Context

DimOS transports currently move typed stream payloads and are mostly stateless per message. Image-specific compression already exists as JPEG transport adapters: `JpegLcmTransport` and `JpegShmTransport` wrap a carrier with an image encoder/decoder while subscribers still receive `Image` objects. This is the right precedent for H.264, but H.264 differs because decoding depends on GOP state rather than one independent compressed frame.

memory2 currently stores image observations through the normal `Backend` path. `codec_for(Image)` selects `JpegCodec`, which is stateless per row. `Observation` already supports lazy payloads through `_UNLOADED` and `_loader`, which is the correct surface for memory2 H.264 decode. The current `Codec.encode(value) -> bytes` contract is not expressive enough for H.264 writes because the encoder is stateful, keyframes are periodic, and later packets depend on earlier packets.

The design therefore introduces H.264 as an image codec layer that can sit above multiple carriers, not as a replacement for LCM, DDS, ROS, SHM, or WebRTC. Carrier adapters move compressed video packets between machines; endpoint adapters decode those packets back into `Image` objects for normal modules.

The aiortc project is the preferred implementation source for the video codec layer now that DimOS already depends on it for WebRTC-related functionality. It implements Python WebRTC/ORTC video send/receive paths, including H.264 encode/decode, H.264 RTP packetization/depacketization, PyAV-backed `libx264` encoding, Baseline/zerolatency-style settings, and WebRTC loss-recovery mechanisms such as NACK/PLI. DimOS should directly wrap aiortc's H.264 encoder/decoder where practical, while converting aiortc RTP payload details into a Foxglove-style complete Annex B access unit before exposing packets to non-WebRTC carriers or memory2.

Foxglove's `CompressedVideo` design is the right compatibility target for DimOS packet shape: one message contains the compressed video data needed for exactly one source frame, H.264 data is Annex B, B-frames are not supported, and every IDR keyframe includes parameter sets such as SPS/PPS. This does not remove the need for keyframes: P-frames still depend on prior decoded reference frames. It does remove the need for DimOS memory2/LCM/DDS consumers to reason about individual RTP fragments.

## Goals / Non-Goals

**Goals:**

- Preserve `Out[Image]` and `In[Image]` as the public module stream contract.
- Add a carrier-neutral per-frame `VideoPacket` representation for complete H.264 Annex B access units, matching Foxglove's one compressed-video message per encoder input frame model.
- Add stateful H.264 encoder/decoder components with deterministic GOP/keyframe behavior.
- Add an LCM carrier adapter first, modeled after `JpegLcmTransport`.
- Add memory2 H.264 storage that keeps one observation per frame, stores one packet blob per frame, and lazily reconstructs `obs.data` as `Image`.
- Provide top-level opt-in configuration for live transports and recorder/store storage.
- Use aiortc directly for H.264 encode/decode through a DimOS adapter, and use aiortc public WebRTC APIs for the future WebRTC carrier.
- Keep JPEG transport and JPEG memory2 storage as defaults.

**Non-Goals:**

- Replacing underlying carriers such as LCM, DDS, ROS, SHM, or WebRTC.
- Making every transport support H.264 in the first implementation; DDS/SHM/WebRTC carriers are follow-ups.
- Exposing aiortc RTP payload fragments or WebRTC session state as public DimOS module, transport, or memory2 storage APIs.
- Supporting depth images, 16-bit images, alpha formats, or arbitrary pixel formats in the first implementation.
- Making `codec_for(Image)` return H.264 by default.
- Guaranteeing random access without decoding from a prior keyframe.
- Exposing video packets to normal module authors as the default stream type.

## DimOS Architecture

### Layering

The design has three layers:

```text
Module API layer
  Out[Image] / In[Image]
        │
        ▼
Codec layer
  H264Encoder / H264Decoder / GopBuffer
  Image ⇄ VideoPacket
        │
        ▼
Carrier layer
  H264LcmTransport first
  DDS / SHM / WebRTC later
```

The carrier still performs inter-process or inter-machine communication. The H.264 layer only changes how image payloads are encoded before carrier publish and decoded after carrier receive.

### Proposed classes and locations

Core packet and codec classes:

- `dimos/msgs/sensor_msgs/VideoPacket.py`
  - Carrier-neutral message for one encoded video frame/access unit.
  - Fields: `seq`, `ts`, `frame_id`, `width`, `height`, `format`, `codec`, `bitstream`, `is_keyframe`, `keyframe_seq`, `pts`, `data`.
  - First supported `codec`: `h264`.
  - First supported `bitstream`: Annex B complete access unit for exactly one source frame, aligned with Foxglove `CompressedVideo` expectations: for every full-frame encoder input call, DimOS creates one `VideoPacket` containing all NAL units emitted for that input frame.
  - A `VideoPacket` is a complete encoded-frame packet, not necessarily an independently decodable image. Keyframe packets must contain enough decoder bootstrap data for late join and random access, including SPS/PPS on every IDR; delta-frame packets require prior decoded GOP state.

- `dimos/protocol/video/h264.py`
  - `H264Config`: bitrate, target fps, keyframe interval, profile, preset/tune, max GOP frames, pixel format.
  - `AiortcH264Codec`: small DimOS adapter around `aiortc.codecs.h264.H264Encoder`, `aiortc.codecs.h264.H264Decoder`, and `aiortc.codecs.h264.h264_depayload`.
  - `H264Encoder`: DimOS-facing wrapper that runs at the publishing or recording endpoint and converts `Image` to ordered `VideoPacket` values using aiortc.
  - `H264Decoder`: DimOS-facing wrapper that runs at the subscribing or replay/decode endpoint and converts ordered `VideoPacket` values to decoded `Image` values using aiortc.
  - `GopBuffer`: tracks the latest keyframe and following delta packets, detects sequence gaps, and suppresses output until the next keyframe after a gap.
  - `H264AccessUnit`: helper that converts aiortc RTP payload batches into a complete Annex B access unit before building a `VideoPacket`.
  - `UnsupportedVideoImageError` / `VideoDecodeGapError`: explicit errors for unsupported image formats and unusable GOP state.

Implementation dependency:

- aiortc's `src/aiortc/codecs/h264.py` provides the mechanics DimOS should call rather than reimplement initially: `H264Encoder.encode()` uses PyAV `libx264`, forces keyframes by setting frame picture type, emits Baseline/zerolatency H.264, and returns RTP-sized H.264 payloads plus timestamp; `h264_depayload()` converts RTP H.264 payloads back to Annex B bytes; `H264Decoder.decode()` decodes a depayloaded `JitterFrame` through PyAV.
- DimOS should assemble the aiortc payloads for one encoded source frame into a single Annex B `VideoPacket.data` value before publication/storage. This packet carries every NAL unit emitted for that encoder input frame, but only IDR/keyframe packets are expected to be independently bootstrappable. WebRTC carriers may keep aiortc RTP packetization internally, but LCM/DDS/memory2 should exchange complete access units.
- The adapter should avoid leaking aiortc classes such as `JitterFrame` and RTP payload descriptors into DimOS public APIs. If future aiortc versions change these codec internals, only `AiortcH264Codec` should need adjustment.

Image lazy data support:

- `dimos/msgs/sensor_msgs/Image.py`
  - Add an explicit lazy pixel path mirroring `Observation`: metadata fields remain available, while `data` materializes pixels on access.
  - `height`, `width`, `format`, `frame_id`, and `ts` must be available without forcing decode.
  - Existing eager construction remains valid.
  - This is needed for transport subscribers that inspect metadata or keep only the latest frame without always decoding pixels.

LCM carrier classes:

- `dimos/protocol/pubsub/impl/h264_lcm.py`
  - `H264LCM`: LCM pubsub encoder/decoder that publishes serialized `VideoPacket` values on the wire and returns `Image` objects to subscribers.
  - Holds one encoder per publisher instance and one `GopBuffer`/decoder per subscriber instance.

- `dimos/core/transport.py`
  - `H264LcmTransport`: mirrors `JpegLcmTransport` and instantiates `H264LCM` lazily to avoid importing video dependencies at normal startup.
  - Reduces to `(H264LcmTransport, (topic, type, config))` for worker serialization.

WebRTC carrier classes, later:

- `dimos/protocol/pubsub/impl/webrtc_video.py`
  - Uses aiortc public APIs such as `RTCPeerConnection`, media tracks, RTP senders/receivers, and RTCP feedback.
  - Lets WebRTC own packetization, jitter buffering, retransmission/NACK, PLI keyframe requests, bitrate adaptation, and NAT traversal.
  - Bridges between DimOS `Image` and WebRTC `VideoFrame` at the module boundary.
  - Optionally exports encoded packets into the DimOS `VideoPacket` format for memory2 recording when aiortc exposes a clean encoded-frame hook; otherwise the first WebRTC integration may decode to `Image` and let memory2 re-encode.
  - If exporting, convert WebRTC RTP payloads into complete Annex B access units first; do not persist raw RTP fragments.

memory2 storage classes:

- `dimos/memory2/video/h264.py`
  - `H264ImageStorageConfig`: mode/config object for opt-in memory2 H.264 image storage.
  - `H264ImageBackend`: image-specific backend or payload strategy that owns encoder state and writes one observation row plus one `VideoPacket` blob per frame.
  - `H264FrameIndexStore`: creates and queries a standalone GOP index table.
  - `H264ObservationLoader`: reconstructs a requested frame by loading the nearest keyframe packet and ordered delta packets through the requested observation.
  - `H264ReplayDecodeSession`: shares decoder state during sequential replay so adjacent frames decode once.

Store/recorder integration:

- `dimos/memory2/store/sqlite.py`
  - Recognize image storage config when creating a stream.
  - Route `Image` streams with `mode="h264"` to the H.264 image backend.
  - Persist storage config in `_streams` so reopening the database selects the right loader.

- `dimos/memory2/module.py`
  - Add recorder-level per-stream image storage configuration.
  - Recorder still subscribes to `In[Image]`; storage mode controls how incoming images are persisted.

### Where components run

Live LCM path across machines:

```text
Source machine / worker process
  module Out[Image]
    └─ H264LcmTransport.broadcast()
         └─ H264Encoder encodes Image -> VideoPacket
              └─ LCM publishes packet bytes

Network / LCM multicast
  carries VideoPacket bytes, not numpy pixels

Subscriber machine / worker process
  H264LcmTransport.subscribe()
    └─ LCM receives packet bytes
         └─ GopBuffer validates seq/keyframe state
              └─ H264Decoder produces Image or lazy Image
                   └─ module In[Image] callback
```

memory2 recording path:

```text
Recorder module process
  In[Image] receives normal Image
    └─ stream.append(Image)
         └─ H264ImageBackend owns encoder state
              ├─ observation table row: ts / pose / tags
              ├─ blob row: serialized VideoPacket with complete Annex B access unit
              └─ h264 frame index: seq / keyframe row / pts / format
```

memory2 replay/decode path:

```text
Replay or query process
  stream query returns Observation[Image] metadata
    └─ obs.data
         └─ H264ObservationLoader loads keyframe + delta packet chain
              └─ H264Decoder reconstructs Image
```

The first implementation may re-encode images when recording a decoded `Image` stream that originally arrived over H.264 transport. Preserving incoming packet bytes end-to-end can be a later optimization via a packet side-channel; it is not required to make the public behavior correct.

WebRTC/aiortc path, later:

```text
Source machine / async WebRTC worker
  Image source track
    └─ aiortc encodes VideoFrame -> RTP/H.264
         └─ WebRTC handles packetization, jitter, NACK/PLI, bandwidth

Network / WebRTC session
  carries RTP media packets

Subscriber machine / async WebRTC worker
  aiortc receives/decodes RTP media
    └─ adapter converts VideoFrame -> Image
         └─ module In[Image] callback
```

This path is intentionally different from LCM and memory2 storage. WebRTC is a session protocol with negotiated codecs and RTP packet state; memory2 still needs deterministic per-observation packet rows and GOP lookup independent of any active peer connection.

### Top-level activation and configuration

Live transport activation should use existing blueprint transport mapping:

```python
from dimos.core.transport import H264LcmTransport
from dimos.protocol.video.h264 import H264Config

blueprint = autoconnect(camera(), consumer()).transports(
    {
        ("color_image", Image): H264LcmTransport(
            "/color_image",
            Image,
            config=H264Config(
                bitrate=2_000_000,
                keyframe_interval=30,
                profile="baseline",
                tune="zerolatency",
            ),
        )
    }
)
```

memory2 direct store activation:

```python
from dimos.memory2.video.h264 import H264ImageStorageConfig
from dimos.protocol.video.h264 import H264Config

stream = store.stream(
    "color_image",
    Image,
    image_storage=H264ImageStorageConfig(
        codec=H264Config(bitrate=2_000_000, keyframe_interval=30),
    ),
)
```

Recorder activation:

```python
MyRecorder.blueprint(
    image_storage={
        "color_image": H264ImageStorageConfig(
            codec=H264Config(bitrate=2_000_000, keyframe_interval=30),
        )
    }
)
```

Default behavior stays unchanged:

```python
store.stream("color_image", Image)  # JPEG-backed memory2 storage
LCMTransport("/color_image", Image)  # normal LCM image transport
```

### Proposed end-to-end test blueprint

Add one runnable synthetic blueprint that proves live H.264 transmission and H.264 memory2 storage through the normal DimOS surfaces, without robot hardware or a physical camera.

Proposed location and registry name:

- `dimos/protocol/video/demo_h264_video_e2e.py`
- Blueprint variable: `demo_h264_video_e2e`
- CLI name after registry generation: `demo-h264-video-e2e`

Components:

- `SyntheticVideoSource(Module)`
  - Publishes deterministic `color_image: Out[Image]` frames.
  - Uses a moving pattern, frame counter overlay/encoded pixels, and fixed metadata: width, height, format, frame_id, timestamp cadence.
  - Defaults to a short loop-friendly rate such as 15 or 30 FPS, with configurable width, height, FPS, frame count, and pattern seed.

- `H264E2ERecorder(Recorder)`
  - Declares `color_image: In[Image]`.
  - Uses recorder-level `image_storage={"color_image": H264ImageStorageConfig(...)}` so memory2 writes the received image stream as H.264 packets rather than JPEG blobs.
  - Defaults `db_path` to an explicit temporary/demo path such as `h264_video_e2e.db` so manual QA can inspect it.

- `H264VideoProbe(Module)`
  - Subscribes to `color_image: In[Image]` after live H.264 transport decode.
  - Tracks received frame count, monotonic timestamps, dimensions, frame_id, and approximate pixel/checksum expectations for the deterministic pattern.
  - Exposes a simple RPC/status method for manual QA, e.g. `summary() -> str`, reporting frames received, drops detected, first/last seq-equivalent frame marker, and validation errors.

Blueprint sketch:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import H264LcmTransport
from dimos.memory2.video.h264 import H264ImageStorageConfig
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.video.h264 import H264Config

h264_config = H264Config(
    bitrate=2_000_000,
    target_fps=30,
    keyframe_interval=30,
    profile="baseline",
    tune="zerolatency",
)

demo_h264_video_e2e = autoconnect(
    SyntheticVideoSource.blueprint(width=640, height=360, fps=30),
    H264E2ERecorder.blueprint(
        db_path="h264_video_e2e.db",
        image_storage={
            "color_image": H264ImageStorageConfig(codec=h264_config),
        },
    ),
    H264VideoProbe.blueprint(expected_width=640, expected_height=360),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            "/demo/h264_video_e2e/color_image",
            Image,
            config=h264_config,
        )
    }
)
```

This blueprint intentionally exercises two independent H.264 paths:

1. **Live transmission:** `SyntheticVideoSource` publishes normal `Image`; `H264LcmTransport` encodes to `VideoPacket`, transmits over LCM, decodes back to `Image`, and delivers to normal `In[Image]` subscribers.
2. **Storage:** `H264E2ERecorder` receives normal `Image` and writes memory2 observations using H.264 image storage, including GOP index rows and one Annex B packet blob per observation.

Manual QA contract:

- Run `dimos run demo-h264-video-e2e --daemon`.
- Confirm logs show H.264 encoder initialization, periodic keyframes, probe frame counts, and recorder append counts.
- Open the produced memory2 store and query `color_image` observations without touching `obs.data`; metadata should be available without decode.
- Access `obs.data` on a keyframe and a mid-GOP delta frame; both should return decoded `Image` pixels, with the mid-GOP read decoding from the nearest prior keyframe.
- Replay the stored stream and confirm decoded images arrive on the normal replay schedule.
- Run a seq-gap variant, either by a test-only packet drop option in `H264LcmTransport` or a direct `GopBuffer` driver, and verify the probe receives no corrupted images and resumes only after the next keyframe.

The blueprint should be excluded from normal hardware requirements and should not require a viewer. If it is registered as a runnable blueprint, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`.

### Storage schema

Use existing per-stream observation and blob tables for primary data:

```text
color_image
  id, ts, value, pose fields, tags

color_image_blob
  id -> serialized VideoPacket Annex B access unit
```

Add a standalone GOP index table for H.264 image streams:

```text
h264_frames
  stream_name
  observation_id
  seq
  keyframe_observation_id
  is_keyframe
  pts
  width
  height
  format
  codec
  bitstream
```

This table is storage-owned metadata. Generic observation tables remain focused on timeline, pose, tags, and scalar values.

### DimOS Spec Protocols, skills/MCP, CLI, generated registries

No new DimOS Python `Spec` Protocol is required for the first version because encode/decode is transport and storage behavior, not cross-module RPC. No skills or MCP tools are exposed.

No CLI command is required for the core feature. The synthetic `demo-h264-video-e2e` blueprint is the manual QA surface for end-to-end live transmission and storage. If the runnable blueprint is added, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`.

## Decisions

1. **Make H.264 a codec layer above carriers, not a carrier itself.**
   - Rationale: publisher and subscriber may run on different machines, and LCM/DDS/SHM/WebRTC remain responsible for communication.
   - Alternative rejected: a monolithic `H264ImageTransport` that hides the underlying carrier, because it does not generalize cleanly to DDS or WebRTC.

2. **Use one complete Annex B `VideoPacket` per source frame.**
   - Rationale: this preserves frame timestamps, sequence numbers, GOP state, and memory2 one-observation-per-frame semantics while matching Foxglove `CompressedVideo` expectations. Each packet contains all NAL units emitted for one encoder input frame.
   - Key detail: "complete packet for a frame" is not the same as "standalone-decodable frame." IDR/keyframe packets can bootstrap decode when they include SPS/PPS; P-frame packets still require prior decoded GOP state.
   - Alternative rejected: MP4 segment files as the primary model, because live transports and per-frame memory2 replay become harder to align.

3. **Keep `codec_for(Image)` as JPEG.**
   - Rationale: H.264 writes need stateful encoder ownership and GOP indexing; the stateless memory2 `Codec` contract should remain simple and backward compatible.

4. **Decode only from valid GOP state.**
   - Rationale: missing H.264 packets can corrupt decoded pixels. After a seq gap, subscribers and storage loaders should suppress or fail decode until a keyframe restores a self-contained GOP.
   - Key detail: complete per-frame access units remove RTP-fragment handling from DimOS storage, but they do not remove inter-frame dependencies; P-frames still require prior decoded reference frames.

5. **Use aiortc's H.264 codec classes through a DimOS adapter.**
   - Rationale: aiortc already depends on PyAV, sets up `libx264`, handles keyframe forcing, produces RTP-sized H.264 payloads, and provides depayload/decode logic. Reusing it reduces new codec code and aligns with the WebRTC transport dependency.
   - Boundary: DimOS stores and transports one logical Annex B `VideoPacket` per source frame; aiortc's multiple RTP payloads are an implementation detail converted before leaving the codec adapter.
   - Alternative rejected: copy aiortc's H.264 implementation into DimOS immediately, because direct wrapping is simpler while aiortc is already a dependency.

6. **Configure WebRTC as a future carrier, not the core codec abstraction.**
   - Rationale: WebRTC already solves live RTP packetization, jitter, packet loss, keyframe requests, NAT traversal, and adaptive bitrate, but memory2 still needs deterministic per-observation packet storage and replay.
   - aiortc should be the preferred Python implementation path for this carrier because it already supports sending and receiving H.264 video and RTCP recovery feedback.

7. **Store one complete access unit per observation, not one observation per RTP fragment.**
   - Rationale: aiortc's encoder returns multiple RTP-sized payloads for one source frame. memory2 should depayload and assemble them into one Annex B `VideoPacket` for the frame, so queries, replay, pose, tags, and GOP indexes remain frame-oriented and Foxglove-compatible.
   - Alternative rejected: storing each RTP fragment as its own observation, because replay and random access would inherit network packetization complexity and break frame-level memory2 semantics.

8. **Repeat decoder parameter sets on every IDR keyframe.**
   - Rationale: late join, random access, and memory2 partial reads require keyframes to bootstrap decoding without relying on stream-start state.
   - Alternative rejected: sending SPS/PPS only at stream startup, because late subscribers and mid-recording reads may never decode.

## Safety / Simulation / Replay

This change affects image transport and recording only. It does not command robot hardware, alter control loops, or expose new skills. Existing hardware safety assumptions remain unchanged.

Simulation and hardware cameras use the same `Image` semantics. Unsupported image formats such as depth or 16-bit images should fail at H.264 configuration/append/publish time with a clear error, not silently convert or corrupt data.

Replay must emit normal decoded `Image` objects on the existing memory2 replay schedule. Sequential replay should share decoder state so normal playback decodes each packet once. Seek or random access may decode from the nearest prior keyframe through the requested frame.

Manual QA should use the synthetic `demo-h264-video-e2e` blueprint so no robot or physical camera is required. The demo should verify live LCM round-trip, memory2 append/query without decode, lazy `obs.data` decode, replay, and seq-gap behavior.

## Risks / Trade-offs

- **Stateful codec complexity:** H.264 has encoder and decoder state. Mitigation: keep state in explicit `H264Encoder`, `H264Decoder`, and `GopBuffer` classes rather than hiding it in `Codec`.
- **Lazy `Image.data` compatibility:** Existing `Image` assumes eager numpy data. Mitigation: add lazy pixel support carefully so metadata properties do not force decode and eager construction remains unchanged.
- **Packet loss:** LCM has no built-in reliable delivery or late-join keyframe durability. Mitigation: periodic IDR frames and seq-gap suppression; later add keyframe request or durable carriers where available.
- **Dependency variability:** aiortc/PyAV/FFmpeg support varies by platform. Mitigation: keep H.264 optional under the extra that already provides aiortc/WebRTC support, preserve JPEG defaults, and fail clearly when video mode is selected without dependencies.
- **aiortc codec API stability:** aiortc codec classes are importable and useful, but the most stable aiortc surface is WebRTC itself. Mitigation: isolate all direct codec imports in `AiortcH264Codec`, pin/verify aiortc versions, and add focused tests around encode/depayload/decode behavior.
- **Double encode on record:** A recorder consuming decoded H.264 transport images may re-encode for memory2 storage. Mitigation: accept this in the first version; consider packet pass-through as a later optimization.
- **Random access latency:** Mid-GOP access requires decoding from a prior keyframe. Mitigation: short GOP defaults and decoder reuse during sequential replay.

## Migration / Rollout

1. Reuse the existing aiortc/WebRTC dependency path for H.264 support; add a lightweight `video` extra only if users need H.264 storage without the broader WebRTC extra.
2. Add `VideoPacket`, H.264 config, `AiortcH264Codec`, DimOS-facing encoder/decoder wrappers, GOP buffer, Annex B access-unit assembly, and explicit errors.
3. Add lazy pixel support to `Image` while preserving eager API behavior.
4. Add `H264LCM` and `H264LcmTransport` as the first live carrier adapter.
5. Add memory2 H.264 storage config, backend/payload strategy, GOP index table, and lazy loader.
6. Add registry serialization so reopened SQLite stores know which streams use H.264 storage.
7. Add `demo_h264_video_e2e` for synthetic end-to-end live transport plus memory2 storage QA.
8. Add tests and synthetic manual QA for live transport, storage, lazy decode, replay, unsupported formats, and seq gaps.
9. Update memory2 and transport docs with opt-in examples and dependency notes.

Rollback is straightforward because all behavior is opt-in. Removing H.264 configuration returns live streams and new recordings to existing transport/JPEG behavior. Existing H.264-backed recordings still require the video dependency to decode pixels, but metadata should remain queryable.

No generated blueprint registry update is needed unless a runnable demo blueprint is added.

## Open Questions

- Should the packet message be named `VideoPacket`, `EncodedImagePacket`, or `CompressedVideoFrame`?
- Should LCM H.264 publish raw packet bytes under an `Image` channel name or use a distinct LCM message type/channel suffix internally?
- What default bitrate, keyframe interval, and target FPS should be used for common DimOS camera streams?
- Should first-version memory2 storage store packet blobs in the existing `{stream}_blob` table or introduce a dedicated packet blob table?
- Should transport subscribers receive lazy `Image` objects by default, or should eager decode remain the default for maximum compatibility?
- Should WebRTC integration reuse this `VideoPacket` abstraction, or map directly between `Image` and WebRTC media tracks with optional packet export for memory2?
- Does aiortc expose a stable encoded-frame hook that can avoid decode/re-encode when recording a WebRTC H.264 stream into memory2?
- Should `AiortcH264Codec` pin to aiortc minor versions or include compatibility tests against the minimum supported aiortc version?
