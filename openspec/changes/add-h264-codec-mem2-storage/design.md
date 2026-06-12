## Context

DimOS modules exchange typed `Image` streams. Existing JPEG compression keeps that public type stable: JPEG is a storage/transport codec detail, and callers usually see decoded raw pixels. H.264 needs a similar opt-in path, but it differs from JPEG because many frames are delta frames that require prior GOP state to decode.

This design keeps the PR minimal and coherent with existing abstractions:

- `Image` remains the public payload type.
- Default image storage remains JPEG.
- memory2 continues to use the normal `Backend` + `Codec` path.
- H.264 live transport owns live encode/decode state.
- H.264 storage stores encoded `Image` values through a normal `H264ImageCodec`, not through a special backend.

Foxglove's H.264 guidance remains the packet-shape target: each encoded message contains all Annex B NAL units emitted for one encoder input frame. A complete encoded frame packet is not necessarily independently decodable; P-frames still require earlier GOP state.

## Goals / Non-Goals

**Goals:**

- Preserve `Out[Image]` and `In[Image]` as the user-facing stream contract.
- Extend `Image` so it can explicitly carry either raw pixels (`encoding="raw"`) or encoded H.264 access-unit bytes (`encoding="h264"`).
- Add H.264 encode/decode sessions with GOP/keyframe tracking, sequence-gap suppression, and explicit unsupported-format/dependency errors.
- Add `H264LcmTransport` with a decode mode:
  - `decode_images=True`: subscribers receive decoded raw `Image` values.
  - `decode_images=False`: subscribers receive encoded `Image` values for storage or inspection.
- Add `H264ImageCodec` so memory2 can store encoded H.264 `Image` values through the existing codec path.
- Keep `codec_for(Image)` as JPEG and require explicit `codec="h264"` for H.264 storage.
- Document v1 best-effort behavior: no transport QoS, durable keyframe cache, keyframe request, or guaranteed arbitrary random pixel decode.

**Non-Goals:**

- Adding a special memory2 backend for H.264.
- Adding a generic payload-strategy framework for this PR.
- Adding lazy pixels to `Image`; `Image.data` remains eager and is either `np.ndarray` for raw images or `bytes` for encoded images.
- Exposing a separate public encoded-video stream type.
- Supporting depth, 16-bit, alpha, or arbitrary pixel formats in the first implementation.
- Making H.264 the default image storage codec.

## Architecture

### Image payload shape

`Image` gains two explicit codec fields:

```python
encoding: str = "raw"
codec_metadata: dict[str, Any] = {}
```

For raw images, `data` is a NumPy array and existing pixel operations work.

For H.264 images, `data` is bytes containing one complete Annex B access unit for one source frame. `format` still describes the decoded pixel layout (for example, RGB or BGR), while `codec_metadata` carries video metadata such as:

```python
{
    "codec": "h264",
    "bitstream": "annex_b",
    "seq": 42,
    "is_keyframe": False,
    "keyframe_seq": 30,
    "pts": 3780,
    "width": 640,
    "height": 480,
    "channels": 3,
    "dtype": "uint8",
}
```

Pixel operations such as `to_rgb()`, `to_bgr()`, `to_opencv()`, `as_numpy()`, `brightness`, and Rerun conversion require `encoding="raw"` and fail clearly for encoded images.

### H.264 codec/session layer

`dimos/protocol/video/h264.py` provides the shared stateful video logic:

- `H264Config`: bitrate, target FPS, keyframe interval, profile/tune/preset, max GOP, supported formats.
- `AiortcH264Codec`: adapter around aiortc/PyAV H.264 encode/decode internals.
- `H264Encoder`: converts raw `Image` to encoded `Image(encoding="h264")`.
- `H264Decoder`: converts encoded H.264 `Image` to raw `Image` when GOP state is valid.
- `GopBuffer`: tracks sequence numbers and keyframe state; suppresses deltas after gaps until a keyframe.
- `H264AccessUnit`: assembles aiortc RTP-sized payloads into one Annex B access unit.

Transport and replay/view code instantiate separate encoder/decoder sessions. They share implementation, not runtime state.

### Live transport

`H264LcmTransport` mirrors the JPEG transport pattern while adding an explicit decode mode.

```python
H264LcmTransport("/camera/color", Image, config=H264Config(...))
```

Default mode decodes on receive, so normal subscribers get raw `Image` values.

```python
H264LcmTransport("/camera/color", Image, config=cfg, decode_images=False)
```

Encoded mode still uses the logical `Image` type, but subscribers receive `Image(encoding="h264")`. This is the mode used by recorders that should persist transport-produced H.264 bytes.

### memory2 storage

memory2 stores H.264 through a normal codec:

```python
store.stream("color_image", Image, codec="h264")
```

or an H.264-specific recorder subclass:

```python
stream = self.store.stream("color_image", Image, codec="h264")
self._port_to_stream("color_image", self.color_image, stream)
```

`H264ImageCodec` only stores/restores encoded `Image` values. It does not decode pixels and does not own GOP state. Reopened stores restore the codec through the existing stream registry `codec_id` field.

This means H.264 recording expects the recorder input to receive encoded Images, typically by subscribing through `H264LcmTransport(..., decode_images=False)`. If a recorder receives raw Images, either use the default JPEG codec or explicitly encode before appending.

### Replay and visualization

memory2 replay of a stream stored with `codec="h264"` emits encoded Images in timestamp order. A separate H.264 decoder session converts that encoded stream to raw Images for Rerun or consumers. V1 decode policy is best effort: if replay starts mid-GOP, deltas are suppressed until the first keyframe at or after the start point.

## Decisions

1. **Use encoded `Image`, not a separate public encoded-video type.**
   - Rationale: the user-facing type remains `Image` across transport and memory2. H.264 packet metadata lives in `Image.codec_metadata`.

2. **Use normal memory2 codecs, not a special backend.**
   - Rationale: memory2 already persists blob payloads through `Codec`. H.264 encoded images can be stored as encoded data without changing `Store` or `Backend` semantics.

3. **Keep `codec_for(Image)` as JPEG.**
   - Rationale: default behavior must remain stateless, compatible, and independent of H.264 dependencies.

4. **Let transport choose decoded vs encoded subscriber payloads.**
   - Rationale: normal modules want raw Images, while recorders may want the H.264 bytes produced by transport. The choice is explicit on `H264LcmTransport`.

5. **Decode only from valid GOP state.**
   - Rationale: complete per-frame access units remove RTP-fragment handling but not inter-frame dependency. P-frames still require prior decoded state.

6. **Defer QoS.**
   - Rationale: LCM is best effort. Keyframe request, durable keyframe cache, retransmission, PLI, and transport QoS belong in a later video-session/QoS design.

## Safety / Replay

This change affects image transport and recording only. It does not command robot hardware or change control loops.

Unsupported image formats must fail explicitly when H.264 encoding is selected. Encoded images must not silently pass through raw-pixel methods.

Replay after arbitrary seek is best effort. A decoder session starts without GOP state, suppresses deltas until the first keyframe at or after the start point, then emits decoded raw Images for that keyframe and following decodable deltas. Full random pixel access to any arbitrary P-frame is not a v1 guarantee.

## Migration / Rollout

1. Extend `Image` with `encoding` and `codec_metadata` while preserving raw eager defaults.
2. Add H.264 encoder/decoder/session classes that produce and consume encoded Images.
3. Add `H264LcmTransport` decode mode.
4. Add `H264ImageCodec` and explicit `codec="h264"` storage.
5. Update demos so recording uses encoded transport mode and replay decodes through an H.264 session before visualization.
6. Update docs/tests/specs to remove obsolete storage-strategy and packet-type language.

Rollback is straightforward for new runs: remove H.264 transport/storage configuration and streams return to normal raw/JPEG behavior. Existing H.264-backed recordings require the H.264 codec path to read encoded Images and a decoder session to view pixels.
