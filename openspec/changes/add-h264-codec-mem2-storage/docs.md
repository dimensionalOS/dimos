## User-Facing Docs

- Update `docs/usage/transports/index.md` or the image-transport-specific transport docs to describe opt-in H.264 image transport behavior:
  - Public module streams remain `Out[Image]` and `In[Image]`.
  - `H264LcmTransport` compresses image payloads internally as H.264 and delivers decoded `Image` objects to subscribers.
  - H.264 packets contain complete Annex B access units for one source frame, matching Foxglove-style `CompressedVideo` expectations.
  - Delta frames require prior GOP state; after packet loss or late join, subscribers resume on the next keyframe.
  - Unsupported image formats fail clearly rather than silently converting.
- Update `docs/usage/blueprints.md` with an opt-in blueprint transport mapping example for `H264LcmTransport` and `H264Config`.
- Update memory2 user docs, likely under `docs/usage/` or the memory2 capability docs, to describe opt-in H.264-backed image storage:
  - Default image storage remains JPEG-backed.
  - Users opt in per stream with `H264ImageStorageConfig`.
  - memory2 still stores one observation per source frame.
  - metadata queries do not require pixel decode.
  - accessing `obs.data` lazily reconstructs an `Image`, decoding from the nearest prior keyframe when needed.
  - replay emits decoded `Image` frames on the normal replay schedule.
- Add a short manual QA section for `demo-h264-video-e2e` after the demo blueprint exists:
  - run `dimos run demo-h264-video-e2e --daemon`
  - inspect probe/recorder logs
  - query the generated memory2 store
  - validate lazy decode, replay, and seq-gap recovery.
- Mention optional video dependencies in the installation or feature docs. Users should know that H.264 mode requires the aiortc/PyAV/FFmpeg dependency path while JPEG defaults remain available without selecting H.264.

## Contributor Docs

- Update `docs/development/testing.md` or a nearby development testing guide with H.264-specific test commands once tests exist:
  - unit tests for `VideoPacket`, H.264 access-unit assembly, GOP buffering, unsupported formats, and lazy `Image.data` behavior
  - memory2 storage tests for append/query/lazy decode/reopen/replay
  - synthetic end-to-end demo/blueprint smoke test for live LCM transmission and memory2 recording.
- Document dependency expectations for contributors who run video tests locally, including how to install the relevant `uv` extras and how tests should skip clearly when video dependencies are unavailable.
- If `demo_h264_video_e2e` is registered as a runnable blueprint, contributor docs should remind maintainers to regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`.

## Coding-Agent Docs

- Update `docs/coding-agents/index.md` or a focused coding-agent guide if agents are expected to modify image transports or memory2 storage:
  - H.264 is opt-in and must not replace JPEG defaults.
  - Keep public module contracts as `Image` streams.
  - Store complete Annex B access units per source frame, not RTP fragments.
  - Preserve one memory2 observation per source frame.
  - Avoid negative-only OpenSpec requirements when adding or editing specs; include positive `MUST`/`SHALL` statements.
- No `AGENTS.md` update is required unless maintainers want the H.264/Foxglove packet-shape rule to become a repo-wide coding-agent constraint.

## Doc Validation

- Run documentation link validation for changed docs if available:
  - `uv run doclinks`
- Run markdown code-block validation for docs that contain executable Python snippets, for example:
  - `uv run md-babel-py run docs/usage/blueprints.md`
  - `uv run md-babel-py run <memory2-doc-path>`
- If diagrams are added or regenerated, run:
  - `bin/gen-diagrams`
- Validate generated blueprint registry freshness if the demo blueprint is registered:
  - `uv run pytest dimos/robot/test_all_blueprints_generation.py`

## No Docs Needed

Documentation is needed. This change adds user-visible opt-in transport and memory2 storage configuration, dependency requirements, replay/lazy-decode behavior, and a runnable synthetic QA blueprint.
