# Topic video: JPEG samples versus direct H.264 tracks

Measurement for PLAN_2b section 3.7, taken on 2026-11-04. It decides the
software encoder behind `video: {encoder: "software"}` and the fixed bitrate
per resolution class in `VIDEO_BITRATES`
([topic_profiles.py](/dimos/web/relay_bridge/topic_profiles.py)), and it
records what the direct path costs and saves against the JPEG profile it
replaces for cameras.

## Setup

- dimOS branch `paul/feat/topic-video-rtc` at `c58b0f9395` (both rate gate
  fixes below included), aiortc 1.14.0, PyAV 16.1.0 with libopenh264 and
  libx264.
- Harness: `dimos-websdk-benchmark` scenario `V2-topic-video` at `994c119`,
  Chromium 149.0.7827.55 (Playwright, headless, software H.264 decode).
- Host: AMD Ryzen 9 9950X, 32 logical CPUs, Linux `7.0.0-38-generic`,
  everything on loopback.
- Camera: the harness's synthetic 1280x720 scene (drifting shapes, mild
  sensor noise) published as rgb8 LCM Images at 30 Hz on a Zenoh session, the
  way a robot camera reaches the standalone topic provider. The provider runs
  as a subprocess with `video.enabled`.
- Viewers: one and three Chromium pages playing the camera either as
  `image.jpeg.balanced.v1` samples decoded with `createImageBitmap` and drawn
  to a canvas on animation frames, or as a `video.h264.v1` track from
  `session.topics.video()` in a `<video>` element.
- Windows: 10 s warmup, 30 s measurement per point. Provider, relay and
  browser CPU come from a 1 Hz process monitor over the window, as a
  percentage of one core.

Every frame carries a bar of 64 black or white blocks in its top rows: 48
bits of publish time in epoch milliseconds and 16 bits of frame counter. Each
page reads the bar back from the picture it displays (the video element on
one path, the bitmap on the other) on every presented frame, so display age
and frame identity are measured the same way on both paths and do not depend
on metadata one path has and the other lacks. Bytes are the JPEG payloads or
the peer's `inbound-rtp` counters.

## Results

Loopback, 720p30, Chromium. "Drawn" is distinct frames presented over frames
published in the window. Age is publish to presentation, p95.

| Path | Viewers | FPS (min view) | Drawn | p95 age | Per view | Provider CPU | Relay CPU | Browser CPU |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| JPEG balanced (q75) | 1 | 30.0 | 0.999 | 23 ms | 13.4 Mbit/s | 13.8% | 6.7% | 17% |
| JPEG balanced (q75) | 3 | 29.9 | 0.996 | 21 ms | 13.4 Mbit/s | 14.1% | 8.4% | 54% |
| H.264 libopenh264 | 1 | 30.0 | 0.999 | 50 ms | 2.5 Mbit/s | 36.7% | 0.1% | 13% |
| H.264 libopenh264 | 3 | 29.1 | 0.969 | 48 ms | 2.5 Mbit/s | 37.6% | 0.1% | 39% |
| H.264 libx264 | 1 | 29.9 | 0.997 | 46 ms | 2.5 Mbit/s | 53.2% | 0.1% | 14% |
| H.264 libx264 | 3 | 30.0 | 1.000 | 62 ms | 2.5 Mbit/s | 54.4% | 0.2% | 43% |

The H.264 rows are the `h264a` run and the JPEG rows the `jpegb` run of the
same day. In every H.264 point each page received and decoded all 900 frames
of the window with zero decoder drops and zero keyframe requests; the
three-viewer openh264 views presented 872 to 884 of them, the rest were
decoded but skipped at presentation while three pages decoded in software on
one machine. The direct path saves 5.4 times the bytes per view and all of
the relay's work, and costs the provider about 23 more percentage points of
one core for the encode. Its p95 display age is about 25 ms above JPEG's:
the software encode of a 720p frame, the receiver's jitter buffer (8 ms mean)
and the decode, where the JPEG path pays a 4 ms encode and an asynchronous
bitmap decode.

Against the loopback gates of section 3.7:

| Gate | Limit | libopenh264 | libx264 |
|---|---|---|---|
| Displayed FPS, each view | at least 28.5 | 30.0 / 29.1 | 29.9 / 30.0 |
| p95 display age | under 200 ms | 50 / 48 ms | 46 / 62 ms |
| Payload per view | at most 5 Mbit/s | 2.5 Mbit/s | 2.5 Mbit/s |
| Encode work for three viewers | none beyond packetization | +0.9 CPU points | +1.2 CPU points |

The provider encoded each selected frame once for the three peers (its
counters at peer close: one encoder, 2527 encodes, 1232 to 1293 frames sent
per peer, no peer or stale drops), and released the encoder within a second
of the last viewer leaving.

### Shaped link

The 200 ms RTT / 1% loss / 100 Mbit/s profile of
[desktop-history.md](/docs/web/benchmarks/desktop-history.md) runs through the
harness's userspace UDP proxy in front of the relay (run `full2`, before the
relay gate fix, which does not change the outcome). The JPEG path collapsed
on it as before: 1.4 displayed FPS and a 37 s p95 age, 0.62 Mbit/s getting
through. The H.264 path measured 29.9 FPS and 60 ms there, but that is not a
shaped result: the direct media between browser and provider does not pass
through the relay, so the proxy shaped its signaling only. Shaping the direct
path needs a network namespace, and adaptation under loss is PLAN_2e's work.

## Decisions

- `software` (and `auto`) selects libopenh264, libx264 when the wheel lacks
  it. Both meet every gate; openh264 does it at 37% of a core against 53%
  for x264 at the same bitrate and age, and the robot's CPU is the scarcer
  resource. `video: {encoder: "libx264"}` selects x264 explicitly.
- `VIDEO_BITRATES` stays at 1.0 / 2.5 / 4.5 Mbit/s for the 360p / 720p /
  1080p classes: 2.5 Mbit/s at 720p30 left the age budget untouched and sits
  at half the per-view gate. PLAN_2e turns the bitrate into an option.
- libx264 runs with `sc_threshold=0`. With scene-cut detection on, this scene
  made x264 emit an IDR every 0.4 s (143 in 57 s), and the one-viewer page
  presented 27.3 FPS with 81 skipped frames; without it x264 keeps the 4 s
  safety keyframe cadence (8 in 30 s) and the numbers above.
- The measurement found and fixed a rate gate fault that predates this work,
  in the provider's mailbox and variant gates and in the relay's per-viewer
  egress gate for latest routes. Each re-anchored its next due time on the
  take or send, so a wake that ran a millisecond late pushed every later
  frame, and a camera at the gate's own rate lost one frame in twenty to
  thirty: the first runs drew 94 to 96 percent on both paths, and the JPEG
  pages still drew 94 percent at a 51 ms p95 age with the provider fixed and
  the relay not. Anchored to their schedule, the gates deliver 99.6 to 100
  percent on both paths, and the JPEG p95 age fell to 23 ms because frames
  no longer wait on a drifted due time.

## Reproducing

```bash
cd /path/to/dimos-websdk-benchmark
bin/webbench --dimos /path/to/dimos run --suite full --only V2-topic-video --browsers chromium
# One path only, no shaped point:
bin/webbench --dimos /path/to/dimos run --suite full --only V2-topic-video --browsers chromium \
  --param 'V2-topic-video.modes=["h264"]' --param 'V2-topic-video.shaped=null'
```

`results/<run-id>/run.json` keeps every view's counters under the
`<point>.views` metric; `scenarios/V2-topic-video/provider-*.log` has the
provider's per-peer counters at close.
