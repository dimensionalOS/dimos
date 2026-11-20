// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//! Each M20 camera captured straight off its V4L2 device as MJPEG and hardware-encoded to H.264, published as
//! `CompressedVideo` (`h264`) — the direct path that replaces the vendor's RTSP server (a `gst-launch` pipeline into
//! `mediamtx`). Encoding H.264 at the source means the Desktop gateway passes it through to every browser instead of
//! decoding the vendor's H.265 and re-encoding H.264, which freed the robot's big cores for navigation.
//!
//! Two ways to do the Rockchip hardware work (JPEG decode → H.264 encode), picked by `Encode::encoder`:
//! - `Encoder::Rust` (the default): in this process, through the `rockchip-h264` crate (the RK3588's JPEG decoder and VEPU580 driven
//!   straight through `/dev/mpp_service`), one blocking thread per camera capturing V4L2 MJPEG and producing one
//!   access unit per frame.
//! - `Encoder::Gstreamer` (opt-in fallback): a `gst-launch-1.0` pipeline per camera (`mppjpegdec` → `mpph264enc`) writing an Annex-B
//!   byte stream to its stdout, split back into access units here. The vendor plugin on AOS leaks a dmabuf fd per
//!   frame (~510k fds and 2.6 fps after a day), which is why the Rust path exists.
//!
//! Either way the SPS/PPS go in front of every keyframe so a subscriber that joins mid-stream starts at the next one,
//! and each access unit (one video slice) is published as one message, every frame.

use std::process::Stdio;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use dimos_module::Output;
use lcm_msgs::builtin_interfaces::Time;
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::sensor_msgs::CameraInfo;
use tokio::io::AsyncReadExt;
use tokio::process::Command;
use tokio::sync::mpsc;
use tracing::{info, warn};

/// One camera: the V4L2 device it captures, where it sits, and where its messages go.
pub struct Camera {
    pub name: String,
    /// the V4L2 device node, e.g. `/dev/video0`
    pub device: String,
    pub frame_id: String,
    /// Center in base_link, and the turn about z.
    pub xyz: [f64; 3],
    pub yaw_rad: f64,
    pub video: Output<CompressedVideo>,
    pub info: Output<CameraInfo>,
    /// The capture size, for its CameraInfo (fixed by the pipeline caps).
    pub size: Mutex<Option<(u32, u32)>>,
}

/// How the pipeline captures and encodes each camera.
#[derive(Clone)]
pub struct Encode {
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub bitrate_bps: u32,
    /// keyframe interval in frames (the pipeline's `gop`)
    pub gop: u32,
    pub encoder: Encoder,
    /// Undistort each frame before encoding (rust encoder only): the raw camera at `width` x `height`, and how
    pub deskew: Option<(
        rockchip_h264::deskew::Intrinsics,
        rockchip_h264::deskew::Options,
    )>,
}

/// Which implementation of the hardware JPEG decode → H.264 encode a camera runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Encoder {
    /// in-process, the `rockchip-h264` crate
    Rust,
    /// a `gst-launch-1.0` child (the vendor's leaking mpp plugin)
    Gstreamer,
}

impl std::str::FromStr for Encoder {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        match s {
            "rust" => Ok(Self::Rust),
            "gstreamer" => Ok(Self::Gstreamer),
            other => bail!("unknown encoder {other:?} (rust or gstreamer)"),
        }
    }
}

/// Relay forever: start the pipeline, publish until it fails or stalls, wait `retry`, repeat; log rates every `stats`.
pub async fn relay(
    camera: Arc<Camera>,
    encode: Encode,
    retry: Duration,
    stall: Duration,
    stats: Duration,
) {
    *camera.size.lock().unwrap() = Some((encode.width, encode.height));
    loop {
        match stream(&camera, &encode, stall, stats).await {
            Ok(()) => warn!(camera = %camera.name, "M20 camera pipeline ended"),
            Err(error) => {
                warn!(camera = %camera.name, error = format!("{error:#}"), "M20 camera pipeline failed")
            }
        }
        tokio::time::sleep(retry).await;
    }
}

/// The `gst-launch-1.0` arguments for one camera: MJPEG off the V4L2 device, Rockchip hardware JPEG-decode and
/// H.264-encode, SPS/PPS in front of every keyframe, Annex-B to stdout.
fn pipeline_args(camera: &Camera, encode: &Encode) -> Vec<String> {
    let caps = format!(
        "image/jpeg,width={},height={},framerate={}/1",
        encode.width, encode.height, encode.fps
    );
    [
        "-q",
        "v4l2src",
        &format!("device={}", camera.device),
        "io-mode=4",
        "!",
        &caps,
        "!",
        "mppjpegdec",
        "!",
        "mpph264enc",
        "rc-mode=cbr",
        &format!("bps={}", encode.bitrate_bps),
        &format!("bps-max={}", encode.bitrate_bps * 5 / 4),
        &format!("gop={}", encode.gop),
        "!",
        "h264parse",
        "config-interval=-1",
        "!",
        "video/x-h264,stream-format=byte-stream,alignment=au",
        "!",
        "fdsink",
        "fd=1",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect()
}

/// Access units waiting to be published: about two seconds at 30 fps. Reading never waits on publishing, so this is
/// the slack that absorbs a slow publish; past it, frames are dropped (see `stream`).
const QUEUED_FRAMES: usize = 60;
/// Below this fraction of the pipeline's configured rate the stream counts as sick. Well under a half, because a UVC
/// camera legitimately halves its rate in low light (30 -> 15) and that must not restart anything; the stuck encoder
/// this guards against managed ~5 fps of 30.
const SICK_FPS_FRACTION: f64 = 0.25;
/// Sick windows in a row before the pipeline is restarted: one is a hiccup, two is the encoder stuck.
const SICK_WINDOWS: u32 = 2;

/// Whether a measured rate is too far under the rate the pipeline was told to capture at.
fn sick(measured_fps: f64, configured_fps: u32) -> bool {
    measured_fps < f64::from(configured_fps) * SICK_FPS_FRACTION
}

async fn stream(camera: &Camera, encode: &Encode, stall: Duration, stats: Duration) -> Result<()> {
    match encode.encoder {
        Encoder::Rust => stream_rust(camera, encode, stall, stats).await,
        Encoder::Gstreamer => stream_gstreamer(camera, encode, stall, stats).await,
    }
}

/// Consecutive frames that fail to transcode before the stream is torn down and reopened (one bad JPEG off a UVC
/// camera is skipped; a second's worth means something is stuck).
fn failed_frames_limit(encode: &Encode) -> u32 {
    encode.fps.max(1)
}

async fn stream_rust(
    camera: &Camera,
    encode: &Encode,
    stall: Duration,
    stats: Duration,
) -> Result<()> {
    let dropped = Arc::new(AtomicU32::new(0));
    let stop = Arc::new(AtomicBool::new(false));
    let (units, queued) = mpsc::channel::<Vec<u8>>(QUEUED_FRAMES);
    // The capture + hardware loop blocks (V4L2 poll, mpp_service poll), so it gets its own thread; like the gst
    // reader, it never waits on publishing and drops a frame instead.
    let worker = tokio::task::spawn_blocking({
        let (device, name, encode) = (camera.device.clone(), camera.name.clone(), encode.clone());
        let (dropped, stop) = (Arc::clone(&dropped), Arc::clone(&stop));
        move || -> Result<()> {
            let config = rockchip_h264::Config {
                width: encode.width,
                height: encode.height,
                fps: encode.fps,
                bitrate_bps: encode.bitrate_bps,
                gop: encode.gop,
                ..rockchip_h264::Config::default()
            };
            let mut transcoder = rockchip_h264::Transcoder::new(config)
                .with_context(|| format!("opening the Rockchip hardware for {name}"))?;
            if let Some((camera, options)) = &encode.deskew {
                // the remap tables are built on the first frame (and on a size change), never per frame
                transcoder.set_deskew(Some(rockchip_h264::deskew::Deskew::new(
                    camera.clone(),
                    *options,
                )?));
            }
            let mut capture = rockchip_h264::v4l2::Capture::open(
                &device,
                encode.width,
                encode.height,
                encode.fps,
                4,
            )?;
            let mut failed = 0;
            while !stop.load(Ordering::Relaxed) && !units.is_closed() {
                let mut unit = Vec::new();
                let result = capture.next(stall, |jpeg| transcoder.transcode(jpeg, &mut unit))?;
                match result {
                    Ok(_) => {
                        failed = 0;
                        if units.try_send(unit).is_err() {
                            dropped.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    Err(error) => {
                        failed += 1;
                        warn!(camera = %name, error = format!("{error:#}"), "M20 camera frame dropped");
                        if failed >= failed_frames_limit(&encode) {
                            bail!("{failed} frames in a row failed to transcode: {error:#}");
                        }
                    }
                }
            }
            Ok(())
        }
    });
    info!(camera = %camera.name, device = %camera.device, "M20 camera encoder started (in-process)");
    let published = publish(camera, encode, stats, queued, &dropped).await;
    // publishing ended (failed, or the worker closed the channel): stop the worker and wait, so the camera and the
    // hardware sessions are closed before a retry opens them again
    stop.store(true, Ordering::Relaxed);
    let worked = worker.await.context("the encoder thread")?;
    published?;
    worked
}

async fn stream_gstreamer(
    camera: &Camera,
    encode: &Encode,
    stall: Duration,
    stats: Duration,
) -> Result<()> {
    let args = pipeline_args(camera, encode);
    let mut child = Command::new("gst-launch-1.0")
        .args(&args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .kill_on_drop(true)
        .spawn()
        .with_context(|| format!("spawning gst-launch-1.0 for {}", camera.name))?;
    let mut stdout = child.stdout.take().context("the pipeline has no stdout")?;
    info!(camera = %camera.name, device = %camera.device, "M20 camera pipeline started");

    // Reading the pipeline and publishing are separate tasks on purpose: a publish that waits must never stop the
    // pipe being drained. fdsink blocks on a full pipe, and the Rockchip encoder answers that back pressure by
    // spinning a core and never recovering — both cameras limped at ~5 fps with no keyframes for hours, which looks
    // to a viewer like no camera at all. Dropping a frame costs the picture until the next keyframe (a GOP, ~1 s);
    // wedging the encoder costs every frame until someone restarts the service.
    let dropped = Arc::new(AtomicU32::new(0));
    let (units, queued) = mpsc::channel::<Vec<u8>>(QUEUED_FRAMES);
    let reader = tokio::spawn({
        let dropped = Arc::clone(&dropped);
        async move {
            let mut splitter = AccessUnits::default();
            let mut buffer = [0u8; 64 * 1024];
            loop {
                let read = match tokio::time::timeout(stall, stdout.read(&mut buffer)).await {
                    Err(_) => bail!("no frame for {stall:?}"),
                    Ok(result) => result.context("reading the pipeline's output")?,
                };
                if read == 0 {
                    // the pipeline closed its stdout: it exited
                    return Ok(());
                }
                for unit in splitter.push(&buffer[..read]) {
                    if units.try_send(unit).is_err() {
                        dropped.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
    });
    publish(camera, encode, stats, queued, &dropped).await?;
    // the reader owns the only sender, so the channel closing means it has finished: its result is the stream's
    reader.await.context("the pipeline reader")?
}

/// Publishes access units as they arrive until the sender finishes, logging rates every `stats`; fails on a publish
/// error or a stream that is up but not delivering.
async fn publish(
    camera: &Camera,
    encode: &Encode,
    stats: Duration,
    mut queued: mpsc::Receiver<Vec<u8>>,
    dropped: &AtomicU32,
) -> Result<()> {
    let (mut frames, mut keyframes, mut bytes, mut window) = (0u32, 0u32, 0usize, Instant::now());
    let mut sick_windows = 0u32;
    while let Some(unit) = queued.recv().await {
        let keyframe = has_keyframe(&unit);
        frames += 1;
        keyframes += keyframe as u32;
        bytes += unit.len();
        let ts = wall_s();
        let message = CompressedVideo {
            timestamp: Time {
                sec: ts.trunc() as i32,
                nanosec: (ts.fract() * 1e9) as i32,
            },
            frame_id: camera.frame_id.clone(),
            data: unit,
            format: "h264".into(),
        };
        camera.video.publish(&message).await?;
        if window.elapsed() >= stats {
            let seconds = window.elapsed().as_secs_f64();
            let fps = f64::from(frames) / seconds;
            info!(
                camera = %camera.name,
                fps,
                keyframes_per_s = f64::from(keyframes) / seconds,
                mbit_per_s = bytes as f64 * 8e-6 / seconds,
                dropped = dropped.swap(0, Ordering::Relaxed),
                "M20 camera stats",
            );
            // The stall timeout only catches a pipeline that stops entirely; a stuck encoder keeps dribbling frames,
            // so watch the rate too and restart on a stream that is up but not delivering.
            sick_windows = if sick(fps, encode.fps) {
                sick_windows + 1
            } else {
                0
            };
            if sick_windows >= SICK_WINDOWS {
                bail!(
                    "{fps:.1} fps of the {} asked for, {SICK_WINDOWS} windows running",
                    encode.fps
                );
            }
            (frames, keyframes, bytes, window) = (0, 0, 0, Instant::now());
        }
    }
    Ok(())
}

/// Whether an Annex-B access unit holds an IDR slice (NAL unit type 5): a decoder can start here.
fn has_keyframe(access_unit: &[u8]) -> bool {
    nal_types(access_unit).into_iter().any(|kind| kind == 5)
}

/// The NAL unit types in an Annex-B buffer, in order (H.264: the low five bits of each unit's first byte).
fn nal_types(data: &[u8]) -> Vec<u8> {
    let mut types = Vec::new();
    let mut at = 0;
    while let Some(found) = data
        .get(at..)
        .and_then(|rest| rest.windows(3).position(|w| w == [0, 0, 1]))
    {
        at += found + 3;
        if let Some(&first) = data.get(at) {
            types.push(first & 0x1f);
        }
    }
    types
}

/// Splits a streaming Annex-B byte stream into access units: one video slice (NAL types 1–5) ends an access unit, so
/// its parameter sets and SEI (which `h264parse config-interval=-1` writes in front of a keyframe) go out with the
/// slice they belong to. The Rockchip encoder emits one slice per frame, so each access unit is one picture.
#[derive(Default)]
struct AccessUnits {
    /// bytes not yet split into whole NAL units (ends mid-unit, since the next start code marks the end of this one)
    buffer: Vec<u8>,
    /// the NAL units of the access unit being built, each with its leading start code, not yet a video slice
    pending: Vec<u8>,
}

impl AccessUnits {
    /// Appends `chunk` and returns every access unit now complete.
    fn push(&mut self, chunk: &[u8]) -> Vec<Vec<u8>> {
        self.buffer.extend_from_slice(chunk);
        let mut units = Vec::new();
        // start-code offsets (the byte after each `00 00 01`); a NAL unit runs from one to the next
        let starts: Vec<usize> = start_codes(&self.buffer);
        if starts.len() < 2 {
            return units;
        }
        // keep the last (possibly incomplete) NAL in the buffer; process the ones before it
        let last = *starts.last().unwrap();
        for pair in starts.windows(2) {
            let (start, next) = (pair[0], pair[1]);
            // the NAL unit, with the three-byte start code that precedes it, minus the next code's leading zeros
            let nal_start = start - 3;
            let nal_end = next - 3;
            let nal = &self.buffer[nal_start..nal_end];
            let kind = self.buffer[start] & 0x1f;
            self.pending.extend_from_slice(nal);
            // a coded slice (1..=5) is the last unit of its access unit
            if (1..=5).contains(&kind) {
                units.push(std::mem::take(&mut self.pending));
            }
        }
        // drop everything consumed; keep the final, still-open NAL unit (from its start code's leading zeros)
        self.buffer.drain(..last - 3);
        units
    }
}

/// The offset just after each Annex-B start code (`00 00 01`, three or four bytes) in `data`.
fn start_codes(data: &[u8]) -> Vec<usize> {
    let mut offsets = Vec::new();
    let mut at = 0;
    while let Some(found) = data
        .get(at..)
        .and_then(|rest| rest.windows(3).position(|w| w == [0, 0, 1]))
    {
        at += found + 3;
        offsets.push(at);
    }
    offsets
}

/// The wall clock in seconds.
pub fn wall_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stream_at_its_asked_for_rate_is_well() {
        assert!(!sick(29.4, 30));
        assert!(!sick(30.0, 30));
    }

    #[test]
    fn low_light_halving_is_not_sick() {
        // a UVC camera drops to 15 fps of 30 on its own in low light: real, and no reason to restart
        assert!(!sick(15.0, 30));
        assert!(!sick(14.0, 30));
    }

    #[test]
    fn encoder_names() {
        assert_eq!("rust".parse::<Encoder>().unwrap(), Encoder::Rust);
        assert_eq!("gstreamer".parse::<Encoder>().unwrap(), Encoder::Gstreamer);
        assert!("gst".parse::<Encoder>().is_err());
    }

    #[test]
    fn the_stuck_encoder_is_sick() {
        // what the Rockchip encoder actually did once fdsink blocked: ~5 fps of the 30 asked for
        assert!(sick(4.7, 30));
        assert!(sick(0.0, 30));
    }
}
