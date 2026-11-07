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
//! Each R1 Pro head camera captured off its V4L2 node as raw UYVY and hardware-encoded to H.264 with the Jetson's
//! NVENC, published as `CompressedVideo` (`h264`) — the R1 twin of the M20 direct relay. Encoding H.264 at the source
//! lets the Desktop gateway pass it through to the browser instead of transcoding, so the Controller matches the
//! native rate.
//!
//! The head is a GMSL2 stereo pair (SENSING ISX031) that the Tegra VI delivers as UYVY at 1920x1536@30 (see
//! `head_cameras.py`), so unlike the M20 (MJPEG off a USB bridge) there is no JPEG decode: `nvvidconv` converts UYVY to
//! NV12 in NVMM memory and `nvv4l2h264enc` encodes it. `h264parse config-interval=-1` puts the SPS/PPS in front of
//! every keyframe. This process splits the Annex-B byte stream into access units (one slice each) and publishes one
//! message per unit, every frame.

use std::process::Stdio;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use dimos_module::Output;
use lcm_msgs::builtin_interfaces::Time;
use lcm_msgs::foxglove_msgs::CompressedVideo;
use tokio::io::AsyncReadExt;
use tokio::process::Command;
use tracing::{info, warn};

/// One head camera: the V4L2 device it captures, its optical frame, and where its messages go.
pub struct Camera {
    pub name: String,
    /// the V4L2 device node (e.g. a `/dev/v4l/by-path/...` for one GMSL link)
    pub device: String,
    pub frame_id: String,
    pub video: Output<CompressedVideo>,
}

/// How the pipeline captures and encodes each camera.
#[derive(Clone)]
pub struct Encode {
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub bitrate_bps: u32,
    /// keyframe interval in frames (NVENC `iframeinterval`)
    pub gop: u32,
}

/// Relay forever: start the pipeline, publish until it fails or stalls, wait `retry`, repeat; log rates every `stats`.
pub async fn relay(
    camera: Arc<Camera>,
    encode: Encode,
    retry: Duration,
    stall: Duration,
    stats: Duration,
) {
    loop {
        match stream(&camera, &encode, stall, stats).await {
            Ok(()) => warn!(camera = %camera.name, "R1 head camera pipeline ended"),
            Err(error) => {
                warn!(camera = %camera.name, error = format!("{error:#}"), "R1 head camera pipeline failed")
            }
        }
        tokio::time::sleep(retry).await;
    }
}

/// The `gst-launch-1.0` arguments for one camera: raw UYVY off the V4L2 node, NVENC H.264, SPS/PPS before every
/// keyframe, Annex-B to stdout.
fn pipeline_args(camera: &Camera, encode: &Encode) -> Vec<String> {
    let raw = format!(
        "video/x-raw,format=UYVY,width={},height={},framerate={}/1",
        encode.width, encode.height, encode.fps
    );
    [
        "-q",
        "v4l2src",
        &format!("device={}", camera.device),
        "io-mode=2",
        "!",
        &raw,
        "!",
        // UYVY -> NV12 in NVMM memory for the hardware encoder
        "nvvidconv",
        "!",
        "video/x-raw(memory:NVMM),format=NV12",
        "!",
        "nvv4l2h264enc",
        &format!("bitrate={}", encode.bitrate_bps),
        &format!("iframeinterval={}", encode.gop),
        "insert-sps-pps=1",
        "maxperf-enable=1",
        "control-rate=1",
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

async fn stream(camera: &Camera, encode: &Encode, stall: Duration, stats: Duration) -> Result<()> {
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
    info!(camera = %camera.name, device = %camera.device, "R1 head camera pipeline started");

    let mut splitter = AccessUnits::default();
    let mut buffer = [0u8; 64 * 1024];
    let (mut frames, mut keyframes, mut bytes, mut window) = (0u32, 0u32, 0usize, Instant::now());
    loop {
        let read = match tokio::time::timeout(stall, stdout.read(&mut buffer)).await {
            Err(_) => bail!("no frame for {stall:?}"),
            Ok(result) => result.context("reading the pipeline's output")?,
        };
        if read == 0 {
            return Ok(());
        }
        for unit in splitter.push(&buffer[..read]) {
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
                info!(
                    camera = %camera.name,
                    fps = frames as f64 / seconds,
                    keyframes_per_s = keyframes as f64 / seconds,
                    mbit_per_s = bytes as f64 * 8e-6 / seconds,
                    "R1 head camera stats",
                );
                (frames, keyframes, bytes, window) = (0, 0, 0, Instant::now());
            }
        }
    }
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

/// Splits a streaming Annex-B byte stream into access units: one video slice (NAL types 1-5) ends an access unit, so
/// its parameter sets go out with the slice they belong to. NVENC emits one slice per frame, so each access unit is one
/// picture.
#[derive(Default)]
struct AccessUnits {
    buffer: Vec<u8>,
    pending: Vec<u8>,
}

impl AccessUnits {
    fn push(&mut self, chunk: &[u8]) -> Vec<Vec<u8>> {
        self.buffer.extend_from_slice(chunk);
        let mut units = Vec::new();
        let starts: Vec<usize> = start_codes(&self.buffer);
        if starts.len() < 2 {
            return units;
        }
        let last = *starts.last().unwrap();
        for pair in starts.windows(2) {
            let (start, next) = (pair[0], pair[1]);
            let nal = &self.buffer[start - 3..next - 3];
            let kind = self.buffer[start] & 0x1f;
            self.pending.extend_from_slice(nal);
            if (1..=5).contains(&kind) {
                units.push(std::mem::take(&mut self.pending));
            }
        }
        self.buffer.drain(..last - 3);
        units
    }
}

/// The offset just after each Annex-B start code (`00 00 01`) in `data`.
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
