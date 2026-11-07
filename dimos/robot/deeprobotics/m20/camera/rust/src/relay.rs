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
//! A `gst-launch-1.0` pipeline per camera does the Rockchip hardware work (`mppjpegdec` → `mpph264enc`) and writes an
//! Annex-B byte stream to its stdout; `h264parse config-interval=-1` puts the SPS/PPS in front of every keyframe so a
//! subscriber that joins mid-stream starts at the next one. This process splits that byte stream back into access
//! units (one video slice each) and publishes one message per unit, every frame.

use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use dimos_module::Output;
use lcm_msgs::builtin_interfaces::Time;
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::sensor_msgs::CameraInfo;
use tokio::io::AsyncReadExt;
use tokio::process::Command;
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
    info!(camera = %camera.name, device = %camera.device, "M20 camera pipeline started");

    let mut splitter = AccessUnits::default();
    let mut buffer = [0u8; 64 * 1024];
    let (mut frames, mut keyframes, mut bytes, mut window) = (0u32, 0u32, 0usize, Instant::now());
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
                    "M20 camera stats",
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
