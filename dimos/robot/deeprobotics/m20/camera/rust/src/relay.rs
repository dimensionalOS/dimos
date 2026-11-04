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
//! One RTSP camera relayed as H.265 access units, every frame, reconnecting on any failure.
//!
//! Each message is one Annex-B access unit. A keyframe carries its VPS/SPS/PPS in band (the vendor
//! server sends them only in the SDP), so a subscriber that joins late starts decoding at the next
//! keyframe with nothing else to fetch.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Context, Result};
use dimos_module::Output;
use futures::StreamExt;
use lcm_msgs::builtin_interfaces::Time;
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::sensor_msgs::CameraInfo;
use retina::client::{
    PlayOptions, Session, SessionOptions, SetupOptions, TcpTransportOptions, Transport,
};
use retina::codec::{CodecItem, FrameFormat, ParametersRef};
use tracing::{info, warn};

/// One camera: where it streams from, where it sits, and where its messages go.
pub struct Camera {
    pub name: String,
    pub url: String,
    pub frame_id: String,
    /// Center in base_link, and the turn about z.
    pub xyz: [f64; 3],
    pub yaw_rad: f64,
    pub video: Output<CompressedVideo>,
    pub info: Output<CameraInfo>,
    /// The stream's pixel size once connected, for its CameraInfo.
    pub size: Mutex<Option<(u32, u32)>>,
}

/// Relay forever: connect, publish until the stream fails or stalls, wait `retry`, repeat; log rates every `stats`.
pub async fn relay(camera: Arc<Camera>, retry: Duration, stall: Duration, stats: Duration) {
    loop {
        match stream(&camera, stall, stats).await {
            Ok(()) => warn!(camera = %camera.name, "M20 camera stream ended"),
            Err(error) => {
                warn!(camera = %camera.name, error = format!("{error:#}"), "M20 camera stream unavailable")
            }
        }
        tokio::time::sleep(retry).await;
    }
}

async fn stream(camera: &Camera, stall: Duration, stats: Duration) -> Result<()> {
    let url = url::Url::parse(&camera.url).context("bad RTSP url")?;
    let mut session = within(stall, Session::describe(url, SessionOptions::default())).await?;
    let index = session
        .streams()
        .iter()
        .position(|s| s.media() == "video" && s.encoding_name().eq_ignore_ascii_case("h265"))
        .ok_or_else(|| anyhow!("no H.265 video stream in the SDP"))?;
    let setup = SetupOptions::default()
        .transport(Transport::Tcp(TcpTransportOptions::default()))
        .frame_format(FrameFormat::SIMPLE);
    within(stall, session.setup(index, setup)).await?;
    let mut playing = within(stall, session.play(PlayOptions::default()))
        .await?
        .demuxed()?;
    info!(camera = %camera.name, "M20 camera stream connected");

    let (mut frames, mut keyframes, mut bytes, mut window) = (0u32, 0u32, 0usize, Instant::now());
    loop {
        let item = match tokio::time::timeout(stall, playing.next()).await {
            Err(_) => return Err(anyhow!("no frame for {stall:?}")),
            Ok(None) => return Ok(()),
            Ok(Some(item)) => item?,
        };
        let CodecItem::VideoFrame(frame) = item else {
            continue;
        };
        if frame.has_new_parameters() || camera.size.lock().unwrap().is_none() {
            if let Some(ParametersRef::Video(params)) = playing.streams()[index].parameters() {
                *camera.size.lock().unwrap() = Some(params.pixel_dimensions());
            }
        }
        frames += 1;
        keyframes += frame.is_random_access_point() as u32;
        let data = frame.into_data();
        bytes += data.len();
        let ts = wall_s();
        let message = CompressedVideo {
            timestamp: Time {
                sec: ts.trunc() as i32,
                nanosec: (ts.fract() * 1e9) as i32,
            },
            frame_id: camera.frame_id.clone(),
            data,
            format: "h265".into(),
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

/// `future`, or an error once `limit` passes without it finishing.
async fn within<T, E>(
    limit: Duration,
    future: impl std::future::Future<Output = Result<T, E>>,
) -> Result<T>
where
    E: std::error::Error + Send + Sync + 'static,
{
    tokio::time::timeout(limit, future)
        .await
        .map_err(|_| anyhow!("no answer for {limit:?}"))?
        .map_err(Into::into)
}

/// The wall clock in seconds.
pub fn wall_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}
