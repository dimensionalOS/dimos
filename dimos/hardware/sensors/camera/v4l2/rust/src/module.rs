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
//! Publishes a V4L2 camera as JPEG `CompressedImage`s stamped with the driver's capture time. On a Jetson the
//! frames never touch the CPU: the VIC converts and NVJPG encodes them. Elsewhere, or if that path fails to
//! start, the raw frames are encoded with libjpeg-turbo.

use std::time::{Duration, Instant};

use dimos_generated_messages::{
    builtin_interfaces::msg::time::Time, sensor_msgs::msg::compressed_image::CompressedImage,
    std_msgs::msg::header::Header,
};
use dimos_module::{cdr, native_config, Module, Output};
use tracing::{info, warn};

use crate::capture::{fourcc, Capture};
use crate::clock::{driver_clocks, wall_s, CaptureClock};
use crate::jpeg::encode_packed_422;

const STATS_EVERY: Duration = Duration::from_secs(10);
const FRAME_TIMEOUT_MS: i32 = 1000;
/// Consecutive frame timeouts before the device is closed and reopened.
const MAX_TIMEOUTS: u32 = 3;

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// /dev/videoN, or better a /dev/v4l/by-path link that survives the nodes being renumbered.
    device: String,
    #[validate(range(min = 1, max = 16384))]
    width: i64,
    #[validate(range(min = 1, max = 16384))]
    height: i64,
    /// V4L2 pixel format the device is asked for, e.g. UYVY or YUYV.
    #[validate(length(equal = 4))]
    fourcc: String,
    frame_id: String,
    #[validate(range(min = 1, max = 100))]
    jpeg_quality: i64,
    /// Use the Jetson's hardware encoder when it is present; the CPU path covers everything else.
    hardware: bool,
    /// Seconds between attempts to open a device that is absent or held by another process.
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
}

#[derive(Module)]
#[module(name = "v4l2_camera", setup = start)]
pub struct V4L2Camera {
    #[output(encode = cdr::encode)]
    jpeg_out: Output<CompressedImage>,

    #[config]
    config: Config,
}

impl V4L2Camera {
    async fn start(&mut self) {
        let config = self.config.clone();
        let output = self.jpeg_out.clone();
        let runtime = tokio::runtime::Handle::current();
        std::thread::Builder::new()
            .name(format!("v4l2:{}", config.device))
            .spawn(move || run(&config, &output, &runtime))
            .expect("spawn the capture thread");
    }
}

fn run(config: &Config, output: &Output<CompressedImage>, runtime: &tokio::runtime::Handle) {
    let code = match fourcc(&config.fourcc) {
        Ok(code) => code,
        Err(error) => return warn!(%error, "bad fourcc, not capturing"),
    };
    let mut warned = false;
    loop {
        match Capture::open(
            &config.device,
            config.width as u32,
            config.height as u32,
            code,
            config.hardware,
            config.jpeg_quality as i32,
        ) {
            Ok(capture) => {
                warned = false;
                info!(device = %config.device, hardware = capture.is_hardware(), "streaming");
                pump(config, code, capture, output, runtime);
            }
            Err(error) if !warned => {
                warn!(device = %config.device, %error, "cannot open; is the vendor camera node holding it? retrying");
                warned = true;
            }
            Err(_) => {}
        }
        std::thread::sleep(Duration::from_secs_f64(config.retry_s));
    }
}

/// Reads, encodes and publishes until the device stops delivering.
fn pump(
    config: &Config,
    code: u32,
    mut capture: Capture,
    output: &Output<CompressedImage>,
    runtime: &tokio::runtime::Handle,
) {
    let mut clock = CaptureClock::new(driver_clocks(), wall_s);
    let (width, height) = (config.width as usize, config.height as usize);
    let mut timeouts = 0;
    let (mut frames, mut encode_s, mut window) = (0u32, 0.0, Instant::now());
    loop {
        let frame = match capture.next(FRAME_TIMEOUT_MS) {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                timeouts += 1;
                if timeouts >= MAX_TIMEOUTS {
                    return warn!(device = %config.device, "stopped delivering frames; reopening");
                }
                continue;
            }
            Err(error) => {
                return warn!(device = %config.device, %error, "capture failed; reopening")
            }
        };
        timeouts = 0;
        let stamp = clock.stamp(frame.driver_stamp_s).unwrap_or_else(wall_s);
        let data = if frame.jpeg {
            encode_s += frame.encode_s;
            frame.data.to_vec()
        } else {
            let started = Instant::now();
            let encoded = encode_packed_422(
                frame.data,
                frame.stride,
                width,
                height,
                code,
                config.jpeg_quality as i32,
            );
            encode_s += started.elapsed().as_secs_f64();
            match encoded {
                Ok(jpeg) => jpeg,
                Err(error) => return warn!(%error, "cannot encode this format; stopping"),
            }
        };
        let message = CompressedImage {
            header: Header {
                stamp: Time {
                    sec: stamp.trunc() as i32,
                    nanosec: (stamp.fract() * 1e9) as u32,
                },
                frame_id: config.frame_id.clone(),
            },
            format: "jpeg".into(),
            data,
        };
        runtime.block_on(output.publish(&message)).ok();
        frames += 1;
        if window.elapsed() >= STATS_EVERY {
            let seconds = window.elapsed().as_secs_f64();
            info!(
                frame_id = %config.frame_id,
                fps = frames as f64 / seconds,
                encode_ms = encode_s * 1e3 / frames as f64,
                hardware = capture.is_hardware(),
                "capture stats",
            );
            (frames, encode_s, window) = (0, 0.0, Instant::now());
        }
    }
}
