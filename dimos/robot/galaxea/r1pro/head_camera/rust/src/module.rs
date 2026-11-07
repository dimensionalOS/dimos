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
//! The R1 Pro's two head cameras captured off V4L2 (raw UYVY) and hardware-encoded to H.264 with the Jetson's NVENC,
//! every frame, published for the Desktop gateway to pass through to the browser.

use std::sync::Arc;
use std::time::Duration;

use dimos_module::{native_config, Module, Output};
use lcm_msgs::foxglove_msgs::CompressedVideo;

use crate::relay::{relay, Camera, Encode};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// V4L2 device nodes of the two head cameras (GMSL links on the Tegra VI)
    left_device: String,
    right_device: String,
    left_frame_id: String,
    right_frame_id: String,
    /// capture + encode: the pipeline forces these caps and the NVENC bitrate and GOP
    #[validate(range(min = 1))]
    width: i64,
    #[validate(range(min = 1))]
    height: i64,
    #[validate(range(min = 1, max = 120))]
    fps: i64,
    #[validate(range(min = 100_000, max = 50_000_000))]
    bitrate_bps: i64,
    #[validate(range(min = 1, max = 600))]
    gop: i64,
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
    #[validate(range(min = 0.1, max = 60.0))]
    stall_timeout_s: f64,
    #[validate(range(min = 0.1, max = 3600.0))]
    stats_period_s: f64,
}

#[derive(Module)]
#[module(name = "r1_head_camera", setup = start)]
pub struct R1HeadCamera {
    #[output(encode = CompressedVideo::encode)]
    head_left_camera: Output<CompressedVideo>,
    #[output(encode = CompressedVideo::encode)]
    head_right_camera: Output<CompressedVideo>,

    #[config]
    config: Config,
}

impl R1HeadCamera {
    async fn start(&mut self) {
        let config = self.config.clone();
        let encode = Encode {
            width: config.width as u32,
            height: config.height as u32,
            fps: config.fps as u32,
            bitrate_bps: config.bitrate_bps as u32,
            gop: config.gop as u32,
        };
        let left = Arc::new(Camera {
            name: "head_left_camera".into(),
            device: config.left_device.clone(),
            frame_id: config.left_frame_id.clone(),
            video: self.head_left_camera.clone(),
        });
        let right = Arc::new(Camera {
            name: "head_right_camera".into(),
            device: config.right_device.clone(),
            frame_id: config.right_frame_id.clone(),
            video: self.head_right_camera.clone(),
        });
        let retry = Duration::from_secs_f64(config.retry_s);
        let stall = Duration::from_secs_f64(config.stall_timeout_s);
        let stats = Duration::from_secs_f64(config.stats_period_s);
        tokio::spawn(relay(left, encode.clone(), retry, stall, stats));
        tokio::spawn(relay(right, encode.clone(), retry, stall, stats));
    }
}
