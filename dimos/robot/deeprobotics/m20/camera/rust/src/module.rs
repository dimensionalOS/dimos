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
//! The M20's front and rear RTSP cameras relayed as H.265 `CompressedVideo`, every frame, with their
//! static transforms and a CameraInfo sized to what each stream actually sends.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use dimos_module::{native_config, Module, Output, Tf};
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::sensor_msgs::CameraInfo;
use tracing::warn;

use crate::geometry::{camera_info, camera_transforms};
use crate::relay::{relay, wall_s, Camera};

#[native_config]
#[derive(Clone)]
pub struct Config {
    front_rtsp_url: String,
    rear_rtsp_url: String,
    /// Camera centers in base_link, and their turn about z.
    front_camera_xyz: [f64; 3],
    front_camera_yaw_rad: f64,
    rear_camera_xyz: [f64; 3],
    rear_camera_yaw_rad: f64,
    /// Focal length calibrated at `focal_length_width_px` wide, scaled to each stream's width.
    #[validate(range(exclusive_min = 0.0))]
    focal_length_px: f64,
    #[validate(range(min = 1))]
    focal_length_width_px: i64,
    /// Seconds between reconnect attempts.
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
    /// Seconds without an answer or a frame before the stream is dropped and reopened.
    #[validate(range(min = 0.1, max = 60.0))]
    stall_timeout_s: f64,
    /// Seconds between publishes of the transforms and CameraInfo.
    #[validate(range(min = 0.01, max = 60.0))]
    metadata_period_s: f64,
}

#[derive(Module)]
#[module(name = "m20_camera", setup = start)]
pub struct M20Camera {
    #[output(encode = CompressedVideo::encode)]
    front_camera: Output<CompressedVideo>,
    #[output(encode = CompressedVideo::encode)]
    rear_camera: Output<CompressedVideo>,
    #[output(encode = CameraInfo::encode)]
    front_camera_info: Output<CameraInfo>,
    #[output(encode = CameraInfo::encode)]
    rear_camera_info: Output<CameraInfo>,
    #[tf]
    tf: Tf,

    #[config]
    config: Config,
}

impl M20Camera {
    async fn start(&mut self) {
        let config = self.config.clone();
        let front = Arc::new(Camera {
            name: "front_camera".into(),
            url: config.front_rtsp_url.clone(),
            frame_id: "front_camera_optical".into(),
            xyz: config.front_camera_xyz,
            yaw_rad: config.front_camera_yaw_rad,
            video: self.front_camera.clone(),
            info: self.front_camera_info.clone(),
            size: Mutex::new(None),
        });
        let rear = Arc::new(Camera {
            name: "rear_camera".into(),
            url: config.rear_rtsp_url.clone(),
            frame_id: "rear_camera_optical".into(),
            xyz: config.rear_camera_xyz,
            yaw_rad: config.rear_camera_yaw_rad,
            video: self.rear_camera.clone(),
            info: self.rear_camera_info.clone(),
            size: Mutex::new(None),
        });
        let retry = Duration::from_secs_f64(config.retry_s);
        let stall = Duration::from_secs_f64(config.stall_timeout_s);
        tokio::spawn(relay(front.clone(), retry, stall));
        tokio::spawn(relay(rear.clone(), retry, stall));
        tokio::spawn(publish_metadata(config, [front, rear], self.tf.clone()));
    }
}

/// The static transforms every period, and each camera's CameraInfo once its size is known.
async fn publish_metadata(config: Config, cameras: [Arc<Camera>; 2], tf: Tf) {
    let mut ticks = tokio::time::interval(Duration::from_secs_f64(config.metadata_period_s));
    loop {
        ticks.tick().await;
        let ts = wall_s();
        let transforms: Vec<_> = cameras
            .iter()
            .flat_map(|camera| camera_transforms(&camera.name, camera.xyz, camera.yaw_rad, ts))
            .collect();
        if let Err(error) = tf.publish(&transforms).await {
            warn!(%error, "cannot publish the camera transforms");
        }
        for camera in &cameras {
            let Some((width, height)) = *camera.size.lock().unwrap() else {
                continue;
            };
            let info = camera_info(
                width,
                height,
                config.focal_length_px,
                config.focal_length_width_px as f64,
                &camera.frame_id,
                ts,
            );
            if let Err(error) = camera.info.publish(&info).await {
                warn!(%error, "cannot publish the camera info");
            }
        }
    }
}
