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
//! The M20's front and rear cameras relayed as H.264 `CompressedVideo`, every frame (optionally deskewed), with
//! their static transforms and a CameraInfo describing what each stream actually sends.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use dimos_module::{native_config, Module, Output, Tf};
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::sensor_msgs::CameraInfo;
use tracing::warn;

use rockchip_h264::deskew::{new_camera, Interpolation, Model, Options, Projection};

use crate::geometry::{camera_info, camera_transforms, intrinsics, rectified_camera_info};
use crate::relay::{relay, wall_s, Camera, Encode, Encoder};

fn valid_model(name: &str) -> Result<(), validator::ValidationError> {
    name.parse::<Model>().map(|_| ()).map_err(|_| {
        validator::ValidationError::new("distortion_model must be plumb_bob or equidistant")
    })
}

fn valid_projection(name: &str) -> Result<(), validator::ValidationError> {
    name.parse::<Projection>().map(|_| ()).map_err(|_| {
        validator::ValidationError::new(
            "deskew_projection must be pinhole, cylindrical or equirectangular",
        )
    })
}

fn valid_interpolation(name: &str) -> Result<(), validator::ValidationError> {
    name.parse::<Interpolation>().map(|_| ()).map_err(|_| {
        validator::ValidationError::new("deskew_interpolation must be nearest or bilinear")
    })
}

fn valid_encoder(name: &str) -> Result<(), validator::ValidationError> {
    name.parse::<Encoder>()
        .map(|_| ())
        .map_err(|_| validator::ValidationError::new("encoder must be rust or gstreamer"))
}

/// The lens settings as a whole: a model takes at most its own number of coefficients.
fn valid_lens(config: &Config) -> Result<(), validator::ValidationError> {
    let camera = config.camera(config.width.max(1) as u32, config.height.max(1) as u32);
    camera.validate().map_err(|error| {
        let mut e = validator::ValidationError::new("distortion");
        e.message = Some(format!("{error:#}").into());
        e
    })
}

#[native_config]
#[derive(Clone)]
#[validate(schema(function = "valid_lens"))]
pub struct Config {
    /// V4L2 device nodes of the two cameras (MJPEG off the vendor AOS board)
    front_device: String,
    rear_device: String,
    /// capture + encode: the pipeline forces these caps and the Rockchip H.264 encoder's bitrate and GOP
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
    /// `rust` (in-process, the rockchip-h264 crate) or `gstreamer` (a gst-launch child per camera)
    #[validate(custom(function = "valid_encoder"))]
    encoder: String,
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
    /// The lenses' distortion, as CameraInfo names it: `plumb_bob` (k1, k2, p1, p2, k3), `rational_polynomial`
    /// (k1, k2, p1, p2, k3, k4, k5, k6), or `equidistant` / `fisheye` / `kannala_brandt` (k1..k4)
    #[validate(custom(function = "valid_model"))]
    distortion_model: String,
    distortion_coefficients: Vec<f64>,
    /// Undistort every frame before encoding (rust encoder only); CameraInfo then describes the deskewed frames
    deskew: bool,
    /// `pinhole` (straight lines stay straight), `cylindrical` or `equirectangular` (wide views without the black
    /// wedges a pinhole needs)
    #[validate(custom(function = "valid_projection"))]
    deskew_projection: String,
    /// Deskewed frame size; 0 keeps the capture's
    #[validate(range(min = 0))]
    deskew_width: i64,
    #[validate(range(min = 0))]
    deskew_height: i64,
    /// 0 keeps only what every edge of the frame saw (no black corners), 1 the whole field of view
    #[validate(range(min = 0.0, max = 1.0))]
    deskew_balance: f64,
    /// `bilinear` or `nearest`
    #[validate(custom(function = "valid_interpolation"))]
    deskew_interpolation: String,
    /// Seconds between reconnect attempts.
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
    /// Seconds without an answer or a frame before the stream is dropped and reopened.
    #[validate(range(min = 0.1, max = 60.0))]
    stall_timeout_s: f64,
    /// Seconds between publishes of the transforms and CameraInfo.
    #[validate(range(min = 0.01, max = 60.0))]
    metadata_period_s: f64,
    /// Seconds between the per-camera fps / keyframe / bitrate log lines.
    #[validate(range(min = 0.1, max = 3600.0))]
    stats_period_s: f64,
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

impl Config {
    /// The deskew options, when deskewing is on.
    fn deskew_options(&self) -> Option<Options> {
        self.deskew.then(|| Options {
            projection: self
                .deskew_projection
                .parse()
                .unwrap_or(Projection::Pinhole),
            output_size: (self.deskew_width > 0 && self.deskew_height > 0)
                .then_some((self.deskew_width as u32, self.deskew_height as u32)),
            balance: self.deskew_balance,
            interpolation: self
                .deskew_interpolation
                .parse()
                .unwrap_or(Interpolation::Bilinear),
        })
    }

    /// The raw camera at a capture size.
    fn camera(&self, width: u32, height: u32) -> rockchip_h264::deskew::Intrinsics {
        intrinsics(
            width,
            height,
            self.focal_length_px,
            self.focal_length_width_px as f64,
            self.distortion_model.parse().unwrap_or(Model::PlumbBob),
            &self.distortion_coefficients,
        )
    }
}

impl M20Camera {
    async fn start(&mut self) {
        let config = self.config.clone();
        let encoder = config.encoder.parse().unwrap_or(Encoder::Rust);
        if config.deskew && encoder != Encoder::Rust {
            warn!("deskew needs the rust encoder; the gstreamer pipeline publishes the frames as captured");
        }
        let encode = Encode {
            width: config.width as u32,
            height: config.height as u32,
            fps: config.fps as u32,
            bitrate_bps: config.bitrate_bps as u32,
            gop: config.gop as u32,
            encoder,
            deskew: config.deskew_options().map(|options| {
                (
                    config.camera(config.width as u32, config.height as u32),
                    options,
                )
            }),
        };
        let front = Arc::new(Camera {
            name: "front_camera".into(),
            device: config.front_device.clone(),
            frame_id: "front_camera_optical".into(),
            xyz: config.front_camera_xyz,
            yaw_rad: config.front_camera_yaw_rad,
            video: self.front_camera.clone(),
            info: self.front_camera_info.clone(),
            size: Mutex::new(None),
        });
        let rear = Arc::new(Camera {
            name: "rear_camera".into(),
            device: config.rear_device.clone(),
            frame_id: "rear_camera_optical".into(),
            xyz: config.rear_camera_xyz,
            yaw_rad: config.rear_camera_yaw_rad,
            video: self.rear_camera.clone(),
            info: self.rear_camera_info.clone(),
            size: Mutex::new(None),
        });
        let retry = Duration::from_secs_f64(config.retry_s);
        let stall = Duration::from_secs_f64(config.stall_timeout_s);
        let stats = Duration::from_secs_f64(config.stats_period_s);
        tokio::spawn(relay(front.clone(), encode.clone(), retry, stall, stats));
        tokio::spawn(relay(rear.clone(), encode.clone(), retry, stall, stats));
        tokio::spawn(publish_metadata(config, [front, rear], self.tf.clone()));
    }
}

fn encoder_is_rust(config: &Config) -> bool {
    config
        .encoder
        .parse::<Encoder>()
        .map_or(true, |e| e == Encoder::Rust)
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
            let raw = config.camera(width, height);
            let info = match config.deskew_options() {
                Some(options) if encoder_is_rust(&config) => {
                    rectified_camera_info(&new_camera(&raw, &options), &camera.frame_id, ts)
                }
                _ => camera_info(&raw, &camera.frame_id, ts),
            };
            if let Err(error) = camera.info.publish(&info).await {
                warn!(%error, "cannot publish the camera info");
            }
        }
    }
}
