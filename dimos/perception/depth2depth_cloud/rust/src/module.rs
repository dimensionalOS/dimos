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

//! `depth2depth_cloud`: a camera-frame point cloud from one colour camera, Depth
//! Anything calibrated per pixel to the recent lidar scans (see the depth2depth crate).
//!
//! A task picks the newest frame whose transform is known, waking when a frame or a transform
//! arrives. A model thread runs Depth Anything on it while a calibration thread calibrates the
//! previous frame on the CPU. Each stage holds only the newest frame waiting for it, so a stage
//! that falls behind drops stale frames (counted in the timing log) instead of queueing them.
//! Frames and lidar scans wait up to `max_tf_lag_s` for their pose. Scans are kept for
//! `lidar_history_s` in the world frame, so the anchors include ground the lidar saw a moment
//! ago and the camera sees now.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use depth2depth::{
    Calibration, CalibrationConfig, CloudOptions, Config as ModelConfig, Depth2Depth,
};
use dimos_generated_messages::builtin_interfaces::msg::time::Time;
use dimos_generated_messages::sensor_msgs::msg::camera_info::CameraInfo;
use dimos_generated_messages::sensor_msgs::msg::compressed_image::CompressedImage;
use dimos_generated_messages::sensor_msgs::msg::point_cloud2::PointCloud2;
use dimos_generated_messages::sensor_msgs::msg::point_field::PointField;
use dimos_generated_messages::std_msgs::msg::header::Header;
use dimos_module::cdr;
use dimos_module::pointcloud::extract_xyz;
use dimos_module::{native_config, warn_throttled, Input, Module, Output, Tf};
use nalgebra::{Isometry3, Point3};
use tokio::sync::Notify;
use tokio::task::JoinHandle;
use tracing::info;

use crate::undistort::{Lens, UndistortMap};

const TIMING_REPORT_EVERY: Duration = Duration::from_secs(5);
/// Scans up to this far after a frame still anchor it...
const SCAN_LEAD_S: f64 = 0.5;
/// ...and history is kept this far past `lidar_history_s`, since frames run behind the newest scan.
const HISTORY_MARGIN_S: f64 = 1.0;

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Model input size; both multiples of 14, smaller is faster. TensorRT builds an engine per size (minutes, once).
    #[validate(range(min = 56, max = 1036))]
    model_height: i64,
    #[validate(range(min = 56, max = 1036))]
    model_width: i64,
    /// JPEG decoded at 1/`decode_scale` of full size (1, 2, 4 or 8), before undistorting.
    #[validate(range(min = 1, max = 8))]
    decode_scale: i64,
    /// The pinhole image Depth Anything sees: size and focal length in pixels, centred. 0 takes the
    /// decoded image's size and the CameraInfo's focal length at that scale.
    #[validate(range(min = 0, max = 4096))]
    undistorted_width: i64,
    #[validate(range(min = 0, max = 4096))]
    undistorted_height: i64,
    #[validate(range(min = 0.0, max = 10000.0))]
    undistorted_focal_px: f64,
    /// Frame the lidar history is kept in; must be fixed while the robot moves.
    world_frame: String,
    /// Seconds of lidar scans used as anchors.
    #[validate(range(min = 0.0, max = 30.0))]
    lidar_history_s: f64,
    /// Lidar points farther than this from the camera are not anchors.
    #[validate(range(min = 0.1, max = 200.0))]
    max_anchor_range_m: f64,
    /// Largest gap between a stamp and the transform used for it.
    #[validate(range(min = 0.0, max = 5.0))]
    tf_tolerance_s: f64,
    /// Frames and scans wait this long for their transform before they are dropped.
    #[validate(range(min = 0.0, max = 2.0))]
    max_tf_lag_s: f64,
    /// Calibration, see `depth2depth::CalibrationConfig`.
    #[validate(range(min = 1.0, max = 1000.0))]
    sigma_px: f64,
    #[validate(range(min = 0.001, max = 10.0))]
    sigma_log_depth: f64,
    #[validate(range(min = 1, max = 256))]
    neighbours: i64,
    #[validate(range(min = 1, max = 64))]
    grid_step: i64,
    #[validate(range(min = 0.01, max = 100.0))]
    reach: f64,
    #[validate(range(min = 0.0, max = 1.0))]
    shape_ema: f64,
    #[validate(range(min = 1, max = 1000000))]
    min_anchors: i64,
    /// Pixels leaning on nearby lidar less than this (0..1) are left out of the cloud; 0 keeps all.
    #[validate(range(min = 0.0, max = 1.0))]
    min_support: f64,
    /// Cloud crop, then decimation (one pixel per block), then a point budget (0 = none).
    #[validate(range(min = 0.0, max = 1000.0))]
    min_range_m: f64,
    #[validate(range(min = 0.0, max = 1000.0))]
    max_range_m: f64,
    #[validate(range(min = 1, max = 64))]
    decimation: i64,
    #[validate(range(min = 0, max = 10000000))]
    max_points: i64,
    /// Frame of the cloud; empty takes the CameraInfo's, then the image's.
    frame_id: String,
    /// Keep only points whose height in `height_frame` is within [min, max]; empty frame keeps all.
    height_frame: String,
    min_height_m: f64,
    max_height_m: f64,
}

#[derive(Module)]
#[module(name = "depth2depth_cloud", setup = start, teardown = stop)]
pub struct Depth2DepthCloud {
    #[input(decode = cdr::decode, handler = on_image)]
    image: Input<CompressedImage>,

    #[input(decode = cdr::decode, handler = on_camera_info)]
    camera_info: Input<CameraInfo>,

    #[input(decode = cdr::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[tf]
    tf: Tf,

    #[output(encode = cdr::encode)]
    depth_cloud: Output<PointCloud2>,

    #[config]
    config: Config,

    shared: Arc<Shared>,
    tasks: Vec<JoinHandle<()>>,
    to_model: Arc<Latest<Picked>>,
    model_thread: Option<std::thread::JoinHandle<()>>,
}

/// What the handlers hand the worker.
#[derive(Default)]
struct Shared {
    frames: Mutex<VecDeque<CompressedImage>>,
    frame_arrived: Notify,
    camera_info: Mutex<Option<CameraInfo>>,
    /// Scans waiting for their transform to the world frame.
    pending: Mutex<VecDeque<PointCloud2>>,
    scan_arrived: Notify,
    history: Mutex<VecDeque<Scan>>,
}

/// One item handed between stages: a newer one replaces one still waiting (and counts it as dropped),
/// so a stage that falls behind always takes the newest.
struct Latest<T> {
    slot: Mutex<(Option<T>, bool)>,
    filled: Condvar,
    dropped: AtomicUsize,
}

impl<T> Default for Latest<T> {
    fn default() -> Self {
        Self {
            slot: Mutex::new((None, false)),
            filled: Condvar::new(),
            dropped: AtomicUsize::new(0),
        }
    }
}

impl<T> Latest<T> {
    fn put(&self, item: T) {
        if self.slot.lock().unwrap().0.replace(item).is_some() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        self.filled.notify_one();
    }

    /// The waiting item, blocking until there is one; None once closed.
    fn take(&self) -> Option<T> {
        let mut slot = self.slot.lock().unwrap();
        loop {
            if let Some(item) = slot.0.take() {
                return Some(item);
            }
            if slot.1 {
                return None;
            }
            slot = self.filled.wait(slot).unwrap();
        }
    }

    fn close(&self) {
        self.slot.lock().unwrap().1 = true;
        self.filled.notify_all();
    }

    /// Items replaced since the last call.
    fn take_dropped(&self) -> usize {
        self.dropped.swap(0, Ordering::Relaxed)
    }
}

/// A frame whose transforms are known, on its way to the model.
struct Picked {
    image: CompressedImage,
    frame_id: String,
    poses: Poses,
    info: CameraInfo,
}

/// A frame through the model, on its way to the calibration.
struct Predicted {
    header: Header,
    frame_id: String,
    poses: Poses,
    pred: Vec<f32>,
    map: Arc<UndistortMap>,
    /// Decode, undistort and model time.
    stages: [Duration; 3],
}

/// A frame's transforms from the camera.
struct Poses {
    world_from_camera: Isometry3<f64>,
    height_from_camera: Option<Isometry3<f64>>,
}

/// One lidar scan, already in the world frame.
struct Scan {
    stamp: f64,
    points: Vec<[f32; 3]>,
}

impl Depth2DepthCloud {
    async fn start(&mut self) {
        let worker = Arc::new(Worker {
            shared: self.shared.clone(),
            tf: self.tf.clone(),
            output: self.depth_cloud.clone(),
            runtime: tokio::runtime::Handle::current(),
            config: self.config.clone(),
            to_model: self.to_model.clone(),
            to_calibrate: Arc::default(),
        });
        let (picker, resolver, model, calibrator) =
            (worker.clone(), worker.clone(), worker.clone(), worker);
        self.tasks = vec![
            tokio::spawn(async move { picker.pick_frames().await }),
            tokio::spawn(async move { resolver.resolve_scans().await }),
        ];
        std::thread::spawn(move || calibrator.calibrate());
        self.model_thread = Some(std::thread::spawn(move || model.run_model()));
    }

    /// Stop the stages and let the model thread drop the model while CUDA is still up.
    async fn stop(&mut self) {
        for task in self.tasks.drain(..) {
            task.abort();
        }
        self.to_model.close();
        if let Some(model_thread) = self.model_thread.take() {
            // A model thread still building its TensorRT engine isn't waited for.
            let joined = tokio::task::spawn_blocking(move || model_thread.join());
            let _ = tokio::time::timeout(Duration::from_secs(2), joined).await;
        }
    }

    async fn on_image(&mut self, msg: CompressedImage) {
        let oldest = seconds(&msg.header.stamp) - self.config.max_tf_lag_s;
        let mut frames = self.shared.frames.lock().unwrap();
        frames.push_back(msg);
        while frames
            .front()
            .is_some_and(|f| seconds(&f.header.stamp) < oldest)
        {
            frames.pop_front();
        }
        drop(frames);
        self.shared.frame_arrived.notify_one();
    }

    async fn on_camera_info(&mut self, msg: CameraInfo) {
        *self.shared.camera_info.lock().unwrap() = Some(msg);
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        // A blueprint may feed this cloud back on the lidar's port (one mapper input for both); it is no anchor.
        if msg.header.frame_id == self.output_frame() {
            return;
        }
        let oldest = seconds(&msg.header.stamp) - self.config.max_tf_lag_s;
        let mut pending = self.shared.pending.lock().unwrap();
        pending.push_back(msg);
        while pending
            .front()
            .is_some_and(|scan| seconds(&scan.header.stamp) < oldest)
        {
            pending.pop_front();
        }
        drop(pending);
        self.shared.scan_arrived.notify_one();
    }
}

impl Depth2DepthCloud {
    fn output_frame(&self) -> String {
        let info = self.shared.camera_info.lock().unwrap();
        let info_frame = info.as_ref().map_or("", |i| i.header.frame_id.as_str());
        resolve_frame_id(&self.config.frame_id, info_frame, "").to_string()
    }
}

struct Worker {
    shared: Arc<Shared>,
    tf: Tf,
    output: Output<PointCloud2>,
    runtime: tokio::runtime::Handle,
    config: Config,
    to_model: Arc<Latest<Picked>>,
    to_calibrate: Arc<Latest<Predicted>>,
}

impl Worker {
    /// Hand the model the newest frame whose transforms are known, waking when a frame arrives or a
    /// transform for the newest one does.
    async fn pick_frames(&self) {
        let cfg = &self.config;
        loop {
            let frame_arrived = self.shared.frame_arrived.notified();
            tokio::pin!(frame_arrived);
            frame_arrived.as_mut().enable();
            let info = self.shared.camera_info.lock().unwrap().clone();
            let Some(info) = info else {
                if !self.shared.frames.lock().unwrap().is_empty() {
                    warn_throttled!(Duration::from_secs(5), "No CameraInfo yet, waiting.");
                }
                frame_arrived.await;
                continue;
            };
            if let Some(picked) = self.next_frame(info.clone()) {
                self.to_model.put(picked);
                continue;
            }
            let newest = self.shared.frames.lock().unwrap().back().map(|image| {
                let frame_id =
                    resolve_frame_id(&cfg.frame_id, &info.header.frame_id, &image.header.frame_id);
                (frame_id.to_string(), seconds(&image.header.stamp))
            });
            let Some((frame_id, stamp)) = newest else {
                frame_arrived.await;
                continue;
            };
            let wait = Duration::from_secs_f64(cfg.max_tf_lag_s);
            let lookup = |target: &str| {
                let tf = self.tf.clone();
                let (target, frame_id) = (target.to_string(), frame_id.clone());
                async move {
                    target.is_empty()
                        || tf
                            .lookup(&target, &frame_id)
                            .at(stamp)
                            .tolerance(cfg.tf_tolerance_s)
                            .within(wait)
                            .await
                            .is_some()
                }
            };
            let transforms =
                async { tokio::join!(lookup(&cfg.world_frame), lookup(&cfg.height_frame)) };
            tokio::select! {
                _ = frame_arrived => {}
                (world, height) = transforms => {
                    // Waited out: this frame and older ones never get their pose.
                    if !(world && height) {
                        self.shared
                            .frames
                            .lock()
                            .unwrap()
                            .retain(|image| seconds(&image.header.stamp) > stamp);
                    }
                }
            }
        }
    }

    /// Decode, undistort and run the model on each frame picked, handing it to `calibrate`.
    fn run_model(&self) {
        let cfg = &self.config;
        let model = match load_model(cfg) {
            Ok(model) => model,
            Err(error) => {
                tracing::error!(%error, "Could not load the depth model; no clouds will be published.");
                self.to_calibrate.close();
                return;
            }
        };
        let mut undistort: Option<(CameraInfo, usize, Arc<UndistortMap>)> = None;
        while let Some(Picked {
            image,
            frame_id,
            poses,
            info,
        }) = self.to_model.take()
        {
            let started = Instant::now();
            let Some((rgb, width, height)) = decode_rgb(&image, cfg.decode_scale as usize) else {
                warn_throttled!(Duration::from_secs(5), format = %image.format, "Could not decode a frame, skipped it.");
                continue;
            };
            let decoded = Instant::now();
            if !undistort.as_ref().is_some_and(|(held, held_width, _)| {
                held.k == info.k
                    && held.d == info.d
                    && held.distortion_model == info.distortion_model
                    && held.width == info.width
                    && *held_width == width
            }) {
                let scale = if info.width > 0 {
                    width as f64 / info.width as f64
                } else {
                    1.0 / cfg.decode_scale as f64
                };
                let Some(lens) = Lens::from_info(&info, scale) else {
                    warn_throttled!(
                        Duration::from_secs(5),
                        model = %info.distortion_model,
                        "CameraInfo has no usable intrinsics or an unsupported distortion model, skipped a frame."
                    );
                    continue;
                };
                let map = UndistortMap::new(
                    &lens,
                    or_derived(cfg.undistorted_width as usize, width),
                    or_derived(cfg.undistorted_height as usize, height),
                    if cfg.undistorted_focal_px > 0.0 {
                        cfg.undistorted_focal_px
                    } else {
                        lens.fx
                    },
                );
                undistort = Some((info.clone(), width, Arc::new(map)));
            }
            let map = undistort.as_ref().unwrap().2.clone();
            let pinhole_rgb = map.apply(&rgb, width, height);
            let undistorted = Instant::now();
            let pred = match model.predict(&pinhole_rgb, map.height, map.width) {
                Ok(pred) => pred,
                Err(error) => {
                    warn_throttled!(Duration::from_secs(5), %error, "Depth model failed on a frame.");
                    continue;
                }
            };
            let frame = Predicted {
                header: image.header,
                frame_id,
                poses,
                pred,
                map,
                stages: [
                    decoded - started,
                    undistorted - decoded,
                    undistorted.elapsed(),
                ],
            };
            self.to_calibrate.put(frame);
        }
        self.to_calibrate.close();
    }

    /// Calibrate each predicted frame to the lidar and publish its cloud.
    fn calibrate(&self) {
        let cfg = &self.config;
        let mut calibration = Calibration::new(CalibrationConfig {
            sigma_px: cfg.sigma_px as f32,
            sigma_log_depth: cfg.sigma_log_depth as f32,
            neighbours: cfg.neighbours as usize,
            grid_step: cfg.grid_step as usize,
            reach: cfg.reach as f32,
            shape_ema: cfg.shape_ema as f32,
            min_anchors: cfg.min_anchors as usize,
        });
        let options = CloudOptions {
            min_range_m: cfg.min_range_m as f32,
            max_range_m: cfg.max_range_m as f32,
            decimation: cfg.decimation as usize,
            max_points: (cfg.max_points > 0).then_some(cfg.max_points as usize),
            ..CloudOptions::default()
        };
        let mut timing = Timing::default();
        while let Some(frame) = self.to_calibrate.take() {
            let started = Instant::now();
            let (map, poses) = (&frame.map, &frame.poses);
            let anchors = self.anchors(
                &poses.world_from_camera.inverse(),
                seconds(&frame.header.stamp),
            );
            let anchored = Instant::now();
            let visible =
                depth2depth::cloud::visible_anchors(&anchors, &map.camera, map.height, map.width);
            let mut calibrated = calibration.apply(&frame.pred, map.height, map.width, &visible);
            for (depth, support) in calibrated.depth.iter_mut().zip(&calibrated.support) {
                if (*support as f64) < cfg.min_support {
                    *depth = 0.0;
                }
            }
            let mut points = calibrated.points(map.height, map.width, &map.camera, &options);
            if let Some(pose) = &poses.height_from_camera {
                points.retain(|&[x, y, z]| {
                    let height = (pose * Point3::new(x as f64, y as f64, z as f64)).z;
                    (cfg.min_height_m..=cfg.max_height_m).contains(&height)
                });
            }
            let calibrated_at = Instant::now();
            let cloud = make_cloud(&points, &frame.header, frame.frame_id);
            if let Err(error) = self.runtime.block_on(self.output.publish(&cloud)) {
                warn_throttled!(Duration::from_secs(5), %error, "Could not publish a cloud.");
            }
            let [decode, undistort, predict] = frame.stages;
            timing.record(
                [
                    decode,
                    undistort,
                    anchored - started,
                    predict,
                    calibrated_at - anchored,
                ],
                visible.len(),
                points.len(),
                [
                    self.to_model.take_dropped(),
                    self.to_calibrate.take_dropped(),
                ],
            );
        }
    }

    /// Move each scan into the history, in the world frame, once its transform arrives.
    async fn resolve_scans(&self) {
        let cfg = &self.config;
        loop {
            let scan_arrived = self.shared.scan_arrived.notified();
            tokio::pin!(scan_arrived);
            scan_arrived.as_mut().enable();
            let Some(msg) = self.shared.pending.lock().unwrap().pop_front() else {
                scan_arrived.await;
                continue;
            };
            let stamp = seconds(&msg.header.stamp);
            let Some(world_from_lidar) = self
                .tf
                .lookup(&cfg.world_frame, &msg.header.frame_id)
                .at(stamp)
                .tolerance(cfg.tf_tolerance_s)
                .within(Duration::from_secs_f64(cfg.max_tf_lag_s))
                .await
            else {
                continue;
            };
            let pose = isometry(&world_from_lidar);
            let points = match extract_xyz(&msg) {
                Ok(points) => points,
                Err(error) => {
                    warn_throttled!(Duration::from_secs(5), %error, "Unreadable lidar scan, dropped it.");
                    continue;
                }
            };
            let points = points
                .into_iter()
                .filter(|p| p.iter().all(|v| v.is_finite()))
                .map(|[x, y, z]| {
                    let p = pose * Point3::new(x as f64, y as f64, z as f64);
                    [p.x as f32, p.y as f32, p.z as f32]
                })
                .collect();
            let mut history = self.shared.history.lock().unwrap();
            history.push_back(Scan { stamp, points });
            let newest = history.iter().map(|scan| scan.stamp).fold(stamp, f64::max);
            history.retain(|scan| scan.stamp >= newest - cfg.lidar_history_s - HISTORY_MARGIN_S);
        }
    }

    /// The newest frame whose transforms are known; it and every older frame leave the queue.
    fn next_frame(&self, info: CameraInfo) -> Option<Picked> {
        let mut frames = self.shared.frames.lock().unwrap();
        let (index, (frame_id, poses)) =
            frames.iter().enumerate().rev().find_map(|(i, image)| {
                let frame_id = resolve_frame_id(
                    &self.config.frame_id,
                    &info.header.frame_id,
                    &image.header.frame_id,
                );
                let poses = self.poses(frame_id, seconds(&image.header.stamp))?;
                Some((i, (frame_id.to_string(), poses)))
            })?;
        let image = frames.drain(..=index).next_back()?;
        Some(Picked {
            image,
            frame_id,
            poses,
            info,
        })
    }

    fn poses(&self, frame_id: &str, stamp: f64) -> Option<Poses> {
        let pose = |target: &str| {
            let transform = self
                .tf
                .lookup(target, frame_id)
                .at(stamp)
                .tolerance(self.config.tf_tolerance_s)
                .get()?;
            Some(isometry(&transform))
        };
        let optional = |target: &str| match target {
            "" => Some(None),
            target => pose(target).map(Some),
        };
        Some(Poses {
            world_from_camera: pose(&self.config.world_frame)?,
            height_from_camera: optional(&self.config.height_frame)?,
        })
    }

    /// The history's points in the camera frame, within range and in front of it.
    fn anchors(&self, camera_from_world: &Isometry3<f64>, stamp: f64) -> Vec<[f32; 3]> {
        let history = self.shared.history.lock().unwrap();
        let max_range = self.config.max_anchor_range_m as f32;
        history
            .iter()
            .filter(|scan| {
                scan.stamp >= stamp - self.config.lidar_history_s
                    && scan.stamp <= stamp + SCAN_LEAD_S
            })
            .flat_map(|scan| scan.points.iter())
            .map(|&[x, y, z]| {
                let p = camera_from_world * Point3::new(x as f64, y as f64, z as f64);
                [p.x as f32, p.y as f32, p.z as f32]
            })
            .filter(|p| {
                p[2] > 0.0 && p[0] * p[0] + p[1] * p[1] + p[2] * p[2] < max_range * max_range
            })
            .collect()
    }
}

/// The model built into the depth2depth crate: TensorRT on a Jetson (candle's CUDA path is bound by kernel
/// launches there, 190 ms a frame against TensorRT's 17; the engine is built once, minutes, and cached),
/// Metal on a Mac, else CPU.
fn load_model(cfg: &Config) -> Result<Depth2Depth, String> {
    info!("Loading the depth model (on a Jetson, building the TensorRT engine first if it is not cached).");
    let model = Depth2Depth::load(ModelConfig {
        model_h: cfg.model_height as usize,
        model_w: cfg.model_width as usize,
        ..ModelConfig::default()
    })
    .map_err(|e| e.to_string())?;
    info!("Depth model loaded.");
    Ok(model)
}

/// Decode a JPEG as RGB at 1/`scale` of its size; None for anything that is not one.
/// Larger than any camera here; a header claiming more is corrupt, and would allocate before failing to decode.
const MAX_DECODED_PIXELS: usize = 64 << 20;

fn decode_rgb(image: &CompressedImage, scale: usize) -> Option<(Vec<u8>, usize, usize)> {
    let format = image.format.to_ascii_lowercase();
    if !(format.contains("jpeg") || format.contains("jpg") || format.is_empty()) {
        return None;
    }
    let scaling = match scale {
        8 => turbojpeg::ScalingFactor::ONE_EIGHTH,
        4 => turbojpeg::ScalingFactor::ONE_QUARTER,
        2 => turbojpeg::ScalingFactor::ONE_HALF,
        _ => turbojpeg::ScalingFactor::ONE,
    };
    let mut decompressor = turbojpeg::Decompressor::new().ok()?;
    let header = decompressor.read_header(&image.data).ok()?;
    decompressor.set_scaling_factor(scaling).ok()?;
    let (width, height) = (scaling.scale(header.width), scaling.scale(header.height));
    if width.saturating_mul(height) > MAX_DECODED_PIXELS {
        return None;
    }
    let mut pixels = vec![0u8; width * height * 3];
    decompressor
        .decompress(
            &image.data,
            turbojpeg::Image {
                pixels: &mut pixels[..],
                width,
                pitch: width * 3,
                height,
                format: turbojpeg::PixelFormat::RGB,
            },
        )
        .ok()?;
    Some((pixels, width, height))
}

fn make_cloud(points: &[[f32; 3]], source: &Header, frame_id: String) -> PointCloud2 {
    let field = |name: &str, offset: u32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32,
        count: 1,
    };
    let data: Vec<u8> = points
        .iter()
        .flatten()
        .flat_map(|v| v.to_le_bytes())
        .collect();
    PointCloud2 {
        header: Header {
            stamp: source.stamp.clone(),
            frame_id,
        },
        height: 1,
        width: points.len() as u32,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * points.len() as u32,
        data,
        is_dense: true,
    }
}

fn isometry(transform: &dimos_module::tf::Transform) -> Isometry3<f64> {
    Isometry3::from_parts(transform.translation().into(), transform.rotation())
}

/// A configured size, or the derived one when it is 0.
fn or_derived(configured: usize, derived: usize) -> usize {
    if configured > 0 {
        configured
    } else {
        derived
    }
}

fn seconds(stamp: &Time) -> f64 {
    stamp.sec as f64 + stamp.nanosec as f64 * 1e-9
}

/// Config, then calibration, then image frame, since drivers stamp frames nobody publishes a transform for.
fn resolve_frame_id<'a>(config: &'a str, info: &'a str, image: &'a str) -> &'a str {
    [config, info, image]
        .into_iter()
        .find(|f| !f.is_empty())
        .unwrap_or("")
}

/// Rolling per-stage timing, logged every few seconds so throughput is visible live.
#[derive(Default)]
struct Timing {
    frames: Vec<[Duration; 5]>,
    anchors: usize,
    points: usize,
    /// Frames replaced while waiting for the model, and for the calibration.
    dropped: [usize; 2],
    since: Option<Instant>,
}

impl Timing {
    fn record(
        &mut self,
        stages: [Duration; 5],
        anchors: usize,
        points: usize,
        dropped: [usize; 2],
    ) {
        let since = *self.since.get_or_insert_with(Instant::now);
        self.frames.push(stages);
        self.dropped = [self.dropped[0] + dropped[0], self.dropped[1] + dropped[1]];
        (self.anchors, self.points) = (anchors, points);
        let elapsed = since.elapsed();
        if elapsed < TIMING_REPORT_EVERY {
            return;
        }
        let median_ms = |stage: usize| {
            let mut ms: Vec<f64> = self
                .frames
                .iter()
                .map(|f| f[stage].as_secs_f64() * 1000.0)
                .collect();
            ms.sort_by(f64::total_cmp);
            ms[ms.len() / 2]
        };
        info!(
            fps = self.frames.len() as f64 / elapsed.as_secs_f64(),
            decode_ms = median_ms(0),
            undistort_ms = median_ms(1),
            anchors_ms = median_ms(2),
            predict_ms = median_ms(3),
            calibrate_ms = median_ms(4),
            anchors = self.anchors,
            points = self.points,
            dropped_waiting_for_model = self.dropped[0],
            dropped_waiting_for_calibration = self.dropped[1],
            "depth2depth_cloud timing (median ms per stage over the window)",
        );
        self.frames.clear();
        self.dropped = [0, 0];
        self.since = Some(Instant::now());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stage_behind_takes_the_newest_item_and_counts_the_ones_it_missed() {
        let latest = Latest::default();
        latest.put(1);
        latest.put(2);
        latest.put(3);
        assert_eq!((latest.take(), latest.take_dropped()), (Some(3), 2));
        latest.close();
        assert_eq!(latest.take(), None);
    }

    #[test]
    fn frame_id_precedence_is_config_then_calibration_then_image() {
        assert_eq!(resolve_frame_id("a", "b", "c"), "a");
        assert_eq!(resolve_frame_id("", "b", "c"), "b");
        assert_eq!(resolve_frame_id("", "", "c"), "c");
    }

    #[test]
    fn a_non_jpeg_format_is_refused_rather_than_fed_to_the_decoder() {
        let png = CompressedImage {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: String::new(),
            },
            format: "png".into(),
            data: Vec::new(),
        };
        assert!(decode_rgb(&png, 1).is_none());
    }

    #[test]
    fn the_cloud_carries_the_points_and_the_source_stamp() {
        let header = Header {
            stamp: Time { sec: 3, nanosec: 4 },
            frame_id: "image".into(),
        };
        let cloud = make_cloud(
            &[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            &header,
            "camera".into(),
        );
        assert_eq!(
            (
                cloud.width,
                cloud.header.frame_id.as_str(),
                cloud.header.stamp.nanosec
            ),
            (2, "camera", 4)
        );
        assert_eq!(
            extract_xyz(&cloud).unwrap(),
            vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        );
    }
}
