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

// RealSense D4xx camera; realsense/camera.py is the wrapper that launches it.
// The capture loop, align and the RGBD cloud run here so the Python process
// never sees a frame.

use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};
use std::ffi::{CStr, CString};
use std::io::Write;
use std::os::raw::c_void;
use std::ptr::{self, NonNull};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_module::nalgebra::{
    Isometry3, Matrix3, Quaternion, Rotation3, Translation3, UnitQuaternion, Vector3,
};
use dimos_module::{native_config, run_with_transport, Module, Output, Tf, Transform};
use lcm_msgs::geometry_msgs::{Quaternion as QuaternionMsg, Vector3 as Vector3Msg};
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2, PointField, RegionOfInterest};
use lcm_msgs::std_msgs::{Header, Time};
use realsense_rust::base::{Rs2Extrinsics, Rs2Intrinsics};
use realsense_rust::config::Config as RsConfig;
use realsense_rust::context::Context;
use realsense_rust::frame::{
    AccelFrame, ColorFrame, CompositeFrame, DepthFrame, FrameEx, GyroFrame, ImageFrame,
    InfraredFrame,
};
use realsense_rust::kind::{
    Rs2CameraInfo, Rs2DistortionModel, Rs2Extension, Rs2Format, Rs2Option, Rs2ProductLine,
    Rs2StreamKind,
};
use realsense_rust::pipeline::{ActivePipeline, InactivePipeline, PipelineProfile};
use realsense_rust::processing_blocks::align::Align;
use realsense_rust::sensor::Sensor;
use realsense_rust::stream_profile::StreamProfile;
use realsense_sys as sys;
use serde::{Deserialize, Serialize};
use tokio::runtime::Handle;

const MOTION_MODULE_NAME: &str = "Motion Module";
const IMU_CLOCK_WINDOW_SECONDS: i64 = 30;
const FRAME_TIMEOUT: Duration = Duration::from_secs(1);
// PointCloud2.from_rgbd's depth_trunc.
const DEPTH_TRUNC_M: f32 = 5.0;
// Body (REP-103) from optical axes, x y z w.
const OPTICAL_ROTATION: [f64; 4] = [-0.5, 0.5, -0.5, 0.5];
// lcm-gen fingerprint of sensor_msgs.ImuInfo; it has no generated bindings.
const IMU_INFO_FINGERPRINT: u64 = 0xe6aa5563f2a33280;

// ---- config, field for field with RealSenseCameraConfig ----

#[derive(Debug, Clone, Deserialize, Serialize)]
struct ImuInfoCfg {
    gyro_noise_density: f64,
    gyro_random_walk: f64,
    accel_noise_density: f64,
    accel_random_walk: f64,
}

/// Python's `None`, sent as a JSON null under a key that is always present.
/// native_config forbids `Option` so an absent key can't pass as None; a null
/// under a present key is what the wrapper guarantees.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(transparent)]
struct Nullable<T>(Option<T>);

#[native_config]
#[derive(Clone)]
struct Config {
    #[validate(range(min = 1))]
    width: i32,
    #[validate(range(min = 1))]
    height: i32,
    #[validate(range(min = 1))]
    fps: i32,
    frame_id: String,
    frame_id_prefix: Nullable<String>,
    align_depth_to_color: bool,
    enable_depth: bool,
    enable_color: bool,
    color_auto_exposure_priority: bool,
    enable_pointcloud: bool,
    enable_infrared: bool,
    emitter_enabled: bool,
    enable_imu: bool,
    #[validate(range(min = 1))]
    imu_hz: i32,
    imu_info: Nullable<ImuInfoCfg>,
    #[validate(range(exclusive_min = 0.0))]
    pointcloud_fps: f64,
    #[validate(range(min = 1))]
    pointcloud_decimation: i32,
    #[validate(range(exclusive_min = 0.0))]
    camera_info_fps: f64,
    serial_number: Nullable<String>,
}

impl Config {
    fn stream_color(&self) -> bool {
        self.enable_color || self.enable_pointcloud
    }
    fn stream_depth(&self) -> bool {
        self.enable_depth || self.enable_pointcloud
    }
    /// `<frame_id_prefix>/<frame_id>_<suffix>`, as Module.frame_id composes it.
    fn frame(&self, suffix: &str) -> String {
        let stem = &self.frame_id;
        match self.frame_id_prefix.0.as_deref() {
            Some(prefix) if !prefix.is_empty() => format!("{prefix}/{stem}_{suffix}"),
            _ => format!("{stem}_{suffix}"),
        }
    }
    fn camera_link(&self) -> String {
        self.frame("link")
    }
    fn color_frame(&self) -> String {
        self.frame("color_frame")
    }
    fn color_optical_frame(&self) -> String {
        self.frame("color_optical_frame")
    }
    fn depth_frame(&self) -> String {
        self.frame("depth_frame")
    }
    fn depth_optical_frame(&self) -> String {
        self.frame("depth_optical_frame")
    }
    fn infra1_frame(&self) -> String {
        self.frame("infra1_frame")
    }
    fn infra1_optical_frame(&self) -> String {
        self.frame("infra1_optical_frame")
    }
    fn infra2_frame(&self) -> String {
        self.frame("infra2_frame")
    }
    fn infra2_optical_frame(&self) -> String {
        self.frame("infra2_optical_frame")
    }
    fn imu_frame(&self) -> String {
        self.frame("imu_frame")
    }
    fn imu_optical_frame(&self) -> String {
        self.frame("imu_optical_frame")
    }
    fn serial(&self) -> Option<&str> {
        self.serial_number.0.as_deref()
    }
}

// ---- sensor_msgs.ImuInfo, hand-encoded ----

#[derive(Clone)]
struct ImuInfo {
    header: Header,
    gyro_noise_density: f64,
    gyro_random_walk: f64,
    accel_noise_density: f64,
    accel_random_walk: f64,
    frequency: f64,
}

impl ImuInfo {
    fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(8 + self.header.encoded_size() + 40);
        buf.write_all(&IMU_INFO_FINGERPRINT.to_be_bytes()).unwrap();
        self.header.encode_one(&mut buf).unwrap();
        for v in [
            self.gyro_noise_density,
            self.gyro_random_walk,
            self.accel_noise_density,
            self.accel_random_walk,
            self.frequency,
        ] {
            buf.write_all(&v.to_be_bytes()).unwrap();
        }
        buf
    }
}

// ---- module ----

/// What the capture thread learns at start and the tickers publish from.
struct CameraInfos {
    color: Option<CameraInfo>,
    depth: Option<CameraInfo>,
    infra1: Option<CameraInfo>,
    infra2: Option<CameraInfo>,
    depth_scale: f32,
}

#[derive(Clone)]
struct Rgbd {
    color: Image,
    depth: Image,
}

#[derive(Default)]
struct Shared {
    stop: AtomicBool,
    last_hardware_ts: Mutex<Option<f64>>,
    infos: Mutex<Option<CameraInfos>>,
    latest_rgbd: Mutex<Option<Rgbd>>,
}

impl Shared {
    fn stopped(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }
}

#[derive(Clone)]
struct Outs {
    color_image: Output<Image>,
    depth_image: Output<Image>,
    infrared_left: Output<Image>,
    infrared_right: Output<Image>,
    imu: Output<Imu>,
    imu_info: Output<ImuInfo>,
    pointcloud: Output<PointCloud2>,
    camera_info: Output<CameraInfo>,
    depth_camera_info: Output<CameraInfo>,
    infrared_left_camera_info: Output<CameraInfo>,
    infrared_right_camera_info: Output<CameraInfo>,
    tf: Tf,
}

#[derive(Module)]
#[module(name = "realsense", setup = start, teardown = stop)]
struct RealSense {
    #[output(encode = Image::encode)]
    color_image: Output<Image>,
    #[output(encode = Image::encode)]
    depth_image: Output<Image>,
    #[output(encode = Image::encode)]
    infrared_left: Output<Image>,
    #[output(encode = Image::encode)]
    infrared_right: Output<Image>,
    #[output(encode = Imu::encode)]
    imu: Output<Imu>,
    #[output(encode = ImuInfo::encode)]
    imu_info: Output<ImuInfo>,
    #[output(encode = PointCloud2::encode)]
    pointcloud: Output<PointCloud2>,
    #[output(encode = CameraInfo::encode)]
    camera_info: Output<CameraInfo>,
    #[output(encode = CameraInfo::encode)]
    depth_camera_info: Output<CameraInfo>,
    #[output(encode = CameraInfo::encode)]
    infrared_left_camera_info: Output<CameraInfo>,
    #[output(encode = CameraInfo::encode)]
    infrared_right_camera_info: Output<CameraInfo>,
    #[tf]
    tf: Tf,
    #[config]
    config: Config,

    shared: Arc<Shared>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl RealSense {
    async fn start(&mut self) {
        let cfg = Arc::new(self.config.clone());
        let handle = Handle::current();
        let outs = Outs {
            color_image: self.color_image.clone(),
            depth_image: self.depth_image.clone(),
            infrared_left: self.infrared_left.clone(),
            infrared_right: self.infrared_right.clone(),
            imu: self.imu.clone(),
            imu_info: self.imu_info.clone(),
            pointcloud: self.pointcloud.clone(),
            camera_info: self.camera_info.clone(),
            depth_camera_info: self.depth_camera_info.clone(),
            infrared_left_camera_info: self.infrared_left_camera_info.clone(),
            infrared_right_camera_info: self.infrared_right_camera_info.clone(),
            tf: self.tf.clone(),
        };

        // The motion module refuses to open once the video pipeline holds the
        // device, and two contexts opening it at once segfault librealsense, so
        // the IMU pipeline is started here, before the capture thread exists.
        if cfg.enable_imu {
            let (pipeline, rx) = imu_open(&cfg);
            let (c, s, h, o) = (
                cfg.clone(),
                self.shared.clone(),
                handle.clone(),
                outs.clone(),
            );
            self.threads.push(std::thread::spawn(move || {
                imu_thread(pipeline, rx, &c, &s, &h, &o)
            }));
        }
        let (c, s, h, o) = (
            cfg.clone(),
            self.shared.clone(),
            handle.clone(),
            outs.clone(),
        );
        self.threads
            .push(std::thread::spawn(move || capture_thread(&c, &s, &h, &o)));
        if cfg.enable_pointcloud {
            let (c, s, h, o) = (
                cfg.clone(),
                self.shared.clone(),
                handle.clone(),
                outs.clone(),
            );
            self.threads.push(std::thread::spawn(move || {
                pointcloud_thread(&c, &s, &h, &o)
            }));
        }
        let (c, s, h, o) = (cfg, self.shared.clone(), handle, outs);
        self.threads.push(std::thread::spawn(move || {
            camera_info_thread(&c, &s, &h, &o)
        }));
    }

    async fn stop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

// ---- small message helpers ----

fn unix_now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}

fn header(frame_id: &str, ts: f64) -> Header {
    let sec = ts.trunc();
    Header {
        seq: 0,
        stamp: Time {
            sec: sec as i32,
            nsec: ((ts - sec) * 1e9) as i32,
        },
        frame_id: frame_id.to_string(),
    }
}

fn with_ts(info: &CameraInfo, ts: f64) -> CameraInfo {
    let mut out = info.clone();
    out.header = header(&info.header.frame_id, ts);
    out
}

fn image<K>(
    frame: &ImageFrame<K>,
    encoding: &str,
    bytes_per_pixel: usize,
    frame_id: &str,
    ts: f64,
) -> Image {
    let data = unsafe {
        std::slice::from_raw_parts(
            frame.get_data() as *const _ as *const u8,
            frame.get_data_size(),
        )
    };
    Image {
        header: header(frame_id, ts),
        height: frame.height() as i32,
        width: frame.width() as i32,
        encoding: encoding.to_string(),
        is_bigendian: 0,
        step: (frame.width() * bytes_per_pixel) as i32,
        data: data.to_vec(),
    }
}

fn camera_info(intrinsics: &Rs2Intrinsics, frame_id: &str) -> CameraInfo {
    let (fx, fy) = (intrinsics.fx() as f64, intrinsics.fy() as f64);
    let (cx, cy) = (intrinsics.ppx() as f64, intrinsics.ppy() as f64);
    let model = match intrinsics.distortion().model {
        Rs2DistortionModel::None => "",
        Rs2DistortionModel::BrownConrady
        | Rs2DistortionModel::BrownConradyModified
        | Rs2DistortionModel::BrownConradyInverse => "plumb_bob",
        Rs2DistortionModel::FThetaFisheye | Rs2DistortionModel::KannalaBrandt => "equidistant",
    };
    CameraInfo {
        header: header(frame_id, 0.0),
        height: intrinsics.height() as i32,
        width: intrinsics.width() as i32,
        distortion_model: model.to_string(),
        D: intrinsics.0.coeffs.iter().map(|&c| c as f64).collect(),
        K: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P: [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        },
    }
}

fn optical_rotation() -> UnitQuaternion<f64> {
    let [x, y, z, w] = OPTICAL_ROTATION;
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
}

/// A librealsense extrinsic (optical axes) in body (REP-103) axes.
fn extrinsics_to_body(extrinsics: &Rs2Extrinsics) -> Isometry3<f64> {
    let body_from_optical = optical_rotation().to_rotation_matrix();
    let rotation: Vec<f64> = extrinsics.rotation().iter().map(|&v| v as f64).collect();
    // librealsense stores the rotation column-major.
    let optical_rotation = Matrix3::from_column_slice(&rotation);
    let translation: Vec<f64> = extrinsics.translation().iter().map(|&v| v as f64).collect();
    let optical_translation = Vector3::from_column_slice(&translation);

    let rotation = body_from_optical * optical_rotation * body_from_optical.transpose();
    let translation = body_from_optical * optical_translation;
    Isometry3::from_parts(
        Translation3::from(translation),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
    )
}

fn sensor_name(sensor: &Sensor) -> String {
    sensor
        .info(Rs2CameraInfo::Name)
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default()
}

fn wait_frames(pipeline: &mut ActivePipeline, shared: &Shared) -> Option<CompositeFrame> {
    match pipeline.wait(Some(FRAME_TIMEOUT)) {
        Ok(frames) => Some(frames),
        Err(_) => {
            // Frame timeouts are transient, from warm-up or USB stalls.
            if !shared.stopped() {
                tracing::warn!("RealSense: no frames within 1s - retrying");
            }
            None
        }
    }
}

fn fail(what: &str, err: impl std::fmt::Display) -> ! {
    tracing::error!("RealSense {what}: {err}");
    std::process::exit(2);
}

// ---- per-stream freshness, the frame-number gate ----

#[derive(Default)]
struct Freshness {
    last_numbers: HashMap<&'static str, u64>,
    dropped: HashMap<&'static str, u64>,
    repeated: HashMap<&'static str, u64>,
}

impl Freshness {
    /// This frame's own timestamp, or None if the stream has not advanced.
    ///
    /// wait() returns the latest frame of each stream, and under load they
    /// desync, so each stream is gated on its own frame number.
    fn check<F: FrameEx>(
        &mut self,
        key: &'static str,
        frame: Option<&F>,
        shared: &Shared,
    ) -> Option<f64> {
        let frame = frame?;
        let number = frame.frame_number();
        let previous = self.last_numbers.get(key).copied();
        if previous == Some(number) {
            *self.repeated.entry(key).or_default() += 1;
            return None;
        }
        if let Some(previous) = previous {
            if number > previous + 1 {
                let missed = number - previous - 1;
                let total = self.dropped.entry(key).or_default();
                *total += missed;
                tracing::warn!(
                    "RealSense {key} dropped {missed} frame(s) at {number} ({total} total)"
                );
            }
        }
        self.last_numbers.insert(key, number);
        let stamp = frame.timestamp() / 1000.0;
        *shared.last_hardware_ts.lock().unwrap() = Some(stamp);
        Some(stamp)
    }
}

// ---- capture thread: video streams, camera infos, tf ----

fn find_profile(
    profiles: &[StreamProfile],
    kind: Rs2StreamKind,
    index: Option<usize>,
) -> Option<&StreamProfile> {
    profiles
        .iter()
        .find(|p| p.kind() == kind && index.is_none_or(|i| p.index() == i))
}

fn capture_thread(cfg: &Config, shared: &Shared, handle: &Handle, outs: &Outs) {
    let context = Context::new().unwrap_or_else(|e| fail("context", e));
    let mut config = RsConfig::new();
    if let Some(serial) = cfg.serial() {
        let serial = CString::new(serial).unwrap_or_else(|e| fail("serial_number", e));
        config
            .enable_device_from_serial(&serial)
            .unwrap_or_else(|e| fail("enable_device", e));
    }
    let (w, h, fps) = (cfg.width as usize, cfg.height as usize, cfg.fps as usize);
    // The camera hands out RGB directly; camera.py asks for BGR and converts.
    if cfg.stream_color() {
        config
            .enable_stream(Rs2StreamKind::Color, None, w, h, Rs2Format::Rgb8, fps)
            .unwrap_or_else(|e| fail("color stream", e));
    }
    if cfg.stream_depth() {
        config
            .enable_stream(Rs2StreamKind::Depth, None, w, h, Rs2Format::Z16, fps)
            .unwrap_or_else(|e| fail("depth stream", e));
    }
    if cfg.enable_infrared {
        // index 1 = left imager, index 2 = right
        for index in 1..=2 {
            config
                .enable_stream(
                    Rs2StreamKind::Infrared,
                    Some(index),
                    w,
                    h,
                    Rs2Format::Y8,
                    fps,
                )
                .unwrap_or_else(|e| fail("infrared stream", e));
        }
    }
    let pipeline = InactivePipeline::try_from(&context).unwrap_or_else(|e| fail("pipeline", e));
    let mut pipeline = pipeline
        .start(Some(config))
        .unwrap_or_else(|e| fail("start", e));

    // Put every sensor on the host clock rather than its own boot clock.
    // The IMU pipeline deliberately keeps the motion module on the hardware clock.
    let sensors = pipeline.profile().device().sensors();
    for mut sensor in sensors {
        let name = sensor_name(&sensor);
        if name == MOTION_MODULE_NAME {
            continue;
        }
        if !sensor.supports_option(Rs2Option::GlobalTimeEnabled) {
            tracing::warn!(
                "RealSense {name} has no global timestamps, so its stream is on the \
                 device's own clock and cannot be fused with the others"
            );
            continue;
        }
        let _ = sensor.set_option(Rs2Option::GlobalTimeEnabled, 1.0);
    }

    // The IR imagers are the depth sensor, and it owns the emitter option.
    let mut depth_scale = 0.001f32;
    if cfg.stream_depth() || cfg.enable_infrared {
        let sensors = pipeline.profile().device().sensors();
        if let Some(mut depth_sensor) = sensors
            .into_iter()
            .find(|s| s.extension() == Rs2Extension::DepthSensor)
        {
            depth_scale = depth_sensor
                .get_option(Rs2Option::DepthUnits)
                .unwrap_or(depth_scale);
            if depth_sensor.supports_option(Rs2Option::EmitterEnabled) {
                let _ = depth_sensor.set_option(
                    Rs2Option::EmitterEnabled,
                    if cfg.emitter_enabled { 1.0 } else { 0.0 },
                );
            }
        }
    }
    if cfg.stream_color() {
        let sensors = pipeline.profile().device().sensors();
        if let Some(mut color_sensor) = sensors
            .into_iter()
            .find(|s| s.extension() == Rs2Extension::ColorSensor)
        {
            if color_sensor.supports_option(Rs2Option::AutoExposurePriority) {
                let _ = color_sensor.set_option(
                    Rs2Option::AutoExposurePriority,
                    if cfg.color_auto_exposure_priority {
                        1.0
                    } else {
                        0.0
                    },
                );
            }
        }
    }

    let mut align = None;
    if cfg.align_depth_to_color && cfg.stream_depth() {
        if cfg.stream_color() {
            align = Some(Align::new(Rs2StreamKind::Color, 1).unwrap_or_else(|e| fail("align", e)));
        } else {
            tracing::info!("align_depth_to_color ignored: color stream is disabled");
        }
    }

    // Camera infos from the running streams.
    let streams = pipeline.profile().streams();
    let color_intrinsics = if cfg.stream_color() {
        find_profile(streams, Rs2StreamKind::Color, None).and_then(|p| p.intrinsics().ok())
    } else {
        None
    };
    let color_info = color_intrinsics
        .as_ref()
        .map(|i| camera_info(i, &cfg.color_optical_frame()));
    let depth_info = if cfg.stream_depth() {
        match (&cfg.align_depth_to_color, &color_intrinsics) {
            // Aligned to color, depth uses color intrinsics and frame.
            (true, Some(i)) => Some(camera_info(i, &cfg.color_optical_frame())),
            _ => find_profile(streams, Rs2StreamKind::Depth, None)
                .and_then(|p| p.intrinsics().ok())
                .map(|i| camera_info(&i, &cfg.depth_optical_frame())),
        }
    } else {
        None
    };
    let (mut infra1_info, mut infra2_info) = (None, None);
    if cfg.enable_infrared {
        let infra1 = find_profile(streams, Rs2StreamKind::Infrared, Some(1));
        let infra2 = find_profile(streams, Rs2StreamKind::Infrared, Some(2));
        if let (Some(p1), Some(p2)) = (infra1, infra2) {
            infra1_info = p1
                .intrinsics()
                .ok()
                .map(|i| camera_info(&i, &cfg.infra1_optical_frame()));
            infra2_info = p2
                .intrinsics()
                .ok()
                .map(|i| camera_info(&i, &cfg.infra2_optical_frame()));
            // P[3] is -fx * baseline; left at 0 a stereo consumer sees infinite depth.
            // The pair is rectified, so the whole of their offset is along x.
            if let (Some(info), Ok(ext)) = (infra2_info.as_mut(), p2.extrinsics(p1)) {
                let baseline = (ext.translation()[0] as f64).abs();
                info.P[3] = -info.P[0] * baseline;
            }
        }
    }
    *shared.infos.lock().unwrap() = Some(CameraInfos {
        color: color_info,
        depth: depth_info,
        infra1: infra1_info,
        infra2: infra2_info,
        depth_scale,
    });

    // Place every imager below camera_link, which sits on the depth imager.
    let device_profiles: Vec<StreamProfile> = pipeline
        .profile()
        .device()
        .sensors()
        .iter()
        .flat_map(|s| s.stream_profiles())
        .collect();
    let mut mount_edges: Vec<(String, String, Isometry3<f64>)> = Vec::new();
    match find_profile(&device_profiles, Rs2StreamKind::Depth, None) {
        None => tracing::warn!("RealSense has no depth stream; publishing no camera frames on tf"),
        Some(origin) => {
            let mut wanted = vec![(cfg.depth_frame(), Rs2StreamKind::Depth, None)];
            if cfg.stream_color() {
                wanted.push((cfg.color_frame(), Rs2StreamKind::Color, None));
            }
            if cfg.enable_infrared {
                wanted.push((cfg.infra1_frame(), Rs2StreamKind::Infrared, Some(1)));
                wanted.push((cfg.infra2_frame(), Rs2StreamKind::Infrared, Some(2)));
            }
            if cfg.enable_imu {
                wanted.push((cfg.imu_frame(), Rs2StreamKind::Accel, None));
            }
            for (frame_id, kind, index) in wanted {
                let Some(profile) = find_profile(&device_profiles, kind, index) else {
                    tracing::warn!("RealSense has no stream for {frame_id}; not published on tf");
                    continue;
                };
                let Ok(extrinsics) = profile.extrinsics(origin) else {
                    tracing::warn!(
                        "RealSense has no extrinsics for {frame_id}; not published on tf"
                    );
                    continue;
                };
                mount_edges.push((
                    cfg.camera_link(),
                    frame_id.clone(),
                    extrinsics_to_body(&extrinsics),
                ));
                let optical = format!("{}_optical_frame", frame_id.trim_end_matches("_frame"));
                mount_edges.push((
                    frame_id,
                    optical,
                    Isometry3::from_parts(Translation3::identity(), optical_rotation()),
                ));
            }
        }
    }

    let depth_frame_id = if align.is_some() {
        cfg.color_optical_frame()
    } else {
        cfg.depth_optical_frame()
    };
    let mut fresh = Freshness::default();

    while !shared.stopped() {
        let Some(frames) = wait_frames(&mut pipeline, shared) else {
            continue;
        };

        // Grab the infrared stereo pair from the raw frameset before align()
        // (align rebuilds the frameset around depth+color and drops IR).
        let (mut infra1, mut infra2) = (None, None);
        if cfg.enable_infrared {
            for frame in frames.frames_of_type::<InfraredFrame>() {
                match frame.stream_profile().index() {
                    1 => infra1 = Some(frame),
                    2 => infra2 = Some(frame),
                    _ => {}
                }
            }
        }
        let frames = match align.as_mut() {
            Some(align) => {
                if let Err(e) = align.queue(frames) {
                    tracing::warn!("RealSense align: {e}");
                    continue;
                }
                match align.wait(FRAME_TIMEOUT) {
                    Ok(frames) => frames,
                    Err(e) => {
                        tracing::warn!("RealSense align: {e}");
                        continue;
                    }
                }
            }
            None => frames,
        };

        let color = if cfg.stream_color() {
            frames.frames_of_type::<ColorFrame>().pop()
        } else {
            None
        };
        let depth = if cfg.stream_depth() {
            frames.frames_of_type::<DepthFrame>().pop()
        } else {
            None
        };

        let color_ts = fresh.check("color", color.as_ref(), shared);
        let depth_ts = fresh.check("depth", depth.as_ref(), shared);
        let infra1_ts = fresh.check("infra1", infra1.as_ref(), shared);
        let infra2_ts = fresh.check("infra2", infra2.as_ref(), shared);

        let color_img = match (&color, color_ts) {
            (Some(frame), Some(ts)) => {
                Some(image(frame, "rgb8", 3, &cfg.color_optical_frame(), ts))
            }
            _ => None,
        };
        if let (Some(img), true) = (&color_img, cfg.enable_color) {
            let _ = handle.block_on(outs.color_image.publish(img));
        }
        let depth_img = match (&depth, depth_ts) {
            (Some(frame), Some(ts)) => Some(image(frame, "16UC1", 2, &depth_frame_id, ts)),
            _ => None,
        };
        if let (Some(img), true) = (&depth_img, cfg.enable_depth) {
            let _ = handle.block_on(outs.depth_image.publish(img));
        }
        if let (Some(frame), Some(ts)) = (&infra1, infra1_ts) {
            let _ = handle.block_on(outs.infrared_left.publish(&image(
                frame,
                "mono8",
                1,
                &cfg.infra1_optical_frame(),
                ts,
            )));
        }
        if let (Some(frame), Some(ts)) = (&infra2, infra2_ts) {
            let _ = handle.block_on(outs.infrared_right.publish(&image(
                frame,
                "mono8",
                1,
                &cfg.infra2_optical_frame(),
                ts,
            )));
        }

        // Latest pair for the pointcloud ticker.
        if let (true, Some(color), Some(depth)) = (cfg.enable_pointcloud, color_img, depth_img) {
            *shared.latest_rgbd.lock().unwrap() = Some(Rgbd { color, depth });
        }

        let latest = [color_ts, depth_ts, infra1_ts, infra2_ts]
            .into_iter()
            .flatten()
            .fold(None, |m: Option<f64>, t| Some(m.map_or(t, |m| m.max(t))));
        if let Some(ts) = latest {
            let mut transforms = Vec::with_capacity(mount_edges.len());
            for (parent, child, iso) in &mount_edges {
                transforms.push(Transform::new(parent.clone(), child.clone(), ts, *iso));
            }
            let _ = handle.block_on(outs.tf.publish(&transforms));
        }
    }

    pipeline.stop();
    if !fresh.dropped.is_empty() || !fresh.repeated.is_empty() {
        tracing::info!(
            "RealSense capture ended: dropped {:?}, repeats suppressed {:?}",
            fresh.dropped,
            fresh.repeated
        );
    }
}

// ---- camera_info ticker ----

fn camera_info_thread(cfg: &Config, shared: &Shared, handle: &Handle, outs: &Outs) {
    let period = Duration::from_secs_f64(1.0 / cfg.camera_info_fps);
    let mut tick = std::time::Instant::now();
    loop {
        std::thread::sleep(period.saturating_sub(tick.elapsed()));
        tick = std::time::Instant::now();
        if shared.stopped() {
            return;
        }
        let Some(ts) = *shared.last_hardware_ts.lock().unwrap() else {
            continue;
        };
        // Copy out under the lock, publish after; the capture thread takes it too.
        let snapshot = shared.infos.lock().unwrap().as_ref().map(|i| {
            (
                i.color.clone(),
                i.depth.clone(),
                i.infra1.clone(),
                i.infra2.clone(),
            )
        });
        let Some((color, depth, infra1, infra2)) = snapshot else {
            continue;
        };
        let published = [
            (&color, &outs.camera_info, cfg.enable_color),
            (&depth, &outs.depth_camera_info, cfg.enable_depth),
            (&infra1, &outs.infrared_left_camera_info, true),
            (&infra2, &outs.infrared_right_camera_info, true),
        ];
        for (info, out, enabled) in published {
            if let (Some(info), true) = (info, enabled) {
                let _ = handle.block_on(out.publish(&with_ts(info, ts)));
            }
        }
        if let (true, Some(imu_info)) = (cfg.enable_imu, &cfg.imu_info.0) {
            let msg = ImuInfo {
                header: header(&cfg.imu_optical_frame(), ts),
                gyro_noise_density: imu_info.gyro_noise_density,
                gyro_random_walk: imu_info.gyro_random_walk,
                accel_noise_density: imu_info.accel_noise_density,
                accel_random_walk: imu_info.accel_random_walk,
                frequency: cfg.imu_hz as f64,
            };
            let _ = handle.block_on(outs.imu_info.publish(&msg));
        }
    }
}

// ---- pointcloud ticker: PointCloud2.from_rgbd on every stride-th pixel ----

fn pointcloud_thread(cfg: &Config, shared: &Shared, handle: &Handle, outs: &Outs) {
    let period = Duration::from_secs_f64(1.0 / cfg.pointcloud_fps);
    let stride = cfg.pointcloud_decimation as usize;
    loop {
        let tick = std::time::Instant::now();
        // Copy out under the locks and release them before any work or sleep;
        // the capture thread takes both, and a std Mutex is not fair.
        let rgbd = shared.latest_rgbd.lock().unwrap().clone();
        let infos = shared
            .infos
            .lock()
            .unwrap()
            .as_ref()
            .map(|i| (i.color.clone(), i.depth_scale));
        if let (Some(rgbd), Some((Some(info), depth_scale))) = (rgbd, infos) {
            let cloud = rgbd_to_cloud(&rgbd, &info, depth_scale, stride);
            let _ = handle.block_on(outs.pointcloud.publish(&cloud));
        }
        std::thread::sleep(period.saturating_sub(tick.elapsed()));
        if shared.stopped() {
            return;
        }
    }
}

fn empty_cloud(header: &Header) -> PointCloud2 {
    let field = |name: &str, offset: i32| PointField {
        name: name.to_string(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    PointCloud2 {
        header: header.clone(),
        height: 0,
        width: 0,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("rgb", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 0,
        data: Vec::new(),
        is_dense: true,
    }
}

fn rgbd_to_cloud(rgbd: &Rgbd, info: &CameraInfo, depth_scale: f32, stride: usize) -> PointCloud2 {
    let (fx, fy) = (info.K[0] as f32, info.K[4] as f32);
    let (cx, cy) = (info.K[2] as f32, info.K[5] as f32);
    let (w, h) = (rgbd.depth.width as usize, rgbd.depth.height as usize);
    let rgb = &rgbd.color.data;
    if rgb.len() != w * h * 3 || rgbd.depth.data.len() != w * h * 2 {
        // Unaligned depth, or streams at different resolutions.
        static WARNED: AtomicBool = AtomicBool::new(false);
        if !WARNED.swap(true, Ordering::Relaxed) {
            tracing::warn!(
                "RealSense pointcloud needs color and depth at one resolution; got color {} bytes, depth {}x{}",
                rgb.len(),
                rgbd.depth.width,
                rgbd.depth.height
            );
        }
        return empty_cloud(&rgbd.depth.header);
    }

    // Every stride-th pixel, back-projected through the color pinhole. A dense
    // image-shaped cloud thins best in pixel space; a voxel hash costs 10x more
    // and keeps most of the points anyway.
    let stride = stride.max(1);
    let mut points: Vec<[f32; 3]> = Vec::with_capacity(w * h / (stride * stride));
    let mut colors: Vec<[f32; 3]> = Vec::with_capacity(points.capacity());
    for v in (0..h).step_by(stride) {
        for u in (0..w).step_by(stride) {
            let i = v * w + u;
            let z = u16::from_le_bytes([rgbd.depth.data[2 * i], rgbd.depth.data[2 * i + 1]]) as f32
                * depth_scale;
            if z <= 0.0 || z > DEPTH_TRUNC_M {
                continue;
            }
            points.push([(u as f32 - cx) * z / fx, (v as f32 - cy) * z / fy, z]);
            let c = &rgb[i * 3..i * 3 + 3];
            colors.push([
                c[0] as f32 / 255.0,
                c[1] as f32 / 255.0,
                c[2] as f32 / 255.0,
            ]);
        }
    }

    // x y z rgb as four float32s; rgb packed [0, r, g, b] per the ROS convention.
    let mut data = Vec::with_capacity(points.len() * 16);
    for (p, c) in points.iter().zip(&colors) {
        for v in p {
            data.extend_from_slice(&v.to_le_bytes());
        }
        let packed =
            ((c[0] * 255.0) as u32) << 16 | ((c[1] * 255.0) as u32) << 8 | (c[2] * 255.0) as u32;
        data.extend_from_slice(&packed.to_le_bytes());
    }
    let field = |name: &str, offset: i32| PointField {
        name: name.to_string(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    let n = points.len() as i32;
    PointCloud2 {
        header: rgbd.depth.header.clone(),
        height: if n == 0 { 0 } else { 1 },
        width: n,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("rgb", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: true,
    }
}

// ---- IMU thread: own pipeline, hardware clock re-anchored to the host ----

/// Map a motion-module hardware stamp onto the host clock.
///
/// Delivery latency only ever pushes an arrival later, so the smallest offset
/// seen recently is the closest estimate of the true one. One minimum per second
/// lets the estimate follow the device's slow drift without rescanning every sample.
#[derive(Default)]
struct HostClock {
    offsets: VecDeque<(i64, f64)>,
    offset: f64,
}

impl HostClock {
    fn host_time(&mut self, device_ts: f64) -> f64 {
        let now = unix_now();
        let offset = now - device_ts;
        let second = now as i64;
        match self.offsets.back_mut() {
            Some(last) if last.0 == second => last.1 = last.1.min(offset),
            _ => {
                self.offsets.push_back((second, offset));
                while self
                    .offsets
                    .front()
                    .is_some_and(|f| f.0 < second - IMU_CLOCK_WINDOW_SECONDS)
                {
                    self.offsets.pop_front();
                }
                self.offset = self
                    .offsets
                    .iter()
                    .map(|o| o.1)
                    .fold(f64::INFINITY, f64::min);
            }
        }
        device_ts + self.offset
    }
}

type Sample = (f64, [f32; 3]);

/// One Imu per gyro sample, with the accelerometer read at that instant.
/// The two are sampled independently and never share a timestamp.
#[derive(Default)]
struct ImuPairer {
    accel_history: VecDeque<Sample>,
    pending_gyro: VecDeque<Sample>,
}

impl ImuPairer {
    fn accel(&mut self, sample: Sample) {
        if self.accel_history.len() == 2 {
            self.accel_history.pop_front();
        }
        self.accel_history.push_back(sample);
    }

    fn gyro(&mut self, sample: Sample) {
        if self.pending_gyro.len() == 16 {
            self.pending_gyro.pop_front();
        }
        self.pending_gyro.push_back(sample);
    }

    fn next(&mut self) -> Option<Sample2> {
        if self.accel_history.len() != 2 {
            return None;
        }
        let (start_ts, start) = self.accel_history[0];
        let (end_ts, end) = self.accel_history[1];
        let (ts, angular) = *self.pending_gyro.front()?;
        if ts > end_ts {
            return None; // no accelerometer sample past it yet
        }
        self.pending_gyro.pop_front();
        let span = end_ts - start_ts;
        let ratio = if span <= 0.0 {
            0.0
        } else {
            ((ts - start_ts) / span).clamp(0.0, 1.0) as f32
        };
        let mut linear = [0.0f32; 3];
        for k in 0..3 {
            linear[k] = start[k] + (end[k] - start[k]) * ratio;
        }
        Some((ts, angular, linear))
    }
}

type Sample2 = (f64, [f32; 3], [f32; 3]);

fn imu_message(frame_id: &str, (ts, angular, linear): Sample2) -> Imu {
    Imu {
        header: header(frame_id, ts),
        orientation: QuaternionMsg {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        orientation_covariance: [0.0; 9],
        angular_velocity: Vector3Msg {
            x: angular[0] as f64,
            y: angular[1] as f64,
            z: angular[2] as f64,
        },
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: Vector3Msg {
            x: linear[0] as f64,
            y: linear[1] as f64,
            z: linear[2] as f64,
        },
        linear_acceleration_covariance: [0.0; 9],
    }
}

/// Every rate the device offers for a stream.
fn stream_rates(context: &Context, cfg: &Config, kind: Rs2StreamKind) -> BTreeSet<i32> {
    let mut rates = BTreeSet::new();
    for device in context.query_devices(HashSet::from([Rs2ProductLine::Any])) {
        if let Some(serial) = cfg.serial() {
            let device_serial = device
                .info(Rs2CameraInfo::SerialNumber)
                .map(|s| s.to_string_lossy().into_owned());
            if device_serial.as_deref() != Some(serial) {
                continue;
            }
        }
        for sensor in device.sensors() {
            for profile in sensor.stream_profiles() {
                if profile.kind() == kind {
                    rates.insert(profile.framerate());
                }
            }
        }
    }
    rates
}

// ---- IMU: librealsense's frame callback, so every sample arrives exactly once ----
//
// realsense-rust only wraps wait_for_frames, whose syncer hands back the latest
// sample of each stream and so repeats or drops IMU samples. The callback start
// is raw realsense-sys; the frames it yields are wrapped back into the crate's
// types, which own and release them.

enum Motion {
    Accel(AccelFrame),
    Gyro(GyroFrame),
}

struct ImuPipeline {
    pipe: *mut sys::rs2_pipeline,
    config: *mut sys::rs2_config,
    context: *mut sys::rs2_context,
    sender: *mut mpsc::Sender<Motion>,
}

// Raw handles used from one thread at a time; librealsense itself is thread-safe.
unsafe impl Send for ImuPipeline {}

impl Drop for ImuPipeline {
    fn drop(&mut self) {
        unsafe {
            // stop() joins the callback thread, so the sender is free afterwards.
            sys::rs2_pipeline_stop(self.pipe, ptr::null_mut());
            sys::rs2_delete_pipeline(self.pipe);
            sys::rs2_delete_config(self.config);
            sys::rs2_delete_context(self.context);
            drop(Box::from_raw(self.sender));
        }
    }
}

/// Exits on a librealsense error, as `fail` does.
unsafe fn rs_check(err: *mut sys::rs2_error, what: &str) {
    if err.is_null() {
        return;
    }
    let msg = CStr::from_ptr(sys::rs2_get_error_message(err))
        .to_string_lossy()
        .into_owned();
    sys::rs2_free_error(err);
    fail(what, msg);
}

unsafe fn frame_kind(frame: NonNull<sys::rs2_frame>) -> Option<Rs2StreamKind> {
    let mut err = ptr::null_mut();
    let profile = sys::rs2_get_frame_stream_profile(frame.as_ptr(), &mut err);
    if !err.is_null() {
        sys::rs2_free_error(err);
        return None;
    }
    let profile = StreamProfile::try_from(NonNull::new(profile.cast_mut())?).ok()?;
    Some(profile.kind())
}

/// `user` is the `Sender<Motion>` leaked by imu_open.
unsafe extern "C" fn on_motion_frame(frame: *mut sys::rs2_frame, user: *mut c_void) {
    let Some(ptr) = NonNull::new(frame) else {
        return;
    };
    let motion = match frame_kind(ptr) {
        Some(Rs2StreamKind::Accel) => AccelFrame::try_from(ptr).ok().map(Motion::Accel),
        Some(Rs2StreamKind::Gyro) => GyroFrame::try_from(ptr).ok().map(Motion::Gyro),
        _ => None,
    };
    match motion {
        Some(motion) => {
            let _ = (*user.cast::<mpsc::Sender<Motion>>()).send(motion);
        }
        None => sys::rs2_release_frame(frame),
    }
}

fn imu_open(cfg: &Config) -> (ImuPipeline, mpsc::Receiver<Motion>) {
    let context = Context::new().unwrap_or_else(|e| fail("context", e));
    let offered = stream_rates(&context, cfg, Rs2StreamKind::Gyro);
    if !offered.contains(&cfg.imu_hz) {
        fail(
            "imu_hz",
            format!(
                "{} is not offered by this camera; it has {:?}",
                cfg.imu_hz, offered
            ),
        );
    }
    let accel_hz = stream_rates(&context, cfg, Rs2StreamKind::Accel)
        .into_iter()
        .max()
        .unwrap_or(cfg.imu_hz);
    drop(context);

    let (tx, rx) = mpsc::channel();
    let sender = Box::into_raw(Box::new(tx));
    let pipeline = unsafe {
        let mut err = ptr::null_mut();
        let context = sys::rs2_create_context(sys::RS2_API_VERSION as i32, &mut err);
        rs_check(err, "imu context");
        let config = sys::rs2_create_config(&mut err);
        rs_check(err, "imu config");
        if let Some(serial) = cfg.serial() {
            let serial = CString::new(serial).unwrap_or_else(|e| fail("serial_number", e));
            sys::rs2_config_enable_device(config, serial.as_ptr(), &mut err);
            rs_check(err, "enable_device");
        }
        let motion = sys::rs2_format_RS2_FORMAT_MOTION_XYZ32F;
        let accel = sys::rs2_stream_RS2_STREAM_ACCEL;
        sys::rs2_config_enable_stream(config, accel, -1, 0, 0, motion, accel_hz, &mut err);
        rs_check(err, "accel stream");
        let gyro = sys::rs2_stream_RS2_STREAM_GYRO;
        sys::rs2_config_enable_stream(config, gyro, -1, 0, 0, motion, cfg.imu_hz, &mut err);
        rs_check(err, "gyro stream");
        let pipe = sys::rs2_create_pipeline(context, &mut err);
        rs_check(err, "imu pipeline");
        let profile = sys::rs2_pipeline_start_with_config_and_callback(
            pipe,
            config,
            Some(on_motion_frame),
            sender.cast(),
            &mut err,
        );
        rs_check(err, "imu start");
        let pipeline = ImuPipeline {
            pipe,
            config,
            context,
            sender,
        };

        // librealsense's global-time fit for the IMU goes wrong once the camera bus is
        // full (IntelRealSense/librealsense#9131, fix unreleased in #15360). The raw
        // hardware clock is stable, so take it and re-anchor it in HostClock::host_time.
        let profile = PipelineProfile::try_from(NonNull::new(profile).unwrap())
            .unwrap_or_else(|e| fail("imu profile", e));
        for mut sensor in profile.device().sensors() {
            if sensor_name(&sensor) == MOTION_MODULE_NAME {
                let _ = sensor.set_option(Rs2Option::GlobalTimeEnabled, 0.0);
            }
        }
        pipeline
    };
    (pipeline, rx)
}

fn imu_thread(
    pipeline: ImuPipeline,
    rx: mpsc::Receiver<Motion>,
    cfg: &Config,
    shared: &Shared,
    handle: &Handle,
    outs: &Outs,
) {
    let frame_id = cfg.imu_optical_frame();
    let mut clock = HostClock::default();
    let mut pairer = ImuPairer::default();
    while !shared.stopped() {
        match rx.recv_timeout(FRAME_TIMEOUT) {
            Ok(Motion::Accel(f)) => {
                pairer.accel((clock.host_time(f.timestamp() / 1000.0), *f.acceleration()))
            }
            Ok(Motion::Gyro(f)) => pairer.gyro((
                clock.host_time(f.timestamp() / 1000.0),
                *f.rotational_velocity(),
            )),
            Err(_) => {
                if !shared.stopped() {
                    tracing::warn!("RealSense: no IMU samples within 1s - retrying");
                }
                continue;
            }
        }
        while let Some(sample) = pairer.next() {
            let _ = handle.block_on(outs.imu.publish(&imu_message(&frame_id, sample)));
        }
    }
    drop(pipeline);
}

#[tokio::main]
async fn main() {
    run_with_transport::<RealSense>().await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pairer_interpolates_accel_at_gyro_time() {
        let mut p = ImuPairer::default();
        p.accel((1.0, [0.0, 0.0, 0.0]));
        p.accel((2.0, [2.0, 4.0, 6.0]));
        p.gyro((1.5, [0.1, 0.2, 0.3]));
        let (ts, ang, lin) = p.next().unwrap();
        assert_eq!(ts, 1.5);
        assert_eq!(ang, [0.1, 0.2, 0.3]);
        assert_eq!(lin, [1.0, 2.0, 3.0]);
        // A gyro sample past the newest accel waits for the next one.
        p.gyro((2.5, [0.0; 3]));
        assert!(p.next().is_none());
    }

    #[test]
    fn optical_edge_matches_python_constant() {
        let q = optical_rotation();
        assert!((q.i + 0.5).abs() < 1e-12 && (q.j - 0.5).abs() < 1e-12);
        assert!((q.k + 0.5).abs() < 1e-12 && (q.w - 0.5).abs() < 1e-12);
    }

    #[test]
    fn imu_info_wire_layout() {
        let msg = ImuInfo {
            header: header("f", 1.5),
            gyro_noise_density: 1.0,
            gyro_random_walk: 2.0,
            accel_noise_density: 3.0,
            accel_random_walk: 4.0,
            frequency: 5.0,
        };
        let bytes = msg.encode();
        assert_eq!(&bytes[..8], &IMU_INFO_FINGERPRINT.to_be_bytes());
        assert_eq!(bytes.len(), 8 + msg.header.encoded_size() + 40);
        assert_eq!(&bytes[bytes.len() - 8..], &5.0f64.to_be_bytes());
    }
}

#[cfg(test)]
mod bench {
    use super::*;

    #[test]
    #[ignore]
    fn cloud_build_time() {
        let (w, h) = (848usize, 480usize);
        let depth: Vec<u8> = (0..w * h)
            .flat_map(|i| ((800 + (i % 1200)) as u16).to_le_bytes())
            .collect();
        let rgbd = Rgbd {
            color: Image {
                header: header("f", 0.0),
                height: h as i32,
                width: w as i32,
                encoding: "rgb8".into(),
                is_bigendian: 0,
                step: (w * 3) as i32,
                data: vec![128; w * h * 3],
            },
            depth: Image {
                header: header("f", 0.0),
                height: h as i32,
                width: w as i32,
                encoding: "16UC1".into(),
                is_bigendian: 0,
                step: (w * 2) as i32,
                data: depth,
            },
        };
        let mut info = camera_info(
            &Rs2Intrinsics(realsense_sys::rs2_intrinsics {
                width: w as i32,
                height: h as i32,
                ppx: 424.0,
                ppy: 240.0,
                fx: 605.0,
                fy: 605.0,
                model: realsense_sys::rs2_distortion_RS2_DISTORTION_NONE,
                coeffs: [0.0; 5],
            }),
            "f",
        );
        info.header.frame_id = "f".into();
        for stride in [1usize, 2, 3] {
            let t = std::time::Instant::now();
            let cloud = rgbd_to_cloud(&rgbd, &info, 0.001, stride);
            let build = t.elapsed();
            let t = std::time::Instant::now();
            let bytes = cloud.encode();
            eprintln!(
                "stride={stride} points={} build={:?} encode={:?} bytes={}",
                cloud.width,
                build,
                t.elapsed(),
                bytes.len()
            );
        }
    }
}
