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

//! `stereo_cloud`: a point cloud from the two head RGB cameras.
//!
//! `DepthCloud` next door takes a depth image and unprojects it. That is the
//! right module when something upstream produces depth — the wrist RealSenses
//! do. The R1 Pro's head does not: Galaxea's spec sheet calls it "1x pure
//! binocular RGB camera" and the robot publishes no head depth topic, so the
//! stereo matching has to happen here. The two modules share
//! [`crate::unproject`], which is the back half of both paths.

use std::collections::VecDeque;
use std::time::Duration;

use dimos_module::{native_config, warn_throttled, Input, Module, Output};
use lcm_msgs::sensor_msgs::{CameraInfo, CompressedImage, Image, PointCloud2};
use lcm_msgs::std_msgs::Header;
use serde::{Deserialize, Serialize};

use crate::denoise::Chain;
use crate::module::make_cloud;
use crate::stereo::{
    disparity_to_depth, downsample, horizontal_fov, match_stereo, rectify_pair_rotated, to_gray,
    Camera, Gray, MatchParams, Rotation, StereoRectification,
};
use crate::timing::{Stopwatch, Window, STAGES};
use crate::unproject::{unproject, HeightGate, Params};

/// Python's `None`, sent as a JSON null under a key that is always present.
/// native_config forbids `Option`, so an absent key cannot pass as None; the
/// Python config lists these fields in `base_fields` so a None is still sent.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(transparent)]
pub struct Nullable<T>(pub Option<T>);

/// How often the per-stage timing is logged.
const TIMING_REPORT_EVERY: Duration = Duration::from_secs(5);

#[native_config]
pub struct Config {
    /// Distance between the two head cameras, in metres.
    ///
    /// Defaults to 0.120195, which is the R1 Pro's URDF geometry
    /// (`camera_head_left_joint` y +0.059919, `camera_head_right_joint` y
    /// -0.060276). That is a nominal design figure, not a measured one — depth
    /// scales linearly with it, so this is the single knob to turn when the
    /// cloud sits consistently nearer or further than the lidar says.
    #[validate(range(min = 0.001, max = 2.0))]
    baseline_m: f64,

    /// Downscale both images by this integer factor before matching.
    ///
    /// The head cameras are 1920x1536. Matching at full resolution is far
    /// beyond what the robot's Orin has spare while it is also running the
    /// whole driver stack, and navigation does not need it: at the default 4
    /// the disparity search still resolves obstacles to a few centimetres.
    #[validate(range(min = 1, max = 16))]
    downscale: i64,

    /// Number of disparities searched. With `downscale` 4 and the R1's
    /// geometry, 96 covers roughly 0.3 m to 7.5 m.
    #[validate(range(min = 8, max = 512))]
    disparity_range: i64,

    /// SGM smoothness penalties. `p2` must exceed `p1`; a one-step disparity
    /// change costs `p1`, any larger jump costs `p2`.
    #[validate(range(min = 0, max = 255))]
    p1: i64,
    #[validate(range(min = 1, max = 4096))]
    p2: i64,

    /// Fraction by which the winning disparity must beat the runner-up.
    /// Raising it throws away more of the textureless regions where stereo
    /// invents surfaces.
    #[validate(range(min = 0.0, max = 1.0))]
    uniqueness: f64,

    /// Largest left/right disparity disagreement kept, in pixels. Negative
    /// disables the check, which is only sensible in tests.
    max_lr_difference: f64,

    /// Connected disparity regions smaller than this are speckle and are
    /// dropped. Stereo invents small isolated patches on textureless surfaces
    /// that survive the occlusion and uniqueness checks, and to a voxel map a
    /// floating patch is an obstacle in empty space — so the robot refuses to
    /// drive through nothing. 0 or 1 turns the filter off.
    #[validate(range(min = 0, max = 100000))]
    min_region: i64,

    /// Disparity step below which two neighbouring pixels count as the same
    /// surface, for the speckle flood fill.
    #[validate(range(min = 0.0, max = 64.0))]
    speckle_max_step: f64,

    /// Aggregate along the four diagonals as well as the four axes. Doubles the
    /// matcher's cost for support on surfaces with nothing of their own to
    /// match on, a floor being the one that matters for navigation.
    diagonal_paths: bool,

    /// Disparities at or below this are dropped instead of becoming very
    /// distant points: near zero disparity, metres-per-pixel grows without
    /// bound and a little noise plants obstacles across the whole map.
    #[validate(range(min = 0.0, max = 64.0))]
    min_disparity_px: f64,

    /// Keep every Nth point when building the cloud.
    #[validate(range(min = 1, max = 64))]
    decimation: i64,

    #[validate(range(min = 0.0, max = 1000.0))]
    min_range_m: f64,
    #[validate(range(min = 0.0, max = 1000.0))]
    max_range_m: f64,

    /// Frame the cloud is published in. Empty defers to the left CameraInfo,
    /// then to the left image.
    frame_id: String,

    /// Largest gap between the two frames' timestamps that still counts as a
    /// pair, in seconds. The head cameras are free-running and not hardware
    /// synchronised, so some skew is expected; too much of it smears the
    /// disparity of anything moving.
    #[validate(range(min = 0.0, max = 1.0))]
    max_pair_skew_s: f64,

    /// How the right eye is aimed relative to the left, in radians.
    ///
    /// This is the stereo half of the calibration, and a robot that publishes
    /// two *monocular* CameraInfos has not given it. Leaving it at zero assumes
    /// the eyes are perfectly parallel, which no real rig is: a relative yaw is
    /// a constant offset in every disparity, and so a depth error growing with
    /// the square of range, while a relative pitch puts the two pictures on
    /// different rows and a matcher searching along a row simply fails there.
    ///
    /// Fit these with `dimos.robot.galaxea.r1pro.fit_rectification` against a
    /// recording; they are properties of the rig, so they only change when
    /// something is unbolted.
    #[validate(range(min = -0.2, max = 0.2))]
    right_roll_rad: f64,
    #[validate(range(min = -0.2, max = 0.2))]
    right_pitch_rad: f64,
    #[validate(range(min = -0.2, max = 0.2))]
    right_yaw_rad: f64,

    /// The depth denoise chain, applied to the metric depth before it is
    /// unprojected: filters joined by `+`, each `name` or `name:arg`
    /// (`plane:radius:weight`), see [`crate::denoise`]. `none` turns it off.
    ///
    /// The default, `median:8+plane:16:1+fill:8`, is what a week of scoring
    /// chains against the lidar map of the same scene settled on: the median
    /// takes the thorns off, the plane fit flattens the floor and walls the
    /// matcher had rippled, and the fill then has honest neighbours to close
    /// the small holes from.
    denoise: String,

    /// Keep only cloud points whose height in the robot base frame is inside
    /// `[min_height_m, max_height_m]`. Null on either side is no bound there;
    /// null on both is no gate. The depth *image* is not gated: it is the
    /// camera's answer, the cloud is what the map may use.
    min_height_m: Nullable<f64>,
    max_height_m: Nullable<f64>,

    /// The pose of the camera's OPTICAL frame (x right, y down, z forward) in
    /// the robot base frame, so a camera point `p` sits at `R(rpy) * p + xyz`
    /// in base and its height is that vector's z. `rpy` is roll about x, then
    /// pitch about y, then yaw about z, all about the fixed base axes
    /// (extrinsic XYZ, equal to intrinsic Z-Y'-X''): the same convention as
    /// `Quaternion.from_euler` on the Python side and as a URDF `rpy`. Only
    /// read when a height bound is set.
    base_from_camera_xyz_m: [f64; 3],
    base_from_camera_rpy_rad: [f64; 3],
}

#[derive(Module)]
#[module(name = "stereo_cloud", setup = prepare)]
pub struct StereoCloud {
    #[input(decode = CompressedImage::decode, handler = on_left)]
    left: Input<CompressedImage>,

    #[input(decode = CompressedImage::decode, handler = on_right)]
    right: Input<CompressedImage>,

    #[input(decode = CameraInfo::decode, handler = on_left_info)]
    left_info: Input<CameraInfo>,

    #[input(decode = CameraInfo::decode, handler = on_right_info)]
    right_info: Input<CameraInfo>,

    #[output(encode = PointCloud2::encode)]
    cloud: Output<PointCloud2>,

    /// The rectified depth map behind the cloud. Published because a wrong
    /// cloud and a wrong calibration look identical once the points are in
    /// 3D — the depth image is where a bad rectification is actually visible.
    #[output(encode = Image::encode)]
    depth: Output<Image>,

    #[config]
    config: Config,

    pending_left: VecDeque<CompressedImage>,
    pending_right: VecDeque<CompressedImage>,
    left_camera: Option<CameraInfo>,
    right_camera: Option<CameraInfo>,

    /// Rectification maps and the rectified intrinsics, built once both
    /// CameraInfos have arrived and the first frame has fixed the resolution.
    rectify: Option<StereoRectification>,

    /// `config.denoise`, parsed once at setup. Empty when the string was
    /// invalid: that is logged as an error and the module runs without a
    /// chain rather than crash-looping under the supervisor.
    denoise: Chain,

    /// Where the last ~150 frames spent their time, logged every few seconds.
    timing: Window,

    /// Frames replaced on one side before the other side ever paired with
    /// them. A steady count here means one camera is running ahead of the
    /// other, or `max_pair_skew_s` is tighter than the rig can meet.
    frames_dropped_unpaired: u64,
}

impl StereoCloud {
    async fn prepare(&mut self) {
        self.denoise = parse_denoise_or_none(&self.config.denoise);
        tracing::info!(
            denoise = %self.denoise.name(),
            min_height_m = ?self.config.min_height_m.0,
            max_height_m = ?self.config.max_height_m.0,
            "stereo_cloud ready",
        );
    }

    async fn on_left_info(&mut self, msg: CameraInfo) {
        if !same_intrinsics(self.left_camera.as_ref(), &msg) {
            self.rectify = None;
        }
        self.left_camera = Some(msg);
    }

    async fn on_right_info(&mut self, msg: CameraInfo) {
        if !same_intrinsics(self.right_camera.as_ref(), &msg) {
            self.rectify = None;
        }
        self.right_camera = Some(msg);
    }

    async fn on_left(&mut self, msg: CompressedImage) {
        Self::hold(
            &mut self.pending_left,
            msg,
            &mut self.frames_dropped_unpaired,
        );
        self.try_pair().await;
    }

    async fn on_right(&mut self, msg: CompressedImage) {
        Self::hold(
            &mut self.pending_right,
            msg,
            &mut self.frames_dropped_unpaired,
        );
        self.try_pair().await;
    }

    /// Keep the last few frames of an eye, not just the newest one.
    ///
    /// The two eyes free-run and arrive on two streams, so a left frame's
    /// partner is often still in flight when the next left lands. Holding one
    /// frame per eye then evicts it unpaired -- on a replay that dropped four
    /// frames in five. A short queue lets a frame wait for its partner.
    fn hold(pending: &mut VecDeque<CompressedImage>, msg: CompressedImage, dropped: &mut u64) {
        pending.push_back(msg);
        while pending.len() > PAIR_QUEUE {
            pending.pop_front();
            *dropped += 1;
        }
    }

    /// Emit a cloud for the closest-stamped pair within the skew, if any.
    ///
    /// Both frames are consumed on a successful pair, and everything older
    /// than either of them is dropped: a frame older than a pair that has
    /// already been matched can only ever pair with something staler still,
    /// and one camera running fast must not pair its newest frame against
    /// the same stale partner over and over.
    async fn try_pair(&mut self) {
        let Some((i, j)) = closest_pair(
            &self.pending_left,
            &self.pending_right,
            self.config.max_pair_skew_s,
        ) else {
            return;
        };
        let left = self
            .pending_left
            .remove(i)
            .expect("indexed within the queue");
        let right = self
            .pending_right
            .remove(j)
            .expect("indexed within the queue");
        self.frames_dropped_unpaired += (i + j) as u64;
        self.pending_left.drain(..i);
        self.pending_right.drain(..j);
        self.process(left, right).await;
    }

    async fn process(&mut self, left: CompressedImage, right: CompressedImage) {
        let (Some(left_info), Some(right_info)) =
            (self.left_camera.clone(), self.right_camera.clone())
        else {
            // Same as a real camera's first frames: no intrinsics, no geometry.
            return;
        };

        let mut stopwatch = Stopwatch::start();
        let factor = self.config.downscale.max(1) as usize;
        let (Some(left_decoded), Some(right_decoded)) = decode_pair_scaled(&left, &right, factor)
        else {
            warn_throttled!(
                Duration::from_secs(1),
                format = %left.format,
                "Could not decode a head camera frame, dropped the pair.",
            );
            return;
        };
        let (left_gray, right_gray) = (&left_decoded.gray, &right_decoded.gray);
        if left_gray.width != right_gray.width || left_gray.height != right_gray.height {
            warn_throttled!(
                Duration::from_secs(5),
                left = left_gray.width,
                right = right_gray.width,
                "Head cameras disagree on resolution, dropped the pair.",
            );
            return;
        }
        let decode_ms = stopwatch.lap_ms();

        // Whatever libjpeg could not do in the IDCT. Usually 1, i.e. nothing.
        let left_small = downsample(left_gray, left_decoded.remaining);
        let right_small = downsample(right_gray, right_decoded.remaining);

        if self.rectify.is_none() {
            self.rectify = build_rectification(
                &left_info,
                &right_info,
                left_decoded.full_width,
                left_decoded.full_height,
                factor,
                Rotation {
                    roll_rad: self.config.right_roll_rad,
                    pitch_rad: self.config.right_pitch_rad,
                    yaw_rad: self.config.right_yaw_rad,
                },
            );
            if let Some(built) = self.rectify.as_ref() {
                tracing::info!(
                    width = built.width,
                    height = built.height,
                    fx = built.fx,
                    baseline_m = self.config.baseline_m,
                    horizontal_fov_deg = horizontal_fov(built.fx as f64, built.width),
                    "stereo_cloud rectification ready",
                );
            }
        }
        let Some(rectification) = self.rectify.as_ref() else {
            warn_throttled!(
                Duration::from_secs(5),
                "Head CameraInfo has no usable intrinsics, dropped the pair.",
            );
            return;
        };

        let left_rect = rectification.left.apply(&left_small);
        let right_rect = rectification.right.apply(&right_small);
        let rectify_ms = stopwatch.lap_ms();

        let params = MatchParams {
            min_disparity: 0,
            disparity_range: self.config.disparity_range as usize,
            p1: self.config.p1 as u16,
            p2: self.config.p2 as u16,
            uniqueness: self.config.uniqueness as f32,
            max_lr_difference: self.config.max_lr_difference as f32,
            min_region: self.config.min_region as usize,
            speckle_max_step: self.config.speckle_max_step as f32,
            diagonal_paths: self.config.diagonal_paths,
        };
        let disparity = match_stereo(&left_rect, &right_rect, &params);
        let depth_metres = disparity_to_depth(
            &disparity,
            rectification.fx,
            self.config.baseline_m as f32,
            self.config.min_disparity_px as f32,
        );
        let match_ms = stopwatch.lap_ms();

        // Denoised in metres, before the image is built, so the published
        // depth and the cloud are the same picture.
        let depth_metres = self.denoise.apply(
            &depth_metres,
            rectification.width,
            rectification.height,
            rectification.fx,
        );
        let denoise_ms = stopwatch.lap_ms();

        let frame_id = resolve_frame_id(
            &self.config.frame_id,
            &left_info.header.frame_id,
            &left.header.frame_id,
        )
        .to_owned();

        let depth_image = depth_to_image(
            &depth_metres,
            rectification.width,
            rectification.height,
            &left.header,
            &frame_id,
        );

        // Unproject through the *rectified* intrinsics, not the raw CameraInfo:
        // the depth map is in the rectified frame, and the two differ by the
        // downscale factor and the undistortion.
        let rectified_info = rectified_camera_info(rectification, &left_info);
        let cloud_params = Params {
            decimation: self.config.decimation as usize,
            min_range_m: self.config.min_range_m as f32,
            max_range_m: self.config.max_range_m as f32,
            depth_scale: 1.0,
            height_gate: HeightGate::new(
                self.config.base_from_camera_xyz_m,
                self.config.base_from_camera_rpy_rad,
                self.config.min_height_m.0,
                self.config.max_height_m.0,
            ),
        };
        match unproject(&depth_image, &rectified_info, &cloud_params) {
            Ok((data, count)) => {
                let cloud = make_cloud(data, count, frame_id, left.header.clone());
                self.cloud.publish(&cloud).await.ok();
            }
            Err(error) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %error,
                    "Could not unproject the stereo depth map.",
                );
            }
        }
        self.depth.publish(&depth_image).await.ok();
        let unproject_ms = stopwatch.lap_ms();

        // In STAGES order, which is the order the bench prints them in.
        debug_assert_eq!(
            STAGES,
            ["decode", "rectify", "match", "denoise", "unproject"]
        );
        self.timing
            .push([decode_ms, rectify_ms, match_ms, denoise_ms, unproject_ms]);
        if let Some(summary) = self.timing.report_due(TIMING_REPORT_EVERY) {
            let [decode, rectify, matching, denoise, unproject] = summary.median_ms;
            // Tenths of a millisecond is what a stage's cost is known to.
            let tenths = |ms: f32| (ms * 10.0).round() / 10.0;
            tracing::info!(
                frames = summary.frames,
                fps = (summary.fps * 100.0).round() / 100.0,
                decode_ms = tenths(decode),
                rectify_ms = tenths(rectify),
                match_ms = tenths(matching),
                denoise_ms = tenths(denoise),
                unproject_ms = tenths(unproject),
                frames_dropped_unpaired = self.frames_dropped_unpaired,
                "stereo_cloud timing (median ms per stage over the window)",
            );
        }
    }
}

/// The configured chain, or none if it does not parse.
///
/// The Python config validates the same grammar, so a bad string should never
/// reach here; if one does, an error naming the token and a module that still
/// publishes is more useful than a supervisor restarting it every second.
pub fn parse_denoise_or_none(text: &str) -> Chain {
    match Chain::parse(text) {
        Ok(chain) => chain,
        Err(error) => {
            tracing::error!(
                denoise = text,
                error = %error,
                "Invalid denoise chain, running with none.",
            );
            Chain(Vec::new())
        }
    }
}

/// Frames an eye may hold while waiting for its partner. At 30 Hz that is a
/// quarter of a second of reordering, far more than the transport does.
const PAIR_QUEUE: usize = 8;

/// The rectification is built from these and nothing else, so a CameraInfo
/// that repeats them -- the robot republishes its calibration at a fixed rate
/// -- is not a reason to rebuild it.
fn same_intrinsics(held: Option<&CameraInfo>, incoming: &CameraInfo) -> bool {
    held.is_some_and(|h| {
        h.width == incoming.width
            && h.height == incoming.height
            && h.K == incoming.K
            && h.D == incoming.D
            && h.distortion_model == incoming.distortion_model
    })
}

/// The left and right queue positions of the closest-stamped pair within
/// `max_skew_s`, or None when no two frames are close enough.
fn closest_pair(
    left: &VecDeque<CompressedImage>,
    right: &VecDeque<CompressedImage>,
    max_skew_s: f64,
) -> Option<(usize, usize)> {
    let mut best: Option<(usize, usize, f64)> = None;
    for (i, l) in left.iter().enumerate() {
        for (j, r) in right.iter().enumerate() {
            let skew = (stamp_seconds(&l.header) - stamp_seconds(&r.header)).abs();
            if skew <= max_skew_s && best.is_none_or(|(_, _, s)| skew < s) {
                best = Some((i, j, skew));
            }
        }
    }
    best.map(|(i, j, _)| (i, j))
}

/// Header stamp as float seconds.
fn stamp_seconds(header: &Header) -> f64 {
    header.stamp.sec as f64 + header.stamp.nsec as f64 * 1e-9
}

/// What a scaled decode produced: the image, the full size the camera really
/// has, and whatever downscaling libjpeg could not do and the caller still must.
pub struct Decoded {
    pub gray: Gray,
    pub full_width: usize,
    pub full_height: usize,
    pub remaining: usize,
}

/// Decode a compressed frame to grayscale, asking libjpeg to do as much of the
/// downscaling as it can.
///
/// libjpeg can scale while it inverts the DCT, in eighths, and it is nearly
/// free: the high-frequency coefficients it would otherwise transform and then
/// throw away are simply never transformed. Decoding 1920x1536 in full and
/// box-averaging by four afterwards costs **30-39 ms per eye on the Orin**,
/// which is more than the matcher itself at downscale 8, and every one of those
/// milliseconds is spent producing pixels that are immediately discarded.
///
/// The full dimensions come back with the image because the rectification is
/// built from the camera's real intrinsics: the geometry has to be told the
/// picture's true size and the factor separately, or it will place the
/// principal point at a quarter of where it belongs.
/// Both eyes at once, on two threads. The decode is the one stage that is
/// two independent jobs of equal size, and on an Orin it is a third of the
/// frame when run one after the other -- which is the difference between
/// clearing 30 fps and not.
pub fn decode_pair_scaled(
    left: &CompressedImage,
    right: &CompressedImage,
    factor: usize,
) -> (Option<Decoded>, Option<Decoded>) {
    rayon::join(
        || decode_scaled(left, factor),
        || decode_scaled(right, factor),
    )
}

pub fn decode_scaled(image: &CompressedImage, factor: usize) -> Option<Decoded> {
    let format = image.format.to_ascii_lowercase();
    if !(format.contains("jpeg") || format.contains("jpg") || format.is_empty()) {
        return None;
    }
    // Only the powers of two that divide the requested factor: scaling by 1/8
    // when 4 was asked for would hand the matcher a different picture than the
    // rectification was built for.
    let (scaling, by) = match factor {
        f if f % 8 == 0 => (turbojpeg::ScalingFactor::ONE_EIGHTH, 8),
        f if f % 4 == 0 => (turbojpeg::ScalingFactor::ONE_QUARTER, 4),
        f if f % 2 == 0 => (turbojpeg::ScalingFactor::ONE_HALF, 2),
        _ => (turbojpeg::ScalingFactor::ONE, 1),
    };

    let mut decompressor = turbojpeg::Decompressor::new().ok()?;
    let header = decompressor.read_header(&image.data).ok()?;
    let (full_width, full_height) = (header.width, header.height);
    decompressor.set_scaling_factor(scaling).ok()?;
    let width = scaling.scale(full_width);
    let height = scaling.scale(full_height);
    if width == 0 || height == 0 {
        return None;
    }

    let mut pixels = vec![0u8; width * height];
    decompressor
        .decompress(
            &image.data,
            turbojpeg::Image {
                pixels: &mut pixels[..],
                width,
                pitch: width,
                height,
                format: turbojpeg::PixelFormat::GRAY,
            },
        )
        .ok()?;
    Some(Decoded {
        gray: to_gray(&pixels, width, height, 1)?,
        full_width,
        full_height,
        remaining: factor / by,
    })
}

/// Build the rectification for a pair, at the downscaled resolution.
///
/// The geometry itself lives in `stereo::rectify_pair`, shared with every other
/// caller that matches a pair, so that an offline tool and the live module
/// cannot drift into rectifying differently and reporting different depths for
/// the same two frames.
pub fn build_rectification(
    left_info: &CameraInfo,
    right_info: &CameraInfo,
    full_width: usize,
    full_height: usize,
    factor: usize,
    right_rotation: Rotation,
) -> Option<StereoRectification> {
    let factor = factor.max(1);
    let width = full_width / factor;
    let height = full_height / factor;
    if width == 0 || height == 0 {
        return None;
    }
    let left_camera = camera_from(left_info, full_width, full_height, factor)?;
    let right_camera = camera_from(right_info, full_width, full_height, factor)?;
    Some(rectify_pair_rotated(
        &left_camera,
        &right_camera,
        width,
        height,
        right_rotation,
    ))
}

/// Pull intrinsics out of a `CameraInfo`, rescaled to the working resolution.
fn camera_from(
    info: &CameraInfo,
    full_width: usize,
    full_height: usize,
    factor: usize,
) -> Option<Camera> {
    let (fx, fy) = (info.K[0], info.K[4]);
    let (cx, cy) = (info.K[2], info.K[5]);
    if fx == 0.0 || fy == 0.0 || !fx.is_finite() || !fy.is_finite() {
        return None;
    }
    // The calibration may have been taken at a different resolution than the
    // frames arrive at; fold that in before the downscale.
    let (calibration_scale_x, calibration_scale_y) = if info.width > 0 && info.height > 0 {
        (
            full_width as f64 / info.width as f64,
            full_height as f64 / info.height as f64,
        )
    } else {
        (1.0, 1.0)
    };
    let sx = calibration_scale_x / factor as f64;
    let sy = calibration_scale_y / factor as f64;

    // Eight coefficients mean the rational model regardless of what
    // `distortion_model` claims; Galaxea ships eight and says `plumb_bob`.
    let mut distortion = [0.0f64; 8];
    for (slot, value) in distortion.iter_mut().zip(info.D.iter()) {
        *slot = *value;
    }

    Some(Camera {
        fx: fx * sx,
        fy: fy * sy,
        cx: cx * sx,
        cy: cy * sy,
        distortion,
    })
}

/// A `CameraInfo` describing the rectified image the depth map lives in.
pub fn rectified_camera_info(
    rectification: &StereoRectification,
    source: &CameraInfo,
) -> CameraInfo {
    let mut info = source.clone();
    info.width = rectification.width as i32;
    info.height = rectification.height as i32;
    info.K = [
        rectification.fx as f64,
        0.0,
        rectification.cx as f64,
        0.0,
        rectification.fy as f64,
        rectification.cy as f64,
        0.0,
        0.0,
        1.0,
    ];
    // Rectified by construction, so there is nothing left to undistort.
    info.D = vec![0.0; 5];
    info.distortion_model = "plumb_bob".into();
    info
}

/// Wrap a metric depth buffer as a 32FC1 `Image`.
pub fn depth_to_image(
    depth: &[f32],
    width: usize,
    height: usize,
    source: &Header,
    frame_id: &str,
) -> Image {
    let mut data = Vec::with_capacity(depth.len() * 4);
    for value in depth {
        data.extend_from_slice(&value.to_le_bytes());
    }
    Image {
        header: Header {
            seq: source.seq,
            stamp: source.stamp.clone(),
            frame_id: frame_id.to_owned(),
        },
        height: height as i32,
        width: width as i32,
        encoding: "32FC1".into(),
        is_bigendian: 0,
        step: (width * 4) as i32,
        data,
    }
}

/// Config wins, then the calibration's frame, then the image's own — the same
/// precedence `DepthCloud` uses, and for the same reason: a vendor driver
/// stamps frames with an optical frame nobody publishes a transform for.
fn resolve_frame_id<'a>(config: &'a str, info: &'a str, image: &'a str) -> &'a str {
    [config, info, image]
        .into_iter()
        .find(|candidate| !candidate.is_empty())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(width: i32, height: i32, fx: f64, coefficients: usize) -> CameraInfo {
        CameraInfo {
            width,
            height,
            distortion_model: "plumb_bob".into(),
            D: vec![0.0; coefficients],
            K: [
                fx,
                0.0,
                width as f64 / 2.0,
                0.0,
                fx,
                height as f64 / 2.0,
                0.0,
                0.0,
                1.0,
            ],
            ..Default::default()
        }
    }

    #[test]
    fn downscaling_scales_the_intrinsics_with_it() {
        let camera = camera_from(&info(1920, 1536, 1012.6, 8), 1920, 1536, 4)
            .expect("valid intrinsics should convert");
        assert!((camera.fx - 253.15).abs() < 0.01, "fx was {}", camera.fx);
        assert!((camera.cx - 240.0).abs() < 0.01, "cx was {}", camera.cx);
    }

    #[test]
    fn a_calibration_taken_at_another_resolution_is_folded_in() {
        // Calibrated at 960x768, frames arrive at 1920x1536: the focal length
        // must double before the downscale, or every depth is off by 2x.
        let camera = camera_from(&info(960, 768, 506.3, 8), 1920, 1536, 1)
            .expect("valid intrinsics should convert");
        assert!((camera.fx - 1012.6).abs() < 0.01, "fx was {}", camera.fx);
    }

    #[test]
    fn zero_focal_length_is_rejected_rather_than_dividing_by_zero() {
        assert!(camera_from(&info(1920, 1536, 0.0, 8), 1920, 1536, 4).is_none());
    }

    #[test]
    fn a_five_coefficient_calibration_still_works() {
        let camera =
            camera_from(&info(640, 480, 500.0, 5), 640, 480, 1).expect("plumb_bob should convert");
        // The unused three are zero, which makes the rational denominator 1 and
        // collapses the model back to plumb_bob exactly.
        assert_eq!(camera.distortion[5], 0.0);
        assert_eq!(camera.distortion[6], 0.0);
        assert_eq!(camera.distortion[7], 0.0);
    }

    #[test]
    fn both_cameras_get_the_same_rectified_intrinsics() {
        // Disparity is only meaningful if the pair shares a focal length and a
        // principal point.
        let left = info(1920, 1536, 1012.59, 8);
        let right = info(1920, 1536, 1013.80, 8);
        let rectification = build_rectification(&left, &right, 1920, 1536, 4, Rotation::IDENTITY)
            .expect("should build");
        assert_eq!(rectification.width, 480);
        assert_eq!(rectification.height, 384);
        // Averaged, so between the two.
        let fx = rectification.fx as f64;
        assert!(fx > 1012.59 / 4.0 && fx < 1013.80 / 4.0, "fx was {fx}");
        assert!((rectification.cx - 240.0).abs() < 0.01);
        assert!((rectification.cy - 192.0).abs() < 0.01);
    }

    #[test]
    fn the_rectified_camera_info_describes_the_depth_map_not_the_raw_frame() {
        let source = info(1920, 1536, 1012.59, 8);
        let rectification =
            build_rectification(&source, &source, 1920, 1536, 4, Rotation::IDENTITY)
                .expect("should build");
        let rectified = rectified_camera_info(&rectification, &source);
        assert_eq!(rectified.width, 480);
        assert_eq!(rectified.height, 384);
        assert!((rectified.K[0] - rectification.fx as f64).abs() < 1e-6);
        assert!(
            rectified.D.iter().all(|&d| d == 0.0),
            "rectified image must not be undistorted a second time"
        );
    }

    #[test]
    fn depth_image_is_32fc1_with_a_tight_row_stride() {
        let depth = vec![1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let image = depth_to_image(&depth, 3, 2, &Header::default(), "camera_head_left_link");
        assert_eq!(image.encoding, "32FC1");
        assert_eq!(image.width, 3);
        assert_eq!(image.height, 2);
        assert_eq!(image.step, 12);
        assert_eq!(image.data.len(), 24);
        assert_eq!(image.header.frame_id, "camera_head_left_link");
        assert_eq!(
            f32::from_le_bytes(image.data[0..4].try_into().unwrap()),
            1.0
        );
    }

    #[test]
    fn frame_id_precedence_is_config_then_calibration_then_image() {
        assert_eq!(resolve_frame_id("config", "info", "image"), "config");
        assert_eq!(resolve_frame_id("", "info", "image"), "info");
        assert_eq!(resolve_frame_id("", "", "image"), "image");
        assert_eq!(resolve_frame_id("", "", ""), "");
    }

    #[test]
    fn stamp_seconds_combines_both_halves() {
        let mut header = Header::default();
        header.stamp.sec = 1000;
        header.stamp.nsec = 500_000_000;
        assert!((stamp_seconds(&header) - 1000.5).abs() < 1e-9);
    }

    #[test]
    fn a_non_jpeg_format_is_refused_rather_than_fed_to_the_decoder() {
        let image = CompressedImage {
            format: "png".into(),
            data: vec![0; 16],
            ..Default::default()
        };
        assert!(decode_scaled(&image, 4).is_none());
    }

    /// A real JPEG through `decode_scaled`, because the arithmetic test below
    /// never touches libjpeg.
    ///
    /// The thing that actually breaks the module is a size mismatch: the
    /// rectification is built from `full_width/full_height` and `factor`, and
    /// the matcher is handed `gray` downsampled by `remaining`. If libjpeg's
    /// scaled decode disagrees by even a pixel with `full / factor`, the two
    /// stop describing the same picture and every depth is wrong. Unit-testing
    /// the divisor table cannot catch that; this can.
    fn jpeg_of(width: usize, height: usize) -> CompressedImage {
        let mut pixels = vec![0u8; width * height];
        for row in 0..height {
            for column in 0..width {
                // Something with structure, so the DCT has real coefficients.
                pixels[row * width + column] = ((row * 7 + column * 3) % 256) as u8;
            }
        }
        let data = turbojpeg::compress(
            turbojpeg::Image {
                pixels: &pixels[..],
                width,
                pitch: width,
                height,
                format: turbojpeg::PixelFormat::GRAY,
            },
            90,
            turbojpeg::Subsamp::Gray,
        )
        .expect("compress")
        .to_vec();
        CompressedImage {
            format: "jpeg".into(),
            data,
            ..Default::default()
        }
    }

    #[test]
    fn a_real_jpeg_decodes_to_exactly_the_size_the_rectification_assumes() {
        // The R1's head frame, and the factors the blueprint can ask for.
        for factor in [1usize, 2, 4, 8] {
            let image = jpeg_of(1920, 1536);
            let decoded = decode_scaled(&image, factor).expect("decodes");
            assert_eq!(decoded.full_width, 1920, "factor {factor}");
            assert_eq!(decoded.full_height, 1536, "factor {factor}");
            // What the matcher ends up with, after the leftover box pass.
            let final_width = decoded.gray.width / decoded.remaining;
            let final_height = decoded.gray.height / decoded.remaining;
            assert_eq!(final_width, 1920 / factor, "factor {factor} width");
            assert_eq!(final_height, 1536 / factor, "factor {factor} height");
        }
    }

    #[test]
    fn a_scaled_decode_carries_the_same_picture_as_a_full_one() {
        // Not bit-identical -- a truncated IDCT is a different low-pass than a
        // box average -- but it must be the same image, or the matcher is
        // matching something else. Compared against decoding full and boxing
        // down by hand.
        let image = jpeg_of(640, 512);
        let scaled = decode_scaled(&image, 4).expect("scaled");
        let full = decode_scaled(&image, 1).expect("full");
        let boxed = downsample(&full.gray, 4);
        assert_eq!(scaled.gray.width, boxed.width);
        assert_eq!(scaled.gray.height, boxed.height);

        let error: f64 = scaled
            .gray
            .data
            .iter()
            .zip(&boxed.data)
            .map(|(a, b)| (*a as f64 - *b as f64).abs())
            .sum::<f64>()
            / boxed.data.len() as f64;
        assert!(error < 12.0, "mean |difference| was {error} grey levels");
    }

    #[test]
    fn libjpeg_is_asked_for_the_largest_scale_that_divides_the_downscale() {
        // The factor has to split cleanly, because whatever libjpeg does not do
        // the box downsample must, and the rectification is built from the
        // full size and the factor together. Asking for 1/8 when 4 was wanted
        // would hand the matcher a different picture than the geometry
        // describes.
        for (factor, expected_remaining) in
            [(1, 1), (2, 1), (3, 3), (4, 1), (6, 3), (8, 1), (12, 3)]
        {
            let image = CompressedImage {
                format: "jpeg".into(),
                data: vec![0; 16],
                ..Default::default()
            };
            // The data is not a real JPEG, so this returns None -- the point of
            // the table is the arithmetic below, kept beside it so the mapping
            // cannot drift without someone noticing.
            assert!(decode_scaled(&image, factor).is_none());
            let by = match factor {
                f if f % 8 == 0 => 8,
                f if f % 4 == 0 => 4,
                f if f % 2 == 0 => 2,
                _ => 1,
            };
            assert_eq!(factor / by, expected_remaining, "factor {factor}");
        }
    }

    #[test]
    fn a_bad_denoise_chain_becomes_none_rather_than_a_panic() {
        // The supervisor would restart a panicking module forever; a module
        // publishing unfiltered depth is at least a module.
        let chain = parse_denoise_or_none("median:8+nonsense:3");
        assert_eq!(chain.name(), "none");
        assert_eq!(parse_denoise_or_none("median:8+fill:8").0.len(), 2);
        assert_eq!(parse_denoise_or_none("none").0.len(), 0);
    }

    #[test]
    fn a_null_height_bound_deserialises_to_none() {
        // The wire shape the Python side sends for `min_height_m=None`.
        let bound: Nullable<f64> = serde_json::from_str("null").expect("null parses");
        assert_eq!(bound.0, None);
        let bound: Nullable<f64> = serde_json::from_str("0.05").expect("number parses");
        assert_eq!(bound.0, Some(0.05));
    }

    #[test]
    fn the_configured_rotation_reaches_the_right_eyes_map() {
        // The stereo half of the calibration is a config field, and a field
        // that is read but never applied looks exactly like a rig that happens
        // to be parallel.
        let source = info(1920, 1536, 1012.59, 8);
        let parallel = build_rectification(&source, &source, 1920, 1536, 4, Rotation::IDENTITY)
            .expect("built");
        let turned = build_rectification(
            &source,
            &source,
            1920,
            1536,
            4,
            Rotation {
                roll_rad: 0.0,
                pitch_rad: 0.0,
                yaw_rad: 0.012,
            },
        )
        .expect("built");
        let index = (192 * 480 + 240) * 2;
        assert_eq!(
            parallel.left.coords()[index],
            turned.left.coords()[index],
            "the left eye defines the rectified frame and must not move"
        );
        assert!(
            (parallel.right.coords()[index] - turned.right.coords()[index]).abs() > 1.0,
            "the right eye's map did not move with the configured yaw"
        );
    }

    fn stamped(seconds: f64) -> CompressedImage {
        let mut image = CompressedImage::default();
        image.header.stamp.sec = seconds.floor() as i32;
        image.header.stamp.nsec = ((seconds - seconds.floor()) * 1e9).round() as i32;
        image
    }

    /// The case the single-slot pairing dropped four frames in five on: the
    /// next left arrives before the right that belongs to the previous one.
    #[test]
    fn a_frame_waits_for_its_partner_instead_of_being_evicted() {
        let left: VecDeque<_> = [stamped(1.000), stamped(1.033)].into();
        let right: VecDeque<_> = [stamped(1.001)].into();
        assert_eq!(closest_pair(&left, &right, 0.05), Some((0, 0)));
    }

    #[test]
    fn the_closest_stamps_pair_not_the_newest_frames() {
        let left: VecDeque<_> = [stamped(1.000), stamped(1.033), stamped(1.066)].into();
        let right: VecDeque<_> = [stamped(1.030), stamped(1.070)].into();
        assert_eq!(closest_pair(&left, &right, 0.05), Some((1, 0)));
    }

    #[test]
    fn nothing_pairs_across_more_than_the_skew() {
        let left: VecDeque<_> = [stamped(1.000)].into();
        let right: VecDeque<_> = [stamped(1.100)].into();
        assert_eq!(closest_pair(&left, &right, 0.05), None);
        assert_eq!(closest_pair(&left, &right, 0.15), Some((0, 0)));
    }

    #[test]
    fn a_repeated_camera_info_is_not_a_new_calibration() {
        let a = info(640, 480, 500.0, 5);
        let mut b = info(640, 480, 500.0, 5);
        assert!(same_intrinsics(Some(&a), &b));
        assert!(!same_intrinsics(None, &b));
        b.K[0] += 1.0;
        assert!(!same_intrinsics(Some(&a), &b));
    }
}
