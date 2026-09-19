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

//! Pinhole unprojection of a depth image into packed xyz+intensity points.
//!
//! Kept free of transport and module types so the geometry can be unit tested
//! on plain structs.
//!
//! The height gate lives here rather than in the module because the cloud's
//! point count and its bytes are produced by one loop: gating afterwards would
//! mean a second pass over the buffer, and gating in the module would mean two
//! places that have to agree on what a point is.

use lcm_msgs::sensor_msgs::{CameraInfo, Image};
use std::fmt;

/// Bytes per point in the emitted buffer: x, y, z, intensity, all f32.
pub const POINT_STEP: usize = 16;

pub struct Params {
    /// Keep every Nth pixel on each axis.
    pub decimation: usize,
    pub min_range_m: f32,
    pub max_range_m: f32,
    /// Multiplier onto metres for integer depth. Float depth is already metres
    /// and ignores this.
    pub depth_scale: f32,
    /// Drop points by their height in the robot's base frame. `HeightGate::OFF`
    /// keeps everything, at no cost per point.
    pub height_gate: HeightGate,
}

/// A band of base-frame heights the cloud keeps, and where the camera sits so
/// a camera-frame point can be measured against it.
///
/// Only the base frame's z row of the pose is stored: the gate never needs a
/// point's base x or y, so one dot product per point is the whole cost.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeightGate {
    /// The third row of `R(base_from_camera)`: dotted with a camera point it
    /// gives that point's height above the camera's own origin, in base axes.
    up: [f32; 3],
    /// The camera origin's own height in the base frame.
    camera_height_m: f32,
    /// `-inf` / `+inf` when unset, so the comparison needs no branch.
    min_height_m: f32,
    max_height_m: f32,
}

impl HeightGate {
    /// No gate: every point passes and nothing is computed per point.
    pub const OFF: Self = Self {
        up: [0.0, 0.0, 0.0],
        camera_height_m: 0.0,
        min_height_m: f32::NEG_INFINITY,
        max_height_m: f32::INFINITY,
    };

    /// `xyz_m` and `rpy_rad` are the pose of the camera's optical frame in the
    /// robot base frame, with the roll-pitch-yaw convention of
    /// [`rotation_from_rpy`]. A camera point `p` lands at `R * p + t` in base,
    /// and its height is that vector's z.
    pub fn new(
        xyz_m: [f64; 3],
        rpy_rad: [f64; 3],
        min_height_m: Option<f64>,
        max_height_m: Option<f64>,
    ) -> Self {
        if min_height_m.is_none() && max_height_m.is_none() {
            return Self::OFF;
        }
        let rotation = rotation_from_rpy(rpy_rad[0], rpy_rad[1], rpy_rad[2]);
        Self {
            up: rotation[2].map(|value| value as f32),
            camera_height_m: xyz_m[2] as f32,
            min_height_m: min_height_m.map_or(f32::NEG_INFINITY, |value| value as f32),
            max_height_m: max_height_m.map_or(f32::INFINITY, |value| value as f32),
        }
    }

    pub fn is_active(&self) -> bool {
        *self != Self::OFF
    }

    /// Whether a camera-frame point sits inside the band.
    #[inline]
    fn admits(&self, x: f32, y: f32, z: f32) -> bool {
        let height = self.up[0] * x + self.up[1] * y + self.up[2] * z + self.camera_height_m;
        height >= self.min_height_m && height <= self.max_height_m
    }
}

/// The rotation matrix for roll, pitch, yaw in radians, row-major.
///
/// `R = Rz(yaw) * Ry(pitch) * Rx(roll)`: roll about the fixed x axis first,
/// then pitch about the fixed y, then yaw about the fixed z (extrinsic XYZ,
/// which is the same rotation as intrinsic Z-Y'-X''). That is the convention
/// of `Quaternion.from_euler` in `dimos/msgs/geometry_msgs/Quaternion.py`, and
/// of ROS's tf, so an rpy copied out of a URDF or a Python transform means the
/// same thing here.
pub fn rotation_from_rpy(roll: f64, pitch: f64, yaw: f64) -> [[f64; 3]; 3] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

#[derive(Debug, PartialEq)]
pub enum UnprojectError {
    /// Colour, or a depth encoding with more than one channel.
    NotSingleChannelDepth(String),
    /// `data` is too short for `height * width`.
    ShortBuffer,
    /// A zero focal length would divide every point to infinity.
    DegenerateIntrinsics,
}

impl fmt::Display for UnprojectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotSingleChannelDepth(enc) => {
                write!(f, "{enc} is not a single-channel depth encoding")
            }
            Self::ShortBuffer => write!(f, "data buffer shorter than height*width"),
            Self::DegenerateIntrinsics => write!(f, "camera_info has a zero focal length"),
        }
    }
}

/// One depth sample, as stored on the wire.
trait DepthPixel: Copy {
    const SIZE: usize;
    fn read(buf: &[u8], offset: usize) -> Self;
    /// `scale` applies to integer depth only; float depth is already metres.
    fn metres(self, scale: f32) -> f32;
}

impl DepthPixel for f32 {
    const SIZE: usize = 4;
    fn read(buf: &[u8], offset: usize) -> Self {
        f32::from_le_bytes([
            buf[offset],
            buf[offset + 1],
            buf[offset + 2],
            buf[offset + 3],
        ])
    }
    fn metres(self, _scale: f32) -> f32 {
        self
    }
}

impl DepthPixel for f64 {
    const SIZE: usize = 8;
    fn read(buf: &[u8], offset: usize) -> Self {
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&buf[offset..offset + 8]);
        f64::from_le_bytes(bytes)
    }
    fn metres(self, _scale: f32) -> f32 {
        self as f32
    }
}

impl DepthPixel for u16 {
    const SIZE: usize = 2;
    fn read(buf: &[u8], offset: usize) -> Self {
        u16::from_le_bytes([buf[offset], buf[offset + 1]])
    }
    fn metres(self, scale: f32) -> f32 {
        self as f32 * scale
    }
}

impl DepthPixel for i16 {
    const SIZE: usize = 2;
    fn read(buf: &[u8], offset: usize) -> Self {
        i16::from_le_bytes([buf[offset], buf[offset + 1]])
    }
    fn metres(self, scale: f32) -> f32 {
        self as f32 * scale
    }
}

struct Intrinsics {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}

/// Registered depth is often published at a different resolution than the
/// CameraInfo was calibrated at; rescale rather than emit skewed geometry.
fn intrinsics_for(
    info: &CameraInfo,
    width: i32,
    height: i32,
) -> Result<Intrinsics, UnprojectError> {
    let (mut fx, mut fy) = (info.K[0] as f32, info.K[4] as f32);
    let (mut cx, mut cy) = (info.K[2] as f32, info.K[5] as f32);
    if info.width != 0 && info.height != 0 {
        let x_ratio = width as f32 / info.width as f32;
        let y_ratio = height as f32 / info.height as f32;
        fx *= x_ratio;
        cx *= x_ratio;
        fy *= y_ratio;
        cy *= y_ratio;
    }
    if fx == 0.0 || fy == 0.0 || !fx.is_finite() || !fy.is_finite() {
        return Err(UnprojectError::DegenerateIntrinsics);
    }
    Ok(Intrinsics { fx, fy, cx, cy })
}

/// Unproject `depth` through `info`, returning the packed point buffer and its
/// point count.
pub fn unproject(
    depth: &Image,
    info: &CameraInfo,
    params: &Params,
) -> Result<(Vec<u8>, i32), UnprojectError> {
    let intrinsics = intrinsics_for(info, depth.width, depth.height)?;
    match depth.encoding.as_str() {
        "32FC1" => run::<f32>(depth, &intrinsics, params),
        "64FC1" => run::<f64>(depth, &intrinsics, params),
        "16UC1" | "mono16" => run::<u16>(depth, &intrinsics, params),
        "16SC1" => run::<i16>(depth, &intrinsics, params),
        other => Err(UnprojectError::NotSingleChannelDepth(other.to_string())),
    }
}

fn run<P: DepthPixel>(
    depth: &Image,
    intrinsics: &Intrinsics,
    params: &Params,
) -> Result<(Vec<u8>, i32), UnprojectError> {
    let width = depth.width.max(0) as usize;
    let height = depth.height.max(0) as usize;
    let tight_row = width * P::SIZE;
    if depth.data.len() < height * tight_row {
        return Err(UnprojectError::ShortBuffer);
    }
    // Honour `step` only when the buffer is actually padded to it; dimos's own
    // Python encoder packs rows tightly and leaves `step` unset.
    let row_stride = match depth.step as usize {
        s if s >= tight_row && depth.data.len() >= height * s => s,
        _ => tight_row,
    };

    let step = params.decimation.max(1);
    let scale = params.depth_scale;
    // Resolved once, outside the loop: the common case is no gate at all, and
    // an ungated point must cost nothing extra.
    let gate = params
        .height_gate
        .is_active()
        .then_some(&params.height_gate);
    let mut data: Vec<u8> =
        Vec::with_capacity(height.div_ceil(step) * width.div_ceil(step) * POINT_STEP);
    let mut count: i32 = 0;

    for row in (0..height).step_by(step) {
        let row_base = row * row_stride;
        let v = row as f32;
        for column in (0..width).step_by(step) {
            let z = P::read(&depth.data, row_base + column * P::SIZE).metres(scale);
            // NaN fails both comparisons, so it is excluded here.
            if !(z >= params.min_range_m && z <= params.max_range_m) {
                continue;
            }
            let x = (column as f32 - intrinsics.cx) * z / intrinsics.fx;
            let y = (v - intrinsics.cy) * z / intrinsics.fy;
            if gate.is_some_and(|gate| !gate.admits(x, y, z)) {
                continue;
            }
            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&0.0f32.to_le_bytes());
            count += 1;
        }
    }
    Ok((data, count))
}

#[cfg(test)]
mod tests {
    use super::*;

    const WIDTH: i32 = 8;
    const HEIGHT: i32 = 6;
    const FX: f64 = 100.0;
    const FY: f64 = 120.0;
    const CX: f64 = 3.5;
    const CY: f64 = 2.5;

    fn camera_info(width: i32, height: i32) -> CameraInfo {
        let mut info = CameraInfo {
            width,
            height,
            ..Default::default()
        };
        info.K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0];
        info
    }

    fn depth_image(metres: &[f32]) -> Image {
        let mut data = Vec::new();
        for m in metres {
            data.extend_from_slice(&m.to_le_bytes());
        }
        Image {
            width: WIDTH,
            height: HEIGHT,
            encoding: "32FC1".into(),
            data,
            ..Default::default()
        }
    }

    fn params() -> Params {
        Params {
            decimation: 1,
            min_range_m: 0.2,
            max_range_m: 6.0,
            depth_scale: 0.001,
            height_gate: HeightGate::OFF,
        }
    }

    fn points(buffer: &[u8]) -> Vec<(f32, f32, f32)> {
        buffer
            .as_chunks::<POINT_STEP>()
            .0
            .iter()
            .map(|p| {
                (
                    f32::from_le_bytes(p[0..4].try_into().unwrap()),
                    f32::from_le_bytes(p[4..8].try_into().unwrap()),
                    f32::from_le_bytes(p[8..12].try_into().unwrap()),
                )
            })
            .collect()
    }

    #[test]
    fn unprojects_with_the_pinhole_model() {
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let (buffer, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        assert_eq!(count, WIDTH * HEIGHT);
        let got = points(&buffer);
        for row in 0..HEIGHT {
            for column in 0..WIDTH {
                let (x, y, z) = got[(row * WIDTH + column) as usize];
                assert!((x - (column as f32 - CX as f32) * 2.0 / FX as f32).abs() < 1e-6);
                assert!((y - (row as f32 - CY as f32) * 2.0 / FY as f32).abs() < 1e-6);
                assert!((z - 2.0).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn float_depth_is_already_metres() {
        // The R1 Pro head publishes 32FC1 metres, so depth_scale must not apply.
        let flat = vec![3.0f32; (WIDTH * HEIGHT) as usize];
        let (buffer, _) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        for (_, _, z) in points(&buffer) {
            assert!((z - 3.0).abs() < 1e-6);
        }
    }

    #[test]
    fn integer_depth_is_scaled() {
        let mut data = Vec::new();
        for _ in 0..(WIDTH * HEIGHT) {
            data.extend_from_slice(&3000u16.to_le_bytes());
        }
        let image = Image {
            width: WIDTH,
            height: HEIGHT,
            encoding: "16UC1".into(),
            data,
            ..Default::default()
        };

        let (buffer, _) = unproject(&image, &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        for (_, _, z) in points(&buffer) {
            assert!((z - 3.0).abs() < 1e-6);
        }
    }

    #[test]
    fn range_gate_drops_out_of_range_pixels() {
        let mut flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        flat[0] = 0.05;
        flat[1] = 50.0;
        flat[2] = 0.0;
        let (_, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        assert_eq!(count, WIDTH * HEIGHT - 3);
    }

    #[test]
    fn nan_depth_is_dropped() {
        let mut flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        flat[0] = f32::NAN;
        let (_, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        assert_eq!(count, WIDTH * HEIGHT - 1);
    }

    #[test]
    fn decimation_keeps_full_resolution_pixel_coordinates() {
        // Decimated points must be a strict subset of the undecimated ones.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let image = depth_image(&flat);
        let info = camera_info(WIDTH, HEIGHT);
        let (every_pixel, _) = unproject(&image, &info, &params()).unwrap();
        let (every_other, count) = unproject(
            &image,
            &info,
            &Params {
                decimation: 2,
                ..params()
            },
        )
        .unwrap();

        assert_eq!(count, (HEIGHT / 2) * (WIDTH / 2));
        let all = points(&every_pixel);
        for point in points(&every_other) {
            assert!(all
                .iter()
                .any(|p| (p.0 - point.0).abs() < 1e-6 && (p.1 - point.1).abs() < 1e-6));
        }
    }

    #[test]
    fn rescales_intrinsics_to_the_depth_resolution() {
        // Registered depth at half the calibrated resolution must not skew geometry.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let (buffer, _) = unproject(
            &depth_image(&flat),
            &camera_info(WIDTH * 2, HEIGHT * 2),
            &params(),
        )
        .unwrap();

        let got = points(&buffer);
        for row in 0..HEIGHT {
            for column in 0..WIDTH {
                let (x, y, _) = got[(row * WIDTH + column) as usize];
                assert!(
                    (x - (column as f32 - CX as f32 / 2.0) * 2.0 / (FX as f32 / 2.0)).abs() < 1e-6
                );
                assert!(
                    (y - (row as f32 - CY as f32 / 2.0) * 2.0 / (FY as f32 / 2.0)).abs() < 1e-6
                );
            }
        }
    }

    #[test]
    fn colour_encodings_are_rejected() {
        let image = Image {
            width: WIDTH,
            height: HEIGHT,
            encoding: "rgb8".into(),
            data: vec![0; (WIDTH * HEIGHT * 3) as usize],
            ..Default::default()
        };

        assert_eq!(
            unproject(&image, &camera_info(WIDTH, HEIGHT), &params()),
            Err(UnprojectError::NotSingleChannelDepth("rgb8".into()))
        );
    }

    #[test]
    fn a_zero_focal_length_is_rejected() {
        // Otherwise every point divides out to infinity and the map is poisoned.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let info = CameraInfo::default();

        assert_eq!(
            unproject(&depth_image(&flat), &info, &params()),
            Err(UnprojectError::DegenerateIntrinsics)
        );
    }

    #[test]
    fn a_short_buffer_is_rejected() {
        let mut image = depth_image(&vec![2.0f32; (WIDTH * HEIGHT) as usize]);
        image.data.truncate(8);

        assert_eq!(
            unproject(&image, &camera_info(WIDTH, HEIGHT), &params()),
            Err(UnprojectError::ShortBuffer)
        );
    }

    #[test]
    fn padded_rows_are_honoured() {
        // A driver that pads rows sets `step`; reading tightly would shear the image.
        let width = 3usize;
        let height = 2usize;
        let stride = width * 4 + 8;
        let mut data = vec![0u8; height * stride];
        for row in 0..height {
            for column in 0..width {
                let z = 1.0f32 + row as f32;
                let at = row * stride + column * 4;
                data[at..at + 4].copy_from_slice(&z.to_le_bytes());
            }
        }
        let image = Image {
            width: width as i32,
            height: height as i32,
            encoding: "32FC1".into(),
            step: stride as i32,
            data,
            ..Default::default()
        };

        let (buffer, count) = unproject(&image, &camera_info(3, 2), &params()).unwrap();

        assert_eq!(count, 6);
        let got = points(&buffer);
        assert!(got[0..3].iter().all(|p| (p.2 - 1.0).abs() < 1e-6));
        assert!(got[3..6].iter().all(|p| (p.2 - 2.0).abs() < 1e-6));
    }

    /// `Quaternion.from_euler(Vector3(0.3, -0.5, 0.9))` on the Python side,
    /// turned into a matrix by scipy. Pinned as numbers rather than rebuilt
    /// from the formula, so this test is about agreeing with Python and not
    /// about agreeing with itself.
    #[test]
    fn rpy_follows_the_python_quaternion_convention() {
        let want = [
            [
                0.5455140684515629,
                -0.8364104391130222,
                -0.05321633644084381,
            ],
            [0.6874340361485555, 0.4828649796432185, -0.542471987644211],
            [0.4794255386042031, 0.2593433800522308, 0.8383866435942036],
        ];
        let got = rotation_from_rpy(0.3, -0.5, 0.9);
        for row in 0..3 {
            for column in 0..3 {
                assert!(
                    (got[row][column] - want[row][column]).abs() < 1e-12,
                    "[{row}][{column}]: {} vs {}",
                    got[row][column],
                    want[row][column]
                );
            }
        }
    }

    /// A head camera looking straight down from 1.2 m: optical z (forward)
    /// becomes base -z, optical x (image right) becomes base -y, and the top of
    /// the image is the robot's forward. That is roll pi, yaw -pi/2 in the
    /// convention above.
    fn looking_down_from(height_m: f64, min: Option<f64>, max: Option<f64>) -> HeightGate {
        HeightGate::new(
            [0.0, 0.0, height_m],
            [std::f64::consts::PI, 0.0, -std::f64::consts::FRAC_PI_2],
            min,
            max,
        )
    }

    /// Rows 0-2 at 0.5 m, rows 3-5 at 1.5 m.
    fn two_bands() -> Vec<f32> {
        (0..(WIDTH * HEIGHT))
            .map(|i| if i / WIDTH < 3 { 0.5 } else { 1.5 })
            .collect()
    }

    #[test]
    fn a_camera_pitched_down_gates_on_depth_below_it() {
        // At 1.2 m up and looking down, 0.5 m of depth is 0.7 m above the
        // floor and 1.5 m of depth is 0.3 m below it: a floor-level gate keeps
        // exactly the near band.
        let gated = Params {
            height_gate: looking_down_from(1.2, Some(0.0), None),
            ..params()
        };
        let (buffer, count) = unproject(
            &depth_image(&two_bands()),
            &camera_info(WIDTH, HEIGHT),
            &gated,
        )
        .unwrap();
        assert_eq!(count, 3 * WIDTH);
        assert!(points(&buffer).iter().all(|p| (p.2 - 0.5).abs() < 1e-6));

        // And the other side of the band: a ceiling at 0.5 m keeps only the
        // far, low points -- the gate is a band, not a floor.
        let capped = Params {
            height_gate: looking_down_from(1.2, None, Some(0.5)),
            ..params()
        };
        let (buffer, count) = unproject(
            &depth_image(&two_bands()),
            &camera_info(WIDTH, HEIGHT),
            &capped,
        )
        .unwrap();
        assert_eq!(count, 3 * WIDTH);
        assert!(points(&buffer).iter().all(|p| (p.2 - 1.5).abs() < 1e-6));
    }

    #[test]
    fn the_pose_translation_reaches_the_height() {
        // Same picture, camera lowered to 0.4 m: now even the near band is
        // below the floor and nothing survives a floor-level gate.
        let gated = Params {
            height_gate: looking_down_from(0.4, Some(0.0), None),
            ..params()
        };
        let (_, count) = unproject(
            &depth_image(&two_bands()),
            &camera_info(WIDTH, HEIGHT),
            &gated,
        )
        .unwrap();
        assert_eq!(count, 0);
    }

    #[test]
    fn an_identity_pose_makes_height_the_optical_z() {
        // With no pose the base frame IS the optical frame, so "height" is
        // optical z, which is depth. Not what anyone wants on a robot -- it is
        // here so the convention is pinned at its simplest.
        let gated = Params {
            height_gate: HeightGate::new([0.0; 3], [0.0; 3], None, Some(1.0)),
            ..params()
        };
        let (buffer, count) = unproject(
            &depth_image(&two_bands()),
            &camera_info(WIDTH, HEIGHT),
            &gated,
        )
        .unwrap();
        assert_eq!(count, 3 * WIDTH);
        assert!(points(&buffer).iter().all(|p| (p.2 - 0.5).abs() < 1e-6));
    }

    #[test]
    fn a_forward_looking_camera_gates_on_optical_y_which_points_down() {
        // The usual optical-in-base pose for a camera looking along base x:
        // roll -pi/2, yaw -pi/2. Optical y points DOWN the image and lands on
        // base -z, so a floor at height zero keeps the rows ABOVE the
        // principal point (v < cy), not below it.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let gated = Params {
            height_gate: HeightGate::new(
                [0.0; 3],
                [
                    -std::f64::consts::FRAC_PI_2,
                    0.0,
                    -std::f64::consts::FRAC_PI_2,
                ],
                Some(0.0),
                None,
            ),
            ..params()
        };
        let (buffer, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &gated).unwrap();
        // cy is 2.5, so rows 0, 1, 2 have y < 0 and so height > 0.
        assert_eq!(count, 3 * WIDTH);
        assert!(points(&buffer).iter().all(|p| p.1 < 0.0));
    }

    #[test]
    fn no_bounds_is_no_gate_whatever_the_pose() {
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let ungated = Params {
            height_gate: HeightGate::new([1.0, 2.0, 3.0], [0.4, 0.5, 0.6], None, None),
            ..params()
        };
        assert!(!ungated.height_gate.is_active());
        let (_, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &ungated).unwrap();
        assert_eq!(count, WIDTH * HEIGHT);
    }
}
