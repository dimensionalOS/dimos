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

//! reflective_column_node (NOS /opt/robot/share/charge_manager/bin, decompiled) on one sweep, step for step as its
//! ProcessPointCloudData does it, in its float types:
//!
//! 1. ReciAndTransformPoints: the vendor driver's base_link points (its f64 transform of the lidar's, as f32) with
//!    intensity above 230, in a box and a +-45 deg sector ahead; then a VoxelGrid of 1 cm and, on 20 points or more, a
//!    RadiusOutlierRemoval ([`super::pcl`]: PCL 1.10's, point order and float sums as it has them).
//! 2. PointsCloudClustering: Euclidean clusters of 5 cm, 30 points or more; FilterReflectiveColumn keeps those taller
//!    than 1.8 x their width (z / y extent).
//! 3. FindTargetRefleColu: the pair of posts whose centroids are nearest 0.4 m apart (within 0.18, 60-120 deg off the x
//!    axis), the most lateral when several are found; FilterOutSuctionPoints keeps each post's points in front of its
//!    centroid within 2 cm (y) / 8 cm (z) of it; CalculateCenter: their centroid, x pushed 0.045 x sin|atan2f(x, y)|.
//! 4. calculateLidarPosition: base_link's position in the posts' frame (origin their midpoint, y along the low-y post
//!    to the high-y one) and the yaw atan2(dx, dy) of that line: the pose reflective_column_node logs as "一次计算--雷达
//!    位置" and its output stage ([`super::vendor::ReflectorFilter`], the docking's) filters and publishes.
//!
//! [`Detector`] turns that into the dock's [`DockPose`] in base_link, placed so charge_manager's inputs from it
//! (docking's target_pose) are the vendor's.

use super::config::{DetectorConfig, DockConfig};
use super::pcl;
use super::DockPose;

/// A lidar's mount in base_link: p_base = rotation * p_lidar + translation.
#[derive(Debug, Clone, Copy)]
pub struct Mount {
    pub translation: [f64; 3],
    pub rotation: [[f64; 3]; 3],
}

impl Mount {
    /// From a position (m) and roll, pitch, yaw (rad; R = Rz(yaw) Ry(pitch) Rx(roll), as tf).
    pub fn from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Mount {
        let (sr, cr) = rpy[0].sin_cos();
        let (sp, cp) = rpy[1].sin_cos();
        let (sy, cy) = rpy[2].sin_cos();
        Mount {
            translation: xyz,
            rotation: [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
        }
    }

    pub fn to_base(&self, p: [f64; 3]) -> [f64; 3] {
        let r = &self.rotation;
        let t = &self.translation;
        [
            r[0][0] * p[0] + r[0][1] * p[1] + r[0][2] * p[2] + t[0],
            r[1][0] * p[0] + r[1][1] * p[1] + r[1][2] * p[2] + t[1],
            r[2][0] * p[0] + r[2][1] * p[1] + r[2][2] * p[2] + t[2],
        ]
    }
}

/// The vendor lidar driver's transform (rs_driver ENABLE_TRANSFORM: translation x Rz(yaw) x Ry(pitch) x Rx(roll) from
/// its f32 parameters, applied in f64, the result cast to f32).
#[derive(Debug, Clone, Copy)]
pub struct DriverTransform {
    rotation: [[f64; 3]; 3],
    translation: [f64; 3],
}

/// Eigen's AngleAxisd(angle, unit axis k).toRotationMatrix().
fn angle_axis(angle: f64, k: usize) -> [[f64; 3]; 3] {
    let axis = [0, 1, 2].map(|i| if i == k { 1.0 } else { 0.0 });
    let (s, c) = angle.sin_cos();
    let sin_axis = axis.map(|a| s * a);
    let cos1_axis = axis.map(|a| (1.0 - c) * a);
    let mut r = [[0.0; 3]; 3];
    let tmp = cos1_axis[0] * axis[1];
    r[0][1] = tmp - sin_axis[2];
    r[1][0] = tmp + sin_axis[2];
    let tmp = cos1_axis[0] * axis[2];
    r[0][2] = tmp + sin_axis[1];
    r[2][0] = tmp - sin_axis[1];
    let tmp = cos1_axis[1] * axis[2];
    r[1][2] = tmp - sin_axis[0];
    r[2][1] = tmp + sin_axis[0];
    for i in 0..3 {
        r[i][i] = cos1_axis[i] * axis[i] + c;
    }
    r
}

fn mat_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    r
}

impl DriverTransform {
    pub fn new(xyz: [f32; 3], rpy: [f32; 3]) -> DriverTransform {
        let [roll, pitch, yaw] = rpy.map(|v| v as f64);
        let zy = mat_mul(&angle_axis(yaw, 2), &angle_axis(pitch, 1));
        DriverTransform {
            rotation: mat_mul(&zy, &angle_axis(roll, 0)),
            translation: xyz.map(|v| v as f64),
        }
    }

    pub fn apply(&self, p: [f32; 3]) -> [f32; 3] {
        let (r, t) = (&self.rotation, &self.translation);
        let [x, y, z] = p.map(|v| v as f64);
        [0, 1, 2].map(|i| (r[i][0] * x + r[i][1] * y + r[i][2] * z + t[i]) as f32)
    }
}

/// One sweep's verdict.
#[derive(Debug, Clone, PartialEq)]
pub enum Detection {
    Found {
        pose: DockPose,
        /// reflective_column_node's pose: base_link's x, y (m) in the posts' frame and the yaw (rad)
        vendor: [f64; 3],
        /// the posts' centres in base_link (x, y): the low-y one, the high-y one
        posts: [[f64; 2]; 2],
        /// each post's points after FilterOutSuctionPoints
        points: [usize; 2],
    },
    /// why not (as reflective_column_node logs it)
    Missing(String),
}

/// What reflective_column_node logs of a sweep, step by step (for checking against its log).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Trace {
    /// 【step1】点云数量: the points after the voxel grid and outlier removal
    pub step1: Option<usize>,
    /// 簇 z/y 比例: each cluster's z / y extent (clusters with a y extent)
    pub ratios: Vec<f32>,
    /// 反光柱数量: the posts (when 2 or more)
    pub columns: Option<usize>,
    /// 距离差, 角度: each pair outside the spacing or the angles
    pub rejected: Vec<(f64, f64)>,
    /// 两反光柱距离误差: the last best spacing error when no pair was kept
    pub no_pair: Option<f64>,
    /// 【step 2】: each post's points after FilterOutSuctionPoints (low-y, high-y)
    pub counts: Option<[usize; 2]>,
    /// 【step 3】: each post's centre (low-y, high-y; z 0)
    pub centers: Option<[[f64; 2]; 2]>,
}

/// reflective_column_node with its numbers, and where our DockPose puts the dock.
#[derive(Debug, Clone)]
pub struct Detector {
    cfg: DetectorConfig,
    driver: DriverTransform,
    /// the dock frame's x of the vendor's posts' frame origin: docked_x_m - pose_offset_x
    axis_shift: f64,
}

/// CalculateCenter: the cloud's centroid (f32), x + push x sinf|atan2f(x, y)| fused in f64.
fn post_center(cloud: &[pcl::Point], push: f64) -> [f64; 2] {
    let c = pcl::centroid(cloud);
    let s = c[0].atan2(c[1]).abs().sin() as f64;
    [s.mul_add(push, c[0] as f64), c[1] as f64]
}

/// calculateLidarPosition(a, b) with z = 0 (its z terms vanish exactly): u = (a - b) / |a - b|, s = a + b; base_link at
/// x = (-uy sx + ux sy) / 2, y = (-ux sx - uy sy) / 2.
fn lidar_position(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let d = [a[0] - b[0], a[1] - b[1]];
    let n2 = d[1] * d[1] + d[0] * d[0];
    let u = if n2 > 0.0 {
        let n = n2.sqrt();
        [d[0] / n, d[1] / n]
    } else {
        d
    };
    let s = [a[0] + b[0], a[1] + b[1]];
    [
        (-u[1] * s[0] + u[0] * s[1]) / 2.0,
        (-u[0] * s[0] + -u[1] * s[1]) / 2.0,
    ]
}

impl Detector {
    pub fn new(dock: &DockConfig) -> Detector {
        let d = &dock.detector;
        Detector {
            cfg: d.clone(),
            driver: DriverTransform::new(d.driver_xyz, d.driver_rpy),
            axis_shift: dock.sequence.docked_x_m - dock.filter.pose_offset_x,
        }
    }

    /// Find the dock in a sweep: `points` are (x, y, z, intensity) in the front lidar's frame.
    pub fn detect(&self, points: impl IntoIterator<Item = [f32; 4]>) -> Detection {
        self.detect_traced(points, &mut Trace::default())
    }

    /// [`Self::detect`], noting what reflective_column_node would log in `trace`.
    pub fn detect_traced(
        &self,
        points: impl IntoIterator<Item = [f32; 4]>,
        trace: &mut Trace,
    ) -> Detection {
        let c = &self.cfg;
        let miss = |why: &str| Detection::Missing(why.to_string());
        // step 1: ReciAndTransformPoints, VoxelGrid, RadiusOutlierRemoval
        let mut cloud: Vec<pcl::Point> = Vec::new();
        for [x, y, z, intensity] in points {
            let [x, y, z] = self.driver.apply([x, y, z]);
            if intensity > c.min_intensity
                && x as f64 > c.x_min
                && x < c.x_max
                && y.abs() < c.y_max
                && z as f64 > c.z_min
                && (z as f64) < c.z_max
                && (y.atan2(x) as f64).abs() <= c.half_angle_rad
            {
                cloud.push([x, y, z, 1.0]);
            }
        }
        let mut cloud = pcl::voxel_grid(&cloud, [c.voxel_leaf; 3]);
        if cloud.len() >= c.outlier_min_cloud {
            cloud = pcl::radius_outlier_removal(
                &cloud,
                c.outlier_radius as f64,
                c.outlier_min_neighbors,
            );
        }
        trace.step1 = Some(cloud.len());
        if cloud.is_empty() {
            return miss("点云聚类失败 (no reflector returns)");
        }
        // step 2: PointsCloudClustering, FilterReflectiveColumn
        let clusters =
            pcl::euclidean_clusters(&cloud, c.cluster_tolerance, c.cluster_min, c.cluster_max);
        if clusters.len() < 2 {
            return miss(&format!(
                "聚类数量 {} (fewer than 2 clusters)",
                clusters.len()
            ));
        }
        let mut columns: Vec<Vec<pcl::Point>> = Vec::new();
        for members in &clusters {
            let points: Vec<pcl::Point> = members.iter().map(|&i| cloud[i]).collect();
            let (lo, hi) = pcl::min_max(&points);
            let width = hi[1] - lo[1];
            if width != 0.0 {
                let ratio = (hi[2] - lo[2]).abs() / width.abs();
                trace.ratios.push(ratio);
                if ratio > c.column_min_ratio {
                    columns.push(points);
                }
            }
        }
        if columns.len() < 2 {
            return miss("反光柱数量少于2个 (fewer than 2 posts)");
        }
        trace.columns = Some(columns.len());
        // step 3: FindTargetRefleColu, FilterOutSuctionPoints, CalculateCenter
        let Some([lo, hi]) = self.find_target(&columns, trace) else {
            return miss("没有找到足够的反光柱簇 (no pair of posts at the spacing)");
        };
        let front = |posts: &[pcl::Point]| -> Vec<pcl::Point> {
            let m = pcl::centroid(posts);
            let (mx, my, mz) = (m[0] as f64, m[1] as f64, m[2] as f64);
            posts
                .iter()
                .filter(|p| {
                    let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
                    x < mx + 0.0
                        && y < my + c.suction_y
                        && my - c.suction_y < y
                        && z < mz + c.suction_z
                        && mz - c.suction_z < z
                })
                .copied()
                .collect()
        };
        let (lo, hi) = (front(&columns[lo]), front(&columns[hi]));
        trace.counts = Some([lo.len(), hi.len()]);
        if lo.is_empty() || hi.is_empty() {
            return miss("a post with no points in front of its centroid");
        }
        let (a, b) = (
            post_center(&lo, c.center_push),
            post_center(&hi, c.center_push),
        );
        trace.centers = Some([a, b]);
        // step 4: calculateLidarPosition(high-y, low-y) and the line's yaw
        let [x, y] = lidar_position(b, a);
        let yaw = (b[0] - a[0]).atan2(b[1] - a[1]);
        Detection::Found {
            pose: DockPose::from_robot_in_dock([self.axis_shift - x, -y, yaw]),
            vendor: [x, y, yaw],
            posts: [a, b],
            points: [lo.len(), hi.len()],
        }
    }

    /// FindTargetRefleColu: (low-y post, high-y post) indices into `columns`.
    fn find_target(&self, columns: &[Vec<pcl::Point>], trace: &mut Trace) -> Option<[usize; 2]> {
        let c = &self.cfg;
        let n = columns.len();
        let mut used = vec![false; n];
        let mut cent = vec![[0.0f32; 4]; n];
        // pairs (i, j) kept, by i; the last improving pair and its spacing error persist across i, as in the binary
        let mut pairs: std::collections::BTreeMap<usize, usize> = Default::default();
        let (mut any, mut best, mut bi, mut bj) = (false, 0.0f64, 0usize, 0usize);
        for i in 0..n - 1 {
            if used[i] {
                continue;
            }
            cent[i] = pcl::centroid(&columns[i]);
            used[i] = true;
            best = f64::MAX;
            for j in i + 1..n {
                if used[j] {
                    continue;
                }
                cent[j] = pcl::centroid(&columns[j]);
                let d: [f32; 4] = [0, 1, 2, 3].map(|k| cent[i][k] - cent[j][k]);
                let sq = d.map(|v| v * v);
                let dist = ((sq[0] + sq[1]) + (sq[2] + sq[3])).sqrt();
                let off = (dist as f64 - c.post_spacing).abs();
                let angle = (cent[j][1] - cent[i][1]).atan2(cent[j][0] - cent[i][0]);
                let deg = (angle as f64 * 180.0) / std::f64::consts::PI;
                if off < c.spacing_tolerance
                    && deg.abs() < c.pair_angle_deg[1]
                    && c.pair_angle_deg[0] < deg.abs()
                {
                    any = true;
                    if best > off {
                        (best, bi, bj) = (off, i, j);
                    }
                } else {
                    trace.rejected.push((off, deg));
                }
            }
            if best < c.pair_max_off {
                pairs.insert(bi, bj);
                used[bj] = true;
            }
        }
        if !any {
            return None;
        }
        if pairs.is_empty() {
            trace.no_pair = Some(best);
            return None;
        }
        if pairs.len() >= 2 {
            // the most lateral pair: the smallest |atan2f(dx, dy)|
            let mut least = f64::MAX;
            for (&i, &j) in &pairs {
                let a = (cent[i][0] - cent[j][0])
                    .atan2(cent[i][1] - cent[j][1])
                    .abs() as f64;
                if a < least {
                    (least, bi, bj) = (a, i, j);
                }
            }
        }
        if (cent[bi][1] + cent[bj][1]) * 0.5 <= cent[bi][1] {
            Some([bj, bi])
        } else {
            Some([bi, bj])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dock::wrap;
    use std::f64::consts::PI;

    fn dock_cfg() -> DockConfig {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        crate::config::Config::load(&path).unwrap().dock
    }

    /// The front Airy's mount (dimos/robot/deeprobotics/m20/constants.py).
    fn front() -> Mount {
        Mount::from_xyz_rpy([0.32028, 0.0, -0.013], [0.0, -PI / 2.0, PI])
    }

    /// The real dock's posts: 0.374 m apart centre to centre, 25 mm radius (world.py charge_dock).
    const SPACING: f64 = 0.374;
    const RADIUS: f64 = 0.025;

    /// Small deterministic PRNG (xorshift) and a Box-Muller normal.
    struct Rng(u64);
    impl Rng {
        fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn normal(&mut self) -> f64 {
            let (u, v) = (self.uniform().max(1e-12), self.uniform());
            (-2.0 * u.ln()).sqrt() * (2.0 * PI * v).cos()
        }
    }

    /// A world of vertical cylinders (x, y, radius, intensity) and a back wall, swept by an Airy-like lidar on the
    /// front mount: 96 beams over 0-90 deg of elevation about the spin axis, 0.2 deg azimuth, Gaussian range noise.
    fn sweep(
        posts: &[([f64; 2], f64, f32)],
        wall_x: Option<f64>,
        noise: f64,
        rng: &mut Rng,
    ) -> Vec<[f32; 4]> {
        let m = front();
        let origin = m.translation;
        let mut out = Vec::new();
        for beam in 0..96 {
            let el = (beam as f64 * 90.0 / 95.0).to_radians();
            if el < 25f64.to_radians() {
                continue;
            }
            for k in 0..1800 {
                let az = (k as f64 * 0.2).to_radians();
                let local = [el.cos() * az.cos(), el.cos() * az.sin(), el.sin()];
                // direction in base_link (rotation only)
                let p = m.to_base(local);
                let d = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
                let mut hit: Option<(f64, f32)> = None;
                for &(c, r, intensity) in posts {
                    // ray-cylinder (vertical axis through c), floor-plane quadratic
                    let (ox, oy) = (origin[0] - c[0], origin[1] - c[1]);
                    let a = d[0] * d[0] + d[1] * d[1];
                    let b = 2.0 * (ox * d[0] + oy * d[1]);
                    let cc = ox * ox + oy * oy - r * r;
                    let disc = b * b - 4.0 * a * cc;
                    if a > 0.0 && disc >= 0.0 {
                        let t = (-b - disc.sqrt()) / (2.0 * a);
                        let z = origin[2] + t * d[2];
                        if t > 0.0 && (-0.47..0.33).contains(&z) && hit.is_none_or(|(h, _)| t < h) {
                            hit = Some((t, intensity));
                        }
                    }
                }
                if let Some(wx) = wall_x {
                    if d[0] > 0.0 {
                        let t = (wx - origin[0]) / d[0];
                        if hit.is_none_or(|(h, _)| t < h) && t < 30.0 {
                            hit = Some((t, 30.0));
                        }
                    }
                }
                if let Some((t, intensity)) = hit {
                    let t = t + noise * rng.normal();
                    out.push([
                        (local[0] * t) as f32,
                        (local[1] * t) as f32,
                        (local[2] * t) as f32,
                        intensity,
                    ]);
                }
            }
        }
        out
    }

    /// Posts `spacing` apart, their midpoint at (x, y) in base_link, the dock's outward axis at `yaw`.
    fn dock_posts(
        x: f64,
        y: f64,
        yaw: f64,
        spacing: f64,
        radius: f64,
    ) -> Vec<([f64; 2], f64, f32)> {
        let side = [-yaw.sin(), yaw.cos()];
        [-1.0, 1.0]
            .iter()
            .map(|s| {
                (
                    [
                        x + s * spacing / 2.0 * side[0],
                        y + s * spacing / 2.0 * side[1],
                    ],
                    radius,
                    250.0,
                )
            })
            .collect()
    }

    fn found(d: &Detection) -> (DockPose, [f64; 3]) {
        match d {
            Detection::Found { pose, vendor, .. } => (*pose, *vendor),
            Detection::Missing(why) => panic!("not found: {why}"),
        }
    }

    #[test]
    fn the_driver_transform_puts_the_spin_axis_forward() {
        let d = dock_cfg().detector;
        let t = DriverTransform::new(d.driver_xyz, d.driver_rpy);
        let fwd = t.apply([0.0, 0.0, 1.0]);
        assert!(
            (fwd[0] - 1.32028).abs() < 1e-5 && fwd[1].abs() < 1e-5,
            "{fwd:?}"
        );
        let up = t.apply([1.0, 0.0, 0.0]);
        assert!((up[2] - (1.0 - 0.013)).abs() < 1e-5, "{up:?}");
    }

    /// reflective_column_node's log of 2026-11-20 16:36:16.732 (docked): its post centres give its position to the
    /// log's 6 decimals.
    #[test]
    fn the_position_from_two_centres_is_the_logged_one() {
        let (lo, hi) = ([0.754140, -0.199226], [0.763291, 0.168756]);
        let [x, y] = lidar_position(hi, lo);
        let yaw = (hi[0] - lo[0]).atan2(hi[1] - lo[1]);
        assert!(
            (x - -0.758860).abs() < 2e-6 && (y - -0.003633).abs() < 2e-6,
            "{x} {y}"
        );
        assert!((yaw - 0.024864).abs() < 2e-6, "{yaw}");
    }

    /// The dock's pose gives charge_manager (docking's target_pose: docked_x_m minus the robot's x out, -y, yaw) the
    /// vendor's position plus pubPose's 0.76, as reflective_column_node publishes it.
    #[test]
    fn charge_managers_inputs_are_the_vendors() {
        let dock = dock_cfg();
        let detector = Detector::new(&dock);
        let mut rng = Rng(5);
        for (dist, lat, yaw_deg) in [(0.76, 0.0, 0.0), (1.2, 0.1, 8.0), (1.8, -0.25, -12.0)] {
            let posts = dock_posts(dist, lat, PI + f64::to_radians(yaw_deg), SPACING, RADIUS);
            let (pose, vendor) =
                found(&detector.detect(sweep(&posts, Some(dist + 0.6), 0.005, &mut rng)));
            let [out, y, yaw] = pose.robot_in_dock();
            let cm = [dock.sequence.docked_x_m - out, -y, yaw];
            let want = [vendor[0] + dock.filter.pose_offset_x, vendor[1], vendor[2]];
            for k in 0..3 {
                assert!((cm[k] - want[k]).abs() < 1e-12, "{cm:?} vs {want:?}");
            }
        }
    }

    /// Synthetic sweeps over distances, lateral offsets and dock yaws (1 cm range noise): the dock lands within the
    /// vendor's centroid method's own error of the posts' axes.
    #[test]
    fn finds_the_dock_on_synthetic_sweeps() {
        let detector = Detector::new(&dock_cfg());
        let mut rng = Rng(0x9e3779b97f4a7c15);
        let mut worst = [0.0f64; 3];
        for &dist in &[0.76, 1.0, 1.5, 2.0] {
            for &lat in &[-0.2, 0.0, 0.15] {
                for &yaw_deg in &[-10.0, 0.0, 10.0] {
                    let yaw = PI + f64::to_radians(yaw_deg);
                    let posts = dock_posts(dist, lat, yaw, SPACING, RADIUS);
                    // a post outside reflective_column_node's +-45 deg sector: skipped
                    if posts
                        .iter()
                        .any(|(c, _, _)| c[1].atan2(c[0]).abs() > 40f64.to_radians())
                    {
                        continue;
                    }
                    let (pose, _) =
                        found(&detector.detect(sweep(&posts, Some(dist + 0.6), 0.01, &mut rng)));
                    let e = [pose.x - dist, pose.y - lat, wrap(pose.yaw - yaw)];
                    println!(
                        "dist {dist:.2} lat {lat:+.2} yaw {yaw_deg:+.0}: x {:+.4} y {:+.4} m yaw {:+.2} deg",
                        e[0],
                        e[1],
                        e[2].to_degrees()
                    );
                    for k in 0..3 {
                        worst[k] = worst[k].max(e[k].abs());
                    }
                }
            }
        }
        assert!(worst[0] < 0.03 && worst[1] < 0.02, "worst {worst:?}");
        assert!(
            worst[2] < 3f64.to_radians(),
            "worst yaw {:.2} deg",
            worst[2].to_degrees()
        );
    }

    #[test]
    fn a_third_reflector_off_the_spacing_is_ignored() {
        let detector = Detector::new(&dock_cfg());
        let mut rng = Rng(7);
        let mut posts = dock_posts(1.2, 0.05, PI, SPACING, RADIUS);
        posts.push(([1.3, 0.6], RADIUS, 250.0));
        let (pose, _) = found(&detector.detect(sweep(&posts, None, 0.005, &mut rng)));
        assert!(
            (pose.x - 1.2).abs() < 0.03 && (pose.y - 0.05).abs() < 0.02,
            "{pose:?}"
        );
    }

    #[test]
    fn plain_surfaces_and_one_post_are_not_a_dock() {
        let detector = Detector::new(&dock_cfg());
        let mut rng = Rng(11);
        let missing = |d: Detection| matches!(d, Detection::Missing(_));
        // below the intensity threshold (230)
        let dull: Vec<_> = dock_posts(1.0, 0.0, PI, SPACING, RADIUS)
            .into_iter()
            .map(|(c, r, _)| (c, r, 200.0))
            .collect();
        assert!(missing(detector.detect(sweep(
            &dull,
            Some(1.6),
            0.01,
            &mut rng
        ))));
        let one = vec![([1.0, 0.15], RADIUS, 250.0f32)];
        assert!(missing(detector.detect(sweep(&one, None, 0.01, &mut rng))));
        // posts 0.7 m apart: outside 0.4 +- 0.18
        let wide = dock_posts(1.0, 0.0, PI, 0.7, RADIUS);
        assert!(missing(detector.detect(sweep(&wide, None, 0.01, &mut rng))));
    }

    #[test]
    fn garbage_points_never_panic() {
        let detector = Detector::new(&dock_cfg());
        let pts = vec![
            [f32::NAN, 0.0, 1.0, 255.0],
            [f32::INFINITY, 1.0, 1.0, 255.0],
            [0.0, 0.0, 0.0, 255.0],
            [0.0, 0.0, 1.0, f32::NAN],
        ];
        assert!(matches!(detector.detect(pts), Detection::Missing(_)));
        assert!(matches!(detector.detect(Vec::new()), Detection::Missing(_)));
    }
}
