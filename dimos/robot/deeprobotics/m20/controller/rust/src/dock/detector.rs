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

//! The dock's two retro-reflective posts in one lidar sweep, as reflective_column_node does it (intensity filter,
//! Euclidean clustering, a pair at the posts' spacing): the high-intensity returns in front of the robot, clustered in
//! the floor plane, each cluster's centre pushed pi/4 of a post radius along its rays (returns lie on the near side), and
//! the pair of clusters whose distance best matches the posts' spacing. The dock's pose is their midpoint and the
//! normal to their line that points at the robot.

use std::collections::HashMap;

use super::config::DetectorConfig;
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

/// One sweep's verdict.
#[derive(Debug, Clone, PartialEq)]
pub enum Detection {
    Found {
        pose: DockPose,
        /// the two posts' centres in base_link (x, y)
        posts: [[f64; 2]; 2],
        /// the reflector returns in each post's cluster
        points: [usize; 2],
    },
    /// why not (no reflector returns, one post, no pair at the spacing, too many returns)
    Missing(String),
}

struct Cluster {
    /// indices of its returns
    members: Vec<usize>,
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

/// The centre of a post of radius `r` through its returns (least squares on |p - c| - r, Gauss-Newton from `start`):
/// it does not lean toward wherever the beams happen to land densest. Falls back to `start` when the returns cannot
/// pin it (a degenerate arc) or it lands more than a radius away (returns bloomed far wider than the post, 1.5 m+ out).
fn fit_circle(points: &[[f64; 2]], members: &[usize], start: [f64; 2], r: f64) -> [f64; 2] {
    let mut c = start;
    for _ in 0..10 {
        let (mut a, mut b) = ([[0.0f64; 2]; 2], [0.0f64; 2]);
        for p in members.iter().filter_map(|&i| points.get(i)) {
            let d = [p[0] - c[0], p[1] - c[1]];
            let n = d[0].hypot(d[1]);
            if n < 1e-9 {
                continue;
            }
            let u = [d[0] / n, d[1] / n];
            for i in 0..2 {
                b[i] += u[i] * (n - r);
                for j in 0..2 {
                    a[i][j] += u[i] * u[j];
                }
            }
        }
        let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        if det.is_nan() || det.abs() <= 1e-6 * (a[0][0] + a[1][1]).powi(2) {
            return start;
        }
        let step = [
            (a[1][1] * b[0] - a[0][1] * b[1]) / det,
            (a[0][0] * b[1] - a[1][0] * b[0]) / det,
        ];
        c = [c[0] + step[0], c[1] + step[1]];
        if step[0].hypot(step[1]) < 1e-6 {
            break;
        }
    }
    let moved = (c[0] - start[0]).hypot(c[1] - start[1]);
    if c.iter().all(|v| v.is_finite()) && moved <= r {
        c
    } else {
        start
    }
}

/// A post's axis from its returns: across the ray the median of their offsets (a saturated reflector's returns bloom
/// about evenly to both sides, so their spread says nothing about its width), along it one radius beyond the front.
fn post_centre(
    points: &[[f64; 2]],
    members: &[usize],
    origin: [f64; 2],
    mean: [f64; 2],
    front: f64,
    r: f64,
) -> [f64; 2] {
    let (dx, dy) = (mean[0] - origin[0], mean[1] - origin[1]);
    let n = dx.hypot(dy);
    if n < 1e-9 {
        return mean;
    }
    let (u, side) = ([dx / n, dy / n], [-dy / n, dx / n]);
    let mut lateral: Vec<f64> = members
        .iter()
        .filter_map(|&i| points.get(i))
        .map(|p| (p[0] - origin[0]) * side[0] + (p[1] - origin[1]) * side[1])
        .collect();
    lateral.sort_by(f64::total_cmp);
    let Some(&l) = lateral.get(lateral.len() / 2) else {
        return mean;
    };
    let a = front + r;
    [
        origin[0] + a * u[0] + l * side[0],
        origin[1] + a * u[1] + l * side[1],
    ]
}

/// Find the dock in a sweep: `points` are (x, y, z, intensity) in the lidar's frame, `mount` places it in base_link.
pub fn detect(
    points: impl IntoIterator<Item = [f32; 4]>,
    mount: &Mount,
    cfg: &DetectorConfig,
) -> Detection {
    let origin = [mount.translation[0], mount.translation[1]];
    let half_angle = cfg.half_angle_deg.to_radians();
    // floor-plane returns as measured, and pushed on toward the post's axis (for clustering and a first centre)
    let mut surface: Vec<[f64; 2]> = Vec::new();
    let mut kept: Vec<[f64; 2]> = Vec::new();
    for [x, y, z, intensity] in points {
        // NaN compares false: never a reflector return
        if (intensity as f64)
            .partial_cmp(&cfg.min_intensity)
            .is_none_or(|o| o.is_lt())
        {
            continue;
        }
        let p = mount.to_base([x as f64, y as f64, z as f64]);
        if !p.iter().all(|v| v.is_finite()) || p[2] < cfg.min_z_m || p[2] > cfg.max_z_m {
            continue;
        }
        let (dx, dy) = (p[0] - origin[0], p[1] - origin[1]);
        let range = dx.hypot(dy);
        if range < cfg.min_range_m || range > cfg.max_range_m || dy.atan2(dx).abs() > half_angle {
            continue;
        }
        if kept.len() >= cfg.max_points {
            return Detection::Missing(format!("more than {} reflector returns", cfg.max_points));
        }
        // returns spread evenly across the post's visible width lie on average pi/4 of a radius in front of its
        // axis (the mean of sqrt(r^2 - u^2) over u in [-r, r]): push each that far on along its ray
        let k = if range > 0.0 {
            std::f64::consts::FRAC_PI_4 * cfg.post_radius_m / range
        } else {
            0.0
        };
        kept.push([p[0] + k * dx, p[1] + k * dy]);
        surface.push([p[0], p[1]]);
    }
    if kept.is_empty() {
        return Detection::Missing("no reflector returns".into());
    }
    // single-linkage clustering on a grid of cluster_distance cells (neighbours only in the 3 x 3 around a cell)
    let cell = cfg.cluster_distance_m;
    let key = |p: &[f64; 2]| ((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, p) in kept.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i);
    }
    let mut parent: Vec<usize> = (0..kept.len()).collect();
    let d2 = cell * cell;
    for (i, p) in kept.iter().enumerate() {
        let (cx, cy) = key(p);
        for nx in cx - 1..=cx + 1 {
            for ny in cy - 1..=cy + 1 {
                let Some(bucket) = grid.get(&(nx, ny)) else {
                    continue;
                };
                for &j in bucket.iter().filter(|&&j| j > i) {
                    let q = &kept[j];
                    if (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) <= d2 {
                        let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                        if a != b {
                            parent[a] = b;
                        }
                    }
                }
            }
        }
    }
    let mut clusters: HashMap<usize, Cluster> = HashMap::new();
    for i in 0..kept.len() {
        let root = find(&mut parent, i);
        clusters
            .entry(root)
            .or_insert(Cluster {
                members: Vec::new(),
            })
            .members
            .push(i);
    }
    let range = |i: usize| (surface[i][0] - origin[0]).hypot(surface[i][1] - origin[1]);
    let posts: Vec<([f64; 2], usize)> = clusters
        .values()
        .filter_map(|c| {
            // a retro-reflector also returns ghosts behind itself along the ray: keep the near band of the cluster
            // (its nearest 2% sets the front)
            let mut ranges: Vec<f64> = c.members.iter().map(|&i| range(i)).collect();
            ranges.sort_by(f64::total_cmp);
            let front = ranges.get(ranges.len() / 50).copied()?;
            let members: Vec<usize> = c
                .members
                .iter()
                .copied()
                .filter(|&i| range(i) <= front + cfg.max_post_depth_m)
                .collect();
            if members.len() < cfg.min_cluster_points {
                return None;
            }
            let (mut lo, mut hi, mut sum) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2], [0.0; 2]);
            for p in members.iter().map(|&i| kept[i]) {
                for k in 0..2 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                    sum[k] += p[k];
                }
            }
            if (hi[0] - lo[0]).hypot(hi[1] - lo[1]) > cfg.max_cluster_width_m {
                return None;
            }
            let mean = [sum[0] / members.len() as f64, sum[1] / members.len() as f64];
            Some((
                {
                    let rough =
                        post_centre(&surface, &members, origin, mean, front, cfg.post_radius_m);
                    fit_circle(&surface, &members, rough, cfg.post_radius_m)
                },
                members.len(),
            ))
        })
        .collect();
    if posts.len() < 2 {
        return Detection::Missing(format!(
            "{} post-sized reflector cluster(s) of {} cluster(s)",
            posts.len(),
            clusters.len()
        ));
    }
    // the pair nearest the spacing; ties (within a millimetre) go to the nearer dock
    let mut best: Option<(f64, f64, usize, usize)> = None;
    for i in 0..posts.len() {
        for j in i + 1..posts.len() {
            let (a, b) = (posts[i].0, posts[j].0);
            let off = ((a[0] - b[0]).hypot(a[1] - b[1]) - cfg.post_spacing_m).abs();
            if off > cfg.spacing_tolerance_m {
                continue;
            }
            let dist = ((a[0] + b[0]) / 2.0).hypot((a[1] + b[1]) / 2.0);
            let better = match best {
                None => true,
                Some((o, d, _, _)) => off < o - 1e-3 || (off < o + 1e-3 && dist < d),
            };
            if better {
                best = Some((off, dist, i, j));
            }
        }
    }
    let Some((_, _, i, j)) = best else {
        return Detection::Missing(format!(
            "no pair of the {} post clusters {:.2} m (+- {:.2}) apart",
            posts.len(),
            cfg.post_spacing_m,
            cfg.spacing_tolerance_m
        ));
    };
    let (a, b) = (posts[i].0, posts[j].0);
    let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
    let (ux, uy) = (b[0] - a[0], b[1] - a[1]);
    // the normal to the posts' line, pointing at the robot (base_link's origin)
    let mut normal = [-uy, ux];
    if normal[0] * -mid[0] + normal[1] * -mid[1] < 0.0 {
        normal = [uy, -ux];
    }
    // left post first, as seen from the robot
    let (left, right) = if a[1] >= b[1] { (i, j) } else { (j, i) };
    Detection::Found {
        pose: DockPose {
            x: mid[0],
            y: mid[1],
            yaw: normal[1].atan2(normal[0]),
        },
        posts: [posts[left].0, posts[right].0],
        points: [posts[left].1, posts[right].1],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dock::wrap;
    use std::f64::consts::PI;

    fn cfg() -> DetectorConfig {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        crate::config::Config::load(&path).unwrap().dock.detector
    }

    /// The front Airy's mount (dimos/robot/deeprobotics/m20/constants.py).
    fn front() -> Mount {
        Mount::from_xyz_rpy([0.32028, 0.0, -0.013], [0.0, -PI / 2.0, PI])
    }

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

    fn found(d: &Detection) -> DockPose {
        match d {
            Detection::Found { pose, .. } => *pose,
            Detection::Missing(why) => panic!("not found: {why}"),
        }
    }

    #[test]
    fn mount_puts_the_spin_axis_forward() {
        let m = front();
        let fwd = m.to_base([0.0, 0.0, 1.0]);
        assert!(
            (fwd[0] - 1.32028).abs() < 1e-9 && fwd[1].abs() < 1e-9,
            "{fwd:?}"
        );
        let up = m.to_base([1.0, 0.0, 0.0]);
        assert!((up[2] - (1.0 - 0.013)).abs() < 1e-9, "{up:?}");
    }

    /// Accuracy over a grid of distances, lateral offsets and dock yaws, with 1 cm range noise: printed (run with
    /// --nocapture) and bounded.
    #[test]
    fn accuracy_on_synthetic_sweeps() {
        let cfg = cfg();
        let mut rng = Rng(0x9e3779b97f4a7c15);
        let (mut worst, mut sum, mut n) = ([0.0f64; 3], [0.0f64; 3], 0);
        for &dist in &[0.6, 0.8, 1.0, 1.5, 2.0, 2.5] {
            for &lat in &[-0.3, 0.0, 0.2] {
                for &yaw_deg in &[-15.0, 0.0, 10.0] {
                    let yaw = PI + f64::to_radians(yaw_deg);
                    let posts = dock_posts(dist, lat, yaw, cfg.post_spacing_m, cfg.post_radius_m);
                    // a post more than 55 deg off the lidar's axis is outside the Airy's forward view: skipped
                    let lidar_x = front().translation[0];
                    if posts
                        .iter()
                        .any(|(c, _, _)| c[1].atan2(c[0] - lidar_x).abs() > 55f64.to_radians())
                    {
                        continue;
                    }
                    let pts = sweep(&posts, Some(dist + 0.6), 0.01, &mut rng);
                    let pose = found(&detect(pts, &front(), &cfg));
                    let e = [pose.x - dist, pose.y - lat, wrap(pose.yaw - yaw)];
                    for k in 0..3 {
                        worst[k] = worst[k].max(e[k].abs());
                        sum[k] += e[k] * e[k];
                    }
                    n += 1;
                    println!(
                        "dist {dist:.1} lat {lat:+.1} yaw {yaw_deg:+.0}: error x {:+.4} y {:+.4} m yaw {:+.2} deg",
                        e[0],
                        e[1],
                        e[2].to_degrees()
                    );
                }
            }
        }
        let rms: Vec<f64> = sum.iter().map(|s| (s / n as f64).sqrt()).collect();
        println!(
            "{n} sweeps: rms x {:.4} y {:.4} m yaw {:.2} deg; worst x {:.4} y {:.4} m yaw {:.2} deg",
            rms[0],
            rms[1],
            rms[2].to_degrees(),
            worst[0],
            worst[1],
            worst[2].to_degrees()
        );
        assert!(worst[0] < 0.004 && worst[1] < 0.006, "worst {worst:?}");
        assert!(
            worst[2] < 1f64.to_radians(),
            "worst yaw {:.2} deg",
            worst[2].to_degrees()
        );
    }

    #[test]
    fn a_third_reflector_off_the_spacing_is_ignored() {
        let cfg = cfg();
        let mut rng = Rng(7);
        let mut posts = dock_posts(1.2, 0.05, PI, cfg.post_spacing_m, cfg.post_radius_m);
        posts.push(([1.0, 0.6], cfg.post_radius_m, 250.0));
        let pose = found(&detect(
            sweep(&posts, None, 0.005, &mut rng),
            &front(),
            &cfg,
        ));
        assert!(
            (pose.x - 1.2).abs() < 0.01 && (pose.y - 0.05).abs() < 0.01,
            "{pose:?}"
        );
    }

    #[test]
    fn plain_surfaces_and_one_post_are_not_a_dock() {
        let cfg = cfg();
        let mut rng = Rng(11);
        let dull: Vec<_> = dock_posts(1.0, 0.0, PI, cfg.post_spacing_m, cfg.post_radius_m)
            .into_iter()
            .map(|(c, r, _)| (c, r, 40.0))
            .collect();
        assert!(matches!(
            detect(sweep(&dull, Some(1.6), 0.01, &mut rng), &front(), &cfg),
            Detection::Missing(_)
        ));
        let one = vec![([1.0, 0.25], cfg.post_radius_m, 250.0f32)];
        assert!(matches!(
            detect(sweep(&one, None, 0.01, &mut rng), &front(), &cfg),
            Detection::Missing(_)
        ));
        // posts at the wrong spacing
        let wide = dock_posts(1.0, 0.0, PI, cfg.post_spacing_m + 0.3, cfg.post_radius_m);
        assert!(matches!(
            detect(sweep(&wide, None, 0.01, &mut rng), &front(), &cfg),
            Detection::Missing(_)
        ));
    }

    #[test]
    fn garbage_points_never_panic() {
        let cfg = cfg();
        let pts = vec![
            [f32::NAN, 0.0, 1.0, 255.0],
            [f32::INFINITY, 1.0, 1.0, 255.0],
            [0.0, 0.0, 0.0, 255.0],
            [0.0, 0.0, 1.0, f32::NAN],
        ];
        assert!(matches!(detect(pts, &front(), &cfg), Detection::Missing(_)));
        // too many returns: refused, not clustered
        let many = vec![[0.0f32, 0.0, 1.0, 255.0]; cfg.max_points + 1];
        assert!(matches!(
            detect(many, &front(), &cfg),
            Detection::Missing(_)
        ));
    }
}
