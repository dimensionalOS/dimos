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

//! The floor under the robot, read off the local map as a plane.

use std::collections::HashMap;

use crate::module::obstacles::{voxel_key, VOXEL};
use crate::planner::Emb;

/// How far around the robot its own floor is read (m).
const GROUND_RADIUS: f64 = 1.0;

/// How far the floor plane is fit (m). With 8 cm voxels a gentle grade only shows over meters.
const PLANE_RADIUS: f64 = 3.0;

/// Floor columns within `GROUND_RADIUS` needed before the map's floor is trusted over the body height.
const GROUND_MIN_COLUMNS: usize = 20;

/// Columns taller than this many voxels near the floor are walls, not floor.
const FLOOR_COLUMN: usize = 2;

/// Steepest floor the fit reports. A one-layer platform edge in the disc reads as a tilt,
/// and this bounds how far that tilt carries across the map.
const MAX_GRADE: f64 = 0.05;

/// Columns further than this off the plane are not floor. One layer off is a tilted floor,
/// a stack of two is an object.
const TRIM: f64 = 1.25 * VOXEL;

/// Fit and trim rounds.
const ROUNDS: usize = 4;

/// The floor as `z + gx * (x - x0) + gy * (y - y0)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub x0: f64,
    pub y0: f64,
    pub z: f64,
    pub gx: f64,
    pub gy: f64,
}

impl Plane {
    /// A level floor at `z`.
    pub fn flat(z: f64) -> Self {
        Plane {
            x0: 0.0,
            y0: 0.0,
            z,
            gx: 0.0,
            gy: 0.0,
        }
    }

    /// Floor height under `(x, y)`.
    pub fn at(&self, x: f64, y: f64) -> f64 {
        self.z + self.gx * (x - self.x0) + self.gy * (y - self.y0)
    }
}

/// The floor the planner reads heights off, and the map voxels it was fit to, moved onto it.
pub struct Ground {
    pub plane: Plane,
    pub support: Vec<[f32; 3]>,
}

/// The map's floor around `(x, y)`, or a level floor at `body_z` with no support until the map shows one.
pub fn read(points: &[[f32; 3]], x: f64, y: f64, body_z: f64, emb: &Emb) -> Ground {
    fit(points, x, y, body_z, emb).unwrap_or(Ground {
        plane: Plane::flat(body_z),
        support: Vec::new(),
    })
}

/// The plane through the short columns around `(x, y)`, seeded at the robot's own floor and
/// trimmed of what stands on the floor. Near the robot a floor runs from a step under the body
/// to half the base height over it, and that band widens by `MAX_GRADE` with distance.
fn fit(points: &[[f32; 3]], x: f64, y: f64, body_z: f64, emb: &Emb) -> Option<Ground> {
    let r2 = PLANE_RADIUS * PLANE_RADIUS;
    let under = body_z + 0.5 * emb.base_height;
    let (low, high) = (body_z - emb.steppable, under + 2.0 * VOXEL);
    let slack = |dx: f64, dy: f64| MAX_GRADE * dx.hypot(dy);
    let mut columns: HashMap<(i64, i64), Vec<[f32; 3]>> = HashMap::new();
    for p in points {
        let (dx, dy, z) = (p[0] as f64 - x, p[1] as f64 - y, p[2] as f64);
        if dx * dx + dy * dy <= r2 && z >= low - slack(dx, dy) && z < high + slack(dx, dy) {
            let k = voxel_key(p);
            columns.entry((k[0], k[1])).or_default().push(*p);
        }
    }
    // One height per floor column: its voxel's center, or the face two stacked voxels share.
    let mut samples: Vec<[f64; 3]> = Vec::new();
    let mut voxels: Vec<Vec<[f32; 3]>> = Vec::new();
    for mut column in columns.into_values() {
        if column.len() > FLOOR_COLUMN {
            continue;
        }
        column.sort_by(|a, b| a[2].total_cmp(&b[2]));
        let (dx, dy) = (column[0][0] as f64 - x, column[0][1] as f64 - y);
        if column[0][2] as f64 >= under + slack(dx, dy) {
            continue;
        }
        let h = match column.as_slice() {
            [p] => p[2] as f64,
            [a, b] if ((b[2] - a[2]) as f64 - VOXEL).abs() < VOXEL / 4.0 => {
                0.5 * (a[2] as f64 + b[2] as f64)
            }
            _ => continue,
        };
        samples.push([dx, dy, h]);
        voxels.push(column);
    }
    let disc = GROUND_RADIUS * GROUND_RADIUS;
    let mut heights: Vec<f64> = samples
        .iter()
        .filter(|s| s[0] * s[0] + s[1] * s[1] <= disc)
        .map(|s| s[2])
        .collect();
    if heights.len() < GROUND_MIN_COLUMNS {
        return None;
    }
    heights.sort_by(f64::total_cmp);
    let mut plane = [heights[heights.len() / 2], 0.0, 0.0];
    let near = |plane: &[f64; 3]| -> Vec<usize> {
        (0..samples.len())
            .filter(|&i| {
                let s = samples[i];
                (s[2] - plane[0] - plane[1] * s[0] - plane[2] * s[1]).abs() <= TRIM
            })
            .collect()
    };
    for _ in 0..ROUNDS {
        let kept = near(&plane);
        if kept.len() < GROUND_MIN_COLUMNS {
            return None;
        }
        plane = least_squares(kept.iter().map(|&i| samples[i]));
    }
    let [z, gx, gy] = plane;
    let plane = Plane {
        x0: x,
        y0: y,
        z,
        gx,
        gy,
    };
    let support = near(&[z, gx, gy])
        .into_iter()
        .flat_map(|i| voxels[i].iter())
        .map(|p| {
            let on = plane.at(p[0] as f64, p[1] as f64) as f32;
            [p[0], p[1], on]
        })
        .collect();
    Some(Ground { plane, support })
}

/// `[z, gx, gy]` of the least-squares plane through `(dx, dy, h)`, its grade capped at `MAX_GRADE`.
fn least_squares(samples: impl Iterator<Item = [f64; 3]>) -> [f64; 3] {
    let (mut n, mut sx, mut sy, mut sh) = (0.0, 0.0, 0.0, 0.0);
    let (mut sxx, mut syy, mut sxy, mut sxh, mut syh) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for [x, y, h] in samples {
        n += 1.0;
        sx += x;
        sy += y;
        sh += h;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sxh += x * h;
        syh += y * h;
    }
    let (mx, my, mh) = (sx / n, sy / n, sh / n);
    let (cxx, cyy, cxy) = (sxx / n - mx * mx, syy / n - my * my, sxy / n - mx * my);
    let (cxh, cyh) = (sxh / n - mx * mh, syh / n - my * mh);
    let det = cxx * cyy - cxy * cxy;
    // Samples along one line fix no grade across it, so the floor is taken as level.
    let (mut gx, mut gy) = if det > 1e-9 {
        ((cxh * cyy - cyh * cxy) / det, (cyh * cxx - cxh * cxy) / det)
    } else {
        (0.0, 0.0)
    };
    let grade = gx.hypot(gy);
    if grade > MAX_GRADE {
        gx *= MAX_GRADE / grade;
        gy *= MAX_GRADE / grade;
    }
    [mh - gx * mx - gy * my, gx, gy]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::module::obstacles::{field_points, load};

    fn fixture() -> Emb {
        Emb::fixture()
    }

    /// The voxel center the ray tracer emits for layer `k`, in its own f32 arithmetic.
    fn center(k: i64) -> f32 {
        let v = VOXEL as f32;
        k as f32 * v + v * 0.5
    }

    fn grade(plane: &Plane) -> f64 {
        plane.gx.hypot(plane.gy)
    }

    /// Voxel centers over the square of half side `half` (m) around the origin, one per
    /// column at the layers `layers(i, j)` gives.
    fn patch(half: f64, layers: impl Fn(i64, i64) -> Vec<i64>) -> Vec<[f32; 3]> {
        let n = (half / VOXEL).round() as i64;
        let v = VOXEL as f32;
        let mut pts = Vec::new();
        for i in -n..n {
            for j in -n..n {
                for k in layers(i, j) {
                    pts.push([i as f32 * v + v * 0.5, j as f32 * v + v * 0.5, center(k)]);
                }
            }
        }
        pts
    }

    /// Voxel centers of a plane floor with this height at the origin and this grade.
    fn sloped(half: f64, z: f64, gx: f64, gy: f64) -> Vec<[f32; 3]> {
        let v = VOXEL;
        patch(half, |i, j| {
            let (x, y) = ((i as f64 + 0.5) * v, (j as f64 + 0.5) * v);
            vec![((z + gx * x + gy * y) / v).floor() as i64]
        })
    }

    fn fit_at_origin(cloud: &[[f32; 3]], body_z: f64) -> Ground {
        fit(cloud, 0.0, 0.0, body_z, &fixture()).expect("a floor")
    }

    #[test]
    fn a_level_floor_is_read_at_its_voxel_centers_at_any_height() {
        for k in -40..=40 {
            let cloud = patch(1.2, |_, _| vec![k]);
            let g = fit_at_origin(&cloud, k as f64 * VOXEL);
            assert!(
                (g.plane.z - center(k) as f64).abs() < 1e-6,
                "layer {k}: {:?}",
                g.plane
            );
            assert!(grade(&g.plane) < 1e-9, "layer {k}: {:?}", g.plane);
        }
    }

    #[test]
    fn a_floor_at_a_voxel_boundary_is_floor() {
        let model = load("body_band", &fixture(), 5.0).expect("known model");
        // columns holding both layers put the ground on their shared face
        let both = patch(1.2, |_, _| vec![0, 1]);
        let g = fit_at_origin(&both, 0.0);
        assert!((g.plane.z - VOXEL).abs() < 1e-6, "{:?}", g.plane);
        // `share` columns in ten one layer up
        let split = |share: i64| {
            patch(2.0, move |i, j| {
                vec![((i * 7 + j * 3).rem_euclid(10) < share) as i64]
            })
        };
        for cloud in [both, split(3), split(5)] {
            let g = fit_at_origin(&cloud, 0.0);
            assert!(grade(&g.plane) < 0.005, "{:?}", g.plane);
            let (hard, soft) = field_points(model.as_ref(), &cloud, &g.plane);
            assert!(hard.is_empty(), "{} hard", hard.len());
            assert!(soft.is_empty(), "{} soft", soft.len());
        }
    }

    #[test]
    fn a_tilted_floor_is_floor_not_low_objects() {
        let model = load("body_band", &fixture(), 5.0).expect("known model");
        // a degree of tilt, a 3 percent walkway and the steepest floor the fit reports
        for g_in in [0.0175, 0.03, MAX_GRADE] {
            let (gx, gy) = (g_in * 0.6, g_in * 0.8);
            let cloud = sloped(3.0, 0.37, gx, gy);
            let g = fit_at_origin(&cloud, 0.37);
            assert!(
                (g.plane.gx - gx).abs() < 0.004 && (g.plane.gy - gy).abs() < 0.004,
                "grade {g_in}: {:?}",
                g.plane
            );
            assert!((g.plane.z - 0.37).abs() < VOXEL / 4.0, "{:?}", g.plane);
            let (hard, soft) = field_points(model.as_ref(), &cloud, &g.plane);
            assert!(hard.is_empty(), "grade {g_in}: {} hard", hard.len());
            assert!(soft.is_empty(), "grade {g_in}: {} soft", soft.len());
        }
    }

    #[test]
    fn the_fit_never_reports_a_grade_past_the_cap() {
        let ramp = sloped(1.2, 0.0, 0.15, 0.0);
        assert!(grade(&fit_at_origin(&ramp, 0.0).plane) <= MAX_GRADE + 1e-12);
        // a one-layer platform edge under the robot reads as a tilt, no steeper than the cap
        let edge = patch(1.2, |i, _| vec![(i >= 0) as i64]);
        let g = fit_at_origin(&edge, 0.0);
        assert!(grade(&g.plane) <= MAX_GRADE + 1e-12, "{:?}", g.plane);
        assert!(grade(&g.plane) > 0.5 * MAX_GRADE, "{:?}", g.plane);
    }

    #[test]
    fn the_ground_is_the_floor_not_the_wall_or_the_block_on_it() {
        let wall = |j: i64| j == 9;
        let block = |i: i64, j: i64| (4..6).contains(&i) && (-2..0).contains(&j);
        let cloud = patch(1.2, |i, j| {
            if wall(j) {
                vec![0, 1, 2, 3, 4]
            } else if block(i, j) {
                vec![1, 2]
            } else {
                vec![0]
            }
        });
        let g = read(&cloud, 0.0, 0.0, 0.0, &fixture());
        assert!((g.plane.z - center(0) as f64).abs() < 1e-6, "{:?}", g.plane);
        assert!(grade(&g.plane) < 1e-9, "{:?}", g.plane);
        for p in &g.support {
            let on = g.plane.at(p[0] as f64, p[1] as f64);
            assert!((p[2] as f64 - on).abs() < 1e-6, "{p:?} is off the plane");
            assert!(
                !(p[1] > 0.72 && p[1] < 0.8),
                "wall voxel in the support: {p:?}"
            );
            assert!(
                !(p[0] > 0.32 && p[0] < 0.48 && p[1] < 0.0 && p[1] > -0.16),
                "block: {p:?}"
            );
        }
        // and every other floor column is in it
        let floor = cloud
            .iter()
            .filter(|p| p[2] < 0.05 && !(p[1] > 0.72 && p[1] < 0.8))
            .count();
        assert_eq!(g.support.len(), floor);
    }

    #[test]
    fn a_floor_more_than_a_step_below_the_body_is_never_the_ground() {
        // a dock edge just ahead: the lower floor fills most of the disc
        let cloud = patch(1.2, |i, _| if i < -3 { vec![0] } else { vec![-10] });
        let g = fit_at_origin(&cloud, 0.0);
        assert!((g.plane.z - center(0) as f64).abs() < 0.02, "{:?}", g.plane);
        assert!(
            g.support.iter().all(|p| p[2] > -0.2),
            "the lower floor is support"
        );
    }

    #[test]
    fn too_little_floor_leaves_the_ground_to_the_body() {
        let sparse: Vec<[f32; 3]> = patch(0.16, |_, _| vec![0]);
        assert!(sparse.len() < GROUND_MIN_COLUMNS);
        // nor does floor only past the robot's own disc, which may be another level
        let far: Vec<[f32; 3]> = (0..60)
            .map(|i| [1.2, -1.2 + 0.04 * i as f32, center(0)])
            .collect();
        for cloud in [sparse, far] {
            let g = read(&cloud, 0.0, 0.0, 0.11, &fixture());
            assert_eq!(g.plane, Plane::flat(0.11));
            assert!(g.support.is_empty());
        }
    }

    #[test]
    fn the_map_floor_wins_over_the_body_height() {
        // the body estimate sits 8 cm over the floor the map shows
        let cloud = patch(1.2, |_, _| vec![-1]);
        let g = read(&cloud, 0.0, 0.0, 0.08, &fixture());
        assert!(
            (g.plane.z - center(-1) as f64).abs() < 1e-6,
            "{:?}",
            g.plane
        );
        assert!(g.support.iter().all(|p| (p[2] - center(-1)).abs() < 1e-6));
    }
}
