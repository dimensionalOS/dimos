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

//! Absolute position priors for the pose graph: positions of a sensor in one or more fixed world frames (GPS, motion capture, a total station), each its own map -> frame alignment, folded into `big_picture_fit`'s solve.

use crate::big_picture_fit::{
    solve_core, Edge, LandmarkSighting, Solution, SolveConfig, Start, Unary,
};
use crate::mat3::{self, Mat3, Vec3};

/// A sensor's position (m) in world frame `frame` (one id per coordinate system), mounted at `lever` on `keyframe`; cov m².
#[derive(Clone, Copy, Debug)]
pub struct PositionPrior {
    pub keyframe: usize,
    pub frame: u32,
    pub lever: Vec3,
    pub world: Vec3,
    pub covariance: Mat3,
}

/// `solve` plus GNC-weighted position priors; other frames align to the most-observed one's solve (two-stage).
pub fn solve_with_priors(
    start: Start,
    odometry: &[Edge],
    closures: &[Edge],
    priors: &[PositionPrior],
    sightings: &[LandmarkSighting],
    config: &SolveConfig,
) -> Solution {
    let Start {
        rotations,
        positions,
        free,
    } = start;
    let mut solution = solve_core(start, odometry, closures, &[], sightings, config);
    let mut frames: Vec<u32> = priors.iter().map(|p| p.frame).collect();
    frames.sort_unstable();
    frames.dedup();
    let of = |frame: u32| -> Vec<usize> {
        (0..priors.len())
            .filter(|&i| priors[i].frame == frame)
            .collect()
    };
    // the sensors' positions in a solution, so a mount offset from the body origin is fitted where it was measured
    let fit = |solution: &Solution, ids: &[usize]| {
        let chosen: Vec<PositionPrior> = ids.iter().map(|&i| priors[i]).collect();
        let sensors: Vec<Vec3> = chosen
            .iter()
            .map(|p| {
                mat3::add(
                    &solution.positions[p.keyframe],
                    &mat3::mat_vec(&solution.rotations[p.keyframe], &p.lever),
                )
            })
            .collect();
        align_priors(&sensors, &chosen)
    };
    // map -> frame, for every frame its priors make observable
    let first: Vec<(u32, (f64, Vec3))> = frames
        .iter()
        .filter_map(|&f| Some((f, fit(&solution, &of(f))?)))
        .collect();
    let Some(&(primary, (yaw, translation))) = first.iter().max_by_key(|(f, _)| of(*f).len())
    else {
        solution.prior_weights = vec![0.0; priors.len()];
        return solution;
    };
    let turn = mat3::rot_z(yaw);
    let world_rotations: Vec<Mat3> = solution
        .rotations
        .iter()
        .map(|r| mat3::mat_mul(&turn, r))
        .collect();
    let world_positions: Vec<Vec3> = solution
        .positions
        .iter()
        .map(|p| mat3::add(&mat3::mat_vec(&turn, p), &translation))
        .collect();
    // primary -> frame for the others: compose the first fits
    let mut relative: Vec<(u32, (f64, Vec3))> = first
        .iter()
        .filter(|(f, _)| *f != primary)
        .map(|&(f, (y, t))| {
            let r = y - yaw;
            (
                f,
                (
                    r,
                    mat3::sub(&t, &mat3::mat_vec(&mat3::rot_z(r), &translation)),
                ),
            )
        })
        .collect();
    let all_free = vec![true; free.len()];
    // the priors of the frames in `frames` as unary factors in the primary frame (others through their alignment)
    let unary_of = |frames: &dyn Fn(u32) -> bool,
                    relative: &[(u32, (f64, Vec3))]|
     -> (Vec<usize>, Vec<Unary>) {
        let mut used = Vec::new();
        let mut unary = Vec::new();
        for (i, p) in priors.iter().enumerate() {
            if !frames(p.frame) {
                continue;
            }
            let (position, covariance) = if p.frame == primary {
                (p.world, p.covariance)
            } else if let Some((_, (r, t))) = relative.iter().find(|(f, _)| *f == p.frame) {
                let back = mat3::rot_z(-r);
                (
                    mat3::mat_vec(&back, &mat3::sub(&p.world, t)),
                    mat3::mat_mul(
                        &back,
                        &mat3::mat_mul(&p.covariance, &mat3::transpose(&back)),
                    ),
                )
            } else {
                continue;
            };
            used.push(i);
            unary.push(Unary {
                keyframe: p.keyframe,
                lever: p.lever,
                position,
                information: if config.planar {
                    horizontal_information(&covariance)
                } else {
                    mat3::inverse(&covariance).unwrap_or([[0.0; 3]; 3])
                },
            });
        }
        (used, unary)
    };
    let mut weights = vec![0.0; priors.len()];
    // stage 1: primary alone, then refit the other alignments to it so their tighter priors can't hold a wrong first guess
    let (used, unary) = unary_of(&|f| f == primary, &relative);
    let primary_only = solve_core(
        Start {
            rotations: &world_rotations,
            positions: &world_positions,
            free: &all_free,
        },
        odometry,
        closures,
        &unary,
        sightings,
        config,
    );
    for (&i, &w) in used.iter().zip(&primary_only.prior_weights) {
        weights[i] = w;
    }
    for (f, alignment) in relative.iter_mut() {
        let ids = of(*f);
        if let Some(refit) = fit(&primary_only, &ids) {
            *alignment = refit;
        }
    }
    // stage 2: every frame together
    let (used, unary) = unary_of(&|_| true, &relative);
    let world = solve_core(
        Start {
            rotations: &primary_only.rotations,
            positions: &primary_only.positions,
            free: &all_free,
        },
        odometry,
        closures,
        &unary,
        sightings,
        config,
    );
    for (&i, &w) in used.iter().zip(&world.prior_weights) {
        weights[i] = w;
    }
    // map -> primary: keyframe 0's map pose onto its solved pose, kept a pure yaw (the frames are gravity-aligned)
    let turned = mat3::mat_mul(&world.rotations[0], &mat3::transpose(&rotations[0]));
    let align_rotation = mat3::rot_z(turned[1][0].atan2(turned[0][0]));
    let align_translation = mat3::sub(
        &world.positions[0],
        &mat3::mat_vec(&align_rotation, &positions[0]),
    );
    let back = mat3::transpose(&align_rotation);
    let mut solution = world;
    solution.rotations = solution
        .rotations
        .iter()
        .map(|r| mat3::mat_mul(&back, r))
        .collect();
    solution.positions = solution
        .positions
        .iter()
        .map(|p| mat3::mat_vec(&back, &mat3::sub(p, &align_translation)))
        .collect();
    solution.landmarks = solution
        .landmarks
        .iter()
        .map(|p| mat3::mat_vec(&back, &mat3::sub(p, &align_translation)))
        .collect();
    solution.landmark_rotations = solution
        .landmark_rotations
        .iter()
        .map(|r| mat3::mat_mul(&back, r))
        .collect();
    let primary_yaw = align_rotation[1][0].atan2(align_rotation[0][0]);
    solution.world_alignment = std::iter::once((primary, primary_yaw, align_translation))
        .chain(relative.iter().map(|&(f, (r, t))| {
            // map -> frame = (primary -> frame) after (map -> primary)
            (
                f,
                primary_yaw + r,
                mat3::add(&mat3::mat_vec(&mat3::rot_z(r), &align_translation), &t),
            )
        }))
        .collect();
    solution.prior_weights = weights;
    solution
}

/// A planar solve's prior information: the inverse of the x-y block, nothing on z (the map is level, its height free).
fn horizontal_information(covariance: &Mat3) -> Mat3 {
    let [[a, b, _], [c, d, _], _] = *covariance;
    let determinant = a * d - b * c;
    // the negated comparison rejects NaN too
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(determinant > 0.0) {
        return [[0.0; 3]; 3];
    }
    [
        [d / determinant, -b / determinant, 0.0],
        [-c / determinant, a / determinant, 0.0],
        [0.0, 0.0, 0.0],
    ]
}

/// Weighted yaw + translation putting `points` on the priors' world positions; None while spread sd <= 2x noise sd.
fn align_priors(points: &[Vec3], priors: &[PositionPrior]) -> Option<(f64, Vec3)> {
    let used: Vec<(Vec3, &PositionPrior, f64)> = priors
        .iter()
        .zip(points)
        .map(|(p, &point)| {
            (
                point,
                p,
                2.0 / (p.covariance[0][0] + p.covariance[1][1]).max(f64::EPSILON),
            )
        })
        .collect();
    if used.len() < 2 {
        return None;
    }
    let total: f64 = used.iter().map(|(_, _, w)| w).sum();
    let mean = |f: &dyn Fn(&Vec3, &PositionPrior) -> Vec3| {
        used.iter().fold([0.0; 3], |m, (point, p, w)| {
            let v = f(point, p);
            [
                m[0] + w * v[0] / total,
                m[1] + w * v[1] / total,
                m[2] + w * v[2] / total,
            ]
        })
    };
    let map_mean = mean(&|point, _| *point);
    let world_mean = mean(&|_, p| p.world);
    let (mut sxx, mut sxy, mut spread, mut noise) = (0.0, 0.0, 0.0, 0.0);
    for (point, p, w) in &used {
        let m = mat3::sub(point, &map_mean);
        let g = mat3::sub(&p.world, &world_mean);
        sxx += w * (m[0] * g[0] + m[1] * g[1]);
        sxy += w * (m[0] * g[1] - m[1] * g[0]);
        spread += w * (g[0] * g[0] + g[1] * g[1]) / total;
        noise += w * (p.covariance[0][0] + p.covariance[1][1]) / total;
    }
    if spread <= 4.0 * noise {
        return None;
    }
    let yaw = sxy.atan2(sxx);
    let rotated = mat3::mat_vec(&mat3::rot_z(yaw), &map_mean);
    Some((yaw, mat3::sub(&world_mean, &rotated)))
}
