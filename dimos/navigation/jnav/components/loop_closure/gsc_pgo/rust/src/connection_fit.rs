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

//! Stage 3, connection fit quality: how well a discovered connection's two places line up, without the
//! graph. Point-to-point ICP of the source keyframe's cloud onto the target's submap, and its fitness
//! (mean squared nearest-neighbour distance).

use rayon::prelude::*;

use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{transform_cloud, KdTree};

pub struct IcpParams {
    pub max_iterations: usize,
    /// Pairs farther apart than this (m) are not correspondences.
    pub max_correspondence_distance: f64,
}

#[derive(Debug, Clone)]
pub struct IcpResult {
    pub converged: bool,
    /// The transform that moves the source onto the target.
    pub rotation: Mat3,
    pub translation: Vec3,
    /// Mean squared nearest-neighbour distance of every source point after the transform (f64::MAX
    /// when the target is empty).
    pub fitness: f64,
}

/// Align `source` onto `target` (whose k-d tree is `tree`), starting from the guess. Converged when an
/// iteration barely moves it (under 0.26 degrees and 1 mm) or the correspondences' mean squared
/// distance stops changing (1e-6 relative), or after `max_iterations`; not converged when fewer than
/// three correspondences remain.
pub fn icp(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    tree: &KdTree,
    rotation: &Mat3,
    translation: &Vec3,
    params: &IcpParams,
) -> IcpResult {
    let (mut rotation, mut translation) = (*rotation, *translation);
    let mut converged = false;
    let limit = (params.max_correspondence_distance * params.max_correspondence_distance) as f32;
    let mut previous_mse = f64::MAX;
    if !source.is_empty() && !target.is_empty() {
        for iteration in 1..=params.max_iterations {
            let moved = transform_cloud(source, &rotation, &translation);
            let pairs: Vec<([f32; 3], [f32; 3], f32)> = moved
                .par_iter()
                .filter_map(|p| {
                    tree.nearest(p)
                        .filter(|&(_, d)| d <= limit)
                        .map(|(i, d)| (*p, target[i], d))
                })
                .collect();
            if pairs.len() < 3 {
                break;
            }
            let (step_rotation, step_translation) = rigid_fit(&pairs);
            rotation = mat3::mat_mul(&step_rotation, &rotation);
            translation = mat3::add(
                &mat3::mat_vec(&step_rotation, &translation),
                &step_translation,
            );
            let cos = (step_rotation[0][0] + step_rotation[1][1] + step_rotation[2][2] - 1.0) / 2.0;
            let mse = pairs.iter().map(|p| p.2 as f64).sum::<f64>() / pairs.len() as f64;
            if iteration == params.max_iterations
                || (cos >= 0.99999 && mat3::norm(&step_translation) <= 1e-3)
                || (previous_mse - mse).abs() <= 1e-6 * previous_mse
            {
                converged = true;
                break;
            }
            previous_mse = mse;
        }
    }
    let fitness = mean_squared_distance(&transform_cloud(source, &rotation, &translation), tree);
    IcpResult {
        converged,
        rotation,
        translation,
        fitness,
    }
}

/// Mean squared nearest-neighbour distance of `points` in the tree's cloud.
pub fn mean_squared_distance(points: &[[f32; 3]], tree: &KdTree) -> f64 {
    let distances: Vec<f64> = points
        .par_iter()
        .filter_map(|p| tree.nearest(p))
        .map(|(_, d)| d as f64)
        .collect();
    if distances.is_empty() {
        f64::MAX
    } else {
        distances.iter().sum::<f64>() / distances.len() as f64
    }
}

/// The rotation and translation that best map each pair's first point onto its second (Horn's
/// quaternion method: the top eigenvector of a 4x4 built from the cross-covariance).
fn rigid_fit(pairs: &[([f32; 3], [f32; 3], f32)]) -> (Mat3, Vec3) {
    let n = pairs.len() as f64;
    let mean = |pick: fn(&([f32; 3], [f32; 3], f32)) -> [f32; 3]| {
        let mut m = [0.0; 3];
        for p in pairs {
            for (k, v) in pick(p).iter().enumerate() {
                m[k] += *v as f64 / n;
            }
        }
        m
    };
    let (from_mean, to_mean) = (mean(|p| p.0), mean(|p| p.1));
    let mut s = [[0.0; 3]; 3];
    for (a, b, _) in pairs {
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += (a[i] as f64 - from_mean[i]) * (b[j] as f64 - to_mean[j]);
            }
        }
    }
    let n4 = [
        [
            s[0][0] + s[1][1] + s[2][2],
            s[1][2] - s[2][1],
            s[2][0] - s[0][2],
            s[0][1] - s[1][0],
        ],
        [
            s[1][2] - s[2][1],
            s[0][0] - s[1][1] - s[2][2],
            s[0][1] + s[1][0],
            s[2][0] + s[0][2],
        ],
        [
            s[2][0] - s[0][2],
            s[0][1] + s[1][0],
            s[1][1] - s[0][0] - s[2][2],
            s[1][2] + s[2][1],
        ],
        [
            s[0][1] - s[1][0],
            s[2][0] + s[0][2],
            s[1][2] + s[2][1],
            s[2][2] - s[0][0] - s[1][1],
        ],
    ];
    let (_, vectors) = mat3::jacobi_eigen(n4);
    let rotation =
        mat3::mat_from_quat(&[vectors[0][3], vectors[1][3], vectors[2][3], vectors[3][3]]);
    (
        rotation,
        mat3::sub(&to_mean, &mat3::mat_vec(&rotation, &from_mean)),
    )
}

/// Write an ICP's exact input and result to `<dir>/<source>_<target><suffix>.icp`: u32 source count, u32
/// target count, f32 xyz points (source, then target), the rotation (row-major) and translation as f64,
/// f64 fitness, u8 converged.
pub fn dump_icp(
    dir: &str,
    source_id: usize,
    target_id: usize,
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    icp: &IcpResult,
    suffix: &str,
) {
    let mut bytes = Vec::with_capacity(8 + 12 * (source.len() + target.len()) + 105);
    bytes.extend_from_slice(&(source.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&(target.len() as u32).to_le_bytes());
    for v in source.iter().chain(target).flatten() {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    for v in icp
        .rotation
        .iter()
        .flatten()
        .chain(&icp.translation)
        .chain([&icp.fitness])
    {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    bytes.push(icp.converged as u8);
    let path = std::path::Path::new(dir).join(format!("{source_id}_{target_id}{suffix}.icp"));
    if let Err(e) = std::fs::create_dir_all(dir).and_then(|_| std::fs::write(&path, bytes)) {
        eprintln!("loop_icp_dump_dir: could not write {}: {e}", path.display());
    }
}
