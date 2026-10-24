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

//! Unit tests for the voxel grid, connection fit (ICP recovers a known transform) and
//! uniqueness (normal-scatter degeneracy).

// Index loops mirror the reference matrix asserts (r[i][j] / t[i]); keep them.
#![allow(clippy::needless_range_loop)]

use dimos_gsc_pgo::connection_fit::{icp, IcpParams};
use dimos_gsc_pgo::mat3;
use dimos_gsc_pgo::pointcloud::{voxel_downsample, KdTree};
use dimos_gsc_pgo::uniqueness::degeneracy;

#[test]
fn voxel_downsample_emits_per_voxel_centroids() {
    let cloud = vec![
        // Voxel (0,0,0) for leaf 1.0: centroid (0.2, 0.3, 0.4).
        [0.1, 0.1, 0.1],
        [0.3, 0.5, 0.7],
        // Voxel (1,0,0): single point survives untouched.
        [1.5, 0.25, 0.75],
        // Voxel (-1,0,0): floor semantics, NOT truncation toward zero.
        [-0.5, 0.5, 0.5],
    ];
    let out = voxel_downsample(&cloud, 1.0);
    assert_eq!(out.len(), 3);
    let find = |x: f32| {
        out.iter()
            .find(|p| (p[0] - x).abs() < 1e-6)
            .copied()
            .unwrap()
    };
    let c0 = find(0.2);
    assert!(
        (c0[1] - 0.3).abs() < 1e-6 && (c0[2] - 0.4).abs() < 1e-6,
        "centroid, not voxel center: {c0:?}"
    );
    let c1 = find(1.5);
    assert!((c1[1] - 0.25).abs() < 1e-6 && (c1[2] - 0.75).abs() < 1e-6);
    let c2 = find(-0.5);
    assert!(
        (c2[1] - 0.5).abs() < 1e-6,
        "negative coords bin by floor: {c2:?}"
    );
    // resolution <= 0 passes through.
    assert_eq!(voxel_downsample(&cloud, 0.0).len(), cloud.len());
}

/// A synthetic structured cloud: three orthogonal wall grids (well
/// conditioned for point-to-point ICP).
fn synthetic_corner_cloud() -> Vec<[f32; 3]> {
    let mut cloud = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            let a = i as f32 * 0.25;
            let b = j as f32 * 0.25;
            cloud.push([a, b, 0.0]); // floor z=0
            cloud.push([a, 0.0, b]); // wall y=0
            cloud.push([0.0, a, b]); // wall x=0
        }
    }
    cloud
}

/// An irregular (deterministic-pseudo-random) three-wall cloud. ICP needs
/// non-repeating structure: on a regular lattice, point-to-point NN
/// correspondences alias to the wrong lattice cell and ICP (PCL's exactly
/// as much as ours) locks into a local minimum.
fn scattered_corner_cloud() -> Vec<[f32; 3]> {
    let mut state = 42u64;
    let mut next = move || {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (((state >> 33) as f32) / (u32::MAX >> 1) as f32, state)
    };
    let mut cloud = Vec::new();
    for _ in 0..1200 {
        let (u, _) = next();
        let (v, _) = next();
        let (w, s) = next();
        let (u, v) = (u * 5.0, v * 5.0);
        match (s >> 7) % 3 {
            0 => cloud.push([u, v, 0.05 * w]), // floor
            1 => cloud.push([u, 0.05 * w, v]), // wall y=0
            _ => cloud.push([0.05 * w, u, v]), // wall x=0
        }
    }
    cloud
}

/// Bit-exact golden test against PCL 1.15 `IterativeClosestPoint<PointXYZI,
/// PointXYZI>` (Eigen 3.4.1, g++ -O2, x86-64 baseline / SSE2): identical
/// clouds fed to the real PCL ICP produced these final-transform f32 bits
/// and fitness f64 bits. The pipeline is IEEE-exact ops only, so the bits
/// are portable — except the covariance GEMM panel size, which follows
/// Eigen's L1-cache heuristic; at this cloud size (~1200 correspondences)
/// every L1 size >= 16 KiB yields the same panel split.
#[test]
fn icp_recovers_known_transform() {
    let source = scattered_corner_cloud();
    let rotation_true = mat3::rot_z(0.12);
    let translation_true = [0.35, -0.25, 0.15];
    let target =
        dimos_gsc_pgo::pointcloud::transform_cloud(&source, &rotation_true, &translation_true);

    let result = icp(
        &source,
        &target,
        &KdTree::build(&target),
        &mat3::identity(),
        &[0.0; 3],
        &IcpParams {
            max_iterations: 50,
            max_correspondence_distance: 10.0,
        },
    );
    assert!(result.converged, "ICP must converge on a clean pair");
    assert!(
        result.fitness < 1e-4,
        "near-exact alignment: fitness {}",
        result.fitness
    );
    let rot_err = mat3::angular_distance(&result.rotation, &rotation_true);
    assert!(rot_err < 1e-3, "rotation recovered: err {rot_err} rad");
    let translation_err = mat3::norm(&mat3::sub(&result.translation, &translation_true));
    assert!(
        translation_err < 1e-2,
        "translation recovered: err {translation_err} m"
    );
}

/// Degeneracy: a flat plane has all normals vertical -> smallest normalized
/// normal-scatter eigenvalue ~ 0; a three-wall corner constrains all axes.
#[test]
fn cloud_degeneracy_separates_planes_from_corners() {
    let mut plane = Vec::new();
    for i in 0..30 {
        for j in 0..30 {
            plane.push([i as f32 * 0.2, j as f32 * 0.2, 0.0]);
        }
    }
    let e_min = degeneracy(&plane);
    assert!(
        (0.0..0.01).contains(&e_min),
        "flat plane is degenerate: e_min {e_min}"
    );

    let corner = synthetic_corner_cloud();
    let e_min = degeneracy(&corner);
    assert!(
        e_min > 0.15,
        "corner scene constrains all axes: e_min {e_min}"
    );

    // too few points -> the -1 sentinel
    assert_eq!(degeneracy(&plane[..10]), -1.0);
}
