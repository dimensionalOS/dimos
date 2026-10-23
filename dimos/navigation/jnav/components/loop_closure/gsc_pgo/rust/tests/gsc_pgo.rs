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

//! Unit tests for keyframe storage and the point-cloud machinery (voxel-grid centroids, ICP recovering
//! a known transform, normal-scatter degeneracy).

// Index loops mirror the reference matrix asserts (r[i][j] / t[i]); keep them.
#![allow(clippy::needless_range_loop)]

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::pointcloud::{
    cloud_degeneracy, icp_point_to_point, voxel_downsample, IcpParams,
};

fn make_pose(rotation: Mat3, translation: Vec3, time: f64) -> PoseWithTime {
    let mut pose = PoseWithTime::new(rotation, translation);
    let sec = time as i32;
    pose.set_time(sec, ((time - sec as f64) * 1e9) as u32);
    pose
}

#[test]
fn keyframe_voxel_size_downsamples_the_stored_cloud_only_when_set() {
    // 1000 points packed in a 10 cm cube: one 0.5 m voxel
    let dense: Vec<[f32; 3]> = (0..1000)
        .map(|i| {
            [
                (i % 10) as f32 * 0.01,
                (i / 10 % 10) as f32 * 0.01,
                (i / 100) as f32 * 0.01,
            ]
        })
        .collect();
    let stored = |keyframe_voxel_size: f64| {
        let mut pgo = GscPgo::new(Config {
            keyframe_voxel_size,
            ..Config::default()
        });
        let cloud_with_pose = CloudWithPose {
            cloud: Some(std::sync::Arc::new(dense.clone())),
            pose: make_pose(mat3::identity(), [0.0, 0.0, 0.0], 0.0),
            frame_id: "odom".to_string(),
        };
        assert!(pgo.add_key_pose(&cloud_with_pose));
        pgo.key_poses()[0].body_cloud.as_ref().unwrap().len()
    };
    assert_eq!(stored(0.0), 1000);
    assert_eq!(stored(0.5), 1);
}

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
fn icp_matches_pcl_reference_bits() {
    // LCG identical to the C++ reference harness.
    let mut state = 42u64;
    let mut next = move || {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (((state >> 40) as f32) / (1u32 << 24) as f32, state)
    };
    // cosf/sinf(0.12f) and cosf/sinf(0.05f) bits from the C++ run, to keep
    // the test independent of libm.
    let cy = f32::from_bits(0x3f7e28b5);
    let sy = f32::from_bits(0x3df52bac);
    let mut source: Vec<[f32; 3]> = Vec::new();
    let mut target: Vec<[f32; 3]> = Vec::new();
    for _ in 0..1200 {
        let (u, _) = next();
        let (v, _) = next();
        let (w, s) = next();
        let (u, v) = (u * 5.0f32, v * 5.0f32);
        let p = match (s >> 7) % 3 {
            0 => [u, v, 0.05f32 * w],
            1 => [u, 0.05f32 * w, v],
            _ => [0.05f32 * w, u, v],
        };
        source.push(p);
        let (n0, _) = next();
        let (n1, _) = next();
        let (n2, _) = next();
        target.push([
            cy * p[0] - sy * p[1] + 0.35f32 + 0.002f32 * n0,
            sy * p[0] + cy * p[1] - 0.25f32 + 0.002f32 * n1,
            p[2] + 0.15f32 + 0.002f32 * n2,
        ]);
    }

    let check = |result: &dimos_gsc_pgo::pointcloud::IcpResult,
                 expected: &[[u32; 4]; 3],
                 expected_fitness: u64,
                 label: &str| {
        assert!(result.converged, "{label}: PCL converged here");
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(
                    (result.rotation[i][j] as f32).to_bits(),
                    expected[i][j],
                    "{label}: r({i},{j})"
                );
            }
            assert_eq!(
                (result.translation[i] as f32).to_bits(),
                expected[i][3],
                "{label}: t({i})"
            );
        }
        assert_eq!(
            result.fitness.to_bits(),
            expected_fitness,
            "{label}: fitness {:e}",
            result.fitness
        );
    };

    // Run 1: non-identity f32 guess (cos/sin(0.05f), small translation).
    let gc = f32::from_bits(0x3f7fae19);
    let gs = f32::from_bits(0x3d4cb6f5);
    let guess_rotation: Mat3 = [
        [f64::from(gc), f64::from(-gs), 0.0],
        [f64::from(gs), f64::from(gc), 0.0],
        [0.0, 0.0, 1.0],
    ];
    let guess_translation: Vec3 = [f64::from(0.1f32), f64::from(-0.05f32), f64::from(0.02f32)];
    let result = icp_point_to_point(
        &source,
        &target,
        &guess_rotation,
        &guess_translation,
        &IcpParams::default(),
    );
    check(
        &result,
        &[
            [0x3f7e28c7, 0xbdf529f2, 0xb633fd02, 0x3eb3b4bf],
            [0x3df529e6, 0x3f7e28c2, 0x3728443e, 0xbe7eff66],
            [0x35a19952, 0xb72918bc, 0x3f800001, 0x3e1aa158],
        ],
        0x3eb07798709051ec,
        "guess run",
    );

    // Run 2: identity guess (skips the initial cloud transform, like PCL).
    let result = icp_point_to_point(
        &source,
        &target,
        &mat3::identity(),
        &[0.0; 3],
        &IcpParams::default(),
    );
    check(
        &result,
        &[
            [0x3f7e28c8, 0xbdf52a00, 0xb6267b50, 0x3eb3b4e7],
            [0x3df529e7, 0x3f7e28cc, 0x372820a2, 0xbe7effd2],
            [0x35c8ce00, 0xb72ca928, 0x3f800008, 0x3e1aa108],
        ],
        0x3eb0778af6af69d0,
        "identity run",
    );
}

/// ICP recovers a known rigid transform between two synthetic clouds.
#[test]
fn icp_recovers_known_transform() {
    let source = scattered_corner_cloud();
    let rotation_true = mat3::rot_z(0.12);
    let translation_true = [0.35, -0.25, 0.15];
    let target =
        dimos_gsc_pgo::pointcloud::transform_cloud(&source, &rotation_true, &translation_true);

    let result = icp_point_to_point(
        &source,
        &target,
        &mat3::identity(),
        &[0.0; 3],
        &IcpParams::default(),
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
    let (e_min, e_mid) = cloud_degeneracy(&plane);
    assert!(
        (0.0..0.01).contains(&e_min),
        "flat plane is degenerate: e_min {e_min}"
    );
    assert!(e_mid >= e_min);

    let corner = synthetic_corner_cloud();
    let (e_min, _) = cloud_degeneracy(&corner);
    assert!(
        e_min > 0.15,
        "corner scene constrains all axes: e_min {e_min}"
    );

    // Too few points -> the (-1, -1) sentinel.
    let (e_min, e_mid) = cloud_degeneracy(&plane[..10]);
    assert_eq!((e_min, e_mid), (-1.0, -1.0));
}
