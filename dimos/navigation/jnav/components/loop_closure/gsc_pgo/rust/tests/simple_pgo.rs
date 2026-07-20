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

//! Port of gsc_pgo's pgo_location_constraint_test.cpp, scenario for
//! scenario — every CHECK becomes an assert — plus unit tests for the
//! point-cloud machinery (voxel-grid centroids, ICP recovering a known
//! transform, normal-scatter degeneracy).

// Index loops mirror the reference matrix asserts (r[i][j] / t[i]); keep them.
#![allow(clippy::needless_range_loop)]

use std::f64::consts::PI;

use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::pointcloud::{
    cloud_degeneracy, icp_point_to_point, voxel_downsample, IcpParams,
};
use dimos_gsc_pgo::simple_pgo::{
    CloudWithPose, Config, LocationConstraintObs, PoseWithTime, SimplePgo,
};

fn yaw(a: f64) -> Mat3 {
    mat3::rot_z(a)
}

fn make_pose(r: Mat3, t: Vec3, time: f64) -> PoseWithTime {
    let mut pose = PoseWithTime::new(r, t);
    let sec = time as i32;
    pose.set_time(sec, ((time - sec as f64) * 1e9) as u32);
    pose
}

/// Feed one scan keyframe (identity orientation unless r given) at world
/// (x,y,z), then run a smoothAndUpdate cycle. Clouds are None (scan-context
/// disabled), so only the location constraints close loops.
fn step(pgo: &mut SimplePgo, t: f64, xyz: Vec3, r: Mat3) {
    let cwp = CloudWithPose {
        cloud: None,
        pose: make_pose(r, xyz, t),
        frame_id: "odom".to_string(),
    };
    if pgo.add_key_pose(&cwp) {
        pgo.search_for_loop_pairs();
        pgo.smooth_and_update();
    }
}

/// A diagonal 6x6 covariance (GTSAM tangent order [rot(3), trans(3)]).
fn make_cov(rot_var: f64, trans_var: f64) -> [f64; 36] {
    let mut cov = [0.0; 36];
    for i in 0..3 {
        cov[i * 6 + i] = rot_var;
    }
    for i in 3..6 {
        cov[i * 6 + i] = trans_var;
    }
    cov
}

/// Ingest a location constraint: (odom_pos, odom_r) is the interpolated
/// odometry pose at the constraint's time.
fn sight(pgo: &mut SimplePgo, odom_pos: Vec3, odom_r: Mat3, ts: f64, c: &LocationConstraintObs) {
    let pose = make_pose(odom_r, odom_pos, ts);
    if pgo.add_location_constraint(&pose, "base_link", c) {
        pgo.smooth_and_update();
    }
}

/// A drifted loop: the robot drives a circle of `radius` over `k` steps and
/// physically returns to the start, but the odometry carries a constant
/// per-step yaw bias, so the fed trajectory spirals open.
#[derive(Clone, Copy)]
struct LoopFrame {
    true_pos: Vec3,
    true_yaw: f64,
    odom_pos: Vec3,
    odom_yaw: f64,
}

fn make_drifted_loop(k: usize, radius: f64, yaw_bias: f64) -> Vec<LoopFrame> {
    let true_pose = |i: usize| -> (Vec3, f64) {
        let a = 2.0 * PI * i as f64 / k as f64;
        ([radius * a.cos(), radius * a.sin(), 0.0], a + PI / 2.0) // tangent heading
    };
    let (p0, th0) = true_pose(0);
    let mut frames = vec![
        LoopFrame {
            true_pos: p0,
            true_yaw: th0,
            odom_pos: p0,
            odom_yaw: th0
        };
        k + 1
    ];
    for i in 1..=k {
        let (pi, thi) = true_pose(i);
        let (pm, thm) = true_pose(i - 1);
        // Relative motion in the body frame.
        let dp_body = mat3::mat_vec(&mat3::transpose(&yaw(thm)), &mat3::sub(&pi, &pm));
        let dth = thi - thm;
        let odom_yaw = frames[i - 1].odom_yaw + dth + yaw_bias; // biased turn
        let odom_pos = mat3::add(
            &frames[i - 1].odom_pos,
            &mat3::mat_vec(&yaw(frames[i - 1].odom_yaw), &dp_body),
        );
        frames[i] = LoopFrame {
            true_pos: pi,
            true_yaw: thi,
            odom_pos,
            odom_yaw,
        };
    }
    frames
}

/// A fixed world location, observed from a TRUE pose (~2 m beyond the start
/// point (30,0)).
const LOC_WORLD: Vec3 = [32.0, 0.0, 0.0];

fn constraint_at(to_id: &str, instance: &str, fr: &LoopFrame, ts: f64) -> LocationConstraintObs {
    LocationConstraintObs {
        to_id: to_id.to_string(),
        constraint_instance_id: instance.to_string(),
        r_body_loc: mat3::identity(),
        t_body_loc: mat3::mat_vec(
            &mat3::transpose(&yaw(fr.true_yaw)),
            &mat3::sub(&LOC_WORLD, &fr.true_pos),
        ),
        covariance: make_cov(0.0025, 0.0025),
        ts,
    }
}

fn pitch_deg(r: &Mat3) -> f64 {
    (-r[2][0]).asin() * 180.0 / PI
}

fn test_config() -> Config {
    Config {
        use_location_constraints: true,
        key_pose_delta_trans: 0.5,
        ..Config::default()
    }
}

// --- Test 1: two sightings of one location close a drifted loop, pulling
// the (badly drifted) end back near the true end (the start). Distinct
// instance ids so neither sighting revises away the other.
#[test]
fn location_closure_pulls_the_drifted_end_back() {
    let mut pgo = SimplePgo::new(test_config());

    let k = 240;
    let frames = make_drifted_loop(k, 30.0, 0.0040);
    for (i, fr) in frames.iter().enumerate() {
        step(&mut pgo, i as f64 * 0.1, fr.odom_pos, yaw(fr.odom_yaw));
        if i == 0 || i == k {
            let c = constraint_at("C", &format!("C{i}"), fr, i as f64 * 0.1);
            sight(&mut pgo, fr.odom_pos, yaw(fr.odom_yaw), i as f64 * 0.1, &c);
        }
    }
    let kps = pgo.key_poses();
    let fed_err = mat3::norm(&mat3::sub(&frames[k].odom_pos, &frames[k].true_pos));
    let cor_err = mat3::norm(&mat3::sub(
        &kps.last().unwrap().t_global,
        &frames[k].true_pos,
    ));
    eprintln!(
        "   true end=({:.1},{:.1}) fed end=({:.1},{:.1}) err {:.1} m -> corrected=({:.1},{:.1}) err {:.1} m",
        frames[k].true_pos[0],
        frames[k].true_pos[1],
        frames[k].odom_pos[0],
        frames[k].odom_pos[1],
        fed_err,
        kps.last().unwrap().t_global[0],
        kps.last().unwrap().t_global[1],
        cor_err
    );
    assert!(
        fed_err > 20.0,
        "odometry drift opens the loop by >20 m: {fed_err}"
    );
    assert!(
        cor_err < fed_err * 0.5,
        "location closure pulls the drifted end back to the true end by >50%: fed {fed_err}, corrected {cor_err}"
    );
}

// --- Test 2: the first-keyframe anchor prior preserves the initial roll/pitch.
// Start with a 30 deg pitch; after constraint-driven optimization, keyframe 0's
// pitch is kept.
#[test]
fn anchor_prior_keeps_kf0_pitch() {
    let mut pgo = SimplePgo::new(test_config());

    let pitch0 = 30.0 * PI / 180.0;
    let r0 = mat3::rot_y(pitch0); // 30 deg pitch about y

    let fr0 = LoopFrame {
        true_pos: [0.0, 0.0, 0.0],
        true_yaw: 0.0,
        odom_pos: [0.0, 0.0, 0.0],
        odom_yaw: 0.0,
    };
    let fr4 = LoopFrame {
        true_pos: [5.0, 5.0, 0.0],
        true_yaw: 0.0,
        odom_pos: [5.0, 5.0, 0.0],
        odom_yaw: 0.0,
    };
    step(&mut pgo, 0.0, [0.0, 0.0, 0.0], r0);
    sight(
        &mut pgo,
        [0.0, 0.0, 0.0],
        r0,
        0.0,
        &constraint_at("G", "G0", &fr0, 0.0),
    );
    step(&mut pgo, 1.0, [10.0, 0.0, 0.0], mat3::identity());
    step(&mut pgo, 2.0, [10.0, 12.0, 0.0], mat3::identity());
    step(&mut pgo, 3.0, [0.0, 8.0, 0.0], mat3::identity());
    step(&mut pgo, 4.0, [5.0, 5.0, 0.0], r0);
    sight(
        &mut pgo,
        [5.0, 5.0, 0.0],
        r0,
        4.0,
        &constraint_at("G", "G4", &fr4, 4.0),
    );

    let p_in = 30.0;
    let p_out = pitch_deg(&pgo.key_poses().first().unwrap().r_global);
    eprintln!("   kf0 pitch in={p_in:.4} out={p_out:.4} deg");
    assert!(
        (p_out - p_in).abs() < 0.1,
        "anchor prior keeps kf0 pitch within 0.1 deg of input: {p_out}"
    );
}

// --- Test 3: revision supersedes a stale (wrong) committed constraint
// factor. A badly-wrong first sighting commits under instance "R0"; a
// corrective sighting reusing "R0" removes it; the final closure then uses
// the correct geometry.
#[test]
fn revision_supersedes_stale_constraint() {
    let k = 240;
    let frames = make_drifted_loop(k, 30.0, 0.0040);

    let run = |corrective: bool| -> f64 {
        let mut pgo = SimplePgo::new(test_config());
        for (i, fr) in frames.iter().enumerate() {
            step(&mut pgo, i as f64 * 0.1, fr.odom_pos, yaw(fr.odom_yaw));
            if i == 0 {
                // Wrong first estimate (off by 15 m in body-y), instance "R0".
                let mut bad = constraint_at("R", "R0", &frames[0], 0.0);
                bad.t_body_loc = mat3::add(&bad.t_body_loc, &[0.0, 15.0, 0.0]);
                sight(
                    &mut pgo,
                    frames[0].odom_pos,
                    yaw(frames[0].odom_yaw),
                    0.0,
                    &bad,
                );
            } else if i == 1 && corrective {
                // Corrective sighting reuses instance "R0" -> removes the stale one.
                let c = constraint_at("R", "R0", &frames[1], 0.1);
                sight(
                    &mut pgo,
                    frames[1].odom_pos,
                    yaw(frames[1].odom_yaw),
                    0.1,
                    &c,
                );
            } else if i == k {
                // Final sighting, distinct instance -> closes the loop.
                let c = constraint_at("R", "RK", &frames[k], i as f64 * 0.1);
                sight(
                    &mut pgo,
                    frames[k].odom_pos,
                    yaw(frames[k].odom_yaw),
                    i as f64 * 0.1,
                    &c,
                );
            }
        }
        mat3::norm(&mat3::sub(
            &pgo.key_poses().last().unwrap().t_global,
            &frames[k].true_pos,
        ))
    };

    let poisoned = run(false);
    let revised = run(true);
    eprintln!("   end error vs true: no-revision={poisoned:.1} m, with-revision={revised:.1} m");
    assert!(
        revised < poisoned - 5.0,
        "revision removes the stale wrong constraint -> cleaner closure: poisoned {poisoned}, revised {revised}"
    );
}

// --- Test 4: the per-keyframe roll/pitch prior keeps EVERY keyframe level
// through a loop closure (not just kf0).
#[test]
fn per_keyframe_rp_prior_keeps_keyframes_level() {
    let k = 240;
    let frames = make_drifted_loop(k, 30.0, 0.0040);
    let run = |per_kf: bool| -> f64 {
        let mut cfg = test_config();
        cfg.per_keyframe_rp_prior = per_kf;
        let mut pgo = SimplePgo::new(cfg);
        for (i, fr) in frames.iter().enumerate() {
            step(&mut pgo, i as f64 * 0.1, fr.odom_pos, yaw(fr.odom_yaw));
            if i == 0 || i == k {
                let c = constraint_at("P", &format!("P{i}"), fr, i as f64 * 0.1);
                sight(&mut pgo, fr.odom_pos, yaw(fr.odom_yaw), i as f64 * 0.1, &c);
            }
        }
        let mut max_pitch = 0.0f64;
        for kp in pgo.key_poses() {
            max_pitch = max_pitch.max(pitch_deg(&kp.r_global).abs());
        }
        max_pitch
    };
    let off = run(false);
    let on = run(true);
    eprintln!("   max |pitch| across keyframes: anchor off={off:.2} deg, on={on:.2} deg");
    assert!(
        on < 0.5,
        "per-keyframe roll/pitch prior keeps all keyframes level (<0.5 deg) through closure: {on}"
    );
}

// --- Point-cloud machinery unit tests --------------------------------------

/// VoxelGrid semantics: each occupied voxel emits the CENTROID of its
/// points (pcl behavior), not the voxel center; floor-binning handles
/// negative coordinates.
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
                    (result.r[i][j] as f32).to_bits(),
                    expected[i][j],
                    "{label}: r({i},{j})"
                );
            }
            assert_eq!(
                (result.t[i] as f32).to_bits(),
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
    let guess_r: Mat3 = [
        [f64::from(gc), f64::from(-gs), 0.0],
        [f64::from(gs), f64::from(gc), 0.0],
        [0.0, 0.0, 1.0],
    ];
    let guess_t: Vec3 = [f64::from(0.1f32), f64::from(-0.05f32), f64::from(0.02f32)];
    let result = icp_point_to_point(&source, &target, &guess_r, &guess_t, &IcpParams::default());
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
    let r_true = mat3::rot_z(0.12);
    let t_true = [0.35, -0.25, 0.15];
    let target = dimos_gsc_pgo::pointcloud::transform_cloud(&source, &r_true, &t_true);

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
    let rot_err = mat3::angular_distance(&result.r, &r_true);
    assert!(rot_err < 1e-3, "rotation recovered: err {rot_err} rad");
    let t_err = mat3::norm(&mat3::sub(&result.t, &t_true));
    assert!(t_err < 1e-2, "translation recovered: err {t_err} m");
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
