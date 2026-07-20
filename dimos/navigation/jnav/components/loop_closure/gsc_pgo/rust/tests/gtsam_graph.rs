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

//! gtsam FFI semantics the SimplePGO port relies on: a loop closure pulling
//! the graph, and factor revision (remove-by-index) relaxing it back —
//! mirroring simple_pgo.cpp's smoothAndUpdate() / constraint-revision paths.

use dimos_gsc_pgo::gtsam::{symbol_key, FactorGraph, Isam2, NoiseModel, Pose3, Values};

fn translation_x(pose: &Pose3) -> f64 {
    pose.t[0]
}

/// Prior + odometry chain 0-1-2-3 at x = 0,1,2,3, then a loop closure
/// claiming node 3 sits at x = 2.5 relative to node 0, with much tighter
/// noise than the odometry. The estimate must be pulled toward 2.5.
#[test]
fn loop_closure_pulls_the_end_node() {
    let mut isam2 = Isam2::new();
    let mut graph = FactorGraph::new();
    let mut values = Values::new();

    let prior_noise = NoiseModel::diagonal_variances(&[1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);
    let odom_noise = NoiseModel::diagonal_variances(&[1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2]);
    let loop_noise = NoiseModel::diagonal_variances(&[1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);

    graph
        .add_prior_pose3(0, &Pose3::identity(), &prior_noise)
        .unwrap();
    values.insert_pose3(0, &Pose3::identity()).unwrap();
    for i in 1..4u64 {
        let step = Pose3::from_translation([1.0, 0.0, 0.0]);
        graph
            .add_between_pose3(i - 1, i, &step, &odom_noise)
            .unwrap();
        values
            .insert_pose3(i, &Pose3::from_translation([i as f64, 0.0, 0.0]))
            .unwrap();
    }
    // Loop closure: node 3 measured at x=2.5 in node 0's frame.
    graph
        .add_between_pose3(0, 3, &Pose3::from_translation([2.5, 0.0, 0.0]), &loop_noise)
        .unwrap();

    let new_indices = isam2.update(&graph, &values, &[]).unwrap();
    assert_eq!(
        new_indices.len(),
        5,
        "prior + 3 odom + 1 loop factors committed"
    );
    // The extra relinearization pass the C++ core always runs.
    isam2.update_empty().unwrap();

    let estimate = isam2.calculate_best_estimate().unwrap();
    let x3 = translation_x(&estimate.pose3(3).unwrap());
    assert!(
        (x3 - 2.5).abs() < 0.1,
        "loop closure (var 1e-4) should dominate the odometry chain (var 3e-2): x3 = {x3}"
    );
    // Inner nodes compress proportionally, staying ordered.
    let x1 = translation_x(&estimate.pose3(1).unwrap());
    let x2 = translation_x(&estimate.pose3(2).unwrap());
    assert!(
        x1 > 0.0 && x1 < x2 && x2 < x3,
        "chain stays ordered: {x1} {x2} {x3}"
    );
    // Missing key reads back as None, not an error.
    assert!(estimate.pose3(99).is_none());
}

/// Revision semantics: commit a tight "correction" factor, capture its
/// iSAM2-assigned factor index from newFactorsIndices, then remove it via
/// update(remove_indices) and check the estimate relaxes back — exactly how
/// the C++ core supersedes location-constraint factors by instance id.
#[test]
fn removing_a_factor_by_index_relaxes_the_estimate() {
    let mut isam2 = Isam2::new();

    // Base graph: prior at origin + one odometry step to x=1.
    let mut graph = FactorGraph::new();
    let mut values = Values::new();
    let prior_noise = NoiseModel::diagonal_variances(&[1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);
    let odom_noise = NoiseModel::diagonal_variances(&[1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2]);
    graph
        .add_prior_pose3(0, &Pose3::identity(), &prior_noise)
        .unwrap();
    graph
        .add_between_pose3(0, 1, &Pose3::from_translation([1.0, 0.0, 0.0]), &odom_noise)
        .unwrap();
    values.insert_pose3(0, &Pose3::identity()).unwrap();
    values
        .insert_pose3(1, &Pose3::from_translation([1.0, 0.0, 0.0]))
        .unwrap();
    isam2.update(&graph, &values, &[]).unwrap();

    let baseline = isam2.calculate_best_estimate().unwrap();
    let x1_before = translation_x(&baseline.pose3(1).unwrap());
    assert!(
        (x1_before - 1.0).abs() < 1e-3,
        "odometry-only estimate: x1 = {x1_before}"
    );

    // Commit a much tighter "correction" claiming node 1 is at x=2.
    let mut correction_graph = FactorGraph::new();
    let empty_values = Values::new();
    let tight_noise = NoiseModel::diagonal_variances(&[1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);
    correction_graph
        .add_between_pose3(
            0,
            1,
            &Pose3::from_translation([2.0, 0.0, 0.0]),
            &tight_noise,
        )
        .unwrap();
    let new_indices = isam2.update(&correction_graph, &empty_values, &[]).unwrap();
    assert_eq!(
        new_indices.len(),
        1,
        "one staged factor, one committed index"
    );
    let correction_index = new_indices[0];
    isam2.update_empty().unwrap();

    let corrected = isam2.calculate_best_estimate().unwrap();
    let x1_corrected = translation_x(&corrected.pose3(1).unwrap());
    assert!(
        x1_corrected > 1.8,
        "tight correction should dominate: x1 = {x1_corrected}"
    );

    // Revision: remove the correction by its factor index (empty new graph),
    // plus the extra relinearization passes the C++ core runs on closures.
    let empty_graph = FactorGraph::new();
    isam2
        .update(&empty_graph, &empty_values, &[correction_index])
        .unwrap();
    for _ in 0..4 {
        isam2.update_empty().unwrap();
    }

    let relaxed = isam2.calculate_best_estimate().unwrap();
    let x1_after = translation_x(&relaxed.pose3(1).unwrap());
    assert!(
        (x1_after - 1.0).abs() < 0.05,
        "after removing the correction the odometry answer returns: x1 = {x1_after}"
    );
}

/// The remaining shim surface used by the port: Symbol keys, gaussian
/// covariance + robust-Huber noise on a factor, and graph clear()/size().
#[test]
fn symbol_keys_and_noise_models_round_trip() {
    // gtsam::Symbol packs the char into the top byte.
    let key = symbol_key('l', 5);
    assert_eq!(key >> 56, 'l' as u64);
    assert_eq!(key & 0x00ff_ffff_ffff_ffff, 5);
    assert_ne!(symbol_key('l', 5), symbol_key('x', 5));

    let mut isam2 = Isam2::new();
    let mut graph = FactorGraph::new();
    let mut values = Values::new();

    let prior_noise = NoiseModel::diagonal_variances(&[1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);
    graph
        .add_prior_pose3(0, &Pose3::identity(), &prior_noise)
        .unwrap();
    values.insert_pose3(0, &Pose3::identity()).unwrap();

    // Location-constraint-style factor: node 0 -> Symbol('l', 0), gaussian
    // covariance wrapped in a robust Huber kernel (the loop_robust_kernel path).
    let mut covariance = [0.0f64; 36];
    for i in 0..6 {
        covariance[i * 6 + i] = 1e-3;
    }
    let gaussian = NoiseModel::gaussian_covariance(&covariance);
    let robust = NoiseModel::robust_huber(0.5, &gaussian);
    let loc_key = symbol_key('l', 0);
    graph
        .add_between_pose3(
            0,
            loc_key,
            &Pose3::from_translation([0.0, 3.0, 0.0]),
            &robust,
        )
        .unwrap();
    values
        .insert_pose3(loc_key, &Pose3::from_translation([0.0, 3.0, 0.0]))
        .unwrap();
    assert_eq!(graph.len(), 2);

    isam2.update(&graph, &values, &[]).unwrap();
    graph.clear();
    values.clear();
    assert_eq!(graph.len(), 0);

    let estimate = isam2.calculate_best_estimate().unwrap();
    let location = estimate.pose3(loc_key).unwrap();
    assert!(
        (location.t[1] - 3.0).abs() < 1e-3,
        "landmark sits where observed: {:?}",
        location.t
    );
}
