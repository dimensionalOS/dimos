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

//! The graph outputs: new_keyframe once per keyframe with the box around its scans, a versioned loop_closure_event
//! naming the keyframes a closure paired, and pose_graph only on request.
#![cfg(feature = "native")]

use std::sync::Arc;

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3;
use dimos_gsc_pgo::records::{GraphOutputs, CLOSED_WITH_NONE};
use lcm_msgs::sensor_msgs::PointCloud2;

/// A scan at (x, 0, 0) facing +x whose body-frame points span x in [1, 3], y in [-1, 1], z in [0, 0.5].
fn scan_at(x: f64, time: f64) -> CloudWithPose {
    let cloud = vec![[1.0, -1.0, 0.0], [3.0, 1.0, 0.5], [2.0, 0.0, 0.25]];
    let mut pose = PoseWithTime::new(mat3::identity(), [x, 0.0, 0.0]);
    pose.set_time(time as i32, ((time - time.floor()) * 1e9) as u32);
    CloudWithPose {
        cloud: Some(Arc::new(cloud)),
        pose,
    }
}

/// A graph built from scans every 0.4 m with keyframes every 1 m (so 0, 1.2, 2.4, ...).
fn graph(scans: usize) -> GscPgo {
    let config = Config {
        keyframe_min_distance_meters: 1.0,
        auto_aggregate_cloud_size: 0,
        ..Config::default()
    };
    let mut pgo = GscPgo::new(config);
    for i in 0..scans {
        pgo.add_key_pose(&scan_at(0.4 * i as f64, i as f64));
    }
    pgo
}

/// Field `name` of record `index` as little-endian bytes.
fn field<'a>(msg: &'a PointCloud2, index: usize, name: &str) -> &'a [u8] {
    let f = msg
        .fields
        .iter()
        .find(|f| f.name == name)
        .unwrap_or_else(|| panic!("no field {name}"));
    let start = index * msg.point_step as usize + f.offset as usize;
    &msg.data[start..start + 4]
}
fn f32_at(msg: &PointCloud2, index: usize, name: &str) -> f32 {
    f32::from_le_bytes(field(msg, index, name).try_into().unwrap())
}
fn u32_at(msg: &PointCloud2, index: usize, name: &str) -> u32 {
    u32::from_le_bytes(field(msg, index, name).try_into().unwrap())
}

#[test]
fn a_keyframe_box_covers_every_scan_since_the_previous_keyframe() {
    let pgo = graph(7);
    let keys = pgo.key_poses();
    assert_eq!(keys.len(), 3, "keyframes at 0, 1.2, 2.4");
    // the first keyframe has only its own scan
    assert_eq!(keys.scan_box(0), [1.0, -1.0, 0.0, 3.0, 1.0, 0.5]);
    // keyframe 1 at x 1.2 covers the scans at 0.4 and 0.8 (reaching back to 1.4 - 1.2) and its own (to 4.2 - 1.2)
    let b = keys.scan_box(1);
    assert!(
        (b[0] - 0.2).abs() < 1e-5 && (b[3] - 3.0).abs() < 1e-5,
        "{b:?}"
    );
    assert_eq!([b[1], b[2], b[4], b[5]], [-1.0, 0.0, 1.0, 0.5]);
}

#[test]
fn new_keyframe_goes_out_once_per_keyframe_with_its_raw_pose_and_box() {
    let mut pgo = graph(4);
    let mut outputs = GraphOutputs::default();
    let first = outputs.step(
        pgo.key_poses(),
        pgo.committed_closures(),
        false,
        false,
        1.0,
        "odom",
    );
    let msg = first.new_keyframe.expect("the first keyframes");
    assert_eq!(msg.width, 2);
    assert_eq!(u32_at(&msg, 1, "node_id"), 1);
    assert!((f32_at(&msg, 1, "x") - 1.2).abs() < 1e-5);
    assert!((f32_at(&msg, 1, "min_x") - 0.2).abs() < 1e-5);
    assert_eq!(f32_at(&msg, 1, "max_z"), 0.5);
    // nothing new: nothing sent; a new keyframe: only it
    assert!(outputs
        .step(
            pgo.key_poses(),
            pgo.committed_closures(),
            false,
            false,
            2.0,
            "odom"
        )
        .new_keyframe
        .is_none());
    for i in 4..7 {
        pgo.add_key_pose(&scan_at(0.4 * i as f64, i as f64));
    }
    let msg = outputs
        .step(
            pgo.key_poses(),
            pgo.committed_closures(),
            false,
            false,
            3.0,
            "odom",
        )
        .new_keyframe
        .expect("keyframe 2");
    assert_eq!((msg.width, u32_at(&msg, 0, "node_id")), (1, 2));
}

#[test]
fn pose_graph_goes_out_only_when_asked() {
    let pgo = graph(4);
    let mut outputs = GraphOutputs::default();
    assert!(outputs
        .step(
            pgo.key_poses(),
            pgo.committed_closures(),
            false,
            false,
            1.0,
            "odom"
        )
        .pose_graph
        .is_none());
    let asked = outputs.step(
        pgo.key_poses(),
        pgo.committed_closures(),
        false,
        true,
        1.0,
        "odom",
    );
    assert_eq!(asked.pose_graph.expect("asked for").width, 2);
}

#[test]
fn loop_closure_events_count_up_and_name_the_keyframes_a_closure_paired() {
    let mut pgo = graph(7);
    let mut outputs = GraphOutputs::default();
    outputs.step(
        pgo.key_poses(),
        pgo.committed_closures(),
        false,
        false,
        1.0,
        "odom",
    );
    pgo.commit_closure(2, 0, mat3::identity(), [-2.4, 0.0, 0.0], 0.1);
    let (event, _) = outputs
        .step(
            pgo.key_poses(),
            pgo.committed_closures(),
            true,
            false,
            2.0,
            "odom",
        )
        .loop_closure_event
        .expect("a closure closed");
    assert_eq!(event.width, 3);
    assert_eq!(
        (0..3)
            .map(|k| u32_at(&event, k, "version"))
            .collect::<Vec<_>>(),
        [1, 1, 1]
    );
    assert_eq!(
        (0..3)
            .map(|k| u32_at(&event, k, "closed_with"))
            .collect::<Vec<_>>(),
        [2, CLOSED_WITH_NONE, 0]
    );
    // the next event is version 2 and reports no closure twice
    let (event, _) = outputs
        .step(
            pgo.key_poses(),
            pgo.committed_closures(),
            true,
            false,
            3.0,
            "odom",
        )
        .loop_closure_event
        .expect("closed again");
    assert_eq!(u32_at(&event, 0, "version"), 2);
    assert!((0..3).all(|k| u32_at(&event, k, "closed_with") == CLOSED_WITH_NONE));
}
