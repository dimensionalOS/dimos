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

//! Standalone helpers for the native PGO module: odometry interpolation, quaternion slerp, the
//! published-message builders and the inbound PointCloud2 parser.

use dimos_gsc_pgo::gsc_pgo::{ClosureEdgeEvent, KeyPoseWithCloud, CLOSURE_EDGE_RECORD_BYTES};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::pointcloud::PointCloud;
use lcm_msgs::geometry_msgs::{Point, Quaternion, Transform, TransformStamped, Vector3};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use lcm_msgs::tf2_msgs::TFMessage;

fn make_time(ts: f64) -> Time {
    Time {
        sec: ts as i32,
        nsec: ((ts - (ts as i32) as f64) * 1e9) as i32,
    }
}

pub fn make_header(frame_id: &str, ts: f64) -> Header {
    Header {
        seq: 0,
        stamp: make_time(ts),
        frame_id: frame_id.to_string(),
    }
}

pub fn build_odometry(
    rotation: &Mat3,
    translation: &Vec3,
    ts: f64,
    frame_id: &str,
    child_frame_id: &str,
) -> Odometry {
    let q = mat3::quat_from_mat(rotation);
    let mut odom = Odometry {
        header: make_header(frame_id, ts),
        child_frame_id: child_frame_id.to_string(),
        ..Default::default()
    };
    odom.pose.pose.position = Point {
        x: translation[0],
        y: translation[1],
        z: translation[2],
    };
    odom.pose.pose.orientation = Quaternion {
        x: q[1],
        y: q[2],
        z: q[3],
        w: q[0],
    };
    odom
}

pub fn build_tf_message(
    correction_rotation: &Mat3,
    correction_translation: &Vec3,
    ts: f64,
    frame_id: &str,
    child_frame_id: &str,
) -> TFMessage {
    let q = mat3::quat_from_mat(correction_rotation);
    TFMessage {
        transforms: vec![TransformStamped {
            header: make_header(frame_id, ts),
            child_frame_id: child_frame_id.to_string(),
            transform: Transform {
                translation: Vector3 {
                    x: correction_translation[0],
                    y: correction_translation[1],
                    z: correction_translation[2],
                },
                rotation: Quaternion {
                    x: q[1],
                    y: q[2],
                    z: q[3],
                    w: q[0],
                },
            },
        }],
    }
}

/// One little-endian record per keyframe, the layout fast-reform consumers
/// decode: x, y, z (f32), node_id (u32), ts (f64), qx, qy, qz, qw (f32).
fn put_keyframe_record(
    data: &mut Vec<u8>,
    id: usize,
    ts: f64,
    rotation: &Mat3,
    translation: &Vec3,
) {
    let q = mat3::quat_from_mat(rotation);
    for v in translation {
        data.extend_from_slice(&(*v as f32).to_le_bytes());
    }
    data.extend_from_slice(&(id as u32).to_le_bytes());
    data.extend_from_slice(&ts.to_le_bytes());
    for v in [q[1], q[2], q[3], q[0]] {
        data.extend_from_slice(&(v as f32).to_le_bytes());
    }
}

/// A PointCloud2 over packed records with these fields, in order.
fn record_cloud(fields: &[(&str, u8)], data: Vec<u8>, frame_id: &str, ts: f64) -> PointCloud2 {
    let mut offset = 0;
    let fields: Vec<PointField> = fields
        .iter()
        .map(|&(name, datatype)| {
            let field = PointField {
                name: name.into(),
                offset,
                datatype,
                count: 1,
            };
            offset += if datatype == PointField::FLOAT64 as u8 {
                8
            } else {
                4
            };
            field
        })
        .collect();
    let n = data.len() as i32 / offset.max(1);
    PointCloud2 {
        header: make_header(frame_id, ts),
        height: 1,
        width: n,
        fields,
        is_bigendian: false,
        point_step: offset,
        row_step: offset * n,
        data,
        is_dense: true,
    }
}

const F32: u8 = PointField::FLOAT32 as u8;
const KEYFRAME_FIELDS: [(&str, u8); 9] = [
    ("x", F32),
    ("y", F32),
    ("z", F32),
    ("node_id", PointField::UINT32 as u8),
    ("ts", PointField::FLOAT64 as u8),
    ("qx", F32),
    ("qy", F32),
    ("qz", F32),
    ("qw", F32),
];
const DELTA_FIELDS: [(&str, u8); 7] = [
    ("dx", F32),
    ("dy", F32),
    ("dz", F32),
    ("dqx", F32),
    ("dqy", F32),
    ("dqz", F32),
    ("dqw", F32),
];

/// pose_graph as packed records, one per keyframe at its current pose.
pub fn build_pose_graph_records(
    key_poses: &[KeyPoseWithCloud],
    ts: f64,
    frame_id: &str,
) -> PointCloud2 {
    let mut data = Vec::with_capacity(key_poses.len() * 40);
    for (i, kp) in key_poses.iter().enumerate() {
        put_keyframe_record(
            &mut data,
            i,
            kp.time,
            &kp.rotation_global,
            &kp.translation_global,
        );
    }
    record_cloud(&KEYFRAME_FIELDS, data, frame_id, ts)
}

/// loop_closure_event as packed records: each keyframe's pre pose, then the
/// delta with post = delta * pre.
pub fn build_loop_closure_records(
    pre_poses: &[(Mat3, Vec3)],
    post_poses: &[KeyPoseWithCloud],
    ts: f64,
    frame_id: &str,
) -> PointCloud2 {
    let count = pre_poses.len().min(post_poses.len());
    let mut data = Vec::with_capacity(count * 68);
    for i in 0..count {
        let (pre_rotation, pre_translation) = &pre_poses[i];
        let post = &post_poses[i];
        let rotation_delta = mat3::mat_mul(&post.rotation_global, &mat3::transpose(pre_rotation));
        let translation_delta = mat3::sub(
            &post.translation_global,
            &mat3::mat_vec(&rotation_delta, pre_translation),
        );
        put_keyframe_record(&mut data, i, post.time, pre_rotation, pre_translation);
        let q = mat3::quat_from_mat(&rotation_delta);
        for v in translation_delta {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
        for v in [q[1], q[2], q[3], q[0]] {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
    }
    let fields: Vec<(&str, u8)> = KEYFRAME_FIELDS
        .iter()
        .chain(&DELTA_FIELDS)
        .copied()
        .collect();
    record_cloud(&fields, data, frame_id, ts)
}

const CLOSURE_EDGE_FIELDS: [(&str, u8); 12] = [
    ("x", F32),
    ("y", F32),
    ("z", F32),
    ("target_x", F32),
    ("target_y", F32),
    ("target_z", F32),
    ("source_id", PointField::UINT32 as u8),
    ("target_id", PointField::UINT32 as u8),
    ("state", PointField::UINT32 as u8),
    ("confidence", F32),
    ("yank_z", F32),
    ("ts", PointField::FLOAT64 as u8),
];

/// closure_edges as packed records, one per lifecycle event (see `ClosureEdgeEvent`).
pub fn build_closure_edge_records(
    events: &[ClosureEdgeEvent],
    key_poses: &[KeyPoseWithCloud],
    ts: f64,
    frame_id: &str,
) -> PointCloud2 {
    let mut data = Vec::with_capacity(events.len() * CLOSURE_EDGE_RECORD_BYTES);
    for event in events {
        event.put_record(&mut data, key_poses);
    }
    record_cloud(&CLOSURE_EDGE_FIELDS, data, frame_id, ts)
}

#[derive(Debug)]
pub struct ExtractError(String);
impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Parse a PointCloud2 into xyz points (x/y/z float32 offsets; intensity is
/// dropped — the PGO core never reads it).
pub fn extract_xyz(msg: &PointCloud2) -> Result<PointCloud, ExtractError> {
    let mut ox = None;
    let mut oy = None;
    let mut oz = None;
    for f in &msg.fields {
        match f.name.as_str() {
            "x" => ox = Some(f.offset as usize),
            "y" => oy = Some(f.offset as usize),
            "z" => oz = Some(f.offset as usize),
            _ => {}
        }
    }
    let (ox, oy, oz) = match (ox, oy, oz) {
        (Some(a), Some(b), Some(c)) => (a, b, c),
        _ => return Err(ExtractError("missing x/y/z fields".into())),
    };
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("zero point_step".into()));
    }
    let num_points = (msg.width as usize) * (msg.height as usize);
    let data = &msg.data;
    let mut out = Vec::with_capacity(num_points);
    for i in 0..num_points {
        let base = i * step;
        if base + step > data.len() {
            break;
        }
        let read = |off: usize| -> f32 {
            let b = &data[base + off..base + off + 4];
            f32::from_le_bytes([b[0], b[1], b[2], b[3]])
        };
        out.push([read(ox), read(oy), read(oz)]);
    }
    Ok(out)
}
