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

//! What the native binaries (`gsc-pgo`, `pgo-eval`) send and dump: the published messages, the packed keyframe,
//! pose-change and closure-edge records, the keyframe / pose-change dump files, and the inbound PointCloud2 parser.

use std::io::Write;
use std::path::Path;

use crate::gsc_pgo::{ClosureEdgeEvent, KeyPoseWithCloud, CLOSURE_EDGE_RECORD_BYTES};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::PointCloud;
use lcm_msgs::geometry_msgs::{Point, Quaternion};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};

/// A keyframe pose change smaller than this waits to add up before it is sent.
const POSE_CHANGE_M: f64 = 1e-3;
const POSE_CHANGE_RAD: f64 = 1e-4;
/// Bytes per pose-change record: a keyframe record (40) then the delta (28).
const POSE_CHANGE_RECORD_BYTES: usize = 68;

pub fn make_header(frame_id: &str, ts: f64) -> Header {
    let stamp = Time {
        sec: ts as i32,
        nsec: ((ts - (ts as i32) as f64) * 1e9) as i32,
    };
    Header {
        seq: 0,
        stamp,
        frame_id: frame_id.to_string(),
    }
}

fn quaternion(rotation: &Mat3) -> Quaternion {
    let [w, x, y, z] = mat3::quat_from_mat(rotation);
    Quaternion { x, y, z, w }
}

pub fn build_odometry(
    rotation: &Mat3,
    translation: &Vec3,
    ts: f64,
    frame_id: &str,
    child_frame_id: &str,
) -> Odometry {
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
    odom.pose.pose.orientation = quaternion(rotation);
    odom
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

/// loop_closure_event from `HeardPoses::take_changes` records.
pub fn build_loop_closure_records(data: Vec<u8>, ts: f64, frame_id: &str) -> PointCloud2 {
    let fields: Vec<(&str, u8)> = KEYFRAME_FIELDS
        .iter()
        .chain(&DELTA_FIELDS)
        .copied()
        .collect();
    record_cloud(&fields, data, frame_id, ts)
}

/// Each keyframe's pose as fast-reform consumers last heard it.
#[derive(Default)]
pub struct HeardPoses(Vec<(Mat3, Vec3)>);

impl HeardPoses {
    /// Keyframes added since the last call: consumers hold their scans where the current offset put them.
    pub fn add_new(&mut self, key_poses: &[KeyPoseWithCloud]) {
        self.0.extend(
            key_poses[self.0.len()..]
                .iter()
                .map(|kp| (kp.rotation_global, kp.translation_global)),
        );
    }

    /// When a closure closed or any keyframe moved past POSE_CHANGE_*, every heard keyframe's pose change as packed
    /// records (pre x y z, id, ts, pre q (xyzw), then dt, dq with post = delta * pre; f32 but id u32, ts f64), now heard.
    pub fn take_changes(
        &mut self,
        key_poses: &[KeyPoseWithCloud],
        closure_closed: bool,
    ) -> Option<Vec<u8>> {
        let moved = key_poses
            .iter()
            .zip(&self.0)
            .any(|(kp, (rotation, translation))| {
                let shift = mat3::sub(&kp.translation_global, translation);
                let turn = mat3::mat_mul(&kp.rotation_global, &mat3::transpose(rotation));
                let cos = ((turn[0][0] + turn[1][1] + turn[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
                mat3::norm(&shift) > POSE_CHANGE_M || cos.acos() > POSE_CHANGE_RAD
            });
        if !closure_closed && !moved {
            return None;
        }
        let count = self.0.len().min(key_poses.len());
        let mut data = Vec::with_capacity(count * POSE_CHANGE_RECORD_BYTES);
        for (i, ((pre_rotation, pre_translation), post)) in self.0.iter().zip(key_poses).enumerate()
        {
            let rotation_delta =
                mat3::mat_mul(&post.rotation_global, &mat3::transpose(pre_rotation));
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
        self.0 = key_poses
            .iter()
            .map(|kp| (kp.rotation_global, kp.translation_global))
            .collect();
        Some(data)
    }
}

/// Append one `take_changes` to a pose-change dump (`<keyframes>.closures`): f64 ts, u32 record count, the records.
pub fn append_pose_changes(file: &mut impl Write, ts: f64, records: &[u8]) -> std::io::Result<()> {
    let mut bytes = Vec::with_capacity(12 + records.len());
    bytes.extend_from_slice(&ts.to_le_bytes());
    bytes.extend_from_slice(&((records.len() / POSE_CHANGE_RECORD_BYTES) as u32).to_le_bytes());
    bytes.extend_from_slice(records);
    file.write_all(&bytes)
}

/// Write each keyframe as little-endian f64s: ts, raw (local) x y z qx qy qz qw, then corrected (global) x y z qx qy
/// qz qw, atomically.
pub fn write_keyframes(path: &Path, key_poses: &[KeyPoseWithCloud]) -> std::io::Result<()> {
    let mut bytes = Vec::with_capacity(key_poses.len() * 15 * 8);
    for kp in key_poses {
        let mut values = vec![kp.time];
        for (rotation, translation) in [
            (&kp.rotation_local, &kp.translation_local),
            (&kp.rotation_global, &kp.translation_global),
        ] {
            let q = mat3::quat_from_mat(rotation);
            values.extend_from_slice(translation);
            values.extend_from_slice(&[q[1], q[2], q[3], q[0]]);
        }
        for v in values {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)
}

/// Closure-edge events as packed records (see `ClosureEdgeEvent::put_record`).
pub fn closure_edge_records(
    events: &[ClosureEdgeEvent],
    key_poses: &[KeyPoseWithCloud],
) -> Vec<u8> {
    let mut data = Vec::with_capacity(events.len() * CLOSURE_EDGE_RECORD_BYTES);
    for event in events {
        event.put_record(&mut data, key_poses);
    }
    data
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
    ("yank_stdevs", F32),
    ("ts", PointField::FLOAT64 as u8),
];

/// closure_edges as packed records, one per lifecycle event (see `ClosureEdgeEvent`).
pub fn build_closure_edge_records(
    events: &[ClosureEdgeEvent],
    key_poses: &[KeyPoseWithCloud],
    ts: f64,
    frame_id: &str,
) -> PointCloud2 {
    record_cloud(
        &CLOSURE_EDGE_FIELDS,
        closure_edge_records(events, key_poses),
        frame_id,
        ts,
    )
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
