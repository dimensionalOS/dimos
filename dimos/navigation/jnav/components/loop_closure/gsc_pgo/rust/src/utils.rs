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

//! Standalone helpers for the native PGO module: odometry interpolation,
//! quaternion slerp, a matrix distance, the deformation-id RNG, plus the
//! wire-message builders (GscPgo state -> published LCM messages) and the
//! inbound PointCloud2 parser.

use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

use dimos_gsc_pgo::gsc_pgo::KeyPoseWithCloud;
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::msgs::{
    DeltaTransform, Edge, Graph3D, GraphDelta3D, Node3D, PoseStamped as WirePose,
};
use dimos_gsc_pgo::pointcloud::PointCloud;
use lcm_msgs::geometry_msgs::{Point, Quaternion, Transform, TransformStamped, Vector3};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use lcm_msgs::tf2_msgs::TFMessage;

use crate::OdomSample;

/// Interpolate the odometry pose at `ts` (slerp rotation, lerp translation).
/// `None` if the buffer is empty or `ts` predates it by more than 1 ms;
/// clamps to the newest sample when `ts` is past it.
pub fn interpolate_odom(buffer: &VecDeque<OdomSample>, ts: f64) -> Option<(Mat3, Vec3, String)> {
    let front = buffer.front()?;
    if ts <= front.ts {
        if front.ts - ts > 1e-3 {
            return None;
        }
        return Some((front.rotation, front.translation, front.frame_id.clone()));
    }
    let back = buffer.back()?;
    if ts >= back.ts {
        return Some((back.rotation, back.translation, back.frame_id.clone()));
    }
    for i in 1..buffer.len() {
        let hi = &buffer[i];
        if hi.ts < ts {
            continue;
        }
        let lo = &buffer[i - 1];
        let span = hi.ts - lo.ts;
        let alpha = if span > 0.0 { (ts - lo.ts) / span } else { 0.0 };
        let translation = [
            lo.translation[0] + alpha * (hi.translation[0] - lo.translation[0]),
            lo.translation[1] + alpha * (hi.translation[1] - lo.translation[1]),
            lo.translation[2] + alpha * (hi.translation[2] - lo.translation[2]),
        ];
        let q = slerp(
            &mat3::quat_from_mat(&lo.rotation),
            &mat3::quat_from_mat(&hi.rotation),
            alpha,
        );
        return Some((mat3::mat_from_quat(&q), translation, lo.frame_id.clone()));
    }
    None
}

/// Quaternion slerp with Eigen's shortest-path semantics (quats are [w,x,y,z]).
fn slerp(a: &[f64; 4], b: &[f64; 4], alpha: f64) -> [f64; 4] {
    let mut dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    dot *= sign;
    let (wa, wb) = if dot > 0.9995 {
        (1.0 - alpha, alpha)
    } else {
        let theta = dot.clamp(-1.0, 1.0).acos();
        let sin_theta = theta.sin();
        (
            ((1.0 - alpha) * theta).sin() / sin_theta,
            (alpha * theta).sin() / sin_theta,
        )
    };
    let mut out = [0.0; 4];
    for i in 0..4 {
        out[i] = wa * a[i] + sign * wb * b[i];
    }
    let norm = out.iter().map(|v| v * v).sum::<f64>().sqrt();
    if norm > 0.0 {
        for v in out.iter_mut() {
            *v /= norm;
        }
    }
    out
}

/// Frobenius norm of `a - b` (sqrt of summed squared element differences).
pub fn frobenius_diff(a: &Mat3, b: &Mat3) -> f64 {
    let mut sum = 0.0;
    for row in 0..3 {
        for col in 0..3 {
            let d = a[row][col] - b[row][col];
            sum += d * d;
        }
    }
    sum.sqrt()
}

/// Tiny non-crypto RNG for the stable deformation-node ids. The ids only need
/// to be stable within a run and unlikely to collide, so there's no rand
/// crate dependency.
pub struct SplitMix64(u64);

impl SplitMix64 {
    pub fn from_entropy() -> Self {
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x9E3779B97F4A7C15)
            ^ (std::process::id() as u64) << 32;
        SplitMix64(seed)
    }

    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
}

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

// Pose-graph metadata ids.
const NODE_KEYFRAME: u64 = 0;
const EDGE_ODOMETRY: u64 = 0;
const EDGE_LOOP_CLOSURE: u64 = 1;

pub fn build_pose_graph(
    key_poses: &[KeyPoseWithCloud],
    loop_pairs: &[(usize, usize)],
    ts: f64,
    frame_id: &str,
) -> Graph3D {
    let mut msg = Graph3D {
        ts,
        nodes: Vec::with_capacity(key_poses.len()),
        edges: Vec::with_capacity(key_poses.len() + loop_pairs.len()),
    };
    for (i, kp) in key_poses.iter().enumerate() {
        let q = mat3::quat_from_mat(&kp.rotation_global);
        msg.nodes.push(Node3D {
            pose: WirePose {
                ts: kp.time,
                frame_id: frame_id.to_string(),
                position: kp.translation_global,
                orientation: [q[1], q[2], q[3], q[0]],
            },
            id: i as u64,
            metadata_id: NODE_KEYFRAME,
        });
    }
    for (i, key_pose) in key_poses.iter().enumerate().skip(1) {
        msg.edges.push(Edge {
            start_id: (i - 1) as u64,
            end_id: i as u64,
            timestamp: key_pose.time,
            metadata_id: EDGE_ODOMETRY,
        });
    }
    for &(first, second) in loop_pairs {
        if first >= key_poses.len() || second >= key_poses.len() {
            continue;
        }
        msg.edges.push(Edge {
            start_id: first as u64,
            end_id: second as u64,
            timestamp: ts,
            metadata_id: EDGE_LOOP_CLOSURE,
        });
    }
    msg
}

const NODE_KEYFRAME_DELTA: u64 = 0;

/// Build a loop-closure event: each pair is (pre-smooth node, SE(3) delta
/// such that post = delta * pre).
pub fn build_loop_closure_event(
    pre_poses: &[(Mat3, Vec3)],
    post_poses: &[KeyPoseWithCloud],
    ts: f64,
    frame_id: &str,
) -> GraphDelta3D {
    let count = pre_poses.len().min(post_poses.len());
    let mut msg = GraphDelta3D {
        ts,
        nodes: Vec::with_capacity(count),
        transforms: Vec::with_capacity(count),
    };
    for i in 0..count {
        let (pre_rotation, pre_translation) = &pre_poses[i];
        let post_rotation = &post_poses[i].rotation_global;
        let post_translation = &post_poses[i].translation_global;

        // SE(3) delta such that post = delta * pre.
        let rotation_delta = mat3::mat_mul(post_rotation, &mat3::transpose(pre_rotation));
        let translation_delta = mat3::sub(
            post_translation,
            &mat3::mat_vec(&rotation_delta, pre_translation),
        );
        let q_pre = mat3::quat_from_mat(pre_rotation);
        let q_delta = mat3::quat_from_mat(&rotation_delta);

        msg.nodes.push(Node3D {
            pose: WirePose {
                ts: post_poses[i].time,
                frame_id: frame_id.to_string(),
                position: *pre_translation,
                orientation: [q_pre[1], q_pre[2], q_pre[3], q_pre[0]],
            },
            id: i as u64,
            metadata_id: NODE_KEYFRAME_DELTA,
        });
        msg.transforms.push(DeltaTransform {
            translation: translation_delta,
            rotation: [q_delta[1], q_delta[2], q_delta[3], q_delta[0]],
        });
    }
    msg
}

/// The `from_pcl` wire layout (x/y/z/intensity float32, 16-byte points).
/// The Rust core's PointCloud carries no intensity, so it's published as 0.
pub fn build_pointcloud2(points: &[[f32; 3]], frame_id: &str, ts: f64) -> PointCloud2 {
    let mut data = Vec::with_capacity(points.len() * 16);
    for p in points {
        data.extend_from_slice(&p[0].to_le_bytes());
        data.extend_from_slice(&p[1].to_le_bytes());
        data.extend_from_slice(&p[2].to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
    }
    let field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    let n = points.len() as i32;
    PointCloud2 {
        header: make_header(frame_id, ts),
        height: 1,
        width: n,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: true,
    }
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
