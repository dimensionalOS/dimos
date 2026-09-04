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

//! Minimal reader for a dimos memory2 SQLite recording, decoding the raw
//! LCM blobs the eval binary feeds through GscPgo.
//!
//! Each stream `S` is two tables: `"S"` (metadata: id, ts, ...) and `"S_blob"`
//! (id, data BLOB). The blob is exactly `value.lcm_encode()` — an 8-byte
//! fingerprint followed by the packed message — so a stream is discriminated by
//! trying each candidate decoder and letting the fingerprint check reject
//! mismatches. Pose values in the metadata columns are deprecated; everything
//! here comes from decoding the blob.

use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use lcm_msgs::geometry_msgs::PoseStamped;
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;
use rusqlite::Connection;

/// One odometry sample: world pose at a timestamp. `frame_id` is the pose's
/// reference (parent) frame; `child_frame_id` is the body frame the pose moves
/// — the frame the paired lidar cloud must already be in (no tf is applied).
/// `child_frame_id` is empty for `PoseStamped` payloads, which carry no child.
pub struct OdomRow {
    pub ts: f64,
    pub translation: Vec3,
    pub rotation: Mat3,
    pub quaternion_xyzw: [f64; 4],
    pub frame_id: String,
    pub child_frame_id: String,
}

/// One lidar scan: xyz points (intensity dropped) at a timestamp.
pub struct ScanRow {
    pub ts: f64,
    pub points: Vec<[f32; 3]>,
    pub frame_id: String,
}

fn quote_ident(name: &str) -> Result<String, String> {
    if name.contains('"') {
        return Err(format!("illegal stream name {name:?}"));
    }
    Ok(format!("\"{name}\""))
}

/// Join a stream's metadata + blob tables on id, ordered by timestamp, and hand
/// each `(ts, blob)` to `decode`. Rows the decoder returns `None` for are
/// skipped (e.g. a corrupt frame), so a single bad message never aborts a run.
///
/// `stride` keeps every `stride`-th row (1 = all). The stride is applied here,
/// before decoding, so strided-out blobs are never decoded or retained — a huge
/// recording can be subsampled without materializing every scan in memory.
fn read_stream<T>(
    connection: &Connection,
    stream: &str,
    stride: usize,
    mut decode: impl FnMut(f64, &[u8]) -> Option<T>,
) -> Result<Vec<T>, String> {
    let stride = stride.max(1);
    let table = quote_ident(stream)?;
    let blob_table = quote_ident(&format!("{stream}_blob"))?;
    let sql = format!(
        "SELECT meta.ts, blob.data FROM {table} AS meta \
         JOIN {blob_table} AS blob ON meta.id = blob.id ORDER BY meta.ts"
    );
    let mut statement = connection.prepare(&sql).map_err(|e| e.to_string())?;
    let rows = statement
        .query_map([], |row| {
            let ts: f64 = row.get(0)?;
            let data: Vec<u8> = row.get(1)?;
            Ok((ts, data))
        })
        .map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    for (index, row) in rows.enumerate() {
        if index % stride != 0 {
            continue;
        }
        let (ts, data) = row.map_err(|e| e.to_string())?;
        if let Some(value) = decode(ts, &data) {
            out.push(value);
        }
    }
    Ok(out)
}

fn odom_from_odometry(ts: f64, message: &Odometry) -> OdomRow {
    let pose = &message.pose.pose;
    odom_from_pose(
        ts,
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        message.header.frame_id.clone(),
        message.child_frame_id.clone(),
    )
}

fn odom_from_pose_stamped(ts: f64, message: &PoseStamped) -> OdomRow {
    let pose = &message.pose;
    odom_from_pose(
        ts,
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        message.header.frame_id.clone(),
        String::new(),
    )
}

fn odom_from_pose(
    ts: f64,
    translation: Vec3,
    quaternion_xyzw: [f64; 4],
    frame_id: String,
    child_frame_id: String,
) -> OdomRow {
    let [qx, qy, qz, qw] = quaternion_xyzw;
    OdomRow {
        ts,
        translation,
        rotation: mat3::mat_from_quat(&[qw, qx, qy, qz]),
        quaternion_xyzw,
        frame_id,
        child_frame_id,
    }
}

/// Read an odometry stream, accepting either `nav_msgs/Odometry` or
/// `geometry_msgs/PoseStamped` payloads (legacy go2 recordings store the
/// latter). The fingerprint check makes the choice unambiguous per blob.
pub fn read_odometry(
    connection: &Connection,
    stream: &str,
    stride: usize,
) -> Result<Vec<OdomRow>, String> {
    let rows = read_stream(connection, stream, stride, |ts, data| {
        if let Ok(message) = Odometry::decode(data) {
            return Some(odom_from_odometry(ts, &message));
        }
        if let Ok(message) = PoseStamped::decode(data) {
            return Some(odom_from_pose_stamped(ts, &message));
        }
        None
    })?;
    if rows.is_empty() {
        return Err(format!(
            "odom stream {stream:?} decoded no Odometry/PoseStamped rows"
        ));
    }
    Ok(rows)
}

/// Extract xyz points from a PointCloud2 (mirrors the module's `extract_xyz`;
/// intensity and other fields are dropped — the PGO core never reads them).
fn extract_xyz(message: &PointCloud2) -> Option<Vec<[f32; 3]>> {
    let mut offset_x = None;
    let mut offset_y = None;
    let mut offset_z = None;
    for field in &message.fields {
        match field.name.as_str() {
            "x" => offset_x = Some(field.offset as usize),
            "y" => offset_y = Some(field.offset as usize),
            "z" => offset_z = Some(field.offset as usize),
            _ => {}
        }
    }
    let (offset_x, offset_y, offset_z) = (offset_x?, offset_y?, offset_z?);
    let step = message.point_step as usize;
    if step == 0 {
        return None;
    }
    let point_count = (message.width as usize) * (message.height as usize);
    let data = &message.data;
    let mut points = Vec::with_capacity(point_count);
    let read = |base: usize, offset: usize| -> f32 {
        let bytes = &data[base + offset..base + offset + 4];
        f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    };
    for index in 0..point_count {
        let base = index * step;
        if base + step > data.len() {
            break;
        }
        points.push([
            read(base, offset_x),
            read(base, offset_y),
            read(base, offset_z),
        ]);
    }
    Some(points)
}

/// Read a lidar stream of PointCloud2 scans.
pub fn read_scans(
    connection: &Connection,
    stream: &str,
    stride: usize,
) -> Result<Vec<ScanRow>, String> {
    let rows = read_stream(connection, stream, stride, |ts, data| {
        let message = PointCloud2::decode(data).ok()?;
        let points = extract_xyz(&message)?;
        Some(ScanRow {
            ts,
            points,
            frame_id: message.header.frame_id.clone(),
        })
    })?;
    if rows.is_empty() {
        return Err(format!(
            "lidar stream {stream:?} decoded no PointCloud2 rows"
        ));
    }
    Ok(rows)
}
