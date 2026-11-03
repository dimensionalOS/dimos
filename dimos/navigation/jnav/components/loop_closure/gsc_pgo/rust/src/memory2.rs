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
use dimos_gsc_pgo::records::extract_xyz;
use lcm_msgs::geometry_msgs::PoseStamped;
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::tf2_msgs::TFMessage;
use rusqlite::Connection;

/// One odometry sample: world pose at a timestamp. `frame_id` is the pose's
/// reference (parent) frame; `child_frame_id` is the frame the pose moves (the
/// eval re-expresses it for the body frame through the recording's tf).
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
fn read_stream<T>(
    connection: &Connection,
    stream: &str,
    mut decode: impl FnMut(f64, &[u8]) -> Option<T>,
) -> Result<Vec<T>, String> {
    let mut out = Vec::new();
    for_each_row(connection, stream, f64::INFINITY, |ts, data| {
        if let Some(value) = decode(ts, data) {
            out.push(value);
        }
        Ok(())
    })?;
    Ok(out)
}

/// Call `visit` with every row's (ts, blob) in time order, one row in memory at a time.
fn for_each_row(
    connection: &Connection,
    stream: &str,
    until_s: f64,
    mut visit: impl FnMut(f64, &[u8]) -> Result<(), String>,
) -> Result<(), String> {
    let table = quote_ident(stream)?;
    let blob_table = quote_ident(&format!("{stream}_blob"))?;
    // order the (small) metadata alone, then fetch each kept blob by id: sorting the joined blobs would spill the whole
    // stream into a temp b-tree (tens of GB on a long recording, enough to fill the disk)
    let sql = format!("SELECT id, ts FROM {table} ORDER BY ts");
    let mut statement = connection.prepare(&sql).map_err(|e| e.to_string())?;
    let mut rows = statement.query([]).map_err(|e| e.to_string())?;
    let mut blob_statement = connection
        .prepare(&format!("SELECT data FROM {blob_table} WHERE id = ?1"))
        .map_err(|e| e.to_string())?;
    let mut first_ts = None;
    while let Some(row) = rows.next().map_err(|e| e.to_string())? {
        let id: i64 = row.get(0).map_err(|e| e.to_string())?;
        let ts: f64 = row.get(1).map_err(|e| e.to_string())?;
        // rows come in time order: past `until_s` of the stream there is nothing more to visit
        if ts - *first_ts.get_or_insert(ts) > until_s {
            break;
        }
        let mut blob_rows = blob_statement.query([id]).map_err(|e| e.to_string())?;
        if let Some(blob_row) = blob_rows.next().map_err(|e| e.to_string())? {
            let data = blob_row.get_ref(0).map_err(|e| e.to_string())?;
            visit(ts, data.as_blob().map_err(|e| e.to_string())?)?;
        }
    }
    Ok(())
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
pub fn read_odometry(connection: &Connection, stream: &str) -> Result<Vec<OdomRow>, String> {
    let rows = read_stream(connection, stream, |ts, data| {
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

/// Call `visit` with each scan of a lidar stream of PointCloud2s, in time order, one at a time, stopping after the
/// stream's first `until_s` seconds.
pub fn for_each_scan(
    connection: &Connection,
    stream: &str,
    until_s: f64,
    mut visit: impl FnMut(ScanRow) -> Result<(), String>,
) -> Result<(), String> {
    for_each_row(connection, stream, until_s, |ts, data| {
        let Ok(message) = PointCloud2::decode(data) else {
            return Ok(());
        };
        let Ok(points) = extract_xyz(&message) else {
            return Ok(());
        };
        visit(ScanRow {
            ts,
            points,
            frame_id: message.header.frame_id.clone(),
        })
    })
}

/// A rigid transform taking points in `child` to `parent` (tf's parent->child edge): p_parent = rotation * p_child + translation.
#[derive(Clone, Copy, Debug)]
pub struct Rigid {
    pub rotation: Mat3,
    pub translation: Vec3,
}

impl Rigid {
    pub fn identity() -> Rigid {
        Rigid {
            rotation: mat3::identity(),
            translation: [0.0; 3],
        }
    }

    /// self * other: apply `other`, then `self`.
    pub fn compose(&self, other: &Rigid) -> Rigid {
        Rigid {
            rotation: mat3::mat_mul(&self.rotation, &other.rotation),
            translation: mat3::add(
                &mat3::mat_vec(&self.rotation, &other.translation),
                &self.translation,
            ),
        }
    }

    pub fn inverse(&self) -> Rigid {
        let rotation = mat3::transpose(&self.rotation);
        let back = mat3::mat_vec(&rotation, &self.translation);
        Rigid {
            rotation,
            translation: [-back[0], -back[1], -back[2]],
        }
    }

    pub fn is_identity(&self) -> bool {
        let r = &self.rotation;
        let i = mat3::identity();
        (0..3).all(|a| (0..3).all(|b| (r[a][b] - i[a][b]).abs() < 1e-12))
            && mat3::norm(&self.translation) < 1e-12
    }
}

/// One tf edge as recorded: its first value, and whether every later message repeated it (a static mount) or not
/// (a moving frame, e.g. odom -> mid360_link, which a mount lookup must not use).
struct TfEdge {
    parent: String,
    child: String,
    transform: Rigid,
    constant: bool,
}

/// The recording's tf tree (the `tf` stream of TFMessages), for looking up its fixed mounts.
pub struct TfTree {
    edges: Vec<TfEdge>,
}

impl TfTree {
    /// Read every transform of the `tf` stream; a recording without one has an empty tree.
    pub fn read(connection: &Connection) -> Result<TfTree, String> {
        let has_tf: bool = connection
            .query_row(
                "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type='table' AND name='tf')",
                [],
                |row| row.get(0),
            )
            .map_err(|e| e.to_string())?;
        let mut edges: Vec<TfEdge> = Vec::new();
        if !has_tf {
            return Ok(TfTree { edges });
        }
        for_each_row(connection, "tf", f64::INFINITY, |_, data| {
            let Ok(message) = TFMessage::decode(data) else {
                return Ok(());
            };
            for stamped in &message.transforms {
                let t = &stamped.transform.translation;
                let q = &stamped.transform.rotation;
                let transform = Rigid {
                    rotation: mat3::mat_from_quat(&[q.w, q.x, q.y, q.z]),
                    translation: [t.x, t.y, t.z],
                };
                let (parent, child) = (&stamped.header.frame_id, &stamped.child_frame_id);
                match edges
                    .iter_mut()
                    .find(|e| e.parent == *parent && e.child == *child)
                {
                    Some(edge) => {
                        let same = (0..3).all(|a| {
                            (edge.transform.translation[a] - transform.translation[a]).abs() < 1e-9
                                && (0..3).all(|b| {
                                    (edge.transform.rotation[a][b] - transform.rotation[a][b]).abs()
                                        < 1e-9
                                })
                        });
                        edge.constant &= same;
                    }
                    None => edges.push(TfEdge {
                        parent: parent.clone(),
                        child: child.clone(),
                        transform,
                        constant: true,
                    }),
                }
            }
            Ok(())
        })?;
        Ok(TfTree { edges })
    }

    /// The fixed transform taking points in `source` to `target`, chained through the tree's constant edges (walked
    /// either way). Identity when the frames are one; an error naming the frames when no constant chain joins them.
    pub fn lookup(&self, target: &str, source: &str) -> Result<Rigid, String> {
        if target == source {
            return Ok(Rigid::identity());
        }
        // breadth-first from `source`, each reached frame holding source -> that frame
        let mut reached: Vec<(String, Rigid)> = vec![(source.to_string(), Rigid::identity())];
        let mut next = 0;
        while next < reached.len() {
            let (frame, to_frame) = reached[next].clone();
            next += 1;
            for edge in self.edges.iter().filter(|e| e.constant) {
                let (neighbour, step) = if edge.child == frame {
                    (&edge.parent, edge.transform)
                } else if edge.parent == frame {
                    (&edge.child, edge.transform.inverse())
                } else {
                    continue;
                };
                if reached.iter().any(|(f, _)| f == neighbour) {
                    continue;
                }
                let to_neighbour = step.compose(&to_frame);
                if neighbour == target {
                    return Ok(to_neighbour);
                }
                reached.push((neighbour.clone(), to_neighbour));
            }
        }
        let moving: Vec<String> = self
            .edges
            .iter()
            .filter(|e| !e.constant)
            .map(|e| format!("{}->{}", e.parent, e.child))
            .collect();
        Err(format!(
            "the recording's tf has no fixed transform from {source:?} to {target:?} \
             (its static edges: {:?}; moving edges, not used: {moving:?})",
            self.edges
                .iter()
                .filter(|e| e.constant)
                .map(|e| format!("{}->{}", e.parent, e.child))
                .collect::<Vec<_>>()
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(
        parent: &str,
        child: &str,
        rotation: Mat3,
        translation: Vec3,
        constant: bool,
    ) -> TfEdge {
        TfEdge {
            parent: parent.into(),
            child: child.into(),
            transform: Rigid {
                rotation,
                translation,
            },
            constant,
        }
    }

    #[test]
    fn lookup_chains_static_edges_either_way_and_skips_moving_ones() {
        let half: f64 = -0.384; // the Mid-360's ~44 degree pitch
        let pitch = mat3::mat_from_quat(&[half.cos(), 0.0, half.sin(), 0.0]);
        let tree = TfTree {
            edges: vec![
                edge("mid360_link", "base_link", pitch, [-0.1, 0.0, -0.3], true),
                edge(
                    "base_link",
                    "l1_link",
                    mat3::identity(),
                    [0.2, 0.0, 0.0],
                    true,
                ),
                edge("odom", "mid360_link", mat3::identity(), [0.0; 3], false),
            ],
        };
        // base_link's pose in mid360_link is the edge itself
        let up = tree.lookup("mid360_link", "base_link").unwrap();
        assert!(mat3::norm(&mat3::sub(&up.translation, &[-0.1, 0.0, -0.3])) < 1e-12);
        // points in mid360_link -> l1_link walks two edges backwards: undoing both returns the point
        let down = tree.lookup("l1_link", "mid360_link").unwrap();
        let back = tree.lookup("mid360_link", "l1_link").unwrap();
        let round_trip = down.compose(&back);
        assert!(mat3::norm(&round_trip.translation) < 1e-12);
        assert!(mat3::angular_distance(&round_trip.rotation, &mat3::identity()) < 1e-9);
        let p = [1.0, 2.0, 3.0];
        let via_base = mat3::sub(
            &mat3::mat_vec(&mat3::transpose(&pitch), &mat3::sub(&p, &[-0.1, 0.0, -0.3])),
            &[0.2, 0.0, 0.0],
        );
        let direct = mat3::add(&mat3::mat_vec(&down.rotation, &p), &down.translation);
        assert!(mat3::norm(&mat3::sub(&via_base, &direct)) < 1e-12);
        assert!(tree.lookup("odom", "base_link").is_err());
    }
}
