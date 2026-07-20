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

//! jnav custom LCM wire formats the PGO module speaks that have no binding in
//! the shared `lcm-msgs` crate. Each is a byte-for-byte port of the Python
//! implementation (the canonical schema):
//!
//! - `Graph3D`       — `dimos/navigation/jnav/msgs/Graph3D.py` (encode)
//! - `GraphDelta3D`  — `dimos/navigation/jnav/msgs/GraphDelta3D.py` (encode)
//! - `DeformationNode` — `dimos/msgs/nav_msgs/DeformationNode.py` (encode)
//! - `LocationConstraint` — `dimos/navigation/jnav/msgs/LocationConstraint.py`
//!   (decode only; the PGO never publishes constraints). The decode mirrors
//!   the C++ helper at gsc_pgo/msgs/LocationConstraint.hpp exactly, including
//!   the tolerant tail read: `map_id`/`kind` are absent on pre-consolidation
//!   payloads and must decode as `""` rather than failing.
//!
//! All formats are big-endian, custom binary (dispatched by channel-name
//! suffix, not an LCM fingerprint). Strings are `u32 len + utf-8 bytes`,
//! no terminator. Quaternions ride the wire in xyzw order.

use std::io;

// ---- byte writers/readers ----------------------------------------------------

fn write_u32_be(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_be_bytes());
}

fn write_u64_be(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_be_bytes());
}

fn write_f64_be(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_be_bytes());
}

fn write_str(out: &mut Vec<u8>, s: &str) {
    write_u32_be(out, s.len() as u32);
    out.extend_from_slice(s.as_bytes());
}

struct Reader<'a> {
    buf: &'a [u8],
    off: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Reader { buf, off: 0 }
    }

    fn remaining(&self) -> usize {
        self.buf.len() - self.off
    }

    fn take(&mut self, n: usize) -> io::Result<&'a [u8]> {
        if self.remaining() < n {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                format!("need {n} bytes, have {}", self.remaining()),
            ));
        }
        let s = &self.buf[self.off..self.off + n];
        self.off += n;
        Ok(s)
    }

    fn f64(&mut self) -> io::Result<f64> {
        Ok(f64::from_be_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn u32(&mut self) -> io::Result<u32> {
        Ok(u32::from_be_bytes(self.take(4)?.try_into().unwrap()))
    }

    fn string(&mut self) -> io::Result<String> {
        let len = self.u32()? as usize;
        let bytes = self.take(len)?;
        String::from_utf8(bytes.to_vec()).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }
}

// ---- shared node pieces --------------------------------------------------------

/// The pose block shared by Graph3D / GraphDelta3D nodes and DeformationNode:
/// `double ts, u32 frame_id_len, frame_id, 7×double pos_xyz + quat_xyzw`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PoseStamped {
    pub ts: f64,
    pub frame_id: String,
    /// x, y, z
    pub position: [f64; 3],
    /// x, y, z, w
    pub orientation: [f64; 4],
}

impl PoseStamped {
    fn write(&self, out: &mut Vec<u8>) {
        write_f64_be(out, self.ts);
        write_str(out, &self.frame_id);
        for v in self.position {
            write_f64_be(out, v);
        }
        for v in self.orientation {
            write_f64_be(out, v);
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Node3D {
    pub pose: PoseStamped,
    pub id: u64,
    pub metadata_id: u64,
}

impl Node3D {
    fn write(&self, out: &mut Vec<u8>) {
        self.pose.write(out);
        write_u64_be(out, self.id);
        write_u64_be(out, self.metadata_id);
    }
}

// ---- Graph3D -------------------------------------------------------------------

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Edge {
    pub start_id: u64,
    pub end_id: u64,
    pub timestamp: f64,
    pub metadata_id: u64,
}

/// Wire: `u64 edge_count, u64 node_count, f64 ts, nodes[], edges[]`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Graph3D {
    pub ts: f64,
    pub nodes: Vec<Node3D>,
    pub edges: Vec<Edge>,
}

impl Graph3D {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(24 + self.nodes.len() * 100 + self.edges.len() * 32);
        write_u64_be(&mut out, self.edges.len() as u64);
        write_u64_be(&mut out, self.nodes.len() as u64);
        write_f64_be(&mut out, self.ts);
        for n in &self.nodes {
            n.write(&mut out);
        }
        for e in &self.edges {
            write_u64_be(&mut out, e.start_id);
            write_u64_be(&mut out, e.end_id);
            write_f64_be(&mut out, e.timestamp);
            write_u64_be(&mut out, e.metadata_id);
        }
        out
    }
}

// ---- GraphDelta3D ----------------------------------------------------------------

/// SE(3) delta: `post_pose = transform * node.pose` (left-multiply).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DeltaTransform {
    /// x, y, z
    pub translation: [f64; 3],
    /// x, y, z, w
    pub rotation: [f64; 4],
}

/// Wire: `u64 node_count, f64 ts, nodes[], transforms[]` — two aligned arrays.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GraphDelta3D {
    pub ts: f64,
    pub nodes: Vec<Node3D>,
    pub transforms: Vec<DeltaTransform>,
}

impl GraphDelta3D {
    pub fn encode(&self) -> Vec<u8> {
        debug_assert_eq!(self.nodes.len(), self.transforms.len());
        let mut out = Vec::with_capacity(16 + self.nodes.len() * (100 + 56));
        write_u64_be(&mut out, self.nodes.len() as u64);
        write_f64_be(&mut out, self.ts);
        for n in &self.nodes {
            n.write(&mut out);
        }
        for t in &self.transforms {
            for v in t.translation {
                write_f64_be(&mut out, v);
            }
            for v in t.rotation {
                write_f64_be(&mut out, v);
            }
        }
        out
    }
}

// ---- DeformationNode -------------------------------------------------------------

/// 64-bit FNV-1a hash (utf-8). Must match DeformationNode.py's `fnv1a_64` so
/// tf_id filtering agrees across the wire.
pub fn fnv1a_64(text: &str) -> u64 {
    const OFFSET_BASIS: u64 = 0xCBF29CE484222325;
    const PRIME: u64 = 0x100000001B3;
    let mut digest = OFFSET_BASIS;
    for byte in text.as_bytes() {
        digest = (digest ^ u64::from(*byte)).wrapping_mul(PRIME);
    }
    digest
}

/// The tf_id for a transform edge: `fnv1a_64(frame_from + "|" + frame_to)`.
pub fn tf_id_for(frame_from: &str, frame_to: &str) -> u64 {
    fnv1a_64(&format!("{frame_from}|{frame_to}"))
}

/// Wire: `u64 id, u64 tf_id, f64 pose_ts, u32 frame_id_len, frame_id, 7×f64`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DeformationNode {
    pub id: u64,
    pub tf_id: u64,
    pub pose: PoseStamped,
}

impl DeformationNode {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(20 + self.pose.frame_id.len() + 56);
        write_u64_be(&mut out, self.id);
        write_u64_be(&mut out, self.tf_id);
        self.pose.write(&mut out);
        out
    }
}

// ---- LocationConstraint (decode only) ----------------------------------------------

/// Wire (see LocationConstraint.py lcm_encode):
/// `f64 ts; str to_id, frame_id, constraint_instance_id; 7×f64 pose
/// (pos_xyz + quat_xyzw); 36×f64 covariance (row-major 6x6, tangent order
/// [rot(3), trans(3)]); TAIL str map_id, kind (absent pre-consolidation ->
/// decode as "").`
#[derive(Debug, Clone, PartialEq)]
pub struct LocationConstraint {
    pub ts: f64,
    pub to_id: String,
    pub frame_id: String,
    pub constraint_instance_id: String,
    pub map_id: String,
    pub kind: String,
    /// x, y, z
    pub position: [f64; 3],
    /// x, y, z, w
    pub orientation: [f64; 4],
    pub covariance: [f64; 36],
}

impl LocationConstraint {
    pub fn decode(data: &[u8]) -> io::Result<Self> {
        let mut r = Reader::new(data);
        let ts = r.f64()?;
        let to_id = r.string()?;
        let frame_id = r.string()?;
        let constraint_instance_id = r.string()?;
        let position = [r.f64()?, r.f64()?, r.f64()?];
        let orientation = [r.f64()?, r.f64()?, r.f64()?, r.f64()?];
        let mut covariance = [0.0f64; 36];
        for v in covariance.iter_mut() {
            *v = r.f64()?;
        }
        // Tail fields (map_id, kind): tolerate their absence (older payloads)
        // exactly like msgs/LocationConstraint.hpp — a missing length prefix
        // means "", but a length prefix promising more bytes than remain is a
        // hard decode error.
        let mut tail = [String::new(), String::new()];
        for field in tail.iter_mut() {
            if r.remaining() < 4 {
                continue;
            }
            *field = r.string()?;
        }
        let [map_id, kind] = tail;
        Ok(LocationConstraint {
            ts,
            to_id,
            frame_id,
            constraint_instance_id,
            map_id,
            kind,
            position,
            orientation,
            covariance,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a_matches_python_reference() {
        // From DeformationNode.py: tf_id_for("map", "odom").
        assert_eq!(tf_id_for("map", "odom"), 3863314957514174452);
    }

    #[test]
    fn location_constraint_round_trip_with_tail() {
        // Hand-built payload following LocationConstraint.py lcm_encode.
        let mut buf = Vec::new();
        write_f64_be(&mut buf, 123.5);
        for s in ["apriltag://36h11/40cm/5", "base_link", "inst-1"] {
            write_str(&mut buf, s);
        }
        for v in [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0] {
            write_f64_be(&mut buf, v);
        }
        for i in 0..36 {
            write_f64_be(&mut buf, i as f64 * 0.5);
        }
        let head_len = buf.len();
        for s in ["map0", "apriltag"] {
            write_str(&mut buf, s);
        }

        let c = LocationConstraint::decode(&buf).unwrap();
        assert_eq!(c.ts, 123.5);
        assert_eq!(c.to_id, "apriltag://36h11/40cm/5");
        assert_eq!(c.frame_id, "base_link");
        assert_eq!(c.constraint_instance_id, "inst-1");
        assert_eq!(c.map_id, "map0");
        assert_eq!(c.kind, "apriltag");
        assert_eq!(c.position, [1.0, 2.0, 3.0]);
        assert_eq!(c.orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(c.covariance[35], 17.5);

        // Pre-consolidation payload (no tail) -> "" for map_id/kind.
        let old = LocationConstraint::decode(&buf[..head_len]).unwrap();
        assert_eq!(old.map_id, "");
        assert_eq!(old.kind, "");
        assert_eq!(old.to_id, c.to_id);
    }
}
