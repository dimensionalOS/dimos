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
//! the shared `lcm-msgs` crate. The canonical schema for each lives in Python:
//!
//! - `Graph3D`       — `dimos/navigation/jnav/msgs/Graph3D.py` (encode)
//! - `GraphDelta3D`  — `dimos/navigation/jnav/msgs/GraphDelta3D.py` (encode)
//! - `DeformationNode` — `dimos/msgs/nav_msgs/DeformationNode.py` (encode)
//!   (decode only; the PGO never publishes constraints). The decode does a
//!   tolerant tail read: `map_id`/`kind` are absent on pre-consolidation
//!   payloads and decode as `""` rather than failing.
//!
//! All formats are big-endian, custom binary (dispatched by channel-name
//! suffix, not an LCM fingerprint). Strings are `u32 len + utf-8 bytes`,
//! no terminator. Quaternions ride the wire in xyzw order.

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
