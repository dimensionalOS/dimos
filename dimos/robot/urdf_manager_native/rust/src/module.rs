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

//! UrdfManager as a native module: poses the URDF from the joint state onto tf, and serves the URDF and the files it
//! names to renderers.

use std::collections::{BTreeSet, HashMap};
use std::io;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_module::{native_config, Input, Module, Output, Tf, Transform};
use lcm_msgs::sensor_msgs::JointState;
use tracing::{error, warn};

use crate::urdf::{files_named_by, joint_transforms, parse_joints, Joint};

// np.allclose's defaults, which UrdfManager compares an edge to the one last sent with.
const UNCHANGED_ATOL: f64 = 1e-6;
const UNCHANGED_RTOL: f64 = 1e-5;

#[native_config]
#[derive(Clone)]
pub struct Config {
    // The URDF as RobotModel materializes it (xacro expanded, package:// and relative paths resolved).
    pub urdf_xml: String,
    // Every single-DOF and fixed URDF joint as a parent -> child tf edge; off where another module owns those frames.
    pub publish_joint_tf: bool,
    // An edge goes out when it moves, at most this often...
    #[validate(range(exclusive_min = 0.0))]
    pub joint_tf_hz: f64,
    // ...and otherwise at this keepalive rate.
    #[validate(range(exclusive_min = 0.0))]
    pub unchanged_tf_hz: f64,
}

/// One file's bytes as found on disk, named by the URI it was requested by (`dimos.msgs.files.File`).
#[derive(Clone, Debug, PartialEq)]
pub struct File {
    pub uri: String,
    pub data: Vec<u8>,
    pub ts: f64,
}

impl File {
    /// Little-endian f64 stamp and u32 URI length, then the URI, then the bytes.
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(12 + self.uri.len() + self.data.len());
        buf.extend_from_slice(&self.ts.to_le_bytes());
        buf.extend_from_slice(&(self.uri.len() as u32).to_le_bytes());
        buf.extend_from_slice(self.uri.as_bytes());
        buf.extend_from_slice(&self.data);
        buf
    }

    pub fn decode(data: &[u8]) -> io::Result<Self> {
        let invalid = |what: &str| io::Error::new(io::ErrorKind::InvalidData, what.to_string());
        let header = data
            .get(..12)
            .ok_or_else(|| invalid("File shorter than its header"))?;
        let ts = f64::from_le_bytes(header[..8].try_into().unwrap());
        let uri_length = u32::from_le_bytes(header[8..].try_into().unwrap()) as usize;
        let uri = data
            .get(12..12 + uri_length)
            .ok_or_else(|| invalid("File shorter than its URI"))?;
        Ok(Self {
            uri: String::from_utf8(uri.to_vec()).map_err(|_| invalid("File URI is not UTF-8"))?,
            data: data[12 + uri_length..].to_vec(),
            ts,
        })
    }
}

/// Per edge, the last transform sent (translation + rotation) and when.
#[derive(Default)]
pub struct SentEdges(HashMap<(String, String), ([f64; 7], Instant)>);

impl SentEdges {
    /// The edges that moved since they were last sent, and those not sent for one keepalive period.
    pub fn due(
        &mut self,
        transforms: Vec<Transform>,
        now: Instant,
        keepalive: Duration,
    ) -> Vec<Transform> {
        transforms
            .into_iter()
            .filter(|transform| {
                let (t, q) = (transform.translation(), transform.rotation());
                let value = [t.x, t.y, t.z, q.i, q.j, q.k, q.w];
                let key = (transform.parent.clone(), transform.child.clone());
                let is_due = self.0.get(&key).is_none_or(|(sent, sent_at)| {
                    now.duration_since(*sent_at) >= keepalive
                        || value
                            .iter()
                            .zip(sent)
                            .any(|(a, b)| (a - b).abs() > UNCHANGED_ATOL + UNCHANGED_RTOL * b.abs())
                });
                if is_due {
                    self.0.insert(key, (value, now));
                }
                is_due
            })
            .collect()
    }
}

#[derive(Module)]
#[module(name = "urdf_manager", setup = load_urdf)]
pub struct UrdfManager {
    // The joints it poses are published on it.
    #[tf]
    tf: Tf,

    #[input(decode = JointState::decode)]
    coordinator_joint_state: Input<JointState>,

    // A request for a *.urdf gets the URDF; one for a file the URDF names by its full path gets that file. Either
    // goes out on file_info named as requested.
    #[input(decode = lcm_msgs::std_msgs::String::decode)]
    render_file_request: Input<lcm_msgs::std_msgs::String>,

    #[output(encode = File::encode)]
    file_info: Output<File>,

    #[config]
    config: Config,

    joints: Vec<Joint>,
    file_paths: BTreeSet<String>,
    next_joint_tf_at: Option<Instant>,
    sent_edges: SentEdges,
}

impl UrdfManager {
    async fn load_urdf(&mut self) {
        let parsed = parse_joints(&self.config.urdf_xml)
            .and_then(|joints| Ok((joints, files_named_by(&self.config.urdf_xml)?)));
        let (joints, file_paths) = match parsed {
            Ok(parsed) => parsed,
            Err(e) => {
                error!("cannot load the URDF: {e}");
                std::process::exit(1);
            }
        };
        for joint in &joints {
            if !joint.is_multi_dof() && joint.parent_from_child(0.0).is_none() {
                warn!(
                    "{} joint {} has no single position, so it is never published on tf",
                    joint.joint_type, joint.name
                );
            }
        }
        self.joints = joints;
        self.file_paths = file_paths;
    }

    async fn handle_render_file_request(&mut self, request: lcm_msgs::std_msgs::String) {
        let name = request.data;
        if name.ends_with(".urdf") {
            let data = self.config.urdf_xml.as_bytes().to_vec();
            self.publish_file(name, data).await;
            return;
        }
        // Only files the URDF names: a request is never an arbitrary path into the filesystem.
        if !self.file_paths.contains(&name) {
            warn!("render_file_request for {name:?}, which is not a file the URDF names");
            return;
        }
        match tokio::fs::read(&name).await {
            Ok(data) => self.publish_file(name, data).await,
            Err(e) => error!("cannot read {name:?}, which the URDF names: {e}"),
        }
    }

    async fn publish_file(&self, uri: String, data: Vec<u8>) {
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0.0, |d| d.as_secs_f64());
        let file = File { uri, data, ts };
        if let Err(e) = self.file_info.publish(&file).await {
            error!("cannot publish {:?} on file_info: {e}", file.uri);
        }
    }

    async fn handle_coordinator_joint_state(&mut self, state: JointState) {
        let now = Instant::now();
        if !self.config.publish_joint_tf || self.next_joint_tf_at.is_some_and(|next| now < next) {
            return;
        }
        self.next_joint_tf_at = Some(now + Duration::from_secs_f64(1.0 / self.config.joint_tf_hz));
        let stamp = &state.header.stamp;
        let ts = stamp.sec as f64 + stamp.nsec as f64 * 1e-9;
        let transforms = joint_transforms(&self.joints, &state.name, &state.position, ts);
        let keepalive = Duration::from_secs_f64(1.0 / self.config.unchanged_tf_hz);
        let due = self.sent_edges.due(transforms, now, keepalive);
        if !due.is_empty() {
            if let Err(e) = self.tf.publish(&due).await {
                error!("cannot publish joint tf: {e}");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_module::nalgebra::Isometry3;

    fn edge(child: &str, x: f64) -> Transform {
        Transform::new("base", child, 0.0, Isometry3::translation(x, 0.0, 0.0))
    }

    fn children(transforms: &[Transform]) -> Vec<&str> {
        transforms.iter().map(|t| t.child.as_str()).collect()
    }

    #[test]
    fn file_encoding_matches_the_python_layout() {
        let file = File {
            uri: "/a.stl".to_string(),
            data: b"solid".to_vec(),
            ts: 1.5,
        };

        let encoded = file.encode();

        let mut expected = 1.5f64.to_le_bytes().to_vec();
        expected.extend_from_slice(&6u32.to_le_bytes());
        expected.extend_from_slice(b"/a.stlsolid");
        assert_eq!(encoded, expected);
        assert_eq!(File::decode(&encoded).unwrap(), file);
    }

    #[test]
    fn an_edge_goes_out_when_it_moves_or_its_keepalive_lapses() {
        let mut sent = SentEdges::default();
        let start = Instant::now();
        let keepalive = Duration::from_millis(250);

        assert_eq!(
            children(&sent.due(vec![edge("a", 0.0), edge("b", 0.0)], start, keepalive)),
            ["a", "b"]
        );
        // within tolerance and keepalive: nothing; a moved edge: just that one
        let later = start + Duration::from_millis(100);
        assert!(sent
            .due(vec![edge("a", 5e-7), edge("b", 0.0)], later, keepalive)
            .is_empty());
        assert_eq!(
            children(&sent.due(vec![edge("a", 0.0), edge("b", 0.01)], later, keepalive)),
            ["b"]
        );
        // a's keepalive lapsed (sent at start), b's has not (sent at 100 ms)
        let lapsed = start + Duration::from_millis(300);
        assert_eq!(
            children(&sent.due(vec![edge("a", 0.0), edge("b", 0.01)], lapsed, keepalive)),
            ["a"]
        );
    }
}
