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

//! Wire-format compatibility tests against the canonical PYTHON message
//! classes: bytes encoded by `dimos_gsc_pgo::msgs` must decode identically in
//! `dimos.navigation.jnav.msgs.{Graph3D,GraphDelta3D,DeformationNode}`, and a Python-encoded
//! `LocationConstraint` (WITH the map_id/kind tail) must decode identically in
//! Rust.
//!
//! Each test shells out to the repo venv's Python (located relative to this
//! crate: 7 directory levels up) with the same environment the jnav test
//! launch uses (PYTHONPATH=<repo>:<cyclonedds shim>, PYTHONSAFEPATH=1).
//! Bytes cross the process boundary as hex on stdin/stdout.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use dimos_gsc_pgo::msgs::{
    DeformationNode, DeltaTransform, Edge, Graph3D, GraphDelta3D, LocationConstraint, Node3D,
    PoseStamped,
};

fn repo_root() -> PathBuf {
    // <repo>/dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(7)
        .expect("repo root 7 levels above the crate")
        .to_path_buf()
}

/// The repo venv's Python, or `None` when it hasn't been created (e.g. the
/// nix CI job, which builds the crate without a Python environment). Tests use
/// this to skip rather than hard-fail when the cross-language check can't run.
fn venv_python() -> Option<PathBuf> {
    let python = repo_root().join(".venv/bin/python");
    python.is_file().then_some(python)
}

/// Run a Python snippet in the repo venv, feeding `stdin` and returning stdout.
fn python(python: &Path, code: &str, stdin: &str) -> String {
    let root = repo_root();
    let mut pythonpath = root.as_os_str().to_owned();
    if let Some(home) = std::env::var_os("HOME") {
        let shim = Path::new(&home).join(".cache/cyclonedds_shim");
        if shim.is_dir() {
            pythonpath.push(":");
            pythonpath.push(shim);
        }
    }
    let mut child = Command::new(python)
        .arg("-c")
        .arg(code)
        .env("PYTHONPATH", pythonpath)
        .env("PYTHONSAFEPATH", "1")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn python");
    use std::io::Write;
    child
        .stdin
        .take()
        .expect("stdin")
        .write_all(stdin.as_bytes())
        .expect("write stdin");
    let out = child.wait_with_output().expect("wait python");
    assert!(
        out.status.success(),
        "python failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).expect("utf-8 stdout")
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(s: &str) -> Vec<u8> {
    let s = s.trim();
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex byte"))
        .collect()
}

fn sample_node(i: u64) -> Node3D {
    Node3D {
        pose: PoseStamped {
            ts: 1000.0 + i as f64 * 0.25,
            frame_id: "map".to_string(),
            position: [1.5 * i as f64, -2.0, 0.125 + i as f64],
            orientation: [0.0, 0.1, 0.2, 0.9747],
        },
        id: i,
        metadata_id: i % 2,
    }
}

#[test]
fn graph3d_decodes_in_python() {
    let graph = Graph3D {
        ts: 1234.5,
        nodes: vec![sample_node(0), sample_node(1), sample_node(2)],
        edges: vec![
            Edge {
                start_id: 0,
                end_id: 1,
                timestamp: 1000.25,
                metadata_id: 0,
            },
            Edge {
                start_id: 1,
                end_id: 2,
                timestamp: 1000.5,
                metadata_id: 0,
            },
            Edge {
                start_id: 0,
                end_id: 2,
                timestamp: 1234.5,
                metadata_id: 1,
            },
        ],
    };
    let Some(py) = venv_python() else {
        eprintln!("skipping: repo .venv/bin/python not present (cross-language check needs the Python venv)");
        return;
    };
    let out = python(
        &py,
        r#"
import sys
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
g = Graph3D.lcm_decode(bytes.fromhex(sys.stdin.read()))
print(repr(g.ts))
for n in g.nodes:
    print(n.id, n.metadata_id, repr(n.pose.ts), n.pose.frame_id,
          repr(n.pose.position.x), repr(n.pose.position.y), repr(n.pose.position.z),
          repr(n.pose.orientation.x), repr(n.pose.orientation.y),
          repr(n.pose.orientation.z), repr(n.pose.orientation.w))
for e in g.edges:
    print(e.start_id, e.end_id, repr(e.timestamp), e.metadata_id)
"#,
        &hex(&graph.encode()),
    );
    let mut lines = out.lines();
    assert_eq!(lines.next().unwrap(), "1234.5");
    for n in &graph.nodes {
        let expect = format!(
            "{} {} {:?} {} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
            n.id,
            n.metadata_id,
            n.pose.ts,
            n.pose.frame_id,
            n.pose.position[0],
            n.pose.position[1],
            n.pose.position[2],
            n.pose.orientation[0],
            n.pose.orientation[1],
            n.pose.orientation[2],
            n.pose.orientation[3],
        );
        assert_eq!(lines.next().unwrap(), expect);
    }
    for e in &graph.edges {
        let expect = format!(
            "{} {} {:?} {}",
            e.start_id, e.end_id, e.timestamp, e.metadata_id
        );
        assert_eq!(lines.next().unwrap(), expect);
    }
    assert_eq!(lines.next(), None);
}

#[test]
fn graph_delta3d_decodes_in_python() {
    let delta = GraphDelta3D {
        ts: 42.75,
        nodes: vec![sample_node(0), sample_node(1)],
        transforms: vec![
            DeltaTransform {
                translation: [0.5, -0.25, 0.0625],
                rotation: [0.0, 0.0, 0.19509, 0.98079],
            },
            DeltaTransform {
                translation: [-1.0, 2.0, -3.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        ],
    };
    let Some(py) = venv_python() else {
        eprintln!("skipping: repo .venv/bin/python not present (cross-language check needs the Python venv)");
        return;
    };
    let out = python(
        &py,
        r#"
import sys
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
d = GraphDelta3D.lcm_decode(bytes.fromhex(sys.stdin.read()))
print(repr(d.ts), len(d.nodes), len(d.transforms))
for n, t in zip(d.nodes, d.transforms):
    print(n.id, n.metadata_id, repr(n.pose.ts), n.pose.frame_id,
          repr(n.pose.position.x), repr(n.pose.position.y), repr(n.pose.position.z),
          repr(t.translation.x), repr(t.translation.y), repr(t.translation.z),
          repr(t.rotation.x), repr(t.rotation.y), repr(t.rotation.z), repr(t.rotation.w))
"#,
        &hex(&delta.encode()),
    );
    let mut lines = out.lines();
    assert_eq!(lines.next().unwrap(), "42.75 2 2");
    for (n, t) in delta.nodes.iter().zip(&delta.transforms) {
        let expect = format!(
            "{} {} {:?} {} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
            n.id,
            n.metadata_id,
            n.pose.ts,
            n.pose.frame_id,
            n.pose.position[0],
            n.pose.position[1],
            n.pose.position[2],
            t.translation[0],
            t.translation[1],
            t.translation[2],
            t.rotation[0],
            t.rotation[1],
            t.rotation[2],
            t.rotation[3],
        );
        assert_eq!(lines.next().unwrap(), expect);
    }
    assert_eq!(lines.next(), None);
}

#[test]
fn deformation_node_decodes_in_python_and_tf_id_agrees() {
    let node = DeformationNode {
        id: 0xDEADBEEF12345678,
        tf_id: dimos_gsc_pgo::msgs::tf_id_for("map", "odom"),
        pose: PoseStamped {
            ts: 987.625,
            frame_id: "map".to_string(),
            position: [10.5, -20.25, 0.75],
            orientation: [0.1, 0.2, 0.3, 0.9273],
        },
    };
    let Some(py) = venv_python() else {
        eprintln!("skipping: repo .venv/bin/python not present (cross-language check needs the Python venv)");
        return;
    };
    let out = python(
        &py,
        r#"
import sys
from dimos.navigation.jnav.msgs.DeformationNode import DeformationNode, tf_id_for
n = DeformationNode.lcm_decode(bytes.fromhex(sys.stdin.read()))
print(n.id, n.tf_id, repr(n.pose.ts), n.pose.frame_id,
      repr(n.pose.position.x), repr(n.pose.position.y), repr(n.pose.position.z),
      repr(n.pose.orientation.x), repr(n.pose.orientation.y),
      repr(n.pose.orientation.z), repr(n.pose.orientation.w))
print(tf_id_for("map", "odom"))
"#,
        &hex(&node.encode()),
    );
    let mut lines = out.lines();
    let expect = format!(
        "{} {} {:?} {} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
        node.id,
        node.tf_id,
        node.pose.ts,
        node.pose.frame_id,
        node.pose.position[0],
        node.pose.position[1],
        node.pose.position[2],
        node.pose.orientation[0],
        node.pose.orientation[1],
        node.pose.orientation[2],
        node.pose.orientation[3],
    );
    assert_eq!(lines.next().unwrap(), expect);
    // tf_id_for must agree across languages, or consumers filter out our nodes.
    assert_eq!(lines.next().unwrap(), node.tf_id.to_string());
}

#[test]
fn location_constraint_python_encode_decodes_in_rust() {
    let Some(py) = venv_python() else {
        eprintln!("skipping: repo .venv/bin/python not present (cross-language check needs the Python venv)");
        return;
    };
    let out = python(
        &py,
        r#"
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
pose = Pose()
pose.position = Vector3(1.25, -2.5, 3.75)
pose.orientation = Quaternion(0.0, 0.0, 0.19509, 0.98079)
cov = [0.0] * 36
for i in range(6):
    cov[i * 6 + i] = 0.5 + i
c = LocationConstraint(
    to_id="apriltag://36h11/40cm/5",
    frame_id="base_link",
    pose=pose,
    covariance=cov,
    constraint_instance_id="instance-7",
    map_id="map0",
    kind="apriltag",
    ts=1717.125,
)
print(c.lcm_encode().hex())
"#,
        "",
    );
    let bytes = unhex(&out);
    let c = LocationConstraint::decode(&bytes).expect("rust decode");
    assert_eq!(c.ts, 1717.125);
    assert_eq!(c.to_id, "apriltag://36h11/40cm/5");
    assert_eq!(c.frame_id, "base_link");
    assert_eq!(c.constraint_instance_id, "instance-7");
    assert_eq!(c.map_id, "map0");
    assert_eq!(c.kind, "apriltag");
    assert_eq!(c.position, [1.25, -2.5, 3.75]);
    assert_eq!(c.orientation, [0.0, 0.0, 0.19509, 0.98079]);
    for i in 0..6 {
        assert_eq!(c.covariance[i * 6 + i], 0.5 + i as f64);
    }
    assert_eq!(c.covariance[1], 0.0);
}
