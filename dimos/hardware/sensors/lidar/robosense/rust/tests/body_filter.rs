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

//! The module's body filter against the Python one (RoboPlan) on real M20 scans, and on a synthetic body cloud.
//! Fixtures (tests/fixtures/make_body_fixtures.py): m20_body_scans.bin = 4 scans (one per lidar and pose) of recording 20261120_163604 (every
//! body point, a sample of the rest) with RoboPlan's mask, at the standing pose and at random joint states;
//! m20_synthetic_body.bin = points inside every collision primitive at the standing pose, in each lidar's frame.

use std::collections::HashMap;

use dimos_robosense::body_cut::{BodyCut, CutConfig, JointSource};
use robosense_airy::{Point, Scan};

const URDF: &str = include_str!("fixtures/m20_collision.urdf");
const SCANS: &[u8] = include_bytes!("fixtures/m20_body_scans.bin");
const SYNTHETIC: &[u8] = include_bytes!("fixtures/m20_synthetic_body.bin");

struct Record {
    frame: String,
    stamp: f64,
    joints: HashMap<String, f64>,
    points: Vec<[f32; 3]>,
    on_body: Vec<bool>,
}

fn take<const N: usize>(bytes: &[u8], at: &mut usize) -> [u8; N] {
    let out = bytes[*at..*at + N].try_into().unwrap();
    *at += N;
    out
}

fn string(bytes: &[u8], at: &mut usize) -> String {
    let len = u32::from_le_bytes(take(bytes, at)) as usize;
    let out = String::from_utf8(bytes[*at..*at + len].to_vec()).unwrap();
    *at += len;
    out
}

fn records(bytes: &[u8]) -> Vec<Record> {
    let (mut at, mut out) = (0, Vec::new());
    while at < bytes.len() {
        let frame = string(bytes, &mut at);
        let stamp = f64::from_le_bytes(take(bytes, &mut at));
        let joint_count = u32::from_le_bytes(take(bytes, &mut at));
        let joints = (0..joint_count)
            .map(|_| {
                (
                    string(bytes, &mut at),
                    f64::from_le_bytes(take(bytes, &mut at)),
                )
            })
            .collect();
        let n = u32::from_le_bytes(take(bytes, &mut at)) as usize;
        let points = (0..n)
            .map(|_| [0; 3].map(|_| f32::from_le_bytes(take(bytes, &mut at))))
            .collect();
        let on_body = bytes[at..at + n].iter().map(|&b| b == 1).collect();
        at += n;
        out.push(Record {
            frame,
            stamp,
            joints,
            points,
            on_body,
        });
    }
    out
}

// The M20's lidar mounts (dimos/robot/deeprobotics/m20/constants.py).
fn mount(frame: &str) -> urdf_body_filter::Transform {
    let (x, yaw) = match frame {
        "front_lidar" => (0.32028, std::f64::consts::PI),
        "rear_lidar" => (-0.32028, 0.0),
        other => panic!("unknown frame {other}"),
    };
    urdf_body_filter::Transform::from_xyz_rpy(
        [x, 0.0, -0.013],
        [0.0, -std::f64::consts::FRAC_PI_2, yaw],
    )
}

fn cut(record: &Record) -> (Scan, Scan) {
    let mut cutter = BodyCut::new(CutConfig {
        urdf: URDF.into(),
        padding_m: 0.01,
        links: None,
        source: JointSource::Fixed,
        fixed_positions: record.joints.clone(),
        state_tolerance_s: 0.02,
        mount_parent: "base_link".into(),
        root_from_lidar: mount(&record.frame),
    })
    .unwrap();
    let points = record
        .points
        .iter()
        .enumerate()
        .map(|(i, &[x, y, z])| Point {
            x,
            y,
            z,
            intensity: i as u8,
            ring: i as u16,
            time: record.stamp,
            feature: 0,
        })
        .collect();
    let scan = Scan {
        seq: 3,
        timestamp: record.stamp,
        rings: 96,
        points,
        complete: true,
    };
    let mut kept = scan.clone();
    assert!(cutter.cut(&mut kept));
    (scan, kept)
}

#[test]
fn real_scans_lose_exactly_the_points_the_python_filter_removes() {
    let records = records(SCANS);
    assert_eq!(records.len(), 4);
    let mut removed = 0;
    for record in &records {
        let (scan, kept) = cut(record);
        let expected: Vec<Point> = scan
            .points
            .iter()
            .zip(&record.on_body)
            .filter(|(_, &body)| !body)
            .map(|(p, _)| *p)
            .collect();
        assert_eq!(
            kept.points, expected,
            "{} at {}",
            record.frame, record.stamp
        );
        assert_eq!(
            (kept.seq, kept.timestamp, kept.rings),
            (scan.seq, scan.timestamp, scan.rings)
        );
        removed += scan.points.len() - kept.points.len();
    }
    // both lidars, standing and moved legs: thousands of body points, every one matched
    assert!(removed > 1_500, "{removed}");
}

#[test]
fn a_synthetic_body_cloud_is_removed_entirely() {
    let records = records(SYNTHETIC);
    assert_eq!(records.len(), 2);
    for record in &records {
        assert!(record.points.len() > 2_000);
        let (_, kept) = cut(record);
        assert!(
            kept.points.is_empty(),
            "{}: {} body points kept",
            record.frame,
            kept.points.len()
        );
    }
}
