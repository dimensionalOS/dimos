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

//! The robot's own body cut out of each scan before it is published (urdf-body-filter), at a fixed joint pose
//! or at the joint state nearest the scan's stamp, with the URDF placed by the lidar's mount.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use lcm_msgs::sensor_msgs::JointState;
use robosense_airy::Scan;
use serde::{Deserialize, Serialize};
use urdf_body_filter::{Body, Options, Transform};

/// Where the joint positions come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum JointSource {
    /// The configured pose, for every scan.
    Fixed,
    /// The `joint_state` input, the sample nearest each scan's stamp.
    JointState,
}

/// The joint states received, oldest first, kept for `history_s` behind the newest.
#[derive(Debug, Default)]
pub struct JointHistory {
    states: VecDeque<(f64, HashMap<String, f64>)>,
}

impl JointHistory {
    /// Buffer one state (a repeated stamp replaces the old one); None when malformed (names vs positions, non-finite).
    pub fn add(&mut self, state: &JointState, history_s: f64) -> Option<()> {
        let ts = state.header.stamp.sec as f64 + state.header.stamp.nsec as f64 * 1e-9;
        if !ts.is_finite() || state.name.len() != state.position.len() {
            return None;
        }
        let positions: HashMap<String, f64> = state
            .name
            .iter()
            .cloned()
            .zip(state.position.iter().copied())
            .collect();
        if positions.len() != state.name.len() || positions.values().any(|p| !p.is_finite()) {
            return None;
        }
        self.states.retain(|(t, _)| *t != ts);
        let at = self.states.partition_point(|(t, _)| *t < ts);
        self.states.insert(at, (ts, positions));
        let newest = self.states.back().map_or(ts, |(t, _)| *t);
        while self
            .states
            .front()
            .is_some_and(|(t, _)| *t < newest - history_s)
        {
            self.states.pop_front();
        }
        Some(())
    }

    /// The state nearest `stamp`, if one is within `tolerance_s` (a tie takes the later one).
    pub fn nearest(&self, stamp: f64, tolerance_s: f64) -> Option<&HashMap<String, f64>> {
        let at = self.states.partition_point(|(t, _)| *t < stamp);
        let before = at.checked_sub(1).and_then(|i| self.states.get(i));
        let after = self.states.get(at);
        let best = match (before, after) {
            (Some(b), Some(a)) => {
                if (a.0 - stamp).abs() <= (stamp - b.0).abs() {
                    a
                } else {
                    b
                }
            }
            (Some(b), None) => b,
            (None, Some(a)) => a,
            (None, None) => return None,
        };
        ((best.0 - stamp).abs() <= tolerance_s).then_some(&best.1)
    }
}

/// What the cutter needs, taken from the module config.
#[derive(Debug, Clone)]
pub struct CutConfig {
    pub urdf: String,
    pub padding_m: f64,
    pub links: Option<Vec<String>>,
    pub source: JointSource,
    pub fixed_positions: HashMap<String, f64>,
    pub state_tolerance_s: f64,
    /// The lidar's mount: its parent frame (must be the URDF root) and its pose there.
    pub mount_parent: String,
    pub root_from_lidar: Transform,
}

/// Running totals for the summary log line.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct CutStats {
    pub scans: u64,
    pub dropped: u64,
    pub points_in: u64,
    pub points_removed: u64,
    pub total_ms: f64,
    pub max_ms: f64,
}

/// Cuts the body out of scans; one per lidar.
pub struct BodyCut {
    body: Body,
    config: CutConfig,
    pub history: Arc<Mutex<JointHistory>>,
    pub stats: CutStats,
}

impl BodyCut {
    pub fn new(config: CutConfig) -> Result<BodyCut, String> {
        let body = Body::from_urdf(
            &config.urdf,
            &Options {
                padding_m: config.padding_m,
                links: config.links.clone(),
            },
        )
        .map_err(|e| e.to_string())?;
        if body.root_link() != config.mount_parent {
            return Err(format!(
                "the body filter needs the lidar mounted in the URDF root {:?}, not {:?}",
                body.root_link(),
                config.mount_parent
            ));
        }
        if config.source == JointSource::Fixed {
            // a fixed pose that cannot pose the body is a config error, not a per-scan drop
            body.pose(
                |j| config.fixed_positions.get(j).copied(),
                &config.root_from_lidar,
            )
            .map_err(|e| e.to_string())?;
        }
        Ok(BodyCut {
            body,
            config,
            history: Arc::default(),
            stats: CutStats::default(),
        })
    }

    pub fn body(&self) -> &Body {
        &self.body
    }

    /// Drop the scan's body points in place; false when there is no joint state to pose the body at the scan's stamp.
    pub fn cut(&mut self, scan: &mut Scan) -> bool {
        let start = Instant::now();
        let posed = match self.config.source {
            JointSource::Fixed => self.body.pose(
                |j| self.config.fixed_positions.get(j).copied(),
                &self.config.root_from_lidar,
            ),
            JointSource::JointState => {
                let history = self
                    .history
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner);
                match history.nearest(scan.timestamp, self.config.state_tolerance_s) {
                    Some(positions) => self
                        .body
                        .pose(|j| positions.get(j).copied(), &self.config.root_from_lidar),
                    None => Err(urdf_body_filter::Error::MissingJoint(
                        "(no joint state near the scan)".into(),
                    )),
                }
            }
        };
        self.stats.scans += 1;
        let Ok(posed) = posed else {
            self.stats.dropped += 1;
            return false;
        };
        self.stats.points_in += scan.points.len() as u64;
        self.stats.points_removed +=
            posed.retain(&mut scan.points, |p| [p.x as f64, p.y as f64, p.z as f64]) as u64;
        let ms = start.elapsed().as_secs_f64() * 1e3;
        self.stats.total_ms += ms;
        self.stats.max_ms = self.stats.max_ms.max(ms);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lcm_msgs::std_msgs::{Header, Time};
    use robosense_airy::Point;

    const URDF: &str = r#"<robot name="r"><link name="base_link"><collision><geometry><box size="0.4 0.4 0.4"/></geometry></collision></link>
        <link name="leg"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
        <joint name="hip" type="prismatic"><parent link="base_link"/><child link="leg"/><axis xyz="1 0 0"/></joint></robot>"#;

    fn state(ts: f64, hip: f64) -> JointState {
        JointState {
            header: Header {
                seq: 0,
                stamp: Time {
                    sec: ts.floor() as i32,
                    nsec: ((ts - ts.floor()) * 1e9).round() as i32,
                },
                frame_id: String::new(),
            },
            name: vec!["hip".into()],
            position: vec![hip],
            velocity: vec![],
            effort: vec![],
        }
    }

    fn scan(ts: f64, xs: &[f32]) -> Scan {
        let points = xs
            .iter()
            .map(|&x| Point {
                x,
                y: 0.0,
                z: 0.0,
                intensity: 1,
                ring: 2,
                time: ts,
                feature: 0,
            })
            .collect();
        Scan {
            seq: 7,
            timestamp: ts,
            rings: 96,
            points,
            complete: true,
        }
    }

    fn cutter(source: JointSource) -> BodyCut {
        BodyCut::new(CutConfig {
            urdf: URDF.into(),
            padding_m: 0.0,
            links: None,
            source,
            fixed_positions: HashMap::from([("hip".to_string(), 1.0)]),
            state_tolerance_s: 0.02,
            mount_parent: "base_link".into(),
            root_from_lidar: Transform::IDENTITY,
        })
        .unwrap()
    }

    #[test]
    fn fixed_pose_cuts_the_body_and_keeps_the_rest_in_order() {
        let mut cut = cutter(JointSource::Fixed);
        let mut s = scan(5.0, &[3.0, 0.1, 1.05, -2.0, 0.0]);
        assert!(cut.cut(&mut s));
        let xs: Vec<f32> = s.points.iter().map(|p| p.x).collect();
        assert_eq!(xs, [3.0, -2.0]);
        assert_eq!((s.seq, s.timestamp), (7, 5.0));
        assert_eq!(cut.stats.points_removed, 3);
    }

    #[test]
    fn joint_state_is_matched_at_the_scan_stamp() {
        let mut cut = cutter(JointSource::JointState);
        {
            let mut history = cut.history.lock().unwrap();
            history.add(&state(10.0, 0.0), 5.0).unwrap();
            history.add(&state(10.1, 2.0), 5.0).unwrap();
        }
        // at 10.01 the leg sits at x=0 (inside the trunk), so a point at x=2 stays
        let mut s = scan(10.01, &[2.0]);
        assert!(cut.cut(&mut s));
        assert_eq!(s.points.len(), 1);
        // at 10.095 the leg is at x=2
        let mut s = scan(10.095, &[2.0]);
        assert!(cut.cut(&mut s));
        assert!(s.points.is_empty());
        // 50 ms from any state: dropped, counted
        let mut s = scan(10.05, &[2.0]);
        assert!(!cut.cut(&mut s));
        assert_eq!((cut.stats.scans, cut.stats.dropped), (3, 1));
    }

    #[test]
    fn history_prunes_and_refuses_malformed_states() {
        let mut history = JointHistory::default();
        history.add(&state(1.0, 0.0), 5.0).unwrap();
        history.add(&state(7.0, 0.0), 5.0).unwrap();
        assert!(history.nearest(1.0, 0.5).is_none());
        let mut bad = state(7.5, 0.0);
        bad.position.push(1.0);
        assert!(history.add(&bad, 5.0).is_none());
        let mut nan = state(7.5, f64::NAN);
        nan.name = vec!["hip".into()];
        assert!(history.add(&nan, 5.0).is_none());
    }

    #[test]
    fn a_mount_outside_the_urdf_root_is_refused() {
        let error = BodyCut::new(CutConfig {
            mount_parent: "odom".into(),
            ..cutter(JointSource::Fixed).config
        })
        .err()
        .unwrap();
        assert!(error.contains("URDF root"), "{error}");
    }
}
