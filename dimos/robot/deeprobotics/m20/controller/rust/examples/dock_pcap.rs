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

//! The dock detector on a recorded capture of the front Airy (tcpdump of its MSOP / DIFOP multicast), one JSON line
//! per sweep: capture time, found, the robot in the dock frame (out from the posts m, y m, yaw rad), the posts'
//! spacing and reflective_column_node's own position (x, y, yaw). Compare with charge_manager's log by
//! dimos/simulation/m20_policy_sim/dock_compare.py, or with reflective_column_node's by tests/vendor_replay.rs.
//!
//!     cargo run --release --example dock_pcap -- CAPTURE.pcap > sweeps.jsonl

use m20_controller::dock::detector::{Detection, Detector};
use robosense_airy::{Decoder, DecoderConfig};

/// The front Airy's multicast group and MSOP / DIFOP ports (dimos/robot/deeprobotics/m20/constants.py).
const FRONT_GROUP: [u8; 4] = [224, 10, 10, 201];
const FRONT_MSOP: u16 = 6691;
const FRONT_DIFOP: u16 = 7781;

fn main() -> anyhow::Result<()> {
    let Some(path) = std::env::args().nth(1) else {
        anyhow::bail!("usage: dock_pcap CAPTURE.pcap");
    };
    let cfg = m20_controller::module::load_controller_config("")?;
    let detector = Detector::new(&cfg.dock);
    let datagrams = robosense_airy::pcap::read_udp(&path).map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut decoder = Decoder::new(DecoderConfig {
        // the vendor driver's (node_driver config.yaml)
        min_distance: 0.2,
        max_distance: 60.0,
        dense_points: true,
        use_lidar_clock: true,
        ts_first_point: true,
        ..Default::default()
    });
    for g in datagrams.iter().filter(|g| g.dst == FRONT_GROUP) {
        if g.dst_port == FRONT_DIFOP {
            let _ = decoder.decode_difop(&g.payload);
            continue;
        }
        if g.dst_port != FRONT_MSOP {
            continue;
        }
        let Ok(Some(scan)) = decoder.decode_msop(&g.payload) else {
            continue;
        };
        let points = scan
            .points
            .iter()
            .filter(|p| p.x.is_finite())
            .map(|p| [p.x, p.y, p.z, p.intensity as f32]);
        let line = match detector.detect(points) {
            Detection::Found {
                pose,
                vendor,
                posts,
                ..
            } => {
                let [out, y, yaw] = pose.robot_in_dock();
                let spacing = (posts[0][0] - posts[1][0]).hypot(posts[0][1] - posts[1][1]);
                serde_json::json!({"t": g.time, "found": true, "out": out, "y": y, "yaw": yaw, "spacing": spacing,
                                   "vendor": vendor})
            }
            Detection::Missing(why) => serde_json::json!({"t": g.time, "found": false, "why": why}),
        };
        println!("{line}");
    }
    Ok(())
}
