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
//! spacing. Compare with charge_manager's log by dimos/simulation/m20_policy_sim/dock_compare.py.
//!
//!     cargo run --release --example dock_pcap -- CAPTURE.pcap > sweeps.jsonl

use std::f64::consts::PI;

use m20_controller::dock::detector::{detect, Detection, Mount};
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
    // FRONT_LIDAR_XYZ_M / FRONT_LIDAR_RPY
    let mount = Mount::from_xyz_rpy([0.32028, 0.0, -0.013], [0.0, -PI / 2.0, PI]);
    let datagrams = robosense_airy::pcap::read_udp(&path).map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut decoder = Decoder::new(DecoderConfig {
        min_distance: 0.1,
        max_distance: 60.0,
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
        let line = match detect(points, &mount, &cfg.dock.detector) {
            Detection::Found { pose, posts, .. } => {
                let [out, y, yaw] = pose.robot_in_dock();
                let spacing = (posts[0][0] - posts[1][0]).hypot(posts[0][1] - posts[1][1]);
                serde_json::json!({"t": g.time, "found": true, "out": out, "y": y, "yaw": yaw, "spacing": spacing})
            }
            Detection::Missing(why) => serde_json::json!({"t": g.time, "found": false, "why": why}),
        };
        println!("{line}");
    }
    Ok(())
}
