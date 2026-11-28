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

//! Joint-state replies in an ECAN capture by record count and odd status words, then DriverWatch on them merged.

use std::collections::BTreeMap;

use anyhow::{bail, Result};

use m20_controller::config::Config;
use m20_controller::hardware::bus::robot_state;
use m20_controller::state_machine::DriverWatch;
use m20_ecan::{decode_ethernet, tap, Calibration, Direction, Message};

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [ecan] = &args[..] else {
        bail!("usage: reply_census ECAN_CAPTURE.pcapng");
    };
    let cfg = Config::load(
        &std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml"),
    )?;
    let calibration = Calibration::m20();
    let bytes = std::fs::read(ecan)?;
    let mut counts: BTreeMap<usize, u64> = BTreeMap::new();
    let mut odd: BTreeMap<(usize, u16), u64> = BTreeMap::new();
    let (mut last, mut watch, mut faults) = (None, DriverWatch::default(), 0u64);
    for (_, direction, frame) in tap::read_capture(&bytes)? {
        if direction != Direction::FromBoard {
            continue;
        }
        let Ok((_, Message::JointStates { joints, .. })) = decode_ethernet(frame, direction) else {
            continue;
        };
        *counts.entry(joints.len()).or_default() += 1;
        for j in &joints {
            if j.status_word != 1 {
                *odd.entry((j.joint.index(), j.status_word)).or_default() += 1;
            }
        }
        let Some(s) = robot_state(&joints, &calibration, last.as_ref()) else {
            continue;
        };
        if let Some(f) = watch.check(&s, &cfg.robot.joint_names) {
            faults += 1;
            if faults <= 5 {
                println!("fault: {f}");
            }
        }
        last = Some(s);
    }
    println!("replies by joint records: {counts:?}");
    println!("records with a status word other than 1 (joint, word): {odd:?}");
    println!("driver faults over the merged replies: {faults}");
    Ok(())
}
