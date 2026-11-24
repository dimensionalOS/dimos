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

//! rl_deploy's faults (the controller's DriverWatch, JointWatch and tilt_fault) over every ECAN reply of a recorded
//! session, with the state rl_deploy was in as the target and the gait it ran; every fault they would have raised is
//! printed (rl_deploy raised none in these sessions).
//!
//!     cargo run --release --example fault_replay -- SESSION.bin ECAN_CAPTURE.pcapng
//!
//! SESSION.bin: tools/policy_steps.py session; ECAN_CAPTURE: the session's ecan.pcapng.

use std::path::PathBuf;

use anyhow::{bail, Context, Result};

use m20_controller::config::Config;
use m20_controller::frames::{decode_state, Mapping};
use m20_controller::state_machine::{tilt_fault, DriverWatch, JointWatch};
use m20_controller::types::Target;
use m20_ecan::{tap, Direction};

struct Reader(Vec<u8>, usize);

impl Reader {
    fn take(&mut self, n: usize) -> &[u8] {
        self.1 += n;
        &self.0[self.1 - n..self.1]
    }
    fn u8(&mut self) -> u8 {
        self.take(1)[0]
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take(8).try_into().unwrap())
    }
    fn f32s<const N: usize>(&mut self) -> [f32; N] {
        std::array::from_fn(|_| f32::from_le_bytes(self.take(4).try_into().unwrap()))
    }
    fn name(&mut self) -> String {
        let n = self.u8() as usize;
        String::from_utf8(self.take(n).to_vec()).unwrap()
    }
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [session, ecan] = &args[..] else {
        bail!("usage: fault_replay SESSION.bin ECAN_CAPTURE.pcapng");
    };
    let mut r = Reader(std::fs::read(session)?, 0);
    if r.take(6) != b"M20SS2" {
        bail!("{session}: not a session export");
    }
    let rows: Vec<f64> = (0..r.u32()).map(|_| r.f64()).collect();
    let imu: Vec<(f64, [f32; 3])> = (0..r.u32())
        .map(|_| {
            let t = r.f64();
            let v: [f32; 9] = r.f32s();
            (t, [v[0], v[1], v[2]])
        })
        .collect();
    let segments: Vec<(String, usize, usize)> = (0..r.u32())
        .map(|_| (r.name(), r.u32() as usize, r.u32() as usize))
        .collect();
    let mut gaits: Vec<(usize, u32)> = Vec::new();
    for _ in 0..r.u32() {
        let _runner = r.name();
        let row = r.u32() as usize;
        let _state_row = r.u32();
        let gait = r.u32();
        r.take(2 + 12);
        for _ in 0..r.u8() {
            r.name();
            r.f64();
        }
        r.take(4 * (16 * 3 + 4));
        gaits.push((row, gait));
    }
    let cfg = Config::load(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml"))?;
    // the ECAN states, in reply order, with the drivers' status words and temperatures
    let map = Mapping::new(&cfg.ecan)?;
    let bytes = std::fs::read(ecan)?;
    let mut states = Vec::new();
    for (_, direction, frame) in tap::read_capture(&bytes)? {
        if direction == Direction::FromBoard {
            if let Ok(Some(s)) = decode_state(frame, cfg.ecan.ethertype, &map) {
                states.push(s);
            }
        }
    }
    if states.len() != rows.len() {
        bail!("{} ECAN states, {} rows", states.len(), rows.len());
    }
    let mut watch = JointWatch::new(&cfg.robot);
    let mut drivers = DriverWatch::default();
    let latest = |t: f64| imu.partition_point(|s| s.0 <= t).checked_sub(1);
    let (mut faults, mut checked) = (0, 0u64);
    let mut gait_at = 0usize;
    for (row, s) in states.iter().enumerate() {
        let state = segments
            .iter()
            .find(|(_, a, b)| *a <= row && row < *b)
            .map(|g| g.0.as_str())
            .context("row outside the segments")?;
        while gait_at + 1 < gaits.len() && gaits[gait_at + 1].0 <= row {
            gait_at += 1;
        }
        let gait = gaits.get(gait_at).filter(|g| g.0 <= row).map_or(0, |g| g.1);
        let target = match state {
            "passive" | "start_damping" => Target::Passive,
            "stand_up" => Target::Stand,
            "rl_control" => Target::RlControl,
            "lie_down" => Target::LieDown,
            _ => Target::JointDamping,
        };
        let driver = drivers.check(s, &cfg.robot.joint_names);
        let joint = watch.check(s, &cfg.robot.joint_names);
        let tilt = latest(rows[row]).and_then(|k| {
            let rad = |d: f32| ((d / 180.0) as f64 * std::f64::consts::PI) as f32;
            tilt_fault(target, gait, rad(imu[k].1[0]), rad(imu[k].1[1]))
        });
        checked += 1;
        for f in [driver, joint, tilt].into_iter().flatten() {
            faults += 1;
            if faults <= 20 {
                println!(
                    "row {row} t {:.3} s {state} gait {gait:#06x}: {f}",
                    rows[row] - rows[0]
                );
            }
        }
    }
    println!("{checked} replies checked, {faults} faults");
    Ok(())
}
