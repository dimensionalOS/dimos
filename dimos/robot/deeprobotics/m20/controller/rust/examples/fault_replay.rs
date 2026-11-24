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

//! rl_deploy's joint-data and tilt faults (the controller's JointWatch and tilt_fault) over every ECAN reply of a
//! recorded session, with the state rl_deploy was in as the target and the gait it ran; every fault they would have
//! raised is printed (rl_deploy raised none in these sessions).
//!
//!     cargo run --release --example fault_replay -- SESSION.bin ECAN_DUMP.bin
//!
//! SESSION.bin: tools/policy_steps.py session; ECAN_DUMP.bin: examples/ecan_dump.rs of the same session.

use std::path::PathBuf;

use anyhow::{bail, Context, Result};

use m20_controller::config::{Config, JOINTS};
use m20_controller::state_machine::{tilt_fault, JointWatch};
use m20_controller::types::{RobotState, Target};

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
        bail!("usage: fault_replay SESSION.bin ECAN_DUMP.bin");
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
    // the ECAN states, in reply order (ecan_dump: kind 0 records)
    let bytes = std::fs::read(ecan)?;
    let mut states = Vec::new();
    let mut o = 0;
    while o < bytes.len() {
        let kind = bytes[o + 8];
        o += 9;
        match kind {
            0 => {
                let v: Vec<f32> = bytes[o..o + 192]
                    .chunks(4)
                    .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
                    .collect();
                states.push(v);
                o += 192;
            }
            1 => o += 258,
            _ => {}
        }
    }
    if states.len() != rows.len() {
        bail!("{} ECAN states, {} rows", states.len(), rows.len());
    }
    let cfg = Config::load(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml"))?;
    let mut watch = JointWatch::new(&cfg.robot);
    let latest = |t: f64| imu.partition_point(|s| s.0 <= t).checked_sub(1);
    let (mut faults, mut checked) = (0, 0u64);
    let mut gait_at = 0usize;
    for (row, v) in states.iter().enumerate() {
        let mut s = RobotState::default();
        for i in 0..JOINTS {
            (s.q[i], s.dq[i], s.tau[i]) =
                (v[3 * i] as f64, v[3 * i + 1] as f64, v[3 * i + 2] as f64);
        }
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
        let joint = watch.check(&s, &cfg.robot.joint_names);
        let tilt = latest(rows[row]).and_then(|k| {
            let rad = |d: f32| ((d / 180.0) as f64 * std::f64::consts::PI) as f32;
            tilt_fault(target, gait, rad(imu[k].1[0]), rad(imu[k].1[1]))
        });
        checked += 1;
        for f in [joint, tilt].into_iter().flatten() {
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
