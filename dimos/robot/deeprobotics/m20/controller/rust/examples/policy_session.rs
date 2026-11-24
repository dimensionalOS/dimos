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

//! Every policy step of a recorded session against rl_deploy's, closed loop (each runner's last action is its own
//! previous output): the runners fed what rl_deploy's were (the ECAN reply before the step, the IMU as
//! RLControlState::UpdateRobotObservation held it, the accelerometer through its 20-tick mean), rl_control entries and
//! gait switches as rl_deploy runs them (OnEnter / OnExit), their leg motor targets and wheel velocity commands against
//! what rl_deploy sent. Within 1e-6 or 4 f32 ULPs, per runner; every step that misses is written to OUT (tsv).
//!
//!     M20_ONNXRUNTIME=libonnxruntime.so.1.20.0 cargo run --release --example policy_session -- SESSION.bin [OUT.tsv]
//!
//! SESSION.bin: tools/policy_steps.py session. --teacher: rl_deploy's previous action as the last action instead.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;

use anyhow::{Context, Result};

use m20_controller::config::{Config, JOINTS};
use m20_controller::frames::{vendor_motor_position, VENDOR_DIRECTION, VENDOR_OFFSET_DEG};
use m20_controller::policy::{PolicyRequest, RunnerSet};
use m20_controller::types::{AccelAverage, RobotState};

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

struct Imu {
    t: f64,
    rpy: [f32; 3],
    gyro: [f32; 3],
    accel: [f32; 3],
}

struct Step {
    runner: String,
    row: usize,
    /// the reply the step read (its on-cadence tick - 1)
    state_row: usize,
    gait: u32,
    axes: [f32; 3],
    params: BTreeMap<String, f64>,
    q: [f32; 16],
    dq: [f32; 16],
    motor: [f32; 16],
    wheel_dq: [f32; 4],
}

fn ulps(a: f32, b: f32) -> u32 {
    let key = |x: f32| {
        let i = x.to_bits() as i32;
        if i < 0 {
            i32::MIN - i
        } else {
            i
        }
    };
    (key(a) as i64 - key(b) as i64).unsigned_abs() as u32
}

fn close(a: f32, b: f32) -> bool {
    (a - b).abs() <= 1e-6 || ulps(a, b) <= 4
}

/// rl_deploy's action from what it sent (legs_by_type_then_wheels or per_leg), for --teacher.
fn sent_action(
    step: &Step,
    default: &[f64; JOINTS],
    scale: f64,
    wheel_scale: f64,
    per_leg: bool,
) -> [f64; JOINTS] {
    const BY_TYPE: [usize; 12] = [0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14];
    const PER_LEG: [usize; 12] = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14];
    let mut a = [0.0; JOINTS];
    for k in 0..12 {
        let i = if per_leg { PER_LEG[k] } else { BY_TYPE[k] };
        let offset = ((VENDOR_OFFSET_DEG[i] as f32 / 180.0) as f64 * std::f64::consts::PI) as f32;
        let target = step.motor[i] * VENDOR_DIRECTION[i] as f32 + offset;
        a[if per_leg { i } else { k }] = ((target - default[i] as f32) / scale as f32) as f64;
    }
    for w in 0..4 {
        let v = step.wheel_dq[w] * VENDOR_DIRECTION[4 * w + 3] as f32;
        a[if per_leg { 4 * w + 3 } else { 12 + w }] = (v / wheel_scale as f32) as f64;
    }
    a
}

/// The live orientation without imu_rpy: yesense-imu's quaternion of the device euler times the mount's inverse
fn live_quaternion(rpy_deg: [f32; 3]) -> [f64; 4] {
    let counts = |d: f32| (d as f64 * 1e6).round() * 1e-6;
    let [roll, pitch, yaw] = [
        counts(rpy_deg[0] + 90.0),
        counts(rpy_deg[1]),
        counts(rpy_deg[2]),
    ]
    .map(f64::to_radians);
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();
    let q = [
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    ];
    // q * mount^-1, mount^-1 = (cos(-pi/4), sin(-pi/4), 0, 0)
    let (mw, mx) = (
        std::f64::consts::FRAC_PI_4.cos(),
        -std::f64::consts::FRAC_PI_4.sin(),
    );
    [
        q[0] * mw - q[1] * mx,
        q[0] * mx + q[1] * mw,
        q[2] * mw + q[3] * mx,
        q[3] * mw - q[2] * mx,
    ]
}

/// experiment knobs (integers from the environment)
fn knob(name: &str, default: i64) -> i64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let teacher = args.iter().any(|a| a == "--teacher");
    let files: Vec<&String> = args.iter().filter(|a| !a.starts_with("--")).collect();
    let path = files
        .first()
        .context("usage: policy_session SESSION.bin [OUT.tsv] [--teacher]")?;
    let mut r = Reader(std::fs::read(path)?, 0);
    assert_eq!(r.take(6), b"M20SS2");
    let rows: Vec<f64> = (0..r.u32()).map(|_| r.f64()).collect();
    let imu: Vec<Imu> = (0..r.u32())
        .map(|_| {
            // M20_IMU_DELAY_US: when rl_deploy sees a sample after the capture did
            let t = r.f64() + knob("M20_IMU_DELAY_US", 0) as f64 * 1e-6;
            Imu {
                t,
                rpy: r.f32s(),
                gyro: r.f32s(),
                accel: r.f32s(),
            }
        })
        .collect();
    let segments: Vec<(String, usize, usize)> = (0..r.u32())
        .map(|_| (r.name(), r.u32() as usize, r.u32() as usize))
        .collect();
    let steps: Vec<Step> = (0..r.u32())
        .map(|_| {
            let runner = r.name();
            let row = r.u32() as usize;
            let state_row = r.u32() as usize;
            let gait = r.u32();
            let _reset = r.u8();
            let _activate = r.u8();
            let axes = r.f32s();
            let params = (0..r.u8()).map(|_| (r.name(), r.f64())).collect();
            Step {
                runner,
                row,
                state_row,
                gait,
                axes,
                params,
                q: r.f32s(),
                dq: r.f32s(),
                motor: r.f32s(),
                wheel_dq: r.f32s(),
            }
        })
        .collect();

    // the IMU sample UpdateRobotObservation held at each row, and its accelerometer mean over the rl_control ticks
    let latest = |t: f64| imu.partition_point(|s| s.t <= t).saturating_sub(1);
    let mut accel_at = vec![[0.0f64; 3]; rows.len()];
    let mut average = AccelAverage::default();
    for (name, a, b) in &segments {
        if name != "rl_control" {
            continue;
        }
        for row in *a..*b {
            // ticks without a recorded reply ran too
            let missing = if row > *a {
                (((rows[row] - rows[row - 1]) * 1000.0).round() as i64 - 1).max(0)
            } else {
                0
            };
            let k = latest(rows[row]);
            let acc = imu[k].accel.map(|x| x as f64);
            for _ in 0..missing {
                average.push(acc);
            }
            accel_at[row] = average.push(acc);
        }
    }

    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let cfg = Config::load(&manifest.join("m20_controller.toml"))?;
    let policies = PathBuf::from(std::env::var("M20_POLICY_DIR").unwrap_or_else(|_| {
        format!(
            "{}/.cache/dimos/m20_policies",
            std::env::var("HOME").unwrap_or_default()
        )
    }));
    let mut runners = RunnerSet::load(&cfg, &policies)?;
    // M20_DUMP=DIR: each step's network inputs and rl_deploy's output, DIR/<row>.bin
    let dump = std::env::var("M20_DUMP").ok().map(PathBuf::from);
    if dump.is_some() {
        runners.keep_inputs();
    }
    let fallback = cfg.states.rl_control.fallback_runner.clone();
    let segment_of = |row: usize| segments.iter().position(|(_, a, b)| *a <= row && row < *b);
    let mut out = files
        .get(1)
        .map(|p| std::fs::File::create(p).map(std::io::BufWriter::new))
        .transpose()?;
    if let Some(o) = &mut out {
        writeln!(o, "row\tt\trunner\tgait\tkind\tleg_err\tleg_ulps\twheel_err\tworst_joint\tours\trl_deploy\twheel_ours\twheel_rl_deploy")?;
    }
    // per runner: steps, leg-target steps within, wheel-command steps within
    let mut max_leg = 0.0f32;
    let mut tally: BTreeMap<String, (u64, u64, u64)> = BTreeMap::new();
    let mut prev_segment = None;
    let mut prev_runner: Option<String> = None;
    for (k, step) in steps.iter().enumerate() {
        let seg = segment_of(step.row);
        let entry = seg != prev_segment;
        prev_segment = seg;
        let current = if entry {
            Some(fallback.clone())
        } else {
            prev_runner.clone()
        };
        let switch = current.as_deref() != Some(step.runner.as_str());
        let kind = if entry {
            "entry"
        } else if switch {
            "switch"
        } else {
            "step"
        };
        prev_runner = Some(step.runner.clone());
        if teacher && !entry && !switch && k > 0 {
            let spec = &cfg.runners[&step.runner];
            let per_leg = spec.action.layout == m20_controller::config::ActionLayout::PerLeg;
            let p = &mut runners.runner_mut(&step.runner).context("runner")?.pipeline;
            if p.collected >= spec.warmup_steps
                && spec.action.leg_mode == m20_controller::config::LegMode::Offset
            {
                p.last_action = sent_action(
                    &steps[k - 1],
                    &p.default,
                    spec.action.leg_scale[0],
                    spec.action.wheel_scale,
                    per_leg,
                );
            }
        }
        let state_row = step.state_row + 1 - knob("M20_LAG", 1) as usize;
        let s = &imu[(latest(rows[state_row]) as i64 + knob("M20_IMU_SHIFT", 0)).max(0) as usize];
        let req = PolicyRequest {
            runner: step.runner.clone(),
            reset: false,
            entry,
            switch,
            activate: switch || entry,
            state: RobotState {
                gyro: s.gyro.map(|x| x as f64),
                accel: accel_at[state_row],
                q: step.q.map(|x| x as f64),
                dq: step.dq.map(|x| x as f64),
                imu_rpy_deg: (knob("M20_QUAT_IMU", 0) == 0).then_some(s.rpy),
                quat: live_quaternion(s.rpy),
                ..Default::default()
            },
            axes: step.axes.map(|x| x as f64),
            params: {
                let mut p = step.params.clone();
                p.insert("gait".into(), step.gait as f64);
                p
            },
            seq: k as u64,
        };
        let targets = runners.serve(&req).targets.map_err(anyhow::Error::msg)?;
        if let Some(dir) = &dump {
            let r = runners.runner_mut(&step.runner).context("runner")?;
            let mut b = Vec::new();
            b.extend((r.last_inputs.len() as u32).to_le_bytes());
            for (data, shape) in &r.last_inputs {
                b.extend((shape.len() as u32).to_le_bytes());
                shape.iter().for_each(|d| b.extend(d.to_le_bytes()));
                data.iter().for_each(|v| b.extend(v.to_le_bytes()));
            }
            step.motor.iter().for_each(|v| b.extend(v.to_le_bytes()));
            step.wheel_dq.iter().for_each(|v| b.extend(v.to_le_bytes()));
            targets
                .action
                .iter()
                .for_each(|v| b.extend((*v as f32).to_le_bytes()));
            std::fs::write(dir.join(format!("{}.bin", step.row)), b)?;
        }
        let (mut leg_err, mut leg_ulps, mut worst) = (0.0f32, 0u32, 0usize);
        let mut leg_ok = true;
        for i in (0..JOINTS).filter(|i| i % 4 != 3) {
            let ours = vendor_motor_position(i, targets.q[i] as f32);
            let e = (ours - step.motor[i]).abs();
            if !close(ours, step.motor[i]) {
                leg_ok = false;
            }
            if e > leg_err || e.is_nan() {
                (leg_err, leg_ulps, worst) = (e, ulps(ours, step.motor[i]), i);
            }
        }
        let (mut wheel_err, mut wheel_pair) = (0.0f32, (0.0f32, 0.0f32));
        let mut wheel_ok = true;
        for w in 0..4 {
            let ours = targets.wheel_velocity[w] as f32 * VENDOR_DIRECTION[4 * w + 3] as f32;
            if (ours - step.wheel_dq[w]).abs() >= wheel_err {
                wheel_err = (ours - step.wheel_dq[w]).abs();
                wheel_pair = (ours, step.wheel_dq[w]);
            }
            wheel_ok &= close(ours, step.wheel_dq[w]);
        }
        max_leg = max_leg.max(leg_err);
        let t = tally.entry(step.runner.clone()).or_default();
        t.0 += 1;
        t.1 += leg_ok as u64;
        t.2 += wheel_ok as u64;
        if !(leg_ok && wheel_ok) {
            if let Some(o) = &mut out {
                writeln!(
                    o,
                    "{}\t{:.3}\t{}\t{:#06x}\t{kind}\t{leg_err:.3e}\t{leg_ulps}\t{wheel_err:.3e}\t{worst}\t{}\t{}\t{}\t{}",
                    step.row,
                    rows[step.row] - rows[0],
                    step.runner,
                    step.gait,
                    vendor_motor_position(worst, targets.q[worst] as f32),
                    step.motor[worst],
                    wheel_pair.0,
                    wheel_pair.1
                )?;
            }
        }
    }
    // the gravity observation each IMU sample gives on the two paths
    let mut worst_g = 0.0f32;
    for s in &imu {
        let a = m20_controller::types::vendor_projected_gravity(s.rpy);
        let b = m20_controller::types::projected_gravity(&live_quaternion(s.rpy));
        for k in 0..3 {
            worst_g = worst_g.max((a[k] - b[k] as f32).abs());
        }
    }
    println!(
        "gravity: rl_deploy's rpy path vs the live quaternion path, max |difference| {worst_g:.3e}"
    );
    let (mut n, mut legs, mut wheels) = (0, 0, 0);
    for (name, (steps, l, w)) in &tally {
        println!(
            "{name:<16} steps {steps:>5}  leg targets within {:>6.2}%  wheel commands within {:>6.2}%",
            100.0 * *l as f64 / *steps as f64,
            100.0 * *w as f64 / *steps as f64
        );
        (n, legs, wheels) = (n + steps, legs + l, wheels + w);
    }
    println!(
        "{:<16} steps {n:>5}  leg targets within {:>6.2}%  wheel commands within {:>6.2}%",
        "all",
        100.0 * legs as f64 / n as f64,
        100.0 * wheels as f64 / n as f64
    );
    Ok(())
}
