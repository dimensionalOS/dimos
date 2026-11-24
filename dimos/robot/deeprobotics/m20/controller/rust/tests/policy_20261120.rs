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

//! Our policy steps against rl_deploy's on the 2026-11-20 14:16 session (motor_data/m20_motor_session_20261120_141603:
//! the ECAN capture and the net pcap with yesense's IMU samples): each step fed the joint state as the ECAN reply
//! before it carried it, the last IMU sample before that reply, the operator's axes, and as last action the action
//! rl_deploy's previous command implies; its leg motor targets against the ones rl_deploy sent, within 1e-6 or 4 f32
//! ULPs. Windows: basic_stair, navi_charge and basic_flat walking runs, each after its first 5 steps (the history).
//!
//! rl_deploy infers with ONNX Runtime 1.20.0 on Linux aarch64 (1 intra-op thread, all graph optimizations); other
//! builds differ in the last bits (macOS's by up to 4e-6, tract by up to 5e-5), so the test needs that library:
//! M20_ONNXRUNTIME=path/to/libonnxruntime.so.1.20.0 (rl_deploy's deb has it), else it skips. M20_POLICY_TOLERANCE
//! widens the tolerance to report how far another build is.
//!
//! Fixture: tests/fixtures/policy_20261120_141603.bin.gz, cut by tools/policy_steps.py (its layout there) from
//! replay.py's plan of the session and examples/ecan_dump.rs's exact frames.

mod common;

use std::io::Read;

use m20_controller::config::JOINTS;
use m20_controller::frames::{vendor_motor_position, VENDOR_DIRECTION, VENDOR_OFFSET_DEG};
use m20_controller::runner::Runner;
use m20_controller::types::RobotState;

struct Step {
    axes: [f32; 3],
    q: [f32; 16],
    dq: [f32; 16],
    rpy_deg: [f32; 3],
    gyro: [f32; 3],
    accel: [f32; 3],
    motor: [f32; 16],
    wheel_dq: [f32; 4],
}

struct Window {
    runner: String,
    /// (reset, activate, axes) of the runner's steps before the window
    prefix: Vec<(bool, bool, [f32; 3])>,
    steps: Vec<Step>,
}

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
    fn f32s<const N: usize>(&mut self) -> [f32; N] {
        std::array::from_fn(|_| f32::from_le_bytes(self.take(4).try_into().unwrap()))
    }
}

fn windows() -> Vec<Window> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/policy_20261120_141603.bin.gz");
    let mut bytes = Vec::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap())
        .read_to_end(&mut bytes)
        .unwrap();
    let mut r = Reader(bytes, 0);
    assert_eq!(r.take(6), b"M20PS1");
    (0..r.u32())
        .map(|_| {
            let n = r.u8() as usize;
            let runner = String::from_utf8(r.take(n).to_vec()).unwrap();
            let prefix = (0..r.u32())
                .map(|_| (r.u8() != 0, r.u8() != 0, r.f32s()))
                .collect();
            let steps = (0..r.u32())
                .map(|_| Step {
                    axes: r.f32s(),
                    q: r.f32s(),
                    dq: r.f32s(),
                    rpy_deg: r.f32s(),
                    gyro: r.f32s(),
                    accel: r.f32s(),
                    motor: r.f32s(),
                    wheel_dq: r.f32s(),
                })
                .collect();
            Window {
                runner,
                prefix,
                steps,
            }
        })
        .collect()
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

/// rl_deploy's action from the targets it sent (its legs_by_type_then_wheels layout): legs (target - default) / leg
/// scale, wheels velocity / wheel scale, in f32.
fn implied_action(
    step: &Step,
    default: &[f64; JOINTS],
    leg_scale: f64,
    wheel_scale: f64,
) -> [f64; JOINTS] {
    const LEGS_BY_TYPE: [usize; 12] = [0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14];
    let mut a = [0.0; JOINTS];
    for (k, &i) in LEGS_BY_TYPE.iter().enumerate() {
        let offset = ((VENDOR_OFFSET_DEG[i] as f32 / 180.0) as f64 * std::f64::consts::PI) as f32;
        let target = step.motor[i] * VENDOR_DIRECTION[i] as f32 + offset;
        a[k] = ((target - default[i] as f32) / leg_scale as f32) as f64;
    }
    for k in 0..4 {
        let velocity = step.wheel_dq[k] * VENDOR_DIRECTION[4 * k + 3] as f32;
        a[12 + k] = (velocity / wheel_scale as f32) as f64;
    }
    a
}

#[test]
fn policy_steps_match_rl_deploy() {
    let Some(dir) = common::policy_dir() else {
        return;
    };
    let Some(library) = std::env::var("M20_ONNXRUNTIME")
        .ok()
        .filter(|p| !p.is_empty())
    else {
        eprintln!(
            "\n\n!!! SKIPPING policy_steps_match_rl_deploy: set M20_ONNXRUNTIME to rl_deploy's ONNX Runtime \
             (libonnxruntime.so.1.20.0, Linux aarch64) to compare our policy steps with rl_deploy's !!!\n\n"
        );
        return;
    };
    let tolerance: f32 = std::env::var("M20_POLICY_TOLERANCE")
        .ok()
        .map(|v| v.parse().unwrap())
        .unwrap_or(1e-6);
    let cfg = common::config();
    let dt = cfg.timing.policy_decimation as f64 / cfg.timing.control_hz;
    let mut misses = Vec::new();
    let mut compared = 0;
    for w in windows() {
        let spec = &cfg.runners[&w.runner];
        let mut runner =
            Runner::with_inference(&w.runner, spec, &cfg.robot, &dir, dt, Some(&library)).unwrap();
        let axes = |a: [f32; 3]| a.map(|x| x as f64);
        for (reset, activate, a) in &w.prefix {
            if *reset {
                runner.reset();
            }
            if *activate {
                runner.pipeline.activate();
            }
            runner.pipeline.update_command(&axes(*a));
            runner.pipeline.advance_phase();
        }
        let default = runner.pipeline.default;
        let (leg_scale, wheel_scale) = (spec.action.leg_scale[0], spec.action.wheel_scale);
        let mut previous: Option<&Step> = None;
        for (k, step) in w.steps.iter().enumerate() {
            if let Some(p) = previous {
                runner.pipeline.last_action = implied_action(p, &default, leg_scale, wheel_scale);
            }
            let s = RobotState {
                gyro: step.gyro.map(|x| x as f64),
                accel: step.accel.map(|x| x as f64),
                q: step.q.map(|x| x as f64),
                dq: step.dq.map(|x| x as f64),
                imu_rpy_deg: Some(step.rpy_deg),
                ..Default::default()
            };
            let targets = runner
                .step(&s, &axes(step.axes), &Default::default())
                .unwrap();
            previous = Some(step);
            if k < 5 {
                continue;
            }
            for i in (0..JOINTS).filter(|i| i % 4 != 3) {
                compared += 1;
                let ours = vendor_motor_position(i, targets.q[i] as f32);
                let theirs = step.motor[i];
                if (ours - theirs).abs() > tolerance && ulps(ours, theirs) > 4 {
                    misses.push(format!(
                        "{} step {k} joint {i}: ours {ours} rl_deploy {theirs} ({:.1e}, {} ULPs)",
                        w.runner,
                        (ours - theirs).abs(),
                        ulps(ours, theirs)
                    ));
                }
            }
        }
    }
    assert!(
        misses.is_empty(),
        "{} of {compared} leg targets miss:\n{}",
        misses.len(),
        misses.join("\n")
    );
}
