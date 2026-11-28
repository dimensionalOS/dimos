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
//! Is the cmd_vel path wired? Joins rl_deploy's topics as the module does and reports, for `--seconds`: whether
//! rl_deploy's REAL_STEER reader matched this writer, the other REAL_STEER writers, and MOTION_INFO / HES_STATUS.
//! With `--write-zeros N` it also writes N zero velocities (one per MOTION_INFO) and waits for rl_deploy to
//! acknowledge them, refusing unless rl_deploy reports a state other than RL control (`--rl-control-state`), so a
//! zero can never cut into a walk. It never writes anything but zero.
//!
//!   m20_steer_check [--seconds 10] [--write-zeros 0] [--domain 0] [--rl-control-state 17]

#![cfg_attr(
    not(test),
    deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

use std::time::{Duration, Instant};

use dimos_m20_cmd_vel::dds::{Discovery, Steering, Topics};
use dimos_m20_cmd_vel::gate::Velocity;
use futures::StreamExt;
use rustdds::dds::statusevents::StatusEvented;

fn arg<T: std::str::FromStr>(name: &str, default: T) -> T {
    let args: Vec<String> = std::env::args().collect();
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let seconds: f64 = arg("--seconds", 10.0);
    let zeros: u32 = arg("--write-zeros", 0);
    let rl_control_state: i32 = arg("--rl-control-state", 17);
    let topics = Topics {
        domain_id: arg("--domain", 0),
        steer_topic: "rt/REAL_STEER".into(),
        steer_type: "drdds::msg::dds_::Steer_".into(),
        motion_info_topic: "rt/MOTION_INFO".into(),
        motion_info_type: "drdds::msg::dds_::MotionInfo_".into(),
        hes_status_topic: "rt/HES_STATUS".into(),
        hes_status_type: "drdds::msg::dds_::StdMsgInt32_".into(),
    };
    let (mut steering, mut readers) = match Steering::open(&topics) {
        Ok(opened) => opened,
        Err(error) => {
            eprintln!("cannot join DDS: {error}");
            std::process::exit(1);
        }
    };
    let (own, writer) = (steering.own(), steering.writer_guid());
    println!("writer {:?}", steering.writer_guid());
    let listener = steering.participant().status_listener();
    let mut events = listener.as_async_status_stream();
    let mut discovery = Discovery::default();
    let (mut infos, mut last_state, mut hes, mut written) = (0u32, None::<i32>, None::<i32>, 0u32);
    let end = tokio::time::Instant::now() + Duration::from_secs_f64(seconds);
    let started = Instant::now();
    loop {
        tokio::select! {
            _ = tokio::time::sleep_until(end) => break,
            sample = readers.motion_info.next() => {
                let Some(Ok(sample)) = sample else { continue };
                let info = sample.into_value();
                if infos == 0 {
                    println!("first MOTION_INFO after {:.2} s: state {} gait {} height {:.3}", started.elapsed().as_secs_f64(), info.motion_state, info.gait, info.height);
                }
                infos += 1;
                last_state = Some(info.motion_state);
                let walking = info.motion_state == rl_control_state;
                if written < zeros && !discovery.matched().is_empty() {
                    if walking {
                        println!("rl_deploy is in RL control (state {}): not writing", info.motion_state);
                    } else {
                        steering.write(Velocity::ZERO);
                        written += 1;
                    }
                }
            }
            sample = readers.hes_status.next() => {
                let Some(Ok(sample)) = sample else { continue };
                hes = Some(sample.into_value().value);
            }
            event = events.next() => {
                let Some(event) = event else { continue };
                discovery.see(&event, &steering.steer_topic, own, writer);
            }
        }
    }
    let acked = written > 0 && steering.acknowledged(Duration::from_secs(2));
    println!("matched REAL_STEER readers: {:?}", discovery.matched());
    println!("other REAL_STEER writers: {:?}", discovery.other_writers());
    println!("MOTION_INFO: {infos} in {seconds} s, last state {last_state:?}; HES_STATUS {hes:?}");
    println!("zeros written: {written}, acknowledged by every matched reader: {acked}");
}
