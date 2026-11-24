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

//! Records the body IMU rl_deploy reads (yesense's rt/IMU_DATA, drdds ImuData, 200 Hz) for a replay: a best-effort,
//! volatile reader (it writes nothing, never acknowledges, and the writer resends nothing to it), one line per sample:
//!
//!   receive_unix_ns frame_id sec nsec roll pitch yaw omega_x omega_y omega_z acc_x acc_y acc_z
//!
//! the floats as Rust prints them (they read back bit for bit); it stops after --seconds or on SIGTERM / SIGINT. On the AOS board beside rl_deploy (rustdds has no shared
//! memory, so yesense's Fast-DDS sends this reader UDP, which the recording's tcpdump also sees).
//!
//!   m20_imu_record --out imu.txt [--seconds 3600] [--domain 0]

#![cfg_attr(
    not(test),
    deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

use std::io::Write;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_m20_cmd_vel::wire::ImuData;
use futures::StreamExt;
use rustdds::policy::{Durability, History, Reliability};
use rustdds::*;

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
    let out: String = arg("--out", String::new());
    let seconds: f64 = arg("--seconds", 3600.0);
    let domain: u16 = arg("--domain", 0);
    let topic_name: String = arg("--topic", "rt/IMU_DATA".to_string());
    if out.is_empty() {
        eprintln!(
            "usage: m20_imu_record --out FILE [--seconds 3600] [--domain 0] [--topic rt/IMU_DATA]"
        );
        std::process::exit(2);
    }
    if let Err(error) = record(&out, seconds, domain, &topic_name).await {
        eprintln!("m20_imu_record: {error}");
        std::process::exit(1);
    }
}

async fn record(out: &str, seconds: f64, domain: u16, topic_name: &str) -> Result<(), String> {
    let qos = QosPolicyBuilder::new()
        .reliability(Reliability::BestEffort)
        .durability(Durability::Volatile)
        .history(History::KeepLast { depth: 64 })
        .build();
    let participant = DomainParticipant::new(domain).map_err(|e| format!("participant: {e:?}"))?;
    let topic = participant
        .create_topic(
            topic_name.into(),
            "drdds::msg::dds_::ImuData_".into(),
            &qos,
            TopicKind::NoKey,
        )
        .map_err(|e| format!("topic: {e:?}"))?;
    let subscriber = participant
        .create_subscriber(&qos)
        .map_err(|e| format!("subscriber: {e:?}"))?;
    let mut samples = subscriber
        .create_datareader_no_key_cdr::<ImuData>(&topic, Some(qos))
        .map_err(|e| format!("reader: {e:?}"))?
        .async_sample_stream();
    let file = std::fs::File::create(out).map_err(|e| format!("{out}: {e}"))?;
    let mut file = std::io::BufWriter::new(file);
    let end = Instant::now() + Duration::from_secs_f64(seconds);
    let mut count: u64 = 0;
    // record.sh stops it early with SIGTERM: what was received is kept
    let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .map_err(|e| format!("SIGTERM handler: {e}"))?;
    eprintln!("recording {topic_name} to {out} for {seconds} s");
    loop {
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        let next = tokio::select! {
            next = tokio::time::timeout(left, samples.next()) => next,
            _ = term.recv() => break,
            _ = tokio::signal::ctrl_c() => break,
        };
        let Ok(Some(sample)) = next else {
            if matches!(next, Ok(None)) {
                break;
            }
            continue;
        };
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        let Ok(sample) = sample else { continue };
        let i = sample.into_value();
        let h = i.header;
        writeln!(
            file,
            "{now} {} {} {} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
            h.frame_id,
            h.sec,
            h.nsec,
            i.roll,
            i.pitch,
            i.yaw,
            i.omega_x,
            i.omega_y,
            i.omega_z,
            i.acc_x,
            i.acc_y,
            i.acc_z
        )
        .map_err(|e| format!("{out}: {e}"))?;
        count += 1;
        if count % 2000 == 0 {
            file.flush().map_err(|e| format!("{out}: {e}"))?;
        }
    }
    file.flush().map_err(|e| format!("{out}: {e}"))?;
    eprintln!("{count} samples");
    Ok(())
}
