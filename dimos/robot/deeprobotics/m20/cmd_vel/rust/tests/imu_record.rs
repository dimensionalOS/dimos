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

//! m20_imu_record against a stand-in for yesense: a reliable rt/IMU_DATA writer (as the vendor's) on this machine
//! publishes samples; the recorder's lines carry each one bit for bit. Skips (saying so) when nothing arrives (DDS
//! discovery needs multicast on this machine).

use std::time::Duration;

use dimos_m20_cmd_vel::wire::{Header, ImuData};
use rustdds::policy::{Durability, History, Reliability};
use rustdds::*;

fn sample(k: u32) -> ImuData {
    let f = |a: f32| a + k as f32 * 1e-3;
    ImuData {
        header: Header {
            frame_id: 0,
            sec: 1_795_470_600 + k as i32,
            nsec: k * 5_000_000,
        },
        roll: f(-1.698143),
        pitch: f(-0.576693),
        yaw: f(-8.955583),
        omega_x: f(0.002021),
        omega_y: f(0.000316),
        omega_z: f(0.004633),
        acc_x: f(0.096795),
        acc_y: f(-0.293907),
        acc_z: f(9.834623),
    }
}

#[test]
fn records_every_sample_bit_for_bit() {
    // a domain of its own, so nothing else on the network is seen
    let domain = 77;
    let dir = std::env::temp_dir().join(format!("m20_imu_record_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let out = dir.join("imu.txt");
    let mut recorder = std::process::Command::new(env!("CARGO_BIN_EXE_m20_imu_record"))
        .args([
            "--out",
            out.to_str().unwrap(),
            "--seconds",
            "60",
            "--domain",
            "77",
        ])
        .spawn()
        .unwrap();
    let qos = QosPolicyBuilder::new()
        .reliability(Reliability::Reliable {
            max_blocking_time: rustdds::Duration::from_millis(100),
        })
        .durability(Durability::Volatile)
        .history(History::KeepLast { depth: 1 })
        .build();
    let participant = DomainParticipant::new(domain).unwrap();
    let topic = participant
        .create_topic(
            "rt/IMU_DATA".into(),
            "drdds::msg::dds_::ImuData_".into(),
            &qos,
            TopicKind::NoKey,
        )
        .unwrap();
    let writer = participant
        .create_publisher(&qos)
        .unwrap()
        .create_datawriter_no_key_cdr::<ImuData>(&topic, None)
        .unwrap();
    // discovery (SPDP / SEDP between the two participants) takes a second or two here: as yesense does, the writer
    // publishes all along (400 samples, k 1000..), then the 200 checked ones (k 0..200), at 200 Hz
    std::thread::sleep(Duration::from_secs(1));
    for k in 1000..1400 {
        writer.write(sample(k), None).unwrap();
        std::thread::sleep(Duration::from_millis(5));
    }
    for k in 0..200 {
        writer.write(sample(k), None).unwrap();
        std::thread::sleep(Duration::from_millis(5));
    }
    // stopped as record.sh stops it (SIGTERM): what it received is kept
    std::thread::sleep(Duration::from_millis(500));
    let killed = std::process::Command::new("kill")
        .args(["-TERM", &recorder.id().to_string()])
        .status()
        .unwrap();
    assert!(killed.success());
    assert!(recorder.wait().unwrap().success());
    let text = std::fs::read_to_string(&out).unwrap();
    let all: Vec<&str> = text.lines().collect();
    let lines: Vec<&str> = all
        .iter()
        .copied()
        .filter(|l| {
            l.split(' ')
                .nth(2)
                .and_then(|s| s.parse::<i32>().ok())
                .is_some_and(|sec| sec < 1_795_470_600 + 1000)
        })
        .collect();
    if all.is_empty() {
        eprintln!("\n*** SKIPPED: nothing recorded: the participants did not discover each other (no multicast here?) ***\n");
        return;
    }
    assert_eq!(
        lines.len(),
        200,
        "{} of the 200 samples after discovery recorded",
        lines.len()
    );
    for l in &lines {
        let v: Vec<&str> = l.split(' ').collect();
        assert_eq!(v.len(), 13, "{l}");
        let sec: i32 = v[2].parse().unwrap();
        let k = (sec - 1_795_470_600) as u32;
        let want = sample(k);
        assert_eq!(v[3].parse::<u32>().unwrap(), want.header.nsec);
        let got: Vec<f32> = v[4..].iter().map(|x| x.parse().unwrap()).collect();
        let exp = [
            want.roll,
            want.pitch,
            want.yaw,
            want.omega_x,
            want.omega_y,
            want.omega_z,
            want.acc_x,
            want.acc_y,
            want.acc_z,
        ];
        for (a, b) in got.iter().zip(exp) {
            assert_eq!(a.to_bits(), b.to_bits(), "{l}");
        }
    }
    println!(
        "all 200 samples after discovery recorded, every value bit for bit ({} in all)",
        all.len()
    );
    let _ = std::fs::remove_dir_all(&dir);
}
