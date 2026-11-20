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

//! `sniff_stats <interface> <participant> <seconds> [min_udp_bytes]`: run the capture alone and print the sample rate, the
//! stamp-to-capture latency and the value ranges, for checking it against the vendor stream.

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("sniff_stats captures with AF_PACKET, which only Linux has");
}

#[cfg(target_os = "linux")]
fn main() -> std::io::Result<()> {
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    use dimos_m20_body_imu_sniff::capture::{wait_readable, SniffCaptures};
    use dimos_m20_body_imu_sniff::rtps::Sniffer;

    let args: Vec<String> = std::env::args().collect();
    let (Some(interface), Some(participant)) = (args.get(1), args.get(2)) else {
        eprintln!("usage: sniff_stats <interface> <participant> <seconds> [min_udp_bytes]");
        std::process::exit(2);
    };
    let seconds: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(10.0);
    let min_udp_bytes = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(200);
    // the module's defaults: domain 0's SPDP port, the user port range, a 4 MiB receive buffer
    let mut captures = SniffCaptures::open(interface, 7400, 7400, 7700, min_udp_bytes, 4 << 20)?;
    let mut sniffer = Sniffer::new(participant);
    let (start, mut first) = (Instant::now(), None::<Instant>);
    let (mut discovery_datagrams, mut datagrams, mut stamps, mut latencies_ms, mut max_accel) =
        (0u64, 0u64, Vec::new(), Vec::new(), 0.0f64);
    let deadline = start + Duration::from_secs_f64(seconds.max(0.0));
    while let Some(left) = deadline.checked_duration_since(Instant::now()) {
        // wake at the deadline even when nothing arrives
        let [discovery_ready, data_ready] =
            wait_readable([&captures.discovery, &captures.data], Some(left))?;
        if discovery_ready {
            discovery_datagrams += 1;
            let known = sniffer.prefix();
            drop(sniffer.feed(captures.discovery.next_udp_payload()?));
            if let (Some(prefix), true) = (sniffer.prefix(), known != sniffer.prefix()) {
                captures.follow(prefix)?;
            }
        }
        if !data_ready {
            continue;
        }
        datagrams += 1;
        for sample in sniffer.feed(captures.data.next_udp_payload()?) {
            first.get_or_insert_with(Instant::now);
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs_f64();
            let stamp = sample.stamp_sec as f64 + sample.stamp_nanosec as f64 * 1e-9;
            latencies_ms.push((now - stamp) * 1e3);
            stamps.push(stamp);
            let a = sample.linear_acceleration;
            max_accel = max_accel.max((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt());
            if stamps.len() == 1 {
                println!("first sample: {sample:?}");
            }
        }
    }
    let gaps: Vec<f64> = stamps.windows(2).map(|w| (w[1] - w[0]) * 1e3).collect();
    let pick = |v: &mut Vec<f64>, q: f64| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.get(((v.len() as f64 - 1.0) * q) as usize)
            .copied()
            .unwrap_or(f64::NAN)
    };
    let (mut gaps, mut lat) = (gaps, latencies_ms);
    let span = stamps.last().unwrap_or(&0.0) - stamps.first().unwrap_or(&0.0);
    println!(
        "kernel drops {}, writer sequence numbers never captured {}",
        captures.data.take_drops() + captures.discovery.take_drops(),
        sniffer.missed
    );
    println!(
        "discovery datagrams {discovery_datagrams}, data datagrams {datagrams}, samples {}, found participant after {:.2} s, rate {:.2} Hz",
        stamps.len(),
        first.map(|f| (f - start).as_secs_f64()).unwrap_or(f64::NAN),
        (stamps.len() as f64 - 1.0) / span
    );
    println!(
        "gap ms min/median/max {:.3}/{:.3}/{:.3}; capture-stamp ms min/median/p99/max {:.3}/{:.3}/{:.3}/{:.3}; max |accel| {:.3} m/s^2",
        pick(&mut gaps, 0.0), pick(&mut gaps, 0.5), pick(&mut gaps, 1.0),
        pick(&mut lat, 0.0), pick(&mut lat, 0.5), pick(&mut lat, 0.99), pick(&mut lat, 1.0), max_accel
    );
    Ok(())
}
