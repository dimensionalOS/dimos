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

//! Closed-loop ping/pong throughput benchmark for the `Transport` impls.
//!
//! One generic loop runs over LCM and then Zenoh and prints a comparison table,
//! measuring transport-stack overhead on loopback rather than cross-host cost.
//!
//! Run with:
//!     cargo run --release --example transport_bench

use std::sync::Arc;
use std::time::{Duration, Instant};

use dimos_module::transport::Dispatch;
use dimos_module::{LcmTransport, Transport, ZenohTransport};
use tokio::sync::mpsc;

const PING_KEY: &str = "dimos_bench/ping";
const PONG_KEY: &str = "dimos_bench/pong";
const WINDOW: Duration = Duration::from_secs(5);

const SIZES: &[usize] = &[32, 1 << 10, 64 << 10, 1 << 20, 4 << 20];

struct Sample {
    bytes: usize,
    transport: &'static str,
    // None when the size could not complete cleanly (publish error or a dropped
    // echo, e.g. a large message past LCM's reassembly limit).
    messages: Option<u64>,
    elapsed: Duration,
}

#[tokio::main]
async fn main() {
    let lcm = bench_pair(
        "LCM",
        Arc::new(LcmTransport::new().await.expect("lcm ping")),
        Arc::new(LcmTransport::new().await.expect("lcm pong")),
    )
    .await;

    let zenoh = bench_pair(
        "Zenoh",
        Arc::new(ZenohTransport::new().await.expect("zenoh ping")),
        Arc::new(ZenohTransport::new().await.expect("zenoh pong")),
    )
    .await;

    print_table(&lcm, &zenoh);
}

async fn bench_pair<T: Transport>(
    transport: &'static str,
    ping: Arc<T>,
    pong: Arc<T>,
) -> Vec<Sample> {
    // pong echoes every ping straight back. The dispatch callback is sync, so it
    // forwards onto a channel that an async task drains and republishes.
    let (echo_tx, mut echo_rx) = mpsc::unbounded_channel::<Vec<u8>>();
    let echo: Dispatch = Arc::new(move |bytes: &[u8]| {
        let _ = echo_tx.send(bytes.to_vec());
    });
    pong.subscribe(PING_KEY, echo)
        .await
        .expect("pong subscribe");
    let pong_pub = Arc::clone(&pong);
    tokio::spawn(async move {
        while let Some(bytes) = echo_rx.recv().await {
            let _ = pong_pub.publish(PONG_KEY, bytes).await;
        }
    });

    // ping signals each returned echo onto a channel its timed loop awaits.
    let (ack_tx, mut ack_rx) = mpsc::unbounded_channel::<()>();
    let ack: Dispatch = Arc::new(move |_bytes: &[u8]| {
        let _ = ack_tx.send(());
    });
    ping.subscribe(PONG_KEY, ack).await.expect("ping subscribe");

    let mut samples = Vec::new();
    for &bytes in SIZES {
        let payload = vec![0u8; bytes];
        samples.push(run_size(transport, &*ping, &payload, &mut ack_rx).await);
    }
    samples
}

async fn run_size<T: Transport>(
    transport: &'static str,
    ping: &T,
    payload: &[u8],
    ack_rx: &mut mpsc::UnboundedReceiver<()>,
) -> Sample {
    let failed = Sample {
        bytes: payload.len(),
        transport,
        messages: None,
        elapsed: WINDOW,
    };

    println!(
        "Running {} benchmark ({})",
        transport,
        human_size(payload.len())
    );

    // Warm up until the first echo returns, covering subscription / multicast
    // join setup, then drop any acks the warmup piled up.
    if !warm_up(ping, payload, ack_rx).await {
        return failed;
    }
    while ack_rx.try_recv().is_ok() {}

    let start = Instant::now();
    let mut messages = 0u64;
    while start.elapsed() < WINDOW {
        if ping.publish(PING_KEY, payload.to_vec()).await.is_err() {
            return failed;
        }
        // Bound the wait so a dropped echo ends the size instead of hanging.
        match tokio::time::timeout(Duration::from_secs(2), ack_rx.recv()).await {
            Ok(Some(())) => messages += 1,
            _ => return failed,
        }
    }

    Sample {
        bytes: payload.len(),
        transport,
        messages: Some(messages),
        elapsed: start.elapsed(),
    }
}

async fn warm_up<T: Transport>(
    ping: &T,
    payload: &[u8],
    ack_rx: &mut mpsc::UnboundedReceiver<()>,
) -> bool {
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        if ping.publish(PING_KEY, payload.to_vec()).await.is_err() {
            return false;
        }
        let echoed = tokio::time::timeout(Duration::from_millis(200), ack_rx.recv()).await;
        if matches!(echoed, Ok(Some(()))) {
            return true;
        }
    }
    false
}

fn print_table(lcm: &[Sample], zenoh: &[Sample]) {
    println!();
    println!(
        "Transport benchmark - closed-loop ping/pong, {}s per size, 1 message in flight",
        WINDOW.as_secs()
    );
    println!();
    println!(
        " {:<8} {:<10} {:>10} {:>10} {:>9} {:>10}",
        "Size", "Transport", "Messages", "Msgs/s", "MB/s", "RTT(us)"
    );
    println!(" {}", "-".repeat(61));
    for (l, z) in lcm.iter().zip(zenoh.iter()) {
        print_row(l);
        print_row(z);
    }
    println!();
}

fn print_row(s: &Sample) {
    let size = human_size(s.bytes);
    match s.messages {
        Some(n) if n > 0 => {
            let secs = s.elapsed.as_secs_f64();
            let msgs_s = n as f64 / secs;
            let mb_s = (n as usize * s.bytes) as f64 / secs / 1e6;
            let rtt_us = s.elapsed.as_micros() as f64 / n as f64;
            println!(
                " {:<8} {:<10} {:>10} {:>10.0} {:>9.2} {:>10.0}",
                size, s.transport, n, msgs_s, mb_s, rtt_us
            );
        }
        _ => println!(
            " {:<8} {:<10} {:>10} {:>10} {:>9} {:>10}",
            size, s.transport, "N/A", "-", "-", "-"
        ),
    }
}

fn human_size(bytes: usize) -> String {
    if bytes >= 1 << 20 {
        format!("{} MB", bytes >> 20)
    } else if bytes >= 1 << 10 {
        format!("{} KB", bytes >> 10)
    } else {
        format!("{} B", bytes)
    }
}
