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

//! The detector against reflective_column_node's own log on the same sweeps: a capture of the front Airy (tcpdump of
//! its multicast) decoded as the vendor driver does it (robosense-airy's fused_multiply_add: rs_driver's GCC aarch64
//! build to the bit), every sweep through the detector, each sweep the vendor logged paired with ours (by time, then by
//! its logged post centres), and every logged value printed as the vendor prints it: the points after the voxel grid
//! and outlier removal, each cluster's z / y ratio, the posts' point counts, their centres and the position and yaw.
//!
//! The captures (each skipped, saying so, when absent): ~/datasets/m20/recording_20261120_163604/lidar_raw.pcap
//! (2026-11-20 16:36-16:38 CST: an undock and a dock from ~1.8 m), ~/datasets/m20/dock_rec_20261121/
//! lidar_20261121_132117.pcap (2026-11-21 13:21-13:28: docks and undocks) and ~/datasets/m20/dock_rec_20261124/nos/
//! lidar_20261124_054951.pcap (2026-11-24 05:50-05:52: an undock and a dock), against the logs cut from those days'
//! (tests/fixtures). M20_DOCK_PCAP + M20_DOCK_RC_LOG (a raw /var/opt/robot/log/<day>/reflective_column_node.<day>.log,
//! or .gz) run one other capture instead.

mod common;

use std::path::PathBuf;

use m20_controller::dock::detector::{Detection, Detector, Trace};
use robosense_airy::{Decoder, DecoderConfig};

/// The front Airy's multicast group and MSOP / DIFOP ports (dimos/robot/deeprobotics/m20/constants.py).
const FRONT_GROUP: [u8; 4] = [224, 10, 10, 201];
const FRONT_MSOP: u16 = 6691;
const FRONT_DIFOP: u16 = 7781;
/// The log's clock: China Standard Time.
const CST_S: f64 = 8.0 * 3600.0;
/// A logged sweep pairs with ours within this (s) of the clock offset.
const PAIR_WINDOW_S: f64 = 0.12;

/// One sweep as reflective_column_node logged it.
#[derive(Debug, Default, Clone)]
struct Logged {
    t: f64,
    trace: Trace,
    /// the z / y ratios as printed (the trace's are f32)
    ratios: Vec<f64>,
    position: Option<[f64; 3]>,
}

fn read_text(path: &PathBuf) -> String {
    use std::io::Read;
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    if path.extension().is_some_and(|e| e == "gz") {
        let mut text = String::new();
        flate2::read::GzDecoder::new(&bytes[..])
            .read_to_string(&mut text)
            .unwrap();
        return text;
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

/// Days since 1970-01-01 of a civil date (Howard Hinnant's days_from_civil).
fn days(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1;
    era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468
}

/// "[2026-11-20 16:36:16.732.262] ..." -> Unix seconds.
fn log_time(line: &str) -> Option<f64> {
    let s = line.get(1..28)?;
    let n = |a: usize, b: usize| s.get(a..b)?.parse::<i64>().ok();
    let secs = days(n(0, 4)?, n(5, 7)?, n(8, 10)?) * 86400
        + n(11, 13)? * 3600
        + n(14, 16)? * 60
        + n(17, 19)?;
    Some(secs as f64 + n(20, 23)? as f64 / 1e3 + n(24, 27)? as f64 / 1e6 - CST_S)
}

/// The numbers after each `key` in a line ("x=0.75, y=-0.19" / "x: -0.75" / "数量: 62").
fn numbers(line: &str) -> Vec<f64> {
    let mut out = Vec::new();
    let mut cur = String::new();
    for c in line.chars().chain([' ']) {
        if c.is_ascii_digit() || c == '.' || (c == '-' && cur.is_empty()) {
            cur.push(c);
        } else {
            if cur.parse::<f64>().is_ok() && cur.contains(|c: char| c.is_ascii_digit()) {
                out.push(cur.parse().unwrap());
            }
            cur.clear();
        }
    }
    out
}

/// Every sweep in the log, from each 【step1】 line to the next.
fn parse_log(text: &str) -> Vec<Logged> {
    let mut out: Vec<Logged> = Vec::new();
    for line in text.lines() {
        let Some(body) = line.split("[Location_node] ").nth(1) else {
            continue;
        };
        if body.starts_with("【step1】") {
            let t = log_time(line).unwrap();
            let n = numbers(body.rsplit(':').next().unwrap())[0] as usize;
            out.push(Logged {
                t,
                trace: Trace {
                    step1: Some(n),
                    ..Default::default()
                },
                ratios: Vec::new(),
                position: None,
            });
            continue;
        }
        let Some(cur) = out.last_mut() else {
            continue;
        };
        let v = numbers(body.split_once(':').map_or(body, |s| s.1));
        if body.starts_with("簇 z/y 比例") {
            cur.trace.ratios.push(v[0] as f32);
            cur.ratios.push(v[0]);
        } else if body.starts_with("反光柱数量:") {
            cur.trace.columns = Some(v[0] as usize);
        } else if body.starts_with("距离差") {
            cur.trace.rejected.push((v[0], v[1]));
        } else if body.starts_with("两反光柱距离误差") {
            cur.trace.no_pair = Some(v[0]);
        } else if body.starts_with("【step 2】") {
            let v = numbers(body);
            cur.trace.counts = Some([v[v.len() - 2] as usize, v[v.len() - 1] as usize]);
        } else if body.starts_with("【step 3】左") || body.starts_with("【step 3】右") {
            let v = numbers(body);
            let c = [v[v.len() - 3], v[v.len() - 2]];
            let centers = cur.trace.centers.get_or_insert([[f64::NAN; 2]; 2]);
            centers[if body.starts_with("【step 3】左") {
                0
            } else {
                1
            }] = c;
        } else if body.starts_with("【step 3】一次计算") {
            let v = numbers(body);
            // x, y, z, yaw
            cur.position = Some([v[v.len() - 4], v[v.len() - 3], v[v.len() - 1]]);
        }
    }
    out
}

/// One of our sweeps: its capture time, trace and position.
struct Ours {
    t: f64,
    trace: Trace,
    position: Option<[f64; 3]>,
}

fn decode_and_detect(pcap: &PathBuf, detector: &Detector) -> Vec<Ours> {
    let datagrams = robosense_airy::pcap::read_udp(pcap).unwrap();
    // the vendor driver's settings (NOS node_driver config.yaml)
    let mut decoder = Decoder::new(DecoderConfig {
        min_distance: 0.2,
        max_distance: 60.0,
        use_lidar_clock: true,
        dense_points: true,
        ts_first_point: true,
        fused_multiply_add: true,
        ..Default::default()
    });
    let mut out = Vec::new();
    for g in datagrams.iter().filter(|g| g.dst == FRONT_GROUP) {
        if g.dst_port == FRONT_DIFOP {
            let _ = decoder.decode_difop(&g.payload);
            continue;
        }
        if g.dst_port != FRONT_MSOP {
            continue;
        }
        let Ok(Some(scan)) = decoder.decode_msop(&g.payload) else {
            continue;
        };
        let mut trace = Trace::default();
        let points = scan
            .points
            .iter()
            .map(|p| [p.x, p.y, p.z, p.intensity as f32]);
        let position = match detector.detect_traced(points, &mut trace) {
            Detection::Found { vendor, .. } => Some(vendor),
            Detection::Missing(_) => None,
        };
        out.push(Ours {
            t: g.time,
            trace,
            position,
        });
    }
    out
}

/// How far a logged sweep's post centres are from ours (inf when either has none).
fn center_gap(a: &Trace, b: &Trace) -> f64 {
    match (a.centers, b.centers) {
        (Some(a), Some(b)) => (0..2)
            .flat_map(|i| (0..2).map(move |k| (a[i][k] - b[i][k]).abs()))
            .fold(0.0, f64::max),
        _ => f64::INFINITY,
    }
}

#[test]
fn every_logged_sweep_matches_reflective_column_node() {
    let home = PathBuf::from(std::env::var("HOME").unwrap_or_default());
    let fixture = |name: &str| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name)
    };
    let mut cases = vec![
        (
            home.join("datasets/m20/recording_20261120_163604/lidar_raw.pcap"),
            fixture("reflective_column_node_20261120.log.gz"),
        ),
        (
            home.join("datasets/m20/dock_rec_20261121/lidar_20261121_132117.pcap"),
            fixture("reflective_column_node_20261121_1321.log.gz"),
        ),
        (
            home.join("datasets/m20/dock_rec_20261124/nos/lidar_20261124_054951.pcap"),
            fixture("reflective_column_node_20261124.log.gz"),
        ),
    ];
    if let Some(pcap) = std::env::var_os("M20_DOCK_PCAP") {
        let log = std::env::var_os("M20_DOCK_RC_LOG").expect("M20_DOCK_PCAP needs M20_DOCK_RC_LOG");
        cases = vec![(PathBuf::from(pcap), PathBuf::from(log))];
    }
    for (pcap, log) in cases {
        if pcap.exists() {
            check(&pcap, &log);
        } else {
            eprintln!("\n*** SKIPPED: no capture at {} ***\n", pcap.display());
        }
    }
}

fn check(pcap: &PathBuf, log: &PathBuf) {
    println!("{} against {}", pcap.display(), log.display());
    let cfg = common::config();
    let detector = Detector::new(&cfg.dock);
    let ours = decode_and_detect(pcap, &detector);
    let logged: Vec<Logged> = parse_log(&read_text(log))
        .into_iter()
        .filter(|l| {
            ours.first().is_some_and(|o| l.t >= o.t - 5.0)
                && ours.last().is_some_and(|o| l.t <= o.t + 5.0)
        })
        .collect();
    println!(
        "{} sweeps decoded, {} logged by reflective_column_node in their span",
        ours.len(),
        logged.len()
    );
    assert!(
        !logged.is_empty(),
        "the log has no sweep in the capture's span"
    );
    // the clock offset (log minus capture): the median over logged sweeps of the best centre match within 2 s
    let mut offsets: Vec<f64> = logged
        .iter()
        .filter_map(|l| {
            ours.iter()
                .filter(|o| (l.t - o.t).abs() < 2.0)
                .map(|o| (center_gap(&l.trace, &o.trace), l.t - o.t))
                .filter(|(gap, _)| *gap < 1e-4)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, dt)| dt)
        })
        .collect();
    assert!(
        offsets.len() > 10,
        "fewer than 10 sweeps pair by their centres"
    );
    offsets.sort_by(f64::total_cmp);
    let offset = offsets[offsets.len() / 2];
    println!("log clock - capture clock: {:.3} s", offset);
    let (mut paired, mut step1_same, mut ratios_same, mut counts_same, mut found_same) =
        (0, 0, 0, 0, 0);
    let (mut posed, mut centers_same, mut poses_same) = (0, 0, 0);
    let (mut max_center, mut max_pose, mut max_ratio) = (0.0f64, [0.0f64; 3], 0.0f64);
    for l in &logged {
        // replay-only pairing, keyed on the expected centres: among sweeps within the window, the best match
        let Some(o) = ours
            .iter()
            .filter(|o| (l.t - offset - o.t).abs() < PAIR_WINDOW_S)
            .min_by(|a, b| {
                let key = |o: &Ours| (center_gap(&l.trace, &o.trace), (l.t - offset - o.t).abs());
                let (ka, kb) = (key(a), key(b));
                ka.0.total_cmp(&kb.0).then(ka.1.total_cmp(&kb.1))
            })
        else {
            continue;
        };
        paired += 1;
        step1_same += (o.trace.step1 == l.trace.step1) as usize;
        let ours_ratios: Vec<f64> = o.trace.ratios.iter().map(|r| *r as f64).collect();
        let logged_ratios = &l.ratios;
        ratios_same += printed(&ours_ratios, logged_ratios) as usize;
        for (a, b) in ours_ratios.iter().zip(logged_ratios) {
            max_ratio = max_ratio.max((a - b).abs());
        }
        counts_same += (o.trace.counts == l.trace.counts) as usize;
        found_same += (o.position.is_some() == l.position.is_some()) as usize;
        if let (Some(a), Some(b), Some(ca), Some(cb)) =
            (o.position, l.position, o.trace.centers, l.trace.centers)
        {
            posed += 1;
            max_center = max_center.max(center_gap(&o.trace, &l.trace));
            for k in 0..3 {
                max_pose[k] = max_pose[k].max((a[k] - b[k]).abs());
            }
            centers_same += printed(&ca.concat(), &cb.concat()) as usize;
            poses_same += printed(&a, &b) as usize;
            if !printed(&a, &b) {
                println!(
                    "  printed differently at {:.3}: ours {a:?} logged {b:?}",
                    l.t
                );
            }
        }
    }
    println!(
        "{paired} paired: step 1 counts equal {step1_same}, z/y ratios {ratios_same} (max diff {max_ratio:.1e}), post \
         counts {counts_same}, found / not found {found_same}; {posed} with both poses: centres {centers_same}, poses \
         {poses_same} as the log prints them; max diff centre {max_center:.2e} m, x {:.2e} m, y {:.2e} m, yaw {:.2e} rad",
        max_pose[0], max_pose[1], max_pose[2]
    );
    assert!(
        paired * 10 >= logged.len() * 9,
        "{paired} of {} logged sweeps paired",
        logged.len()
    );
    assert!(
        posed * 10 >= paired * 8,
        "{posed} of {paired} with both poses"
    );
    // every value reflective_column_node logs, as it prints it (%f)
    assert_eq!(step1_same, paired, "point counts after step 1 differ");
    assert_eq!(ratios_same, paired, "cluster z/y ratios print differently");
    assert_eq!(counts_same, paired, "post point counts differ");
    assert_eq!(found_same, paired, "found / not found differ");
    assert_eq!(centers_same, posed, "post centres print differently");
    assert_eq!(poses_same, posed, "positions print differently");
}

/// Ours printed as the vendor prints (%f, 6 decimals) equals the logged values.
fn printed(ours: &[f64], logged: &[f64]) -> bool {
    ours.len() == logged.len()
        && ours
            .iter()
            .zip(logged)
            .all(|(a, b)| format!("{a:.6}").parse::<f64>().unwrap() == *b)
}
