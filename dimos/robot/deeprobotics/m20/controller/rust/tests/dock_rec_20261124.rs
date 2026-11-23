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

//! The dock sequence against the vendor's on the 2026-11-24 recording (NOS 05:50-05:52 CST: an undock from charging,
//! the operator driving, a dock, the charge confirmed; ~/datasets/m20/dock_rec_20261124): what charge_manager wrote to
//! rl_deploy on the wire (/MOTION_STATE, /GAIT, /CHARGE_CMD steer, /BATTERY_CHARGE_ENABLE: RTPS in the NOS DDS
//! capture) against what [`Docking`] writes, fed what charge_manager was fed: rl_deploy's /MOTION_INFO and
//! /BATTERY_DATA as they arrived on NOS, the ChargeEnable service calls and obstacle readings as its log has them, and
//! the dock poses of our detector on the front Airy's sweeps that reflective_column_node processed, at the time it
//! published each. One 1 kHz tick per ms of the NOS clock.
//!
//! Fixtures (tests/fixtures, cut from the recording): dock_rec_20261124_dds.txt.gz (every user DDS sample of
//! charge_manager and rl_deploy, tools/dds_samples.py), dock_rec_20261124_sweeps.txt.gz (our detector on each sweep
//! reflective_column_node logged, made and checked against the capture by `sweeps_are_the_detector_on_the_capture`
//! when ~/datasets/m20/dock_rec_20261124 is present; M20_REGEN=1 rewrites it), charge_manager_20261124_dock_rec.log.gz,
//! reflective_column_node_20261124.log.gz (the vendor's logs).

mod common;

use std::io::Read;
use std::path::PathBuf;

use m20_controller::dock::docking::{Docking, Phase, View, Write};
use m20_controller::dock::vendor::{ChargeManagerToml, Steer};
use m20_controller::dock::{ChargeReading, DockPose};
use m20_controller::state_machine::State;

/// The replay's tick 0 on the NOS clock (Unix s): charge_manager's charging loop wrote a charge-enable then
/// (1795470616.210740 on the wire), the last before the undock's request.
const T0: f64 = 1795470616.210740;
/// The replay's end: the DDS capture's.
const T_END: f64 = 1795470712.5;
/// The logs' clock: China Standard Time.
const CST_S: f64 = 8.0 * 3600.0;
/// charge_manager's and rl_deploy's DDS participants (GUID prefixes) and their writers (entity ids)
const CHARGE_MANAGER: &str = "010f6742d20a0da300000000";
const RL_DEPLOY: &str = "010fd21b22879f2b00000000";
const STEER: &str = "00000803";
const MOTION_STATE: &str = "00000503";
const GAIT: &str = "00000603";
const CHARGE_ENABLE: &str = "00000703";
const MOTION_INFO: &str = "00001603";
const BATTERY_DATA: &str = "00000c03";

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn read_gz(path: &PathBuf) -> String {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut text = String::new();
    flate2::read::GzDecoder::new(&bytes[..])
        .read_to_string(&mut text)
        .unwrap();
    text
}

// ------------------------------------------------------------------------------------------------- the wire (CDR)

/// One DDS sample: NOS capture time (Unix s), writer GUID (prefix + entity), serialized payload after its 4-byte
/// encapsulation header (little-endian CDR).
struct Sample {
    t: f64,
    prefix: String,
    entity: String,
    cdr: Vec<u8>,
}

fn samples() -> Vec<Sample> {
    read_gz(&fixture("dock_rec_20261124_dds.txt.gz"))
        .lines()
        .filter_map(|l| {
            let v: Vec<&str> = l.split(' ').collect();
            let bytes: Vec<u8> = (0..v[4].len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&v[4][i..i + 2], 16).unwrap())
                .collect();
            // little-endian CDR (the ChargeEnable service's request / reply carry their own header)
            (bytes[..2] == [0, 1]).then_some(())?;
            Some(Sample {
                t: v[0].parse::<f64>().unwrap() / 1e9,
                prefix: v[2][..24].to_string(),
                entity: v[2][24..].to_string(),
                cdr: bytes[4..].to_vec(),
            })
        })
        .collect()
}

fn f32_at(d: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(d[o..o + 4].try_into().unwrap())
}

fn u32_at(d: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(d[o..o + 4].try_into().unwrap())
}

/// dr_msgs Steer, MotionState, Gait (a MetaType header: u64 frame_id, i32 sec, u32 nsec, then the value) and
/// StdMsgInt32 as charge_manager wrote them.
fn vendor_writes(all: &[Sample]) -> Vec<(f64, Write)> {
    all.iter()
        // after the write at T0 (the replay starts from it)
        .filter(|s| s.prefix == CHARGE_MANAGER && s.t > T0 + 5e-4 && s.t < T_END)
        .filter_map(|s| {
            let d = &s.cdr;
            let w = match s.entity.as_str() {
                STEER => {
                    let [x, y, z, roll, pitch, yaw] =
                        [16, 20, 24, 28, 32, 36].map(|o| f32_at(d, o));
                    assert_eq!(
                        (roll, pitch),
                        (0.0, 0.0),
                        "charge_manager steers no roll / pitch"
                    );
                    Write::Steer(Steer { x, y, yaw, z })
                }
                MOTION_STATE => Write::MotionState(u32_at(d, 16) as i32),
                GAIT => Write::Gait(u32_at(d, 16)),
                CHARGE_ENABLE => Write::ChargeEnable(u32_at(d, 0) as i32),
                _ => return None,
            };
            Some((s.t, w))
        })
        .collect()
}

/// rl_deploy's /MOTION_INFO: (time, motion state, gait).
fn motion_info(all: &[Sample]) -> Vec<(f64, i32, u32)> {
    all.iter()
        .filter(|s| s.prefix == RL_DEPLOY && s.entity == MOTION_INFO)
        .map(|s| (s.t, u32_at(&s.cdr, 32) as i32, u32_at(&s.cdr, 36)))
        .collect()
}

/// rl_deploy's /BATTERY_DATA as charge_manager's BatteryStatusCallback reads it (mos_detection on, current_detection
/// off): (time, packs with a battery level, of those with the charge MOS (bit 0) on).
fn battery(all: &[Sample]) -> Vec<(f64, u8, u8)> {
    all.iter()
        .filter(|s| s.prefix == RL_DEPLOY && s.entity == BATTERY_DATA)
        .map(|s| {
            let d = &s.cdr;
            let n = u32_at(d, 16) as usize;
            let mut o = 20;
            let (mut present, mut mos) = (0, 0);
            for _ in 0..n {
                // 9 u16, then software_version, battery_level, mos_state, battery_quantity, battery_ntc (u8)
                o = (o + 1) & !1;
                let (level, mos_state) = (d[o + 19], d[o + 20]);
                o += 23;
                o = (o + 3) & !3;
                let temps = u32_at(d, o) as usize;
                o += 4 + 4 * temps + 32;
                if level != 0 {
                    present += 1;
                    mos += (mos_state & 1) as u8;
                }
            }
            (s.t, present, mos)
        })
        .collect()
}

// -------------------------------------------------------------------------------------------------------- the logs

/// "[2026-11-24 05:50:21.858.430] ..." -> Unix s.
fn log_time(line: &str) -> Option<f64> {
    let s = line.get(1..28)?;
    let n = |a: usize, b: usize| s.get(a..b)?.parse::<i64>().ok();
    let (y, m, d) = (n(0, 4)?, n(5, 7)?, n(8, 10)?);
    // days from civil (Howard Hinnant)
    let yy = if m <= 2 { y - 1 } else { y };
    let era = yy.div_euclid(400);
    let yoe = yy - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1;
    let days = era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468;
    let secs = days * 86400 + n(11, 13)? * 3600 + n(14, 16)? * 60 + n(17, 19)?;
    Some(secs as f64 + n(20, 23)? as f64 / 1e3 + n(24, 27)? as f64 / 1e6 - CST_S)
}

/// The numbers in a line, in order.
fn numbers(line: &str) -> Vec<f64> {
    let mut out = Vec::new();
    let mut cur = String::new();
    for c in line.chars().chain([' ']) {
        if c.is_ascii_digit() || c == '.' || (c == '-' && cur.is_empty()) {
            cur.push(c);
        } else {
            if let Ok(v) = cur.parse::<f64>() {
                if cur.contains(|c: char| c.is_ascii_digit()) {
                    out.push(v);
                }
            }
            cur.clear();
        }
    }
    out
}

/// What charge_manager's log has that the replay feeds: the ChargeEnable calls (time, command), the obstacle check's
/// readings (time, m), the undock's goal from the dock before ("mInitX" while undocking) and its detector service calls
/// (time each returned).
struct CmLog {
    requests: Vec<(f64, i32)>,
    /// when it took a pose (logged it, or found the dock with it)
    poses: Vec<f64>,
    obstacles: Vec<(f64, f32)>,
    start_point: [f32; 3],
    service_calls: Vec<f64>,
}

fn cm_log() -> CmLog {
    let text = read_gz(&fixture("charge_manager_20261124_dock_rec.log.gz"));
    let mut log = CmLog {
        requests: Vec::new(),
        poses: Vec::new(),
        obstacles: Vec::new(),
        start_point: [0.0; 3],
        service_calls: Vec::new(),
    };
    for line in text.lines() {
        let Some(t) = log_time(line) else { continue };
        if line.contains("get Target position:")
            || line.contains("mInitYaw x:")
            || line.contains("Find target success")
        {
            log.poses.push(t);
        }
        if line.contains("Receive ChargeEnable, command is [") {
            log.requests
                .push((t, numbers(line.rsplit('[').next().unwrap())[0] as i32));
        } else if let Some(d) = line.split("最近障碍物距离为: ").nth(1) {
            log.obstacles.push((t, d.trim().parse().unwrap()));
        } else if line.contains("mInitX:") && log.start_point == [0.0; 3] {
            let v: Vec<f32> = ["mInitX:", "mInitY:", "mInitYaw:"]
                .iter()
                .map(|k| {
                    let s = line.split(k).nth(1).unwrap();
                    s[..s.find([',', ' ']).unwrap_or(s.len())].parse().unwrap()
                })
                .collect();
            log.start_point = [v[0], v[1], v[2]];
        } else if line.contains("location service call success") {
            log.service_calls.push(t);
        }
    }
    log
}

/// One sweep reflective_column_node processed: when its last line was logged (it publishes then), its step 1 time,
/// and the position it logged ("一次计算--雷达位置": x, y, yaw), if any.
struct RcSweep {
    t_step1: f64,
    t_done: f64,
    position: Option<[f64; 3]>,
}

fn rc_log() -> Vec<RcSweep> {
    let text = read_gz(&fixture("reflective_column_node_20261124.log.gz"));
    let mut out: Vec<RcSweep> = Vec::new();
    for line in text.lines() {
        let Some(body) = line.split("[Location_node] ").nth(1) else {
            continue;
        };
        let t = log_time(line).unwrap();
        if body.starts_with("【step1】") {
            out.push(RcSweep {
                t_step1: t,
                t_done: t,
                position: None,
            });
            continue;
        }
        let Some(cur) = out.last_mut() else { continue };
        if body.starts_with("ServiceCallback") || body.starts_with("=====") {
            continue;
        }
        if t - cur.t_step1 < 0.08 {
            cur.t_done = t;
        }
        if body.starts_with("【step 3】一次计算") {
            let v = numbers(body);
            cur.position = Some([v[v.len() - 4], v[v.len() - 3], v[v.len() - 1]]);
        }
    }
    out
}

// ------------------------------------------------------------------------------------------------ the dock poses

/// Our detector on one sweep reflective_column_node logged: (its log's step 1 time, capture time, reflective_column_
/// node's position as ours computes it, our dock pose), Rust's shortest round-trip decimals.
#[derive(Debug, Clone, PartialEq)]
struct Sweep {
    t_step1: f64,
    t_capture: f64,
    found: Option<([f64; 3], DockPose)>,
}

fn sweep_line(s: &Sweep) -> String {
    match &s.found {
        Some((v, p)) => format!(
            "{:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
            s.t_step1, s.t_capture, v[0], v[1], v[2], p.x, p.y, p.yaw
        ),
        None => format!("{:?} {:?}", s.t_step1, s.t_capture),
    }
}

fn sweeps() -> Vec<Sweep> {
    read_gz(&fixture("dock_rec_20261124_sweeps.txt.gz"))
        .lines()
        .map(|l| {
            let v: Vec<f64> = l.split(' ').map(|x| x.parse().unwrap()).collect();
            Sweep {
                t_step1: v[0],
                t_capture: v[1],
                found: (v.len() == 8).then(|| {
                    (
                        [v[2], v[3], v[4]],
                        DockPose {
                            x: v[5],
                            y: v[6],
                            yaw: v[7],
                        },
                    )
                }),
            }
        })
        .collect()
}

/// Printed as the vendor prints (%f) equals the logged value.
fn printed(ours: f64, logged: f64) -> bool {
    format!("{ours:.6}").parse::<f64>().unwrap() == logged
}

/// Our detector's position on every sweep reflective_column_node logged (the committed sweeps) is the one it logged,
/// as it prints it (%f), and found / not found the same.
#[test]
fn every_logged_position_is_ours_as_printed() {
    let rc = rc_log();
    let ours = sweeps();
    assert_eq!(ours.len(), rc.len());
    let (mut posed, mut max) = (0, 0.0f64);
    for (o, r) in ours.iter().zip(&rc) {
        assert_eq!(o.t_step1, r.t_step1);
        assert_eq!(o.found.is_some(), r.position.is_some(), "at {}", r.t_step1);
        if let (Some((v, _)), Some(p)) = (&o.found, r.position) {
            posed += 1;
            for k in 0..3 {
                assert!(
                    printed(v[k], p[k]),
                    "at {}: ours {v:?}, logged {p:?}",
                    r.t_step1
                );
                max = max.max((v[k] - p[k]).abs());
            }
        }
    }
    println!(
        "{} sweeps, {posed} positions, max |ours - logged| {max:.1e}",
        rc.len()
    );
}

/// The detector on the capture (when present) gives the committed sweeps, bit for bit, and reflective_column_node's
/// logged position on each, as it prints it. M20_REGEN=1 writes them.
#[test]
fn sweeps_are_the_detector_on_the_capture() {
    use m20_controller::dock::detector::{Detection, Detector};
    use robosense_airy::{Decoder, DecoderConfig};
    let pcap = std::env::var_os("M20_DOCK_REC_20261124")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(std::env::var("HOME").unwrap_or_default())
                .join("datasets/m20/dock_rec_20261124")
        })
        .join("nos/lidar_20261124_054951.pcap");
    if !pcap.exists() {
        eprintln!(
            "\n\n*** SKIPPED: no capture at {} (set M20_DOCK_REC_20261124 to the recording's directory); the committed \
             sweeps are not checked against the detector ***\n",
            pcap.display()
        );
        return;
    }
    let cfg = common::config();
    let detector = Detector::new(&cfg.dock);
    let mut decoder = Decoder::new(DecoderConfig {
        // the vendor driver's (NOS node_driver config.yaml)
        min_distance: 0.2,
        max_distance: 60.0,
        use_lidar_clock: true,
        dense_points: true,
        ts_first_point: true,
        fused_multiply_add: true,
        ..Default::default()
    });
    let rc = rc_log();
    let (first, last) = (rc[0].t_step1 - 1.0, rc[rc.len() - 1].t_step1 + 1.0);
    let mut ours: Vec<(f64, Option<([f64; 3], DockPose)>)> = Vec::new();
    for g in robosense_airy::pcap::read_udp(&pcap)
        .unwrap()
        .iter()
        .filter(|g| g.dst == [224, 10, 10, 201])
    {
        if g.dst_port == 7781 {
            let _ = decoder.decode_difop(&g.payload);
            continue;
        }
        if g.dst_port != 6691 {
            continue;
        }
        let Ok(Some(scan)) = decoder.decode_msop(&g.payload) else {
            continue;
        };
        if !(first..last).contains(&g.time) {
            continue;
        }
        let points = scan
            .points
            .iter()
            .map(|p| [p.x, p.y, p.z, p.intensity as f32]);
        ours.push((
            g.time,
            match detector.detect(points) {
                Detection::Found { pose, vendor, .. } => Some((vendor, pose)),
                Detection::Missing(_) => None,
            },
        ));
    }
    // each logged sweep is the last captured before its step 1 line (it processes a sweep ~15 ms after it ends)
    let made: Vec<Sweep> = rc
        .iter()
        .map(|r| {
            let (t, found) = ours
                .iter()
                .rev()
                .find(|(t, _)| *t < r.t_step1 && r.t_step1 - t < 0.06)
                .unwrap_or_else(|| panic!("no captured sweep for the one logged at {}", r.t_step1))
                .clone();
            Sweep {
                t_step1: r.t_step1,
                t_capture: t,
                found,
            }
        })
        .collect();
    let (mut posed, mut same) = (0, 0);
    for (m, r) in made.iter().zip(&rc) {
        assert_eq!(
            m.found.is_some(),
            r.position.is_some(),
            "found / not found differ at {}",
            r.t_step1
        );
        if let (Some((v, _)), Some(p)) = (&m.found, r.position) {
            posed += 1;
            same += (0..3).all(|k| printed(v[k], p[k])) as usize;
        }
    }
    println!(
        "{} sweeps logged, {posed} with a position, {same} printed as reflective_column_node printed them",
        rc.len()
    );
    assert_eq!(same, posed);
    let text: String = made.iter().map(|s| sweep_line(s) + "\n").collect();
    let path = fixture("dock_rec_20261124_sweeps.txt.gz");
    if std::env::var_os("M20_REGEN").is_some() {
        use std::io::Write as _;
        let mut gz = flate2::write::GzEncoder::new(
            std::fs::File::create(&path).unwrap(),
            flate2::Compression::best(),
        );
        gz.write_all(text.as_bytes()).unwrap();
        gz.finish().unwrap();
        println!("wrote {}", path.display());
    }
    assert_eq!(
        made,
        sweeps(),
        "the committed sweeps are not the detector's"
    );
}

// --------------------------------------------------------------------------------------------------- the replay

/// What one write of ours is compared on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Kind {
    MotionState,
    Gait,
    Steer,
    ChargeEnable,
}

fn kind(w: &Write) -> Kind {
    match w {
        Write::MotionState(_) => Kind::MotionState,
        Write::Gait(_) => Kind::Gait,
        Write::Steer(_) => Kind::Steer,
        Write::ChargeEnable(_) => Kind::ChargeEnable,
    }
}

/// f32 units in the last place between a and b.
fn ulps(a: f32, b: f32) -> u32 {
    if a == b {
        return 0;
    }
    let key = |f: f32| {
        let i = f.to_bits() as i32;
        if i < 0 {
            i32::MIN - i
        } else {
            i
        }
    };
    (key(a) as i64 - key(b) as i64).unsigned_abs() as u32
}

/// The replay: Docking fed the recorded inputs at their NOS times, one tick per ms from T0; its writes with their times.
/// `start_point`: the undock's goal from the dock before the recording.
fn replay(start_point: [f32; 3]) -> Vec<(f64, Write)> {
    replay_traced(start_point).0
}

/// [`replay`], and when the sequence entered each phase.
fn replay_traced(start_point: [f32; 3]) -> (Vec<(f64, Write)>, Vec<(f64, Phase)>) {
    let cfg = common::config();
    let all = samples();
    let infos = motion_info(&all);
    let batteries = battery(&all);
    let log = cm_log();
    let rc = rc_log();
    let sweeps = sweeps();
    assert_eq!(sweeps.len(), rc.len());
    let vendor = ChargeManagerToml::load(fixture("charge_manager.toml").to_str().unwrap())
        .unwrap()
        .0;
    let mut docking = Docking::new(&cfg, true, Some(&vendor));
    docking.record_writes();
    // charging since the dock before the recording: its start point as the undock logs it, restoring rl_control on
    // basic_stair (0x3003, "mInitRobotGaitStatus_ :0x3003")
    docking.resume_charging(start_point, (17, 0x3003));
    let tick_of = |t: f64| ((t - T0) * 1000.0).ceil().max(0.0) as u64;
    let mut view = View::default();
    let set_view = |view: &mut View, state: i32, gait: u32| {
        let state = match state {
            1 => State::StandUp,
            4 => State::LieDown,
            17 => State::RlControl,
            _ => State::Passive,
        };
        if view.state != Some(state) {
            view.state = Some(state);
            view.state_ticks = 0;
        }
        view.gait = gait;
        view.runner = Some(cfg.runner_for_gait(gait).to_string());
    };
    if let Some(&(_, s, g)) = infos.iter().rev().find(|i| i.0 <= T0) {
        set_view(&mut view, s, g);
    }
    // the poses: each found sweep, when charge_manager took it (logged it), else when reflective_column_node logged
    // its last line for it (published it)
    let poses: Vec<(u64, DockPose)> = sweeps
        .iter()
        .zip(&rc)
        .enumerate()
        .filter_map(|(i, (s, r))| {
            let next = rc.get(i + 1).map_or(f64::INFINITY, |n| n.t_step1);
            let taken = log
                .poses
                .iter()
                .find(|t| **t >= r.t_done && **t < next.min(r.t_done + 0.05))
                .copied()
                .unwrap_or(r.t_done);
            s.found.as_ref().map(|(_, p)| (tick_of(taken), *p))
        })
        .collect();
    let (mut ii, mut bi, mut pi, mut oi, mut ri) = (0, 0, 0, 0, 0);
    let (mut pose, mut charge, mut obstacle): (
        Option<(DockPose, u64)>,
        Option<ChargeReading>,
        Option<(Option<f32>, u64)>,
    ) = (None, None, None);
    let end = tick_of(T_END);
    let mut phases = vec![(T0, docking.phase())];
    for k in 1..=end {
        let now = T0 + k as f64 / 1000.0;
        while ii < infos.len() && infos[ii].0 <= now {
            if infos[ii].0 > T0 {
                set_view(&mut view, infos[ii].1, infos[ii].2);
            }
            ii += 1;
        }
        while bi < batteries.len() && batteries[bi].0 <= now {
            let (_, present, mos) = batteries[bi];
            charge = Some(ChargeReading {
                seq: bi as u64 + 1,
                packs_present: present,
                packs_charge_mos_on: mos,
                charger_connected: false,
            });
            bi += 1;
        }
        while pi < poses.len() && poses[pi].0 <= k {
            pose = Some((poses[pi].1, pi as u64 + 1));
            pi += 1;
        }
        while oi < log.obstacles.len() && log.obstacles[oi].0 <= now {
            obstacle = Some((Some(log.obstacles[oi].1), oi as u64 + 1));
            oi += 1;
        }
        docking.ingest(pose, charge, obstacle);
        while ri < log.requests.len() && log.requests[ri].0 <= now {
            let request = match log.requests[ri].1 {
                0 => m20_controller::dock::docking::Request::Undock,
                1 => m20_controller::dock::docking::Request::Dock,
                _ => m20_controller::dock::docking::Request::Cancel,
            };
            docking
                .request(request, &view)
                .unwrap_or_else(|e| panic!("{request:?} refused at {now}: {e}"));
            ri += 1;
        }
        docking.advance(&view);
        view.state_ticks += 1;
        if phases.last().map(|p| p.1) != Some(docking.phase()) {
            phases.push((now, docking.phase()));
        }
    }
    let writes = docking
        .take_writes()
        .into_iter()
        .map(|(tick, w)| (T0 + tick as f64 / 1000.0, w))
        .collect();
    (writes, phases)
}

/// Steers of `ours` more than 4 f32 ULPs from the vendor's, pairing writes in order.
fn steers_off(ours: &[(f64, Write)], theirs: &[(f64, Write)]) -> usize {
    ours.iter()
        .zip(theirs)
        .filter(|((_, a), (_, b))| match (a, b) {
            (Write::Steer(p), Write::Steer(q)) => {
                [(p.x, q.x), (p.y, q.y), (p.yaw, q.yaw), (p.z, q.z)]
                    .iter()
                    .any(|(u, v)| ulps(*u, *v) > 4)
            }
            _ => false,
        })
        .count()
}

/// The undock's goal (the start point of the dock before the recording, charge_manager's f32) is in its log only to 6
/// decimals: of the f32 x values that print as logged, the one the undock's steers imply (fewest steers off). Its y and
/// yaw enter only steers clamped at their limits here.
fn start_point(theirs: &[(f64, Write)]) -> [f32; 3] {
    let logged = cm_log().start_point;
    let mut best = (usize::MAX, logged);
    for k in -8i32..=8 {
        let x = f32::from_bits((logged[0].to_bits() as i32 + k) as u32);
        if (x as f64 - logged[0] as f64).abs() > 5e-7 {
            continue;
        }
        let p = [x, logged[1], logged[2]];
        let off = steers_off(&replay(p), theirs);
        if off < best.0 {
            best = (off, p);
        }
    }
    println!(
        "the undock's goal: logged {logged:?}, implied {:?} ({} steers off)",
        best.1, best.0
    );
    best.1
}

/// The dock ends charging on the tick charge_manager confirmed the charge ("获取电流成功，上桩成功"), and the undock
/// ends undocked.
#[test]
fn the_charge_is_confirmed_on_charge_managers_tick() {
    let theirs = vendor_writes(&samples());
    let (_, phases) = replay_traced(start_point(&theirs));
    let log = log_text();
    let confirmed = log
        .lines()
        .find(|l| l.contains("获取电流成功"))
        .and_then(log_time)
        .unwrap();
    // after the charging the replay starts in
    let charging: Vec<f64> = phases
        .iter()
        .skip(1)
        .filter(|(_, p)| *p == Phase::Charging)
        .map(|(t, _)| *t)
        .collect();
    println!("phases: {phases:?}");
    assert_eq!(charging.len(), 1, "one charge confirmed: {phases:?}");
    println!(
        "charging from {:.6}, charge_manager confirmed at {confirmed:.6}: {:+.3} ms",
        charging[0],
        (charging[0] - confirmed) * 1e3
    );
    assert!((charging[0] - confirmed).abs() < 1e-3);
    assert_eq!(phases.last().unwrap().1, Phase::Charging);
}

#[test]
fn the_dock_and_undock_write_what_charge_manager_wrote() {
    let theirs = vendor_writes(&samples());
    let ours = replay(start_point(&theirs));
    println!(
        "ours {} writes, charge_manager's {}",
        ours.len(),
        theirs.len()
    );
    let n = ours.len().max(theirs.len());
    let mut first_bad = None;
    for i in 0..n {
        let (a, b) = (ours.get(i), theirs.get(i));
        let same = match (a, b) {
            (Some((_, x)), Some((_, y))) => match (x, y) {
                (Write::Steer(p), Write::Steer(q)) => [
                    ulps(p.x, q.x),
                    ulps(p.y, q.y),
                    ulps(p.yaw, q.yaw),
                    ulps(p.z, q.z),
                ]
                .iter()
                .all(|u| *u <= 4),
                _ => x == y,
            },
            _ => false,
        };
        if !same && first_bad.is_none() {
            first_bad = Some(i);
        }
    }
    if let Some(i) = first_bad {
        for j in i.saturating_sub(4)..(i + 8).min(n) {
            println!(
                "{j}: ours {:?}\n    vendor {:?}",
                ours.get(j).map(|(t, w)| (format!("{t:.6}"), *w)),
                theirs.get(j).map(|(t, w)| (format!("{t:.6}"), *w))
            );
        }
        panic!("write {i} differs");
    }
    // every write the same, in the same order (steers within 4 f32 ULPs); now the times. A write is on the vendor's
    // tick when within 1 ms of it, except where charge_manager waited on its detector process (service calls, in its
    // log) or on a 200 ms pose wait that restarts after its own logging (each ~0.2 ms late): see `Late`.
    let late = late_writes(&log_text());
    let mut table: std::collections::BTreeMap<Kind, (usize, f64, u32, f64)> = Default::default();
    let mut off_tick = Vec::new();
    // pose-wait stops in a row so far
    let mut waits = 0;
    for (i, ((ta, a), (tb, b))) in ours.iter().zip(&theirs).enumerate() {
        let e = table.entry(kind(a)).or_default();
        e.0 += 1;
        if let (Write::Steer(p), Write::Steer(q)) = (a, b) {
            for (u, v) in [(p.x, q.x), (p.y, q.y), (p.yaw, q.yaw), (p.z, q.z)] {
                e.1 = e.1.max((u as f64 - v as f64).abs());
                e.2 = e.2.max(ulps(u, v));
            }
        }
        let dt = ta - tb;
        match late.iter().find(|l| l.covers(*tb)) {
            Some(l) => {
                off_tick.push((i, *b, dt, l.why));
                if l.shift == 0.0 {
                    // ours every pose_wait_ms exactly; the vendor's later by up to 0.5 ms a wait
                    waits += 1;
                    assert!(
                        dt < 1e-3 && dt > -5e-4 * waits as f64,
                        "pose-wait stop {waits} (write {i}): {:.3} ms",
                        dt * 1e3
                    );
                    continue;
                }
                waits = 0;
                // on the vendor's tick once its own delay is taken out
                assert!(
                    (dt + l.shift).abs() < 1e-3,
                    "write {i} {b:?}: {:.3} ms from the vendor's after its {:.3} ms {}",
                    (dt + l.shift) * 1e3,
                    l.shift * 1e3,
                    l.why
                );
            }
            None => {
                waits = 0;
                e.3 = e.3.max(dt.abs());
                assert!(
                    dt.abs() < 1e-3,
                    "write {i} {b:?} at {tb:.6}: ours {:.3} ms off",
                    dt * 1e3
                );
            }
        }
    }
    for (k, (count, diff, ulp, dt)) in &table {
        println!(
            "{k:?}: {count} writes, max |diff| {diff:.2e} ({ulp} ulp), max |dt| {:.3} ms (on the vendor's tick)",
            dt * 1e3
        );
    }
    for (i, w, dt, why) in &off_tick {
        println!("  write {i} {w:?}: {:+.3} ms ({why})", dt * 1e3);
    }
}

fn log_text() -> String {
    read_gz(&fixture("charge_manager_20261124_dock_rec.log.gz"))
}

/// Vendor writes that come later than the sequence decides them, from what charge_manager's log shows it waiting on.
struct Late {
    from: f64,
    to: f64,
    /// how much later (s), 0 when it grows write by write
    shift: f64,
    why: &'static str,
}

impl Late {
    fn covers(&self, t: f64) -> bool {
        (self.from..self.to).contains(&t)
    }
}

fn late_writes(log: &str) -> Vec<Late> {
    let lines: Vec<(f64, &str)> = log
        .lines()
        .filter_map(|l| log_time(l).map(|t| (t, l)))
        .collect();
    let find = |after: f64, what: &str| {
        lines
            .iter()
            .find(|(t, l)| *t >= after && l.contains(what))
            .map(|(t, _)| *t)
            .unwrap_or_else(|| panic!("no {what:?} after {after}"))
    };
    let close = "Close location service call success";
    let mut out = Vec::new();
    // the undock's end: restoreInitRobotGaitStatus, then two detector service calls before Close24vPower
    let restored = find(0.0, "mInitRobotGaitStatus_");
    let closed = find(find(restored, close) + 1e-6, close);
    out.push(Late {
        from: closed,
        to: closed + 0.01,
        shift: closed - restored,
        why: "charge_manager closed its detector process (2 service calls) before Close24vPower",
    });
    // charging: the loop starts once the detector process is closed after the charge is confirmed
    let charged = find(0.0, "获取电流成功");
    let loop_start = find(charged, close);
    out.push(Late {
        from: loop_start,
        to: f64::INFINITY,
        shift: loop_start - charged,
        why: "charge_manager's charging loop starts after a detector service call",
    });
    // a pose wait's stop: its 200 ms restart after the last one's logging and write
    for (t, l) in &lines {
        if l.contains("超时，停止运动") {
            out.push(Late {
                from: *t,
                to: t + 0.002,
                shift: 0.0,
                why: "charge_manager's 200 ms pose wait restarts after its log line and write (~0.2 ms each)",
            });
        }
    }
    out
}
