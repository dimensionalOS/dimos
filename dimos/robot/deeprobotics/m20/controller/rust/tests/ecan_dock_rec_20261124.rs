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

//! The controller against rl_deploy on the 2026-11-24 vendor undock and dock (AOS ecan.pcapng: every frame rl_deploy
//! and the motor board exchanged, 05:49:56-05:51:55 CST). Our Controller runs every recorded tick on the joint state
//! rl_deploy computed that tick from (the newest reply before its previous send), stamped with rl_deploy's clock, and
//! gets the commands rl_deploy got when it got them (rl_deploy.log: /MOTION_STATE, /GAIT). Each of its commands,
//! encoded, is compared with rl_deploy's frame of that tick: every byte but the f32 targets equal, the targets within 4
//! f32 ULPs (or 1e-6). The charge-enable requests (/BATTERY_CHARGE_ENABLE as they reached the AOS board) go through
//! the bus's request rule; its frames must be rl_deploy's, byte for byte, on the same ticks.
//!
//! rl_deploy's stand-up / lie-down clock is f32(seconds since it started) (EcanHardwareInterface::Handler stamps each
//! reply): 1.5 to 3 days that morning, so its splines stepped every 1/64 s. The capture cannot say when rl_deploy
//! started; the phase of that clock (one number in [0, 1/64 s)) is fitted on the stand-up and must then hold, tick for
//! tick, through the lie-down 61 s later.
//!
//! The body IMU went over shared memory that day (in no capture), so rl_control's policy targets cannot be compared:
//! the policies run on an upright, still body, and only what does not depend on the IMU is compared in rl_control
//! (gains, control words, the ticks the targets change: start-up gains, stand-ins, gait switches). That part needs
//! the extracted policies (skipped, saying so, without them).
//!
//! Fixture: tests/fixtures/ecan_dock_rec_20261124.bin.gz (64 KB), windows of the capture (tools/ecan_windows.py): the
//! stand-up to rl_control, the gait switches, the lie-down, every charge-enable burst. M20_DOCK_REC_ECAN=PATH (the
//! whole aos/ecan.pcapng of ~/datasets/m20/dock_rec_20261124) runs the whole session instead.

mod common;

use std::collections::BTreeMap;
use std::path::PathBuf;

use m20_controller::config::{Config, JOINTS};
use m20_controller::frames::{decode_command, decode_state, encode_command, Mapping};
use m20_controller::hardware::bus::ReplyLag;
use m20_controller::hardware::ecan::{encode_charge_enable, Charge};
use m20_controller::policy::{InlinePolicy, PolicyService, RunnerSet};
use m20_controller::state_machine::{Controller, State};
use m20_controller::types::{JointCommand, RobotState, Target, UserCommand};
use m20_ecan::{tap, Direction};

const CST_NS: u64 = 8 * 3600 * 1_000_000_000;
/// The one place rl_deploy's clock is not the capture's: at 05:51:26.725644 its loop sent 39 us late (every other send
/// is at .xxx605 +-10 us) and its targets took the lie-down's 1/64 s step a tick early (its stamp is its own
/// clock_gettime when the reply reaches it, which the capture cannot see). Those two ticks are reported, not failed.
const KNOWN_CLOCK_SLIP: [&str; 2] = ["05:51:26.725644", "05:51:26.726606"];
/// A /BATTERY_CHARGE_ENABLE that reached the AOS board's NIC this long before a send is in that send's tick; later, in
/// the next. Fitted on the capture: the 16 requests arrived 6 and 81 us (next tick) or 151 us and more (this tick)
/// before a send, so rl_deploy decides between 81 and 151 us before it sends (its DDS delivery plus its loop's work).
const DECIDE_NS: u64 = 116_000;
/// The reference for rl_deploy's clock: 05:50:00 CST, on this capture's day.
const CLOCK_REF: &str = "05:50:00";
/// rl_deploy's seconds since start at CLOCK_REF, less the fitted phase: inside [2^17, 2^18) s (its 1/64 s steps)
const CLOCK_BASE_S: f64 = 196608.0;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// (capture time ns, direction, frame)
type Frame = (u64, Direction, Vec<u8>);

/// The capture: M20_DOCK_REC_ECAN (the whole pcapng), else the committed windows.
fn capture() -> (Vec<Frame>, bool) {
    if let Some(path) = std::env::var_os("M20_DOCK_REC_ECAN") {
        let bytes = std::fs::read(&path).expect("M20_DOCK_REC_ECAN");
        let frames = tap::read_capture(&bytes)
            .unwrap()
            .into_iter()
            .map(|(t, d, f)| (t, d, f.to_vec()))
            .collect();
        return (frames, true);
    }
    println!(
        "the committed windows (M20_DOCK_REC_ECAN=.../aos/ecan.pcapng replays the whole session)"
    );
    (
        read_compact(&fixture("ecan_dock_rec_20261124.bin.gz")),
        false,
    )
}

/// tools/ecan_windows.py's fixture back into frames: times to the microsecond; commands whole (their header the first
/// command's with their sequence number, its checksum recomputed); replies whole in its full windows, else a 24-byte
/// stand-in (a reply that carries no joint state: rl_deploy's stamp only).
fn read_compact(path: &PathBuf) -> Vec<Frame> {
    use std::io::Read;
    let gz = std::fs::read(path).unwrap();
    let mut b = Vec::new();
    flate2::read::GzDecoder::new(&gz[..])
        .read_to_end(&mut b)
        .unwrap();
    let n = u32::from_le_bytes(b[..4].try_into().unwrap()) as usize;
    let header = b[4..28].to_vec();
    let kinds = b[28..28 + n].to_vec();
    let frames = kinds.iter().filter(|k| **k != b'Q').count();
    let mut times = 28 + n;
    let mut side = times + 8 + 4 * (frames - 1);
    let side_len: usize = kinds
        .iter()
        .map(|k| match k {
            b'S' => 452,
            b'B' => 24,
            b'Q' => 1,
            _ => 0,
        })
        .sum();
    let mut units = side + side_len;
    let (mut t, mut seq, mut prev_units) = (0u64, 0u8, vec![0u8; 16 * 24]);
    let mut out = Vec::new();
    for (i, k) in kinds.iter().enumerate() {
        if *k == b'Q' {
            seq = b[side].wrapping_sub(1);
            side += 1;
            continue;
        }
        if i == 0 || out.is_empty() {
            t = i64::from_le_bytes(b[times..times + 8].try_into().unwrap()) as u64 * 1000;
            times += 8;
        } else {
            t = (t as i64
                + i32::from_le_bytes(b[times..times + 4].try_into().unwrap()) as i64 * 1000)
                as u64;
            times += 4;
        }
        let frame = match k {
            b'S' => {
                side += 452;
                (Direction::FromBoard, b[side - 452..side].to_vec())
            }
            b's' => {
                let mut f = vec![0u8; 24];
                f[19] = 0x89;
                (Direction::FromBoard, f)
            }
            _ => {
                seq = seq.wrapping_add(1);
                match k {
                    b'B' => {
                        side += 24;
                        (Direction::ToBoard, b[side - 24..side].to_vec())
                    }
                    b'C' | b'r' => {
                        if *k == b'C' {
                            for (j, u) in prev_units.iter_mut().enumerate() {
                                *u ^= b[units + j];
                            }
                            units += 16 * 24;
                        }
                        let mut f = header.clone();
                        f[16] = seq;
                        let sum = prev_units.iter().fold(0u32, |a, &x| a + x as u32) as u16;
                        f[22..24].copy_from_slice(&sum.to_le_bytes());
                        f.extend_from_slice(&prev_units);
                        (Direction::ToBoard, f)
                    }
                    other => panic!("fixture kind {other}"),
                }
            }
        };
        out.push((t, frame.0, frame.1));
    }
    out
}

/// One tick: the frame rl_deploy sent, and the reply it computed it from.
#[derive(Clone)]
struct Tick {
    t: u64,
    frame: Vec<u8>,
    /// the newest decoded joint state before rl_deploy's previous send (None in a cut)
    state: Option<RobotState>,
    /// that reply's capture time (rl_deploy's stamp)
    state_t: Option<u64>,
}

fn load(frames: &[Frame], cfg: &Config) -> Vec<Tick> {
    let map = Mapping::new(&cfg.ecan).unwrap();
    let mut out = Vec::new();
    // the live bus's lag (BusHardware): each send hands the next tick the newest reply before it
    let mut lag: ReplyLag<(RobotState, u64)> = ReplyLag::default();
    let mut state: Option<RobotState> = None;
    let mut computed_from: Option<(RobotState, u64)> = None;
    let mut last_t = 0;
    for (t, d, f) in frames {
        let (t, d, f) = (*t, *d, &f[..]);
        // a gap (between windows): nothing carries over
        if t > last_t + 50_000_000 {
            state = None;
            lag = ReplyLag::default();
            computed_from = None;
        }
        last_t = t;
        match d {
            Direction::FromBoard if f.get(19) == Some(&0x89) => {
                // a reply cut to its header keeps the state before it (its time is what rl_deploy stamps)
                if let Ok(Some(s)) = decode_state(f, cfg.ecan.ethertype, &map) {
                    state = Some(s);
                }
                if let Some(s) = state.clone() {
                    lag.reply((s, t));
                }
            }
            Direction::FromBoard => {}
            Direction::ToBoard => {
                out.push(Tick {
                    t,
                    frame: f.to_vec(),
                    state: computed_from.as_ref().map(|c| c.0.clone()),
                    state_t: computed_from.as_ref().map(|c| c.1),
                });
                computed_from = lag.send();
            }
        }
    }
    out
}

/// A capture time from "HH:MM:SS.ffffff" (CST) on the capture's day.
fn at(first: u64, hms: &str) -> u64 {
    let day = (first + CST_NS) / 86_400_000_000_000 * 86_400_000_000_000;
    let v: Vec<f64> = hms.split(':').map(|x| x.parse().unwrap()).collect();
    day + ((v[0] * 3600.0 + v[1] * 60.0) * 1e9) as u64 + (v[2] * 1e9).round() as u64 - CST_NS
}

fn clock(t: u64) -> String {
    let t = t + CST_NS;
    let s = t / 1_000_000_000 % 86400;
    format!(
        "{:02}:{:02}:{:02}.{:06}",
        s / 3600,
        s / 60 % 60,
        s % 60,
        t % 1_000_000_000 / 1000
    )
}

/// rl_deploy's stamp of a reply (s since it started), for a clock phase (s).
fn stamp(first: u64, t: u64, phase: f64) -> f64 {
    let r = at(first, CLOCK_REF);
    CLOCK_BASE_S + phase + (t as f64 - r as f64) / 1e9
}

/// What rl_deploy was told and when (rl_deploy.log, AOS clock): (time, state code or gait).
fn commands(first: u64) -> Vec<(u64, Option<i32>, Option<u32>)> {
    let text = std::fs::read_to_string(fixture("rl_deploy_20261124.log")).unwrap();
    text.lines()
        .filter_map(|l| {
            // "[2026-11-24 05:50:24.839.702]" -> "05:50:24.839702"
            let hms = l.get(12..28)?;
            let t = at(first, &format!("{}{}", &hms[..12], &hms[13..]));
            let n = |key: &str| {
                l.split(key)
                    .nth(1)
                    .and_then(|r| r.trim().parse::<i64>().ok())
            };
            match (n("[Target_State]"), n("[Target_Gait]")) {
                (Some(s), _) => Some((t, Some(s as i32), None)),
                (_, Some(g)) => Some((t, None, Some(g as u32))),
                _ => None,
            }
        })
        .collect()
}

/// The charge-enable requests as they reached the AOS board (capture ns, value).
fn charge_requests() -> Vec<(u64, i32)> {
    std::fs::read_to_string(fixture("charge_enable_20261124.txt"))
        .unwrap()
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|l| {
            let v: Vec<&str> = l.split(' ').collect();
            (v[0].parse().unwrap(), v[1].parse().unwrap())
        })
        .collect()
}

fn ulps(a: f32, b: f32) -> u64 {
    let k = |x: f32| {
        let i = x.to_bits() as i32 as i64;
        if i < 0 {
            i32::MIN as i64 - i
        } else {
            i
        }
    };
    (k(a) - k(b)).unsigned_abs()
}

/// One compared output: how many, the worst, and every cell outside the tolerance (first few kept).
#[derive(Debug, Default)]
struct Cell {
    n: usize,
    max_ulp: u64,
    max_abs: f64,
    bad: usize,
    first_bad: Vec<String>,
}

impl Cell {
    fn fail(&mut self, what: String) {
        self.bad += 1;
        if self.first_bad.len() < 8 {
            self.first_bad.push(what);
        }
    }
}

#[derive(Debug, Default)]
struct Report {
    /// deterministic states' targets (q, dq, tau as f32 on the wire)
    targets: Cell,
    /// every byte but the targets (unit ids, gains, words)
    bytes: Cell,
    /// rl_control: the ticks rl_deploy's targets changed vs ours
    updates: Cell,
    /// charge-enable frames: same ticks, same bytes
    charge: Cell,
    /// KNOWN_CLOCK_SLIP's ticks: their targets, reported apart
    slip: Cell,
    /// runs of ticks whose bytes differ: (first, last, ticks, ours kp/kd of unit 1, rl_deploy's)
    byte_runs: Vec<(String, String, usize, String, String)>,
    /// state transitions: (tick, from, to)
    transitions: Vec<(usize, State, State)>,
    ticks: usize,
}

/// Compare our encoded command with rl_deploy's frame: targets by ULP (when `targets`), everything else byte for byte.
fn compare(ours: &[u8], theirs: &[u8], targets: bool, when: &str, r: &mut Report) {
    r.bytes.n += 1;
    let mut bytes_ok = ours.len() == theirs.len();
    for k in 0..JOINTS {
        let (a, b) = (
            &ours[24 + k * 24..48 + k * 24],
            &theirs[24 + k * 24..48 + k * 24],
        );
        if a[..4] != b[..4] || a[16..] != b[16..] {
            bytes_ok = false;
            r.bytes.fail(format!(
                "{when} unit {k}: {:02x?} vs rl_deploy {:02x?}",
                &a[16..22],
                &b[16..22]
            ));
        }
        if !targets {
            continue;
        }
        for (name, o) in [("q", 4usize), ("dq", 8), ("tau", 12)] {
            let x = f32::from_le_bytes(a[o..o + 4].try_into().unwrap());
            let y = f32::from_le_bytes(b[o..o + 4].try_into().unwrap());
            let (u, d) = (ulps(x, y), (x - y).abs() as f64);
            r.targets.n += 1;
            r.targets.max_ulp = r.targets.max_ulp.max(u);
            r.targets.max_abs = r.targets.max_abs.max(d);
            if u > 4 && d > 1e-6 {
                r.targets.fail(format!(
                    "{when} unit {k} {name}: {x} vs rl_deploy {y} ({u} ulp)"
                ));
            }
        }
    }
    if bytes_ok && ours[..12] != theirs[..12] {
        r.bytes.fail(format!("{when}: Ethernet header"));
    }
}

/// The policies, or None (rl_control is then not replayed). Each has run before, as rl_deploy's had (up 1.5 days):
/// a sequence runner's history is full, so it infers from its first step instead of holding its warm-up pose.
fn policies(cfg: &Config) -> Option<RunnerSet> {
    let mut runners = RunnerSet::load(cfg, &common::policy_dir()?).unwrap();
    let mut s = RobotState {
        quat: [1.0, 0.0, 0.0, 0.0],
        accel: [0.0, 0.0, 9.81],
        ..Default::default()
    };
    s.q = m20_controller::state_machine::leg_pose_for_height(&cfg.robot, 0.38, 0.0);
    for (k, name) in cfg.runners.keys().enumerate() {
        for i in 0..40 {
            runners.serve(&m20_controller::policy::PolicyRequest {
                runner: name.clone(),
                reset: false,
                entry: false,
                switch: false,
                activate: false,
                state: s.clone(),
                axes: [0.0; 3],
                params: BTreeMap::new(),
                seq: (k * 40 + i) as u64,
            });
        }
    }
    Some(runners)
}

struct Replay {
    cfg: Config,
    map: Mapping,
    first: u64,
    phase: f64,
}

/// A window of ticks to replay: the controller starts in `start` (lying since long before, or on rl_control with the
/// gait in force), the comparison from `compare_from`.
struct Window {
    from: u64,
    to: u64,
    start: State,
    /// starting in rl_control: the gait in force before the window (its runner's gains bridge a switch at the start)
    before: Option<u32>,
}

impl Replay {
    fn user(&self, cmds: &[(u64, Option<i32>, Option<u32>)], until: u64) -> UserCommand {
        let (mut target, mut gait) = (Target::LieDown, 0);
        for &(t, s, g) in cmds.iter().filter(|c| c.0 <= until) {
            let _ = t;
            if let Some(s) = s {
                target = match s {
                    1 => Target::Stand,
                    17 => Target::RlControl,
                    4 => Target::LieDown,
                    2 => Target::JointDamping,
                    _ => Target::Passive,
                };
            }
            if let Some(g) = g {
                gait = g;
            }
        }
        UserCommand {
            target,
            gait,
            ..Default::default()
        }
    }

    /// Replay the ticks in `w`, comparing from its start; `runners` for rl_control (else the window must not reach it).
    fn run(&self, ticks: &[Tick], w: &Window, runners: Option<RunnerSet>) -> Report {
        let mut r = Report::default();
        let cmds = commands(self.first);
        let requests = charge_requests();
        let have_policies = runners.is_some();
        let mut policy = InlinePolicy::new(runners.unwrap_or_else(RunnerSet::empty));
        let mut c = Controller::new(self.cfg.clone());
        // rl_deploy's faults are not ours to raise on a body we cannot see (no IMU)
        c.set_fault_checks(false);
        let mut charge = Charge::new();
        charge.set_rule(false);
        let idx: Vec<usize> = (0..ticks.len())
            .filter(|&k| ticks[k].t >= w.from && ticks[k].t <= w.to)
            .collect();
        let Some(&k0) = idx.first() else {
            panic!("no ticks in {}..{}", clock(w.from), clock(w.to));
        };
        let state_of = |k: usize| -> Option<RobotState> {
            let mut s = ticks[k].state.clone()?;
            s.stamp = stamp(self.first, ticks[k].state_t?, self.phase);
            s.quat = [1.0, 0.0, 0.0, 0.0];
            s.accel = [0.0, 0.0, 9.81];
            Some(s)
        };
        let s0 = state_of(k0).expect("a joint state at the window's start");
        let cmd0 = self.user(&cmds, ticks[k0 - 1].t);
        match w.start {
            State::LieDown => {
                // lying since long before
                let mut early = s0.clone();
                early.stamp -= 60.0;
                c.force(State::LieDown, &early, &cmd0, false);
            }
            State::RlControl => {
                assert!(have_policies);
                let mut cmd0 = cmd0.clone();
                if let Some(g) = w.before {
                    cmd0.gait = g;
                }
                // rl_control is entered on an entry gait (else rl_deploy protects to BasicFlat): on another, entered
                // on NaviCharge as on the robot, then switched to it (the warm-up's 2 s are past its stand-in)
                let entry = &self.cfg.states.rl_control.entry_gaits;
                let mut enter = cmd0.clone();
                if !entry.contains(&cmd0.gait) {
                    enter.gait = 0x3001;
                }
                c.force(State::RlControl, &s0, &enter, false);
                // the runner in force well past its start-up gains (as rl_deploy's was), its 50 Hz steps on
                // rl_deploy's: an offer on warm-up tick j lands on tick j + 1, so the first target change in the
                // window (i0) fixes the warm-up's length
                let dec = self.cfg.timing.policy_decimation as usize;
                let i0 = idx
                    .iter()
                    .position(|&k| ticks[k].frame[24..] != ticks[k - 1].frame[24..])
                    .expect("a policy step in the window");
                let warm = 2000 + (dec + 1 - i0 % dec) % dec;
                for _ in 0..warm {
                    c.tick(&s0, &cmd0, &mut policy);
                }
                // a result of the warm-up must not land in the window
                let _ = policy.take_result();
            }
            other => panic!("no start in {other:?}"),
        }
        let mut last_req = ticks[k0 - 1].t.saturating_sub(DECIDE_NS);
        let (mut prev_ours, mut prev_theirs): (Option<Vec<u8>>, Option<Vec<u8>>) = (None, None);
        for &k in &idx {
            let tick = &ticks[k];
            let when = clock(tick.t);
            // rl_deploy computes frame k right after sending frame k - 1: what reached it by then
            let cmd = self.user(&cmds, ticks[k - 1].t);
            let s = state_of(k).unwrap_or_else(|| s0.clone());
            let before = c.state;
            if c.state == State::RlControl && !have_policies {
                break;
            }
            let ours = c.tick(&s, &cmd, &mut policy);
            if c.state != before {
                r.transitions.push((k, before, c.state));
            }
            // charge-enable requests that arrived since the previous send go out on this tick and the next two
            let decide = tick.t.saturating_sub(DECIDE_NS);
            for &(_, v) in requests.iter().filter(|q| q.0 > last_req && q.0 <= decide) {
                if v == 1 {
                    charge.request();
                }
            }
            last_req = decide;
            r.ticks += 1;
            let mac: [u8; 6] = tick.frame[6..12].try_into().unwrap();
            if charge.take_tick() {
                r.charge.n += 1;
                let frame = encode_charge_enable(tick.frame[16], mac, &self.cfg.ecan);
                if frame != tick.frame {
                    r.charge.fail(format!(
                        "{when}: ours a charge-enable frame, rl_deploy's {:02x?}",
                        &tick.frame[..tick.frame.len().min(24)]
                    ));
                }
                prev_ours = None;
                prev_theirs = None;
                continue;
            }
            if tick.frame[19] != 0x89 {
                r.charge.n += 1;
                r.charge.fail(format!(
                    "{when}: rl_deploy sent a charge-enable frame, ours none"
                ));
                continue;
            }
            let enc = encode_command(&ours, tick.frame[16], mac, &self.cfg.ecan, &self.map);
            let in_policy = c.state == State::RlControl && !ours.enable;
            if KNOWN_CLOCK_SLIP.contains(&when.as_str()) {
                let mut s = Report::default();
                compare(&enc, &tick.frame, true, &when, &mut s);
                r.slip.n += s.targets.n;
                r.slip.max_ulp = r.slip.max_ulp.max(s.targets.max_ulp);
                r.slip.max_abs = r.slip.max_abs.max(s.targets.max_abs);
                r.slip.bad += s.targets.bad;
                r.bytes.n += s.bytes.n;
                r.bytes.bad += s.bytes.bad;
                r.bytes.first_bad.extend(s.bytes.first_bad);
                prev_ours = Some(enc);
                prev_theirs = Some(tick.frame.clone());
                continue;
            }
            let bad_before = r.bytes.bad;
            compare(&enc, &tick.frame, !in_policy, &when, &mut r);
            if r.bytes.bad > bad_before {
                let g = |f: &[u8]| format!("{:02x?}", &f[24 + 24 + 16..24 + 24 + 22]);
                match r.byte_runs.last_mut() {
                    Some(run) if run.2 > 0 && run.1 == clock(ticks[k - 1].t) => {
                        run.1 = when.clone();
                        run.2 += 1;
                    }
                    _ => r
                        .byte_runs
                        .push((when.clone(), when.clone(), 1, g(&enc), g(&tick.frame))),
                }
            }
            if in_policy {
                // the ticks the targets change (a policy step landing) are rl_deploy's
                let changed = |now: &[u8], prev: &Option<Vec<u8>>| {
                    prev.as_ref().is_none_or(|p| p[24..] != now[24..])
                };
                let (a, b) = (
                    changed(&enc, &prev_ours),
                    changed(&tick.frame, &prev_theirs),
                );
                r.updates.n += 1;
                if a != b {
                    r.updates.fail(format!(
                        "{when}: targets changed: ours {a}, rl_deploy's {b}"
                    ));
                }
            }
            prev_ours = Some(enc);
            prev_theirs = Some(tick.frame.clone());
        }
        r
    }
}

/// rl_deploy's clock phase: the one in [0, 1/64 s) under which its stand-up targets change exactly on the ticks its
/// stamp's f32 changes (first and last of the best interval, 10 us steps).
fn fit_phase(ticks: &[Tick], first: u64, windows: &[(u64, u64)]) -> f64 {
    let idx: Vec<usize> = (1..ticks.len())
        .filter(|&k| {
            windows.iter().any(|&(a, b)| ticks[k].t >= a && ticks[k].t <= b)
                && ticks[k].state_t.is_some()
                && ticks[k - 1].state_t.is_some()
                // around a charge-enable burst there is no command to compare with the one before
                && ticks[k].frame[19] == 0x89
                && ticks[k - 1].frame[19] == 0x89
        })
        .collect();
    let observed: Vec<bool> = idx
        .iter()
        .map(|&k| ticks[k].frame[24..] != ticks[k - 1].frame[24..])
        .collect();
    let score = |phase: f64| -> usize {
        idx.iter()
            .zip(&observed)
            .filter(|(&k, &o)| {
                let s = |t: Option<u64>| t.map(|t| stamp(first, t, phase) as f32);
                (s(ticks[k].state_t) != s(ticks[k - 1].state_t)) == o
            })
            .count()
    };
    let steps: Vec<f64> = (0..1563).map(|i| i as f64 * 1e-5).collect();
    let scores: Vec<usize> = steps.iter().map(|&p| score(p)).collect();
    let best = *scores.iter().max().unwrap();
    let hits: Vec<f64> = steps
        .iter()
        .zip(&scores)
        .filter(|(_, &s)| s == best)
        .map(|(&p, _)| p)
        .collect();
    println!(
        "clock phase: {} of {} stand-up / lie-down ticks at {:.5}..{:.5} s",
        best,
        idx.len(),
        hits[0],
        hits[hits.len() - 1]
    );
    let phase = (hits[0] + hits[hits.len() - 1]) / 2.0;
    for (&k, &o) in idx.iter().zip(&observed) {
        let s = |t: Option<u64>| t.map(|t| stamp(first, t, phase) as f32);
        if (s(ticks[k].state_t) != s(ticks[k - 1].state_t)) != o {
            println!(
                "  misfit {} (reply {} / {}): rl_deploy changed {o}",
                clock(ticks[k].t),
                clock(ticks[k].state_t.unwrap()),
                clock(ticks[k - 1].state_t.unwrap())
            );
        }
    }
    let misfits: Vec<String> = idx
        .iter()
        .zip(&observed)
        .filter(|(&k, &o)| {
            let s = |t: Option<u64>| t.map(|t| stamp(first, t, phase) as f32);
            (s(ticks[k].state_t) != s(ticks[k - 1].state_t)) != o
        })
        .map(|(&k, _)| clock(ticks[k].t))
        .collect();
    assert!(
        misfits
            .iter()
            .all(|m| KNOWN_CLOCK_SLIP.contains(&m.as_str())),
        "no phase explains every stand-up / lie-down update: {misfits:?}"
    );
    (hits[0] + hits[hits.len() - 1]) / 2.0
}

fn setup() -> (Replay, Vec<Tick>, bool) {
    let mut cfg = common::config();
    // rl_deploy's own clock, from its start, to replay its 1/64 s steps
    cfg.timing.f32_state_clock_from_start = true;
    let (frames, whole) = capture();
    let ticks = load(&frames, &cfg);
    let first = ticks[0].t;
    // the 1/64 s steps of both splines: phase 1 of the stand-up, the lie-down's (61 s later)
    let phase = fit_phase(
        &ticks,
        first,
        &[
            (at(first, "05:50:24.845"), at(first, "05:50:26.300")),
            (at(first, "05:51:25.840"), at(first, "05:51:27.800")),
        ],
    );
    let map = Mapping::new(&cfg.ecan).unwrap();
    (
        Replay {
            cfg,
            map,
            first,
            phase,
        },
        ticks,
        whole,
    )
}

fn show(name: &str, r: &Report) {
    println!(
        "{name}: {} ticks; targets {} (max {} ulp, {:.2e}) bad {}; bytes {} bad {}; target-change ticks {} bad {}; \
         charge-enable frames {} bad {}; clock-slip ticks' targets {} (max {:.2e}) out {}; transitions {:?}",
        r.ticks,
        r.targets.n,
        r.targets.max_ulp,
        r.targets.max_abs,
        r.targets.bad,
        r.bytes.n,
        r.bytes.bad,
        r.updates.n,
        r.updates.bad,
        r.charge.n,
        r.charge.bad,
        r.slip.n,
        r.slip.max_abs,
        r.slip.bad,
        r.transitions
    );
    for run in &r.byte_runs {
        println!(
            "  bytes differ {} .. {} ({} ticks): ours {} rl_deploy {}",
            run.0, run.1, run.2, run.3, run.4
        );
    }
    for c in [&r.targets, &r.bytes, &r.updates, &r.charge] {
        for b in &c.first_bad {
            println!("  {b}");
        }
    }
}

fn clean(r: &Report) -> bool {
    r.targets.bad + r.bytes.bad + r.updates.bad + r.charge.bad == 0
}

#[test]
fn stand_up_and_lie_down_match_rl_deploy_tick_for_tick() {
    let (rp, ticks, _) = setup();
    let f = rp.first;
    // lying (charging), stand-up at 05:50:24.8397, its 3 s, rl_control from 27.85
    let stand = Window {
        from: at(f, "05:50:24.837"),
        to: at(f, "05:50:28.450"),
        start: State::LieDown,
        before: None,
    };
    let runners = policies(&rp.cfg);
    let r = rp.run(&ticks, &stand, runners);
    show("stand-up", &r);
    let names: Vec<(State, State)> = r.transitions.iter().map(|t| (t.1, t.2)).collect();
    assert_eq!(names[0], (State::LieDown, State::StandUp));
    // rl_deploy: the first enable frame at 05:50:24.841620, rl_control's at 05:50:27.851615
    assert_eq!(clock(ticks[r.transitions[0].0].t), "05:50:24.841620");
    if names.len() > 1 {
        assert_eq!(names[1], (State::StandUp, State::RlControl));
        assert_eq!(clock(ticks[r.transitions[1].0].t), "05:50:27.851615");
    }
    assert!(r.targets.n > 3000 * 48, "{}", r.targets.n);
    // on FinetuneStand, 05:51:25.8316 lie down: 2 s spline, 2 s damping, then all zero (and charge-enable bursts)
    let lie = Window {
        from: at(f, "05:51:25.8315"),
        to: at(f, "05:51:29.900"),
        start: State::RlControl,
        before: None,
    };
    let r2 = match policies(&rp.cfg) {
        Some(p) => {
            let r2 = rp.run(&ticks, &lie, Some(p));
            show("lie-down", &r2);
            assert_eq!(
                r2.transitions
                    .first()
                    .map(|t| (t.1, t.2, clock(ticks[t.0].t))),
                Some((State::RlControl, State::LieDown, "05:51:25.832607".into()))
            );
            Some(r2)
        }
        None => None,
    };
    assert!(clean(&r), "stand-up differs from rl_deploy");
    if let Some(r2) = r2 {
        assert!(clean(&r2), "lie-down differs from rl_deploy");
    }
}

/// rl_control through the undock, the operator's gait switches and the dock (gains, control words, the ticks the
/// targets change, stand-ins and start-up gains): one replay of the whole session from the capture, else a window per
/// switch, each starting on rl_control with the gait in force before it (whose runner's gains bridge the switch).
#[test]
fn rl_control_gains_and_switches_match_rl_deploy() {
    let (rp, ticks, whole) = setup();
    let Some(_) = policies(&rp.cfg) else {
        println!("SKIPPED: rl_control needs the extracted policies");
        return;
    };
    let f = rp.first;
    let windows: Vec<(&str, Window)> = if whole {
        vec![(
            "session",
            Window {
                from: at(f, "05:50:24.800"),
                to: at(f, "05:51:25.8315"),
                start: State::LieDown,
                before: None,
            },
        )]
    } else {
        [
            ("0x3003", "05:50:37.507", "05:50:37.600", 0x3001),
            ("0x1001", "05:50:38.397", "05:50:38.470", 0x3003),
            ("0x3003 again", "05:50:39.422", "05:50:39.480", 0x1001),
            ("0xf001 -> 0x3001", "05:51:08.727", "05:51:10.900", 0x3003),
            ("0x3004", "05:51:21.217", "05:51:22.050", 0x3001),
        ]
        .into_iter()
        .map(|(n, a, b, before)| {
            (
                n,
                Window {
                    from: at(f, a),
                    to: at(f, b),
                    start: State::RlControl,
                    before: Some(before),
                },
            )
        })
        .collect()
    };
    let mut ok = true;
    for (name, w) in &windows {
        let r = rp.run(&ticks, w, policies(&rp.cfg));
        show(name, &r);
        ok &= clean(&r);
    }
    assert!(ok, "rl_control differs from rl_deploy");
}

/// Every charge-enable request reached rl_deploy as 3 frames on the next 3 ticks; ours, through the bus's request
/// rule, the same frames on the same ticks (lying: between them the all-zero command).
#[test]
fn charge_enable_frames_match_rl_deploy() {
    let (rp, ticks, _) = setup();
    let f = rp.first;
    let bursts = [
        "05:50:01.2105",
        "05:50:06.2110",
        "05:50:11.2110",
        "05:50:16.2110",
        "05:50:21.2110",
        "05:51:34.8910",
        "05:51:39.8910",
        "05:51:44.8920",
        "05:51:49.8920",
        "05:51:54.8920",
    ];
    let (mut frames, mut ok) = (0, true);
    for b in bursts {
        let t = at(f, b);
        let r = rp.run(
            &ticks,
            &Window {
                from: t - 3_000_000,
                to: t + 7_000_000,
                start: State::LieDown,
                before: None,
            },
            None,
        );
        show(b, &r);
        frames += r.charge.n;
        ok &= clean(&r);
    }
    assert_eq!(frames, 3 * bursts.len());
    assert!(ok, "charge-enable frames differ from rl_deploy's");
}

/// Every joint command rl_deploy sent, decoded (rl_deploy's f32 calibration) and encoded again: the same bytes, the
/// targets within 4 f32 ULPs.
#[test]
fn every_rl_deploy_command_re_encodes() {
    let (rp, ticks, _) = setup();
    let mut r = Report::default();
    for t in ticks.iter().filter(|t| t.frame[19] == 0x89) {
        let sent = decode_command(&t.frame, rp.cfg.ecan.ethertype, &rp.map)
            .unwrap()
            .unwrap();
        let enc = encode_command(
            &sent.command,
            t.frame[16],
            t.frame[6..12].try_into().unwrap(),
            &rp.cfg.ecan,
            &rp.map,
        );
        // rl_deploy's control word per frame (4, or 2 on its enable frames) is ours
        compare(&enc, &t.frame, true, &clock(t.t), &mut r);
    }
    show("re-encode", &r);
    assert!(r.bytes.n > 9000, "{}", r.bytes.n);
    assert!(clean(&r));
}

#[allow(dead_code)]
fn unused(_: BTreeMap<u8, u8>, _: JointCommand, _: &dyn PolicyService) {}
