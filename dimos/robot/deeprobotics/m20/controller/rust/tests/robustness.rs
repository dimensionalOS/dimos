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

//! Does any command make the robot freak out? The controller and its policies (lockstep, one tick of latency) against
//! the MuJoCo sim (server.py): every gait code to every other from each start state, every motion command at those
//! moments, rapid toggles, out-of-range / NaN / inf commands, stale commands and usage-mode changes, each scored on
//! falls, faults, joint-limit contact, target jumps, torque saturation, body rate and policy-clamp hits.
//!
//! `cargo test --release --test robustness -- --ignored matrix --nocapture` writes ROBUSTNESS.tsv.gz (one row per case).
//! M20_ROBUSTNESS_FILTER (comma-separated substrings of the case name) narrows it; M20_ROBUSTNESS_FULL=1 starts every
//! case from lying (stand-up, the 0x1001 stand-in, basic_flat) instead of standing on navi_charge, for validation;
//! M20_ROBUSTNESS_WORKERS sets the parallel sims; M20_ROBUSTNESS_OUT the table's path. The regression tests below run
//! the cases the controller once failed. Needs the extracted policies and a sim Python (M20_SIM_PYTHON); skips
//! otherwise.

mod common;

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};

use m20_controller::basic_server::{BasicServer, ChargerState, UsageMode};
use m20_controller::commands::{CommandSource, ScriptStep, ScriptedCommands};
use m20_controller::config::{Config, JOINTS};
use m20_controller::hardware::mujoco::MujocoSim;
use m20_controller::hardware::Hardware;
use m20_controller::policy::{InlinePolicy, RunnerSet};
use m20_controller::runner::joint_limits;
use m20_controller::state_machine::{Controller, State};
use m20_controller::types::{JointCommand, Target, UserCommand};

// ------------------------------------------------------------------------------------------------- what is a freak-out
// The calm runs' extremes (family "baseline": every runner standing and walking at each start state's speed) set these
// with margin; ROBUSTNESS.md lists the baseline values.

/// a held body tilted past this (rad) has fallen over
const FALL_TILT_RAD: f64 = 1.0;
/// tilt past this (deg) in a held state is a stumble
const STUMBLE_TILT_DEG: f64 = 30.0;
/// a joint this close to its range (rad) is on its stop
const LIMIT_MARGIN_RAD: f64 = 0.01;
/// a leg position target moving more than this between two ticks (rad) is a jump: walking at the full command the
/// policies' own steps reach 1.2 rad (basic_flat, axes (1, -1, 1)), a calm walk 0.6
const JUMP_RAD: f64 = 1.5;
/// a leg joint torque-saturated for this long in a row (ms) is a struggle
const SATURATION_MS: u64 = 100;
/// body angular rate above this (rad/s) is a spin or a flail
const GYRO_RAD_S: f64 = 6.0;

// ------------------------------------------------------------------------------------------------- the sim

struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../../../..")
        .canonicalize()
        .unwrap()
}

fn sim_python() -> Option<PathBuf> {
    let python = std::env::var("M20_SIM_PYTHON")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(std::env::var("HOME").unwrap_or_default())
                .join("repos/dimos/.venv/bin/python")
        });
    if python.exists() {
        Some(python)
    } else {
        eprintln!(
            "skipping: no sim python at {} (set M20_SIM_PYTHON)",
            python.display()
        );
        None
    }
}

fn start_sim(python: &Path) -> (Server, MujocoSim) {
    let mut child = Command::new(python)
        .args([
            "-m",
            "dimos.simulation.m20_policy_sim.server",
            "--port",
            "0",
            "--once",
        ])
        .env("PYTHONPATH", repo_root())
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn sim server");
    let mut line = String::new();
    BufReader::new(child.stdout.take().unwrap())
        .read_line(&mut line)
        .unwrap();
    let port: u16 = line
        .trim()
        .rsplit(':')
        .next()
        .and_then(|p| p.parse().ok())
        .unwrap_or_else(|| panic!("no port in {line:?}"));
    let sim = MujocoSim::connect(&format!("127.0.0.1:{port}")).unwrap();
    (Server(child), sim)
}

// ------------------------------------------------------------------------------------------------- cases

/// Every gait code basic_server's SwitchGait sends that rl_deploy has a runner for, plus 0xf003.
const CODES: [u32; 17] = [
    0x1001, 0x1002, 0x1003, 0x1004, 0x2001, 0x2002, 0x2003, 0x2004, 0x3001, 0x3002, 0x3003, 0x3004,
    0x3005, 0x5001, 0xf001, 0xf002, 0xf003,
];
/// one code per runner
const REPS: [u32; 12] = [
    0x1001, 0x1002, 0x1003, 0x1004, 0x2001, 0x3001, 0x3002, 0x3004, 0x3005, 0x5001, 0xf001, 0xf002,
];
/// codes no runner lists: 0x1005 passes basic_server's SwitchGait, 0x9999 only the UDP path
const UNKNOWN: [u32; 2] = [0x1005, 0x9999];

/// Moving start states: normalized axes held from 3 s to the event.
const MOVING: [(&str, [f64; 3]); 5] = [
    ("fwd_fast", [1.0, 0.0, 0.0]),
    ("fwd_slow", [0.15, 0.0, 0.0]),
    ("backward", [-0.6, 0.0, 0.0]),
    ("strafe", [0.0, 0.8, 0.0]),
    ("turn", [0.0, 0.0, 1.0]),
];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Setup {
    /// standing: rl_control forced on navi_charge from a standing pose, the source gait 1 s in
    Standing,
    /// lying: passive, stand at 2 s, rl_control on 0x1001 at 2.1 s (the stand-in), basic_flat from ~8.5 s
    Lying,
}

#[derive(Debug, Clone)]
struct Case {
    name: String,
    family: &'static str,
    start: String,
    from: u32,
    event: String,
    setup: Setup,
    steps: Vec<ScriptStep>,
    /// usage-mode requests (controller time, mode); the case runs through basic_server's layer when any
    usage: Vec<(f64, i32)>,
    /// the operator falls silent at the event (commands expire after command_timeout_s)
    silent_after_event: bool,
    event_at: f64,
    end: f64,
    /// the event asks the robot to go limp or lie (joint damping, lie-down): the body may touch down
    collapse_ok: bool,
}

/// A command script built up step by step; each step sends the whole current command.
struct Script {
    steps: Vec<ScriptStep>,
    cur: UserCommand,
    /// added to every time (the lying setup's stand-up and stand-in in full mode)
    offset: f64,
}

impl Script {
    fn new(offset: f64) -> Script {
        Script {
            steps: Vec::new(),
            cur: UserCommand::default(),
            offset,
        }
    }
    fn push(&mut self, at: f64) -> &mut Self {
        self.steps.push(ScriptStep {
            at: at + self.offset,
            command: self.cur.clone(),
        });
        self
    }
    fn gait(&mut self, at: f64, g: u32) -> &mut Self {
        self.cur.gait = g;
        self.cur.gait_seq += 1;
        self.cur.motion_seq += 1;
        self.push(at)
    }
    fn axes(&mut self, at: f64, a: [f64; 3]) -> &mut Self {
        self.cur.axes = a;
        self.cur.velocity = None;
        self.push(at)
    }
    fn target(&mut self, at: f64, t: Target) -> &mut Self {
        self.cur.target = t;
        self.cur.motion_seq += 1;
        self.push(at)
    }
    fn edit(&mut self, at: f64, f: impl FnOnce(&mut UserCommand)) -> &mut Self {
        f(&mut self.cur);
        self.push(at)
    }
}

/// The event time in the standing setup; the source gait is in from 1 s, a moving start state from 3 s.
const EVENT: f64 = 5.5;
const END: f64 = 9.0;
/// the lying setup's stand-up and 0x1001 stand-in (3 s + 0.5 s from rl_control at ~5 s) end by here
const LYING_READY: f64 = 8.0;

fn full_mode() -> bool {
    std::env::var("M20_ROBUSTNESS_FULL").is_ok_and(|v| v == "1")
}

/// The setup to standing in gait `a`: forced navi_charge (or, in full mode, the lying setup's stand-up and stand-in),
/// `a` at 1 s.
fn standing(a: u32) -> (Script, Setup) {
    let full = full_mode();
    let mut s = Script::new(0.0);
    if full {
        s.target(0.0, Target::Passive).gait(0.0, 0x1001);
        s.target(2.0, Target::Stand).target(2.1, Target::RlControl);
        s.offset = LYING_READY;
    } else {
        s.cur.target = Target::RlControl;
        s.gait(0.0, 0x3001);
    }
    if !(a == 0x3001 && !full) && !(a == 0x1001 && full) {
        s.gait(1.0, a);
    }
    (s, if full { Setup::Lying } else { Setup::Standing })
}

fn hex(g: u32) -> String {
    format!("{g:#06x}")
}

#[allow(clippy::too_many_arguments)]
fn case(
    family: &'static str,
    start: &str,
    from: u32,
    event: String,
    (s, setup): (Script, Setup),
    usage: Vec<(f64, i32)>,
    silent: bool,
    collapse_ok: bool,
) -> Case {
    let offset = s.offset;
    Case {
        name: format!("{family}/{start}/{}/{event}", hex(from)),
        family,
        start: start.to_string(),
        from,
        event,
        setup,
        steps: s.steps,
        usage: usage.into_iter().map(|(t, m)| (t + offset, m)).collect(),
        silent_after_event: silent,
        event_at: EVENT + offset,
        end: END + offset,
        collapse_ok,
    }
}

/// A third gait for a mid-switch that is neither `a`'s runner nor `b`'s.
fn third(cfg: &Config, a: u32, b: u32) -> u32 {
    let runners = [cfg.runner_for_gait(a), cfg.runner_for_gait(b)];
    *[0x3003, 0x1001, 0x3001]
        .iter()
        .find(|g| !runners.contains(&cfg.runner_for_gait(**g)))
        .unwrap()
}

/// A start state's commands before the event (standing still, moving, or a gait switch in flight).
fn start_state(cfg: &Config, s: &mut Script, start: &str, a: u32, b: u32) {
    if let Some((_, axes)) = MOVING.iter().find(|(n, _)| *n == start) {
        s.axes(3.0, *axes);
    } else if start == "mid_switch" {
        s.gait(EVENT - 0.05, third(cfg, a, b));
    }
}

fn motion_event(s: &mut Script, m: &str) {
    match m {
        "stand" => s.target(EVENT, Target::Stand),
        "lie_down" => s.target(EVENT, Target::LieDown),
        "passive" => s.target(EVENT, Target::Passive),
        "damping" => s.target(EVENT, Target::JointDamping),
        "rl_control" => s.target(EVENT, Target::RlControl),
        "align" => s.target(EVENT, Target::RlControl).gait(EVENT, 0xf003),
        _ => unreachable!(),
    };
}

/// Out-of-range and non-finite commands at the event (and what the start state does before them).
fn extreme(s: &mut Script, e: &str) {
    let p = |s: &mut Script, kv: &[(&str, f64)]| {
        let kv: Vec<(String, f64)> = kv.iter().map(|(k, v)| (k.to_string(), *v)).collect();
        s.edit(EVENT, |c| c.params.extend(kv));
    };
    let pose = ["height", "roll", "pitch", "yaw", "dx", "dy"];
    match e {
        "axes_full" => {
            s.axes(EVENT, [1.0, 1.0, 1.0]);
        }
        "axes_full_neg" => {
            s.axes(EVENT, [-1.0, -1.0, -1.0]);
        }
        "axes_over" => {
            s.axes(EVENT, [10.0, -10.0, 10.0]);
        }
        "axes_nan" => {
            s.axes(EVENT, [f64::NAN, 0.0, 0.0]);
        }
        "axes_inf" => {
            s.axes(EVENT, [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY]);
        }
        "velocity_huge" => {
            s.edit(EVENT, |c| c.velocity = Some([100.0, -100.0, 100.0]));
        }
        "velocity_nan" => {
            s.edit(EVENT, |c| c.velocity = Some([f64::NAN; 3]));
        }
        "reverse" => {
            s.axes(3.0, [1.0, 0.0, 0.0]).axes(EVENT, [-1.0, 0.0, 0.0]);
        }
        "pose_over" => p(s, &pose.map(|k| (k, 5.0))),
        "pose_under" => p(s, &pose.map(|k| (k, -5.0))),
        "pose_nan" => p(s, &pose.map(|k| (k, f64::NAN))),
        "mode_over" => p(s, &[("mode", 5.0), ("mode_byte", 7.0), ("feet_fixed", 3.0)]),
        "params_inf" => p(s, &[("height", f64::INFINITY), ("mode", f64::NEG_INFINITY)]),
        _ => unreachable!(),
    }
}

const EXTREMES: [&str; 13] = [
    "axes_full",
    "axes_full_neg",
    "axes_over",
    "axes_nan",
    "axes_inf",
    "velocity_huge",
    "velocity_nan",
    "reverse",
    "pose_over",
    "pose_under",
    "pose_nan",
    "mode_over",
    "params_inf",
];
const MOTIONS: [&str; 6] = [
    "stand",
    "lie_down",
    "passive",
    "damping",
    "rl_control",
    "align",
];

fn all_cases(cfg: &Config) -> Vec<Case> {
    let mut out = Vec::new();
    let reps_and_unknown: Vec<u32> = REPS.iter().chain(&UNKNOWN[1..]).copied().collect();
    // baseline: each runner standing, and at each moving start state's speed through the end
    for &a in &REPS {
        out.push(case(
            "baseline",
            "still",
            a,
            "none".into(),
            standing(a),
            vec![],
            false,
            false,
        ));
        for (start, axes) in MOVING {
            let mut s = standing(a);
            s.0.axes(3.0, axes);
            out.push(case(
                "baseline",
                start,
                a,
                "none".into(),
                s,
                vec![],
                false,
                false,
            ));
        }
    }
    // every code to every other, standing still
    for &a in &CODES {
        for &b in CODES.iter().chain(&UNKNOWN) {
            if a == b {
                continue;
            }
            let mut s = standing(a);
            s.0.gait(EVENT, b);
            out.push(case("gait", "still", a, hex(b), s, vec![], false, false));
        }
    }
    // runner to runner from moving: the switch at the stop (rl_deploy waits for a still command), moving again 0.1 s on
    for (start, axes) in MOVING {
        for &a in &REPS {
            for &b in &reps_and_unknown {
                if a == b {
                    continue;
                }
                let mut s = standing(a);
                s.0.axes(3.0, axes);
                s.0.gait(EVENT, b)
                    .axes(EVENT, [0.0; 3])
                    .axes(EVENT + 0.1, axes)
                    .axes(EVENT + 1.6, [0.0; 3]);
                // long enough for a switch held until the body stops, then a stand-in
                let mut c = case("gait", start, a, hex(b), s, vec![], false, false);
                c.end += 3.0;
                out.push(c);
            }
        }
    }
    // a switch while another is in flight (50 ms before)
    for &a in &REPS {
        for &b in &reps_and_unknown {
            if a == b {
                continue;
            }
            let mut s = standing(a);
            start_state(cfg, &mut s.0, "mid_switch", a, b);
            s.0.gait(EVENT, b);
            out.push(case(
                "gait",
                "mid_switch",
                a,
                hex(b),
                s,
                vec![],
                false,
                false,
            ));
        }
    }
    // gaits around the stand-up: commanded while lying, right as the stand-in hands over, resent through the stand-in
    for &b in CODES.iter().chain(&UNKNOWN) {
        let lying = |f: &dyn Fn(&mut Script)| {
            let mut s = Script::new(0.0);
            s.target(0.0, Target::Passive).gait(0.0, 0x1001);
            f(&mut s);
            (s, Setup::Lying)
        };
        let mk = |start: &str, s| {
            let mut c = case("gait", start, 0x1001, hex(b), s, vec![], false, false);
            c.event_at = 2.0;
            c.end = 14.0;
            c
        };
        out.push(mk(
            "lying",
            lying(&|s| {
                s.gait(1.0, b)
                    .target(2.0, Target::Stand)
                    .target(2.1, Target::RlControl);
            }),
        ));
        let mut c = mk(
            "after_standup",
            lying(&|s| {
                s.target(2.0, Target::Stand)
                    .target(2.1, Target::RlControl)
                    .gait(8.52, b);
            }),
        );
        c.event_at = 8.52;
        out.push(c);
        out.push(mk(
            "during_standin",
            lying(&|s| {
                s.target(2.0, Target::Stand).target(2.1, Target::RlControl);
                for k in 0..40 {
                    s.gait(2.1 + 0.25 * k as f64, b);
                }
            }),
        ));
    }
    // every motion command at each start state
    for start in [
        "still",
        "fwd_fast",
        "backward",
        "strafe",
        "turn",
        "mid_switch",
    ] {
        for &a in &REPS {
            for m in MOTIONS {
                let mut s = standing(a);
                start_state(cfg, &mut s.0, start, a, 0x1001);
                motion_event(&mut s.0, m);
                let collapse = matches!(m, "damping" | "lie_down");
                out.push(case(
                    "motion",
                    start,
                    a,
                    m.into(),
                    s,
                    vec![],
                    false,
                    collapse,
                ));
            }
        }
    }
    // A -> B -> A within 50 ms; stand / lie spam; gait spam
    for &a in &REPS {
        for &b in &REPS {
            if a == b {
                continue;
            }
            let mut s = standing(a);
            s.0.gait(EVENT, b).gait(EVENT + 0.05, a);
            out.push(case(
                "toggle",
                "still",
                a,
                format!("{}_back", hex(b)),
                s,
                vec![],
                false,
                false,
            ));
        }
        let mut s = standing(a);
        for k in 0..30 {
            let t = [Target::LieDown, Target::Stand, Target::RlControl][k % 3];
            s.0.target(EVENT + 0.1 * k as f64, t);
        }
        s.0.target(EVENT + 3.0, Target::RlControl);
        out.push(case(
            "toggle",
            "still",
            a,
            "motion_spam".into(),
            s,
            vec![],
            false,
            true,
        ));
        let mut s = standing(a);
        for k in 0..30 {
            s.0.gait(EVENT + 0.05 * k as f64, REPS[(k * 5) % REPS.len()]);
        }
        s.0.gait(EVENT + 1.5, a);
        out.push(case(
            "toggle",
            "still",
            a,
            "gait_spam".into(),
            s,
            vec![],
            false,
            false,
        ));
        let mut s = standing(a);
        s.0.axes(3.0, [1.0, 0.0, 0.0]);
        for k in 0..30 {
            s.0.gait(EVENT + 0.05 * k as f64, REPS[(k * 5) % REPS.len()]);
        }
        s.0.gait(EVENT + 1.5, a).axes(EVENT + 2.0, [0.0; 3]);
        out.push(case(
            "toggle",
            "fwd_fast",
            a,
            "gait_spam".into(),
            s,
            vec![],
            false,
            false,
        ));
    }
    // out-of-range and non-finite commands, standing and walking
    for &a in &REPS {
        for e in EXTREMES {
            for start in ["still", "fwd_fast"] {
                let mut s = standing(a);
                start_state(cfg, &mut s.0, start, a, a);
                extreme(&mut s.0, e);
                out.push(case("extreme", start, a, e.into(), s, vec![], false, false));
            }
        }
    }
    // the operator falls silent while moving
    for &a in &REPS {
        for start in ["fwd_fast", "turn", "strafe"] {
            let mut s = standing(a);
            start_state(cfg, &mut s.0, start, a, a);
            out.push(case(
                "stale",
                start,
                a,
                "silent".into(),
                s,
                vec![],
                true,
                false,
            ));
        }
    }
    // usage mode (basic_server's layer): navigation from 0.5 s, then regular / assist (both SwitchGait(0x1001))
    for &a in &REPS {
        for (start, to) in [("fwd_fast", 0), ("fwd_fast", 2), ("still", 0), ("turn", 2)] {
            let mut s = standing(a);
            start_state(cfg, &mut s.0, start, a, a);
            let usage = vec![(0.5, 1), (EVENT, to)];
            out.push(case(
                "usage",
                start,
                a,
                format!("mode_1_to_{to}"),
                s,
                usage,
                false,
                false,
            ));
        }
    }
    out
}

// ------------------------------------------------------------------------------------------------- running

/// The operator's script with no charge manager (basic_server's layer reads its state).
struct Operator(ScriptedCommands);

impl CommandSource for Operator {
    fn poll(&mut self, now: f64) -> Option<UserCommand> {
        self.0.poll(now)
    }
}

impl ChargerState for Operator {
    fn charger_state(&self) -> i32 {
        0
    }
}

#[derive(Debug, Clone, Default)]
struct Metrics {
    fell_at: Option<f64>,
    max_tilt_deg: f64,
    min_z: f64,
    limit_ticks: u64,
    limit_joint: Option<String>,
    max_jump: f64,
    sat_run_ms: u64,
    max_gyro: f64,
    clamp_steps: u64,
    max_action: f64,
    /// joint damping or a fault the event did not ask for
    fault: Option<String>,
    nonfinite: bool,
}

#[derive(Debug, Clone)]
struct Outcome {
    case: Case,
    before: Metrics,
    after: Metrics,
    transitions: Vec<(f64, State, State)>,
    runners: Vec<String>,
    final_state: State,
    final_speed: f64,
    error: Option<String>,
}

impl Outcome {
    /// What freaked out after the event (empty: calm).
    fn freakouts(&self) -> Vec<String> {
        let m = &self.after;
        let mut v = Vec::new();
        if let Some(e) = &self.error {
            v.push(format!("error {e}"));
        }
        if let Some(t) = m.fell_at {
            v.push(format!("fell {t:.2}s"));
        }
        if let Some(f) = &m.fault {
            v.push(format!("fault ({f})"));
        }
        if m.nonfinite {
            v.push("nonfinite command".into());
        }
        if m.max_tilt_deg > STUMBLE_TILT_DEG && m.fell_at.is_none() {
            v.push(format!("tilt {:.0}deg", m.max_tilt_deg));
        }
        if m.limit_ticks > 0 {
            v.push(format!(
                "limit {} ({} ms)",
                m.limit_joint.as_deref().unwrap_or("?"),
                m.limit_ticks
            ));
        }
        if m.max_jump > JUMP_RAD {
            v.push(format!("jump {:.2}rad", m.max_jump));
        }
        if m.sat_run_ms >= SATURATION_MS {
            v.push(format!("saturated {} ms", m.sat_run_ms));
        }
        if m.max_gyro > GYRO_RAD_S {
            v.push(format!("gyro {:.1}rad/s", m.max_gyro));
        }
        v
    }

    /// A gait case's gait took over by the end (None: not a gait case, or one rl_deploy drops).
    fn switched(&self, cfg: &Config) -> Option<bool> {
        let c = &self.case;
        if c.family != "gait" && c.family != "toggle" {
            return None;
        }
        let want = if c.family == "toggle" {
            cfg.runner_for_gait(c.from)
        } else {
            u32::from_str_radix(c.event.trim_start_matches("0x"), 16)
                .map_or("", |g| cfg.runner_for_gait(g))
        };
        // a gait commanded while lying never applies: rl_control enters on its entry rule
        if c.start == "lying" || want.is_empty() {
            return None;
        }
        Some(self.runners.last().map(String::as_str) == Some(want))
    }

    fn setup_ok(&self) -> bool {
        self.before.fell_at.is_none() && self.before.fault.is_none()
    }
}

struct Tracker<'a> {
    cfg: &'a Config,
    m: Metrics,
    prev: Option<JointCommand>,
    sat_run: u64,
    steps_seen: u64,
    limits: ([f32; JOINTS], [f32; JOINTS]),
    torque: [f64; JOINTS],
}

impl<'a> Tracker<'a> {
    fn new(cfg: &'a Config) -> Tracker<'a> {
        Tracker {
            cfg,
            m: Metrics {
                min_z: f64::INFINITY,
                ..Default::default()
            },
            prev: None,
            sat_run: 0,
            steps_seen: 0,
            limits: joint_limits(&cfg.robot),
            torque: std::array::from_fn(|i| cfg.robot.torque_limit[i % 4]),
        }
    }

    fn tick(&mut self, c: &Controller, s: &m20_controller::types::RobotState, cmd: &JointCommand) {
        let held = matches!(c.state, State::RlControl);
        let r = &self.cfg.robot;
        let finite = [&cmd.kp, &cmd.q, &cmd.kd, &cmd.dq, &cmd.tau]
            .iter()
            .all(|a| a.iter().all(|v| v.is_finite()));
        if !finite {
            self.m.nonfinite = true;
        }
        if held {
            for i in (0..JOINTS).filter(|i| i % 4 != 3) {
                if s.q[i] < r.joint_lower[i] + LIMIT_MARGIN_RAD
                    || s.q[i] > r.joint_upper[i] - LIMIT_MARGIN_RAD
                {
                    self.m.limit_ticks += 1;
                    self.m
                        .limit_joint
                        .get_or_insert_with(|| r.joint_names[i].clone());
                    break;
                }
            }
            let g = s.gyro.iter().map(|v| v * v).sum::<f64>().sqrt();
            self.m.max_gyro = self.m.max_gyro.max(g);
        }
        if let Some(p) = &self.prev {
            for i in (0..JOINTS).filter(|i| i % 4 != 3) {
                if p.kp[i] > 0.0 && cmd.kp[i] > 0.0 && !cmd.enable && !p.enable {
                    self.m.max_jump = self.m.max_jump.max((cmd.q[i] - p.q[i]).abs());
                }
            }
        }
        let mut saturated = false;
        for i in (0..JOINTS).filter(|i| i % 4 != 3) {
            let tau =
                cmd.kp[i] * (cmd.q[i] - s.q[i]) + cmd.kd[i] * (cmd.dq[i] - s.dq[i]) + cmd.tau[i];
            if tau.abs() >= self.torque[i] {
                saturated = true;
            }
        }
        self.sat_run = if saturated && held {
            self.sat_run + 1
        } else {
            0
        };
        self.m.sat_run_ms = self.m.sat_run_ms.max(self.sat_run);
        if c.counters.policy_steps != self.steps_seen {
            self.steps_seen = c.counters.policy_steps;
            if let Some(t) = c.last_targets() {
                let on_clamp = (0..JOINTS).filter(|i| i % 4 != 3).any(|i| {
                    let q = t.q[i] as f32;
                    q == self.limits.0[i] || q == self.limits.1[i]
                });
                if on_clamp {
                    self.m.clamp_steps += 1;
                }
                let a = t.action.iter().fold(0.0f64, |m, v| m.max(v.abs()));
                self.m.max_action = self.m.max_action.max(a);
            }
        }
        self.prev = Some(*cmd);
    }
}

fn run_case(sim: &mut MujocoSim, cfg: &Config, dir: &Path, case: &Case) -> Outcome {
    match try_run(sim, cfg, dir, case) {
        Ok(o) => o,
        Err(e) => Outcome {
            case: case.clone(),
            before: Metrics::default(),
            after: Metrics::default(),
            transitions: vec![],
            runners: vec![],
            final_state: State::Passive,
            final_speed: 0.0,
            error: Some(format!("{e:#}")),
        },
    }
}

fn standing_pose(cfg: &Config) -> Vec<f64> {
    cfg.runners["navi_charge"].default_pose.clone()
}

fn try_run(sim: &mut MujocoSim, cfg: &Config, dir: &Path, case: &Case) -> anyhow::Result<Outcome> {
    let pose = match case.setup {
        Setup::Standing => serde_json::json!(standing_pose(cfg)),
        Setup::Lying => serde_json::json!("lie"),
    };
    let mut state =
        sim.reset(&serde_json::json!({"pose": pose, "height_map": {"rows": 41, "cols": 11}}))?;
    let mut policy = InlinePolicy::new(RunnerSet::load(cfg, dir)?);
    let mut controller = Controller::new(cfg.clone());
    let scripted = ScriptedCommands::new(case.steps.clone());
    let usage = Arc::new(UsageMode::default());
    let mut commands: Box<dyn CommandSource> = if case.usage.is_empty() {
        Box::new(scripted)
    } else {
        let mut b = BasicServer::new(
            Operator(scripted),
            usage.clone(),
            cfg.basic_server.clone(),
            cfg.real_steer.clone(),
        );
        b.extre_authorized = true;
        Box::new(b)
    };
    let mut user = UserCommand::default();
    if case.setup == Setup::Standing {
        user = commands.poll(0.0).unwrap_or_default();
        controller.force(State::RlControl, &state, &user, false);
    }
    let mut usage_left = case.usage.clone();
    let (mut before, mut after) = (Tracker::new(cfg), Tracker::new(cfg));
    let mut last_command_at = 0.0;
    let mut runners: Vec<String> = Vec::new();
    let mut final_speed = 0.0;
    let mut ticks = 0u64;
    loop {
        let now = controller.time();
        if now >= case.end {
            break;
        }
        while usage_left.first().is_some_and(|(t, _)| *t <= now) {
            let (_, m) = usage_left.remove(0);
            *usage.request.lock().unwrap() = Some(m);
        }
        if let Some(c) = commands.poll(now) {
            user = c;
            last_command_at = now;
        }
        // until the event the operator keeps sending (an expiring source); a silent case stops then
        if !case.silent_after_event || now < case.event_at {
            last_command_at = now;
        }
        let mut effective = user.clone();
        if now - last_command_at > cfg.safety.command_timeout_s {
            effective.axes = [0.0; 3];
            effective.velocity = effective.velocity.map(|_| [0.0; 3]);
        }
        let cmd = controller.tick(&state, &effective, &mut policy);
        commands.observe(&controller);
        let tracker = if now < case.event_at {
            &mut before
        } else {
            &mut after
        };
        tracker.tick(&controller, &state, &cmd);
        state = sim.exchange(&cmd)?;
        ticks += 1;
        if let Some(r) = controller.current_runner() {
            if runners.last().map(String::as_str) != Some(r) && controller.state == State::RlControl
            {
                runners.push(r.to_string());
            }
        }
        if ticks.is_multiple_of(20) {
            let truth = sim.truth()?;
            let tracker = if now < case.event_at {
                &mut before
            } else {
                &mut after
            };
            let m = &mut tracker.m;
            let held = controller.state == State::RlControl
                || (controller.state == State::StandUp
                    && controller
                        .transitions
                        .last()
                        .is_some_and(|t| now - t.0 > 1.6));
            let tilt_deg = truth.tilt.to_degrees();
            if held {
                m.max_tilt_deg = m.max_tilt_deg.max(tilt_deg);
                m.min_z = m.min_z.min(truth.position[2]);
            }
            let down = truth.tilt > FALL_TILT_RAD
                || (held && truth.trunk_contact && !(case.collapse_ok && now >= case.event_at));
            if down && m.fell_at.is_none() {
                m.fell_at = Some(now);
            }
            final_speed = (truth.velocity[0].powi(2) + truth.velocity[1].powi(2)).sqrt();
        }
        let tracker = if now < case.event_at {
            &mut before
        } else {
            &mut after
        };
        if controller.state == State::JointDamping && tracker.m.fault.is_none() {
            let asked = effective.target == Target::JointDamping && case.collapse_ok;
            if !asked {
                tracker.m.fault = Some(
                    controller
                        .fault
                        .clone()
                        .unwrap_or_else(|| "joint damping".into()),
                );
            }
        }
    }
    Ok(Outcome {
        case: case.clone(),
        before: before.m,
        after: after.m,
        transitions: controller.transitions.clone(),
        runners,
        final_state: controller.state,
        final_speed,
        error: None,
    })
}

/// Run `cases` on parallel sims; None when the sim or the policies are missing.
fn run_all(cases: Vec<Case>) -> Option<Vec<Outcome>> {
    let dir = common::policy_dir()?;
    let python = sim_python()?;
    let cfg = common::config();
    let workers = std::env::var("M20_ROBUSTNESS_WORKERS")
        .ok()
        .and_then(|w| w.parse().ok())
        .unwrap_or_else(|| {
            std::thread::available_parallelism().map_or(4, |n| n.get().saturating_sub(4).max(1))
        })
        .min(cases.len().max(1));
    let total = cases.len();
    let queue = Arc::new(Mutex::new(
        cases.into_iter().enumerate().collect::<Vec<_>>(),
    ));
    let results = Arc::new(Mutex::new(Vec::new()));
    let started = std::time::Instant::now();
    std::thread::scope(|scope| {
        for _ in 0..workers {
            let (queue, results, cfg, dir, python) = (
                queue.clone(),
                results.clone(),
                cfg.clone(),
                dir.clone(),
                python.clone(),
            );
            scope.spawn(move || {
                let (_server, mut sim) = start_sim(&python);
                loop {
                    let Some((k, case)) = queue.lock().unwrap().pop() else {
                        break;
                    };
                    let outcome = run_case(&mut sim, &cfg, &dir, &case);
                    let mut r = results.lock().unwrap();
                    r.push((k, outcome));
                    if r.len() % 50 == 0 || r.len() == total {
                        eprintln!(
                            "{} / {total} cases, {:.0} s",
                            r.len(),
                            started.elapsed().as_secs_f64()
                        );
                    }
                }
            });
        }
    });
    let mut r = std::mem::take(&mut *results.lock().unwrap());
    r.sort_by_key(|(k, _)| *k);
    Some(r.into_iter().map(|(_, o)| o).collect())
}

fn tsv(outcomes: &[Outcome]) -> String {
    let cfg = common::config();
    let mut s = String::from(
        "case\tfamily\tstart\tfrom\tevent\tverdict\tfreakouts\tswitched\tsetup_ok\tfell_at\tmax_tilt_deg\tmin_z\tlimit_ms\t\
         max_jump_rad\tsat_run_ms\tmax_gyro\tclamp_steps\tmax_action\tfinal_state\tfinal_speed\trunners\ttransitions\n",
    );
    for o in outcomes {
        let m = &o.after;
        let f = o.freakouts();
        let tr: Vec<String> = o
            .transitions
            .iter()
            .map(|(t, a, b)| format!("{t:.2}:{a:?}>{b:?}"))
            .collect();
        s += &format!(
            "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:.1}\t{:.3}\t{}\t{:.3}\t{}\t{:.2}\t{}\t{:.1}\t{:?}\t{:.2}\t{}\t{}\n",
            o.case.name,
            o.case.family,
            o.case.start,
            hex(o.case.from),
            o.case.event,
            if f.is_empty() { "calm" } else { "FREAKOUT" },
            f.join("; "),
            o.switched(&cfg).map_or(String::from("-"), |b| b.to_string()),
            o.setup_ok(),
            m.fell_at.map_or(String::new(), |t| format!("{t:.2}")),
            m.max_tilt_deg,
            if m.min_z.is_finite() { m.min_z } else { 0.0 },
            m.limit_ticks,
            m.max_jump,
            m.sat_run_ms,
            m.max_gyro,
            m.clamp_steps,
            m.max_action,
            o.final_state,
            o.final_speed,
            o.runners.join(">"),
            tr.join(" "),
        );
    }
    s
}

fn filtered(cases: Vec<Case>) -> Vec<Case> {
    match std::env::var("M20_ROBUSTNESS_FILTER") {
        Ok(f) if !f.is_empty() => {
            let parts: Vec<String> = f.split(',').map(String::from).collect();
            cases
                .into_iter()
                .filter(|c| parts.iter().any(|p| c.name.contains(p.as_str())))
                .collect()
        }
        _ => cases,
    }
}

/// The whole matrix (or M20_ROBUSTNESS_FILTER's part of it) into ROBUSTNESS.tsv.
#[test]
#[ignore = "the matrix: ~2000 sim cases (cargo test --release --test robustness -- --ignored matrix --nocapture)"]
fn matrix() {
    let cfg = common::config();
    let cases = filtered(all_cases(&cfg));
    eprintln!("{} cases", cases.len());
    let Some(outcomes) = run_all(cases) else {
        return;
    };
    let out = std::env::var("M20_ROBUSTNESS_OUT")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(if full_mode() {
                "ROBUSTNESS_full.tsv.gz"
            } else {
                "ROBUSTNESS.tsv.gz"
            })
        });
    // gzipped (the repo's large-file limit); zcat reads it
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    std::io::Write::write_all(&mut gz, tsv(&outcomes).as_bytes()).unwrap();
    std::fs::write(&out, gz.finish().unwrap()).unwrap();
    let mut by_family: BTreeMap<(&str, &str), (usize, usize)> = BTreeMap::new();
    for o in &outcomes {
        let e = by_family.entry((o.case.family, "")).or_default();
        e.0 += 1;
        if !o.freakouts().is_empty() {
            e.1 += 1;
        }
    }
    for ((family, _), (n, bad)) in &by_family {
        eprintln!("{family}: {bad} of {n} freak out");
    }
    eprintln!("wrote {}", out.display());
}

/// The cases rl_deploy's behavior freaks out on in the sim (ROBUSTNESS.md), calm with safe_switch, finite_commands,
/// default_command_slew and the runners' param_limits; each gait case's gait takes over.
#[test]
fn fixed_cases_stay_calm() {
    let cfg = common::config();
    let names = [
        "gait/still/0x1001/0x3002",
        "gait/still/0x1004/0x2001",
        "gait/still/0x1004/0xf002",
        "gait/fwd_fast/0x3002/0xf002",
        "gait/fwd_fast/0x2001/0x3001",
        "gait/backward/0xf002/0x3004",
        "toggle/still/0x1001/0x3002_back",
        "motion/fwd_fast/0x3002/lie_down",
        "motion/fwd_fast/0x2001/lie_down",
        "extreme/still/0x1001/axes_nan",
        "extreme/fwd_fast/0x3002/velocity_nan",
        "extreme/still/0xf001/pose_nan",
        "extreme/still/0x2001/mode_over",
        "extreme/still/0x1002/reverse",
    ];
    let cases: Vec<Case> = all_cases(&cfg)
        .into_iter()
        .filter(|c| names.contains(&c.name.as_str()))
        .collect();
    assert_eq!(cases.len(), names.len());
    let Some(outcomes) = run_all(cases) else {
        return;
    };
    for o in &outcomes {
        let f = o.freakouts();
        println!("{}: {:?} {}", o.case.name, f, o.runners.join(">"));
        assert!(o.setup_ok(), "{}: setup", o.case.name);
        assert!(f.is_empty(), "{}: {f:?}", o.case.name);
        assert_ne!(
            o.switched(&cfg),
            Some(false),
            "{}: never switched",
            o.case.name
        );
    }
}
