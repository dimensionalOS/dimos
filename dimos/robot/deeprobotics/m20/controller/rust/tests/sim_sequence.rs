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

//! The controller against the MuJoCo sim (server.py), threaded policy and all: lie -> stand-up -> rl_control -> walk
//! -> stop -> lie-down, without falling. The sim steps once per exchange and the script runs on controller time
//! (ticks), so the walk tests are paced by step count with the policy worker held to one tick of latency: the same
//! run on an idle or a loaded machine. The real-time variant (wall-clock pacing, the worker free-running, overruns
//! counted) is `--ignored`, for an idle machine. Needs the extracted policies and a Python with mujoco and
//! onnxruntime (M20_SIM_PYTHON, default ~/repos/dimos/.venv/bin/python); skips otherwise.

mod common;

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::AtomicBool;

use m20_controller::commands::{ScriptStep, ScriptedCommands};
use m20_controller::hardware::mujoco::{MujocoSim, Truth};
use m20_controller::policy::{
    PolicyRequest, PolicyResult, PolicyService, RunnerSet, ThreadedPolicy,
};
use m20_controller::rt::{self, Pacing};
use m20_controller::state_machine::{Controller, State};
use m20_controller::types::{Target, UserCommand};

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

/// Starts the sim on a free port; returns it and the port it took.
fn start_server() -> Option<(Server, u16)> {
    let python = std::env::var("M20_SIM_PYTHON")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(std::env::var("HOME").unwrap_or_default())
                .join("repos/dimos/.venv/bin/python")
        });
    if !python.exists() {
        eprintln!(
            "skipping: no sim python at {} (set M20_SIM_PYTHON)",
            python.display()
        );
        return None;
    }
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
    assert!(line.contains("listening"), "sim server said {line:?}");
    let port = line.trim().rsplit(':').next().and_then(|p| p.parse().ok());
    let port = port.unwrap_or_else(|| panic!("no port in {line:?}"));
    Some((Server(child), port))
}

fn step(at: f64, target: Target, gait: u32, axes: [f64; 3]) -> ScriptStep {
    ScriptStep {
        at,
        command: UserCommand {
            target,
            gait,
            axes,
            ..Default::default()
        },
    }
}

/// The threaded policy with its worker's answer always collected on the next tick: the robot's worker answers within
/// a tick or two, and here a loaded machine cannot stretch that, so no offer is refused and none is late.
struct Lockstep {
    inner: ThreadedPolicy,
    pending: bool,
}

impl PolicyService for Lockstep {
    /// Never refused: the last answer was collected, so the worker is idle and only its lock can be briefly held
    /// (between storing a result and waiting for the next request), which the real loop would retry a tick later.
    fn offer(&mut self, request: PolicyRequest) -> bool {
        while !self.inner.offer(request.clone()) {
            std::thread::yield_now();
        }
        self.pending = true;
        true
    }
    fn take_result(&mut self) -> Option<PolicyResult> {
        if !std::mem::take(&mut self.pending) {
            return None;
        }
        // a hang guard only (a dead worker), far above any inference: it never decides a result
        let give_up = std::time::Instant::now() + std::time::Duration::from_secs(60);
        loop {
            if let Some(result) = self.inner.take_result() {
                return Some(result);
            }
            assert!(
                std::time::Instant::now() < give_up,
                "policy worker never answered"
            );
            std::thread::yield_now();
        }
    }
    /// The worker's lock misses the offer retried past; none reaches the controller.
    fn dropped(&self) -> u64 {
        0
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    /// step-count pacing, the worker held to one tick (deterministic)
    Lockstep,
    /// wall-clock pacing, the worker free-running (as on the robot)
    Realtime,
}

/// One finished run: the trace, the controller after it, the loop's stats and the offers the policy dropped.
struct Run {
    trace: Vec<Sample>,
    controller: Controller,
    stats: rt::LoopStats,
    dropped: u64,
}

struct Sample {
    t: f64,
    state: State,
    runner: Option<String>,
    truth: Truth,
}

/// Run the script; returns a 50 Hz (controller time) trace of ground truth.
fn run_script(mode: Mode, script: Vec<ScriptStep>, seconds: f64) -> Option<Run> {
    let dir = common::policy_dir()?;
    let (_server, port) = start_server()?;
    let cfg = common::config();
    let mut sim = MujocoSim::connect(&format!("127.0.0.1:{port}")).unwrap();
    sim.reset(&serde_json::json!({ "pose": "lie" })).unwrap();
    let threaded = ThreadedPolicy::spawn(RunnerSet::load(&cfg, &dir).unwrap(), || {}).unwrap();
    let (mut lockstep, mut free);
    let (policy, pacing): (&mut dyn PolicyService, Pacing) = match mode {
        Mode::Lockstep => {
            lockstep = Lockstep {
                inner: threaded,
                pending: false,
            };
            (&mut lockstep, Pacing::Free)
        }
        Mode::Realtime => {
            free = threaded;
            (&mut free, Pacing::Realtime)
        }
    };
    let mut commands = ScriptedCommands::new(script);
    let mut controller = Controller::new(cfg.clone());
    let stop = AtomicBool::new(false);
    let mut trace = Vec::new();
    let stats = rt::run(
        &cfg,
        &mut sim,
        &mut controller,
        policy,
        &mut commands,
        pacing,
        &stop,
        |hw, tick| {
            let c = tick.controller;
            if tick.stats.ticks % 20 == 0 {
                let truth = hw.truth().unwrap();
                trace.push(Sample {
                    t: c.time(),
                    state: c.state,
                    runner: c.current_runner().map(String::from),
                    truth,
                });
            }
            if c.time() >= seconds {
                stop.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        },
    )
    .unwrap();
    let dropped = policy.dropped();
    Some(Run {
        trace,
        controller,
        stats,
        dropped,
    })
}

fn mean_vx(trace: &[Sample], t0: f64, t1: f64) -> f64 {
    let v: Vec<f64> = trace
        .iter()
        .filter(|s| s.t >= t0 && s.t < t1)
        .map(|s| s.truth.velocity[0])
        .collect();
    v.iter().sum::<f64>() / v.len() as f64
}

fn check(mode: Mode, run: &Run, walk: (f64, f64), speed: f64, runner: &str) {
    let Run {
        trace,
        controller,
        stats,
        dropped,
    } = run;
    let dropped = *dropped;
    let states: Vec<State> = controller.transitions.iter().map(|t| t.2).collect();
    println!("transitions {:?}", controller.transitions);
    println!("counters {:?} loop {:?}", controller.counters, stats);
    // the trailing joint damping is the shutdown sequence after the test stops the loop
    assert_eq!(
        states[..3],
        [State::StandUp, State::RlControl, State::LieDown],
        "transitions"
    );
    for s in trace.iter().filter(|s| s.state == State::RlControl) {
        assert!(
            !s.truth.trunk_contact && s.truth.tilt < 1.0,
            "fell at {:.2} s: {:?}",
            s.t,
            s.truth
        );
    }
    let walking: Vec<&Sample> = trace
        .iter()
        .filter(|s| s.t >= walk.0 && s.t < walk.1)
        .collect();
    assert!(
        walking.iter().all(|s| s.runner.as_deref() == Some(runner)),
        "walked on the wrong runner"
    );
    let vx = mean_vx(trace, (walk.0 + walk.1) / 2.0, walk.1);
    println!("{runner}: commanded {speed} m/s, walked {vx:.3} m/s");
    assert!((vx - speed).abs() < 0.15, "vx {vx:.3} vs {speed}");
    let end = trace.last().unwrap();
    assert!(
        end.truth.position[2] < 0.2,
        "not lying at the end: z {:.3}",
        end.truth.position[2]
    );
    let rl = &controller.transitions;
    let rl_s = rl[2].0 - rl[1].0;
    let expected = rl_s * controller.config().timing.control_hz
        / controller.config().timing.policy_decimation as f64;
    let steps = controller.counters.policy_steps as f64;
    println!("policy steps {steps} of {expected:.0} at 50 Hz, {dropped} offers dropped");
    match mode {
        // every offer taken and answered: a policy step every policy_decimation ticks, to the tick
        Mode::Lockstep => {
            assert_eq!(
                controller.counters.policy_offers_refused, 0,
                "offers refused"
            );
            assert!(
                (steps - expected).abs() <= 1.0,
                "policy steps {steps} of {expected:.0}"
            );
        }
        // the Mac and the Python sim are not real-time: a tick that lands > 1 ms late counts as an overrun; the
        // policy must still keep its 50 Hz in controller time
        Mode::Realtime => {
            let overrun = stats.overruns as f64 / stats.ticks as f64;
            println!(
                "overruns {:.2}% (max tick {:.1} ms)",
                overrun * 100.0,
                stats.max_tick_s * 1e3
            );
            assert!(overrun < 0.2, "overruns {:.2}%", overrun * 100.0);
            assert!(
                steps > 0.95 * expected,
                "policy fell behind: {steps} of {expected:.0}"
            );
        }
    }
}

fn navi_flat(mode: Mode) {
    let script = vec![
        step(0.0, Target::Passive, 0x3001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x3001, [0.0; 3]),
        step(8.5, Target::RlControl, 0x3002, [0.0; 3]),
        step(10.0, Target::RlControl, 0x3002, [0.25, 0.0, 0.0]),
        step(15.0, Target::RlControl, 0x3002, [0.0; 3]),
        step(17.0, Target::LieDown, 0x3002, [0.0; 3]),
    ];
    let Some(run) = run_script(mode, script, 24.0) else {
        return;
    };
    check(mode, &run, (10.5, 15.0), 0.5, "navi_flat");
}

fn basic_flat(mode: Mode) {
    let script = vec![
        step(0.0, Target::Passive, 0x1001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x1001, [0.0; 3]),
        step(12.5, Target::RlControl, 0x1001, [0.25, 0.0, 0.0]),
        step(17.5, Target::RlControl, 0x1001, [0.0; 3]),
        step(19.5, Target::LieDown, 0x1001, [0.0; 3]),
    ];
    let Some(run) = run_script(mode, script, 26.5) else {
        return;
    };
    check(mode, &run, (13.0, 17.5), 0.5, "basic_flat");
}

/// M20Controller.standup(): rl_control on 0x1001, the navigation gait as soon as the state reads RlControl (held
/// through the stand-in, so the stand-in hands over straight to the gait's runner), then 0.5 m/s forward for 5 s.
fn standup_rpc(mode: Mode, gait: u32, runner: &str) {
    let script = vec![
        step(0.0, Target::Passive, 0x1001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x1001, [0.0; 3]),
        step(5.2, Target::RlControl, gait, [0.0; 3]),
        step(11.0, Target::RlControl, gait, [0.25, 0.0, 0.0]),
        step(16.0, Target::RlControl, gait, [0.0; 3]),
        step(18.0, Target::LieDown, gait, [0.0; 3]),
    ];
    let Some(run) = run_script(mode, script, 25.0) else {
        return;
    };
    let runners: Vec<&str> = run
        .trace
        .iter()
        .filter(|s| s.state == State::RlControl)
        .filter_map(|s| s.runner.as_deref())
        .fold(Vec::new(), |mut v, r| {
            if v.last() != Some(&r) {
                v.push(r);
            }
            v
        });
    assert_eq!(runners, ["six_axis_stand", runner], "runner sequence");
    check(mode, &run, (11.5, 16.0), 0.5, runner);
}

/// standup() on the stairs terrain (0x3003 NaviStair, the gait the vendor captures walk on).
#[test]
fn standup_rpc_navi_stair() {
    standup_rpc(Mode::Lockstep, 0x3003, "basic_stair");
}

/// standup() on the flat terrain (0x3002 NaviFlat): safe only because BasicFlat never runs before it.
#[test]
fn standup_rpc_navi_flat() {
    standup_rpc(Mode::Lockstep, 0x3002, "navi_flat");
}

/// NaviCharge entry (1.5 s finetune_stand_15kg stand-in), then NaviFlat walking at 0.5 m/s.
#[test]
fn stand_walk_lie_navi_flat() {
    navi_flat(Mode::Lockstep);
}

/// The default path: BasicFlat entry runs the finetune_stand_15kg stand-in (6.14 s, as on the robot), then walks.
#[test]
fn stand_walk_lie_basic_flat() {
    basic_flat(Mode::Lockstep);
}

/// Both walks in wall-clock time with the free-running worker; needs an idle machine.
#[test]
#[ignore = "real time: needs an idle machine (cargo test --test sim_sequence -- --ignored)"]
fn realtime_stand_walk_lie() {
    navi_flat(Mode::Realtime);
    basic_flat(Mode::Realtime);
}

// ------------------------------------------------------------------------------------------------- docking

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use m20_controller::dock::detector::{Detection, Detector, Mount};
use m20_controller::dock::docking::{DockFeed, Docking, DockingCommands, Phase, Request, Status};
use m20_controller::dock::obstacle::{body_height, nearest_obstacle, GridParams};
use m20_controller::dock::vendor::ChargeManagerToml;
use m20_controller::dock::{wrap, ChargeReading};

/// The sim's charging dock (world.py charge_dock): the posts' midpoint in the world and the dock's turn.
#[derive(Clone, Copy)]
struct DockScene {
    post_x: f64,
    post_y: f64,
    post_yaw: f64,
}

impl DockScene {
    /// The robot's truth pose in the dock frame (x out along the axis, y left of it, yaw 0 = facing the posts).
    fn robot(&self, t: &Truth) -> [f64; 3] {
        let axis = self.post_yaw + std::f64::consts::PI;
        let (dx, dy) = (t.position[0] - self.post_x, t.position[1] - self.post_y);
        let (s, c) = axis.sin_cos();
        [
            c * dx + s * dy,
            -s * dx + c * dy,
            wrap(t.yaw - axis - std::f64::consts::PI),
        ]
    }
}

/// How the sim's batteries answer: the charge MOS comes on lying with the legs passive (when m20-battery's rule has
/// sent the request) when base_link was within `contact` (m, x and y in the dock frame) of the docked point as the
/// fine-tune ended (the sit began); never with `contact` None.
struct DockRun {
    scene: DockScene,
    contact: Option<[f64; 2]>,
    /// seconds of controller time to run at most
    seconds: f64,
}

struct DockResult {
    /// every status change with the controller time and the truth pose in the dock frame
    statuses: Vec<(f64, Status, [f64; 3])>,
    controller: Controller,
    trace: Vec<Sample>,
    /// where charge_manager docks the sim robot (truth, m out): docked_x_m less the detector's x bias on the sim's
    /// posts, measured before docking (reflective_column_node's centroid centres sit nearer the axes of the sim's
    /// ideal cylinders than of the real posts, which docked_x_m was measured on)
    docked_x: f64,
}

fn front_mount() -> ([f64; 3], [f64; 3]) {
    // dimos/robot/deeprobotics/m20/constants.py FRONT_LIDAR_XYZ_M / FRONT_LIDAR_RPY
    (
        [0.32028, 0.0, -0.013],
        [0.0, -std::f64::consts::FRAC_PI_2, std::f64::consts::PI],
    )
}

fn rear_mount() -> ([f64; 3], [f64; 3]) {
    // REAR_LIDAR_XYZ_M / REAR_LIDAR_RPY
    (
        [-0.32028, 0.0, -0.013],
        [0.0, -std::f64::consts::FRAC_PI_2, 0.0],
    )
}

/// NOS's charge_manager.toml (tests/fixtures).
fn vendor() -> ChargeManagerToml {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/charge_manager.toml");
    ChargeManagerToml::load(path.to_str().unwrap()).unwrap().0
}

/// Stand up on NaviCharge, dock(), and once it ends (charging or not) undock(); stop when that ends.
fn dock_run(run: DockRun) -> Option<DockResult> {
    let dir = common::policy_dir()?;
    let (_server, port) = start_server()?;
    let cfg = common::config();
    let mut sim = MujocoSim::connect(&format!("127.0.0.1:{port}")).unwrap();
    let s = run.scene;
    sim.reset(&serde_json::json!({
        "pose": "lie",
        "terrain": {"kind": "charge_dock", "post_x": s.post_x, "post_y": s.post_y, "post_yaw": s.post_yaw},
    }))
    .unwrap();
    let threaded = ThreadedPolicy::spawn(RunnerSet::load(&cfg, &dir).unwrap(), || {}).unwrap();
    let mut policy = Lockstep {
        inner: threaded,
        pending: false,
    };
    let mut lie = step(0.0, Target::Passive, 0x3001, [0.0; 3]);
    lie.command.motion_seq = 1;
    let mut stand = step(2.0, Target::RlControl, 0x3001, [0.0; 3]);
    stand.command.motion_seq = 2;
    let feed = Arc::new(DockFeed::default());
    let now = Arc::new(AtomicU64::new(0));
    let changes: Arc<Mutex<Vec<(u64, Status)>>> = Arc::default();
    let (now_cb, changes_cb) = (now.clone(), changes.clone());
    let mut commands = DockingCommands::new(
        ScriptedCommands::new(vec![lie, stand]),
        Docking::new(&cfg, true, Some(&vendor())),
        feed.clone(),
        Box::new(move |status| {
            let tick = now_cb.load(Ordering::Relaxed);
            println!("{:7.2} s dock {:?}", tick as f64 / 1000.0, status);
            changes_cb.lock().unwrap().push((tick, status.clone()));
        }),
    );
    let mut controller = Controller::new(cfg.clone());
    let stop = AtomicBool::new(false);
    let (xyz, rpy) = front_mount();
    let mount = Mount::from_xyz_rpy(xyz, rpy);
    let detector = Detector::new(&cfg.dock);
    let (rear_xyz, rear_rpy) = rear_mount();
    let rear = Mount::from_xyz_rpy(rear_xyz, rear_rpy);
    let grid = GridParams::from_toml(&vendor());
    let min_range = cfg.dock.obstacle.min_lidar_range_m;
    let mut obstacle_checks = 0u64;
    let elevations: Vec<f64> = (0..96)
        .map(|b| b as f64 * 90.0 / 95.0)
        .filter(|e| *e >= 25.0)
        .collect();
    let azimuths: Vec<f64> = (0..900).map(|k| k as f64 * 0.4).collect();
    let mut docked_x = cfg.dock.sequence.docked_x_m;
    let mut biases: Vec<f64> = Vec::new();
    let lie_ticks = (2.0 * cfg.states.lie_down.phase_s * cfg.timing.control_hz) as u64;
    let (mut sweeps, mut reports, mut lying_ticks) = (0u64, 0u64, 0u64);
    let mut lie_start: Option<[f64; 3]> = None;
    let (mut dock_asked, mut undock_asked) = (false, false);
    let mut statuses = Vec::new();
    let mut seen = 0usize;
    let mut trace = Vec::new();
    rt::run(
        &cfg,
        &mut sim,
        &mut controller,
        &mut policy,
        &mut commands,
        Pacing::Free,
        &stop,
        |hw, tick| {
            let c = tick.controller;
            let ticks = tick.stats.ticks;
            now.store(ticks, Ordering::Relaxed);
            let truth = hw.truth().unwrap();
            if ticks % 20 == 0 {
                trace.push(Sample {
                    t: c.time(),
                    state: c.state,
                    runner: c.current_runner().map(String::from),
                    truth,
                });
            }
            // the front Airy at 10 Hz through the detector, as m20_dock_detector_module does
            if ticks % 100 == 0 {
                let request = serde_json::json!({
                    "mount_xyz": xyz, "mount_rpy": rpy, "elevations_deg": elevations,
                    "azimuths_deg": azimuths, "range_noise_m": 0.01, "seed": 1,
                });
                let points = hw.lidar(&request).unwrap();
                // charge_manager's obstacle check while open, as m20_dock_detector_module runs it (both Airys)
                if let Some(direction) = statuses
                    .last()
                    .and_then(|(_, st, _): &(f64, Status, [f64; 3])| st.obstacle)
                {
                    let rear_request = serde_json::json!({
                        "mount_xyz": rear_xyz, "mount_rpy": rear_rpy, "elevations_deg": elevations,
                        "azimuths_deg": azimuths, "range_noise_m": 0.01, "seed": 2,
                    });
                    let rear_points = hw.lidar(&rear_request).unwrap();
                    let base = |p: &[f32; 4], m: &Mount| {
                        let v = [p[0] as f64, p[1] as f64, p[2] as f64];
                        let r = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
                        (r >= min_range).then(|| {
                            let b = m.to_base(v);
                            [b[0] as f32, b[1] as f32, b[2] as f32]
                        })
                    };
                    let all = points
                        .iter()
                        .filter_map(|p| base(p, &mount))
                        .chain(rear_points.iter().filter_map(|p| base(p, &rear)));
                    let height =
                        body_height(&cfg.robot, &tick.state.q, cfg.dock.obstacle.wheel_radius_m);
                    obstacle_checks += 1;
                    *feed.obstacle.lock().unwrap() = Some((
                        nearest_obstacle(all, direction, height, &grid),
                        obstacle_checks,
                    ));
                }
                if let Detection::Found { pose, .. } = detector.detect(points) {
                    let b = pose.robot_in_dock()[0] - s.robot(&truth)[0];
                    if !dock_asked {
                        biases.push(b);
                    }
                    sweeps += 1;
                    *feed.pose.lock().unwrap() = Some((pose, sweeps));
                }
            }
            // the battery packs at 1 Hz
            lying_ticks = if c.state == State::LieDown {
                lying_ticks + 1
            } else {
                0
            };

            if ticks % 1000 == 0 {
                reports += 1;
                let on_contacts = match (run.contact, lie_start) {
                    (Some(w), Some(r)) => (r[0] - docked_x).abs() <= w[0] && r[1].abs() <= w[1],
                    _ => false,
                };
                let charging = on_contacts && lying_ticks >= lie_ticks;
                *feed.charge.lock().unwrap() = Some(ChargeReading {
                    seq: reports,
                    packs_present: 2,
                    packs_charge_mos_on: if charging { 2 } else { 0 },
                    charger_connected: charging,
                });
            }
            let changed: Vec<(u64, Status)> = changes.lock().unwrap()[seen..].to_vec();
            seen += changed.len();
            for (at, status) in changed {
                // the pose charge_manager logs its last on-dock delta at: the fine-tune's end, as the sit begins
                if status.phase == Phase::SitHold {
                    lie_start = Some(s.robot(&truth));
                }
                statuses.push((at as f64 / 1000.0, status, s.robot(&truth)));
            }
            let phase = statuses.last().map(|(_, st, _)| st.phase);
            if dock_asked && ticks % 500 == 0 && std::env::var_os("M20_DOCK_TRACE").is_some() {
                let r = s.robot(&truth);
                println!(
                    "{:7.2} s {:?} {:?} axes {:+.3?} truth x {:+.4} y {:+.4} yaw {:+.2} deg",
                    c.time(),
                    phase,
                    c.current_runner(),
                    tick.user.axes,
                    r[0] - docked_x,
                    r[1],
                    r[2].to_degrees()
                );
            }
            // stood up (no stand-in on a 0x3001 entry): dock
            if !dock_asked && c.state == State::RlControl && c.time() > 8.0 {
                *feed.request.lock().unwrap() = Some(Request::Dock);
                dock_asked = true;
                biases.sort_by(f64::total_cmp);
                let bias = biases[biases.len() / 2];
                docked_x -= bias;
                println!("sim detector x bias {bias:+.4} m: docked at {docked_x:.4} m out");
            }
            let ended = |p: Option<Phase>| matches!(p, Some(Phase::Charging) | Some(Phase::Idle));
            let docking_started = statuses
                .iter()
                .any(|(_, st, _)| st.phase == Phase::EnterGait);
            // undock from charging, or (after a failure) as charge_manager allows
            if docking_started && !undock_asked && ended(phase) {
                let at = statuses.last().map_or(0.0, |s| s.0);
                // a moment of lying (charging) or standing first
                if c.time() > at + 2.0 {
                    *feed.request.lock().unwrap() = Some(Request::Undock);
                    undock_asked = true;
                }
            }
            let undock_started = statuses.iter().any(|(_, st, _)| st.phase == Phase::StandUp);
            let done = undock_asked && undock_started && phase == Some(Phase::Idle);
            if done || c.time() >= run.seconds {
                stop.store(true, Ordering::Relaxed);
            }
        },
    )
    .unwrap();
    Some(DockResult {
        statuses,
        controller,
        trace,
        docked_x,
    })
}

fn no_fall(result: &DockResult) {
    for s in result.trace.iter().filter(|s| s.state == State::RlControl) {
        assert!(
            !s.truth.trunk_contact && s.truth.tilt < 1.0,
            "fell at {:.2} s: {:?}",
            s.t,
            s.truth
        );
    }
    assert!(
        result.controller.fault.is_none(),
        "fault {:?}",
        result.controller.fault
    );
}

/// The first status in `phase`, and the robot (dock frame) then.
fn entered(result: &DockResult, phase: Phase) -> Option<&(f64, Status, [f64; 3])> {
    result.statuses.iter().find(|(_, s, _)| s.phase == phase)
}

fn dock_and_undock(scene: DockScene) {
    let cfg = common::config();
    let seq = &cfg.dock.sequence;
    // where charging succeeds, from the real docks: charge_manager's delta at the end of the on-dock fine-tune
    // ("已到达桩上", just before the sit and lie-down) was within 3.4 mm along the dock and 2.3 mm across on
    // 2026-11-24 05:51:24 (-0.0034, +0.0023) and 2026-11-21 13:28:07 (+0.0027, -0.0002), and both charged; neither
    // recording has a failed dock or the pose after lying. So the sim charges when the fine-tune ends within 1 cm of
    // the docked point either way (the real successes with margin; where real charging stops working is not
    // measured), and the sim's slide while sitting and lying (2-5 cm back) is not counted: the real one's is unknown.
    let Some(result) = dock_run(DockRun {
        scene,
        contact: Some([0.01, 0.01]),
        seconds: 150.0,
    }) else {
        return;
    };
    no_fall(&result);
    let fine = entered(&result, Phase::EnterFinetune).expect("never reached the fine point");
    let e = [
        fine.2[0] - result.docked_x,
        fine.2[1],
        fine.2[2].to_degrees(),
    ];
    println!(
        "fine point at {:.1} s: truth error x {:+.4} m y {:+.4} m yaw {:+.2} deg; measured {:?}",
        fine.0, e[0], e[1], e[2], fine.1.fine_point
    );
    let lying = entered(&result, Phase::ConfirmCharge).expect("never lay down");
    println!(
        "lying at {:.1} s: x {:+.4} m y {:+.4} m yaw {:+.2} deg from the docked point",
        lying.0,
        lying.2[0] - result.docked_x,
        lying.2[1],
        lying.2[2].to_degrees()
    );
    let charging = entered(&result, Phase::Charging).expect("never charging");
    assert!(charging.1.charging && charging.1.packs_charge_mos_on == 2);
    let start = charging.1.start_point.expect("no start point");
    let last = result.statuses.last().unwrap();
    println!(
        "undocked at {:.1} s: {:.4} m out (start point {start:?} + docked), y {:+.4} m, yaw {:+.2} deg; {:?}",
        last.0,
        last.2[0],
        last.2[1],
        last.2[2].to_degrees(),
        last.1
    );
    // the fine point as charge_manager measured it, inside its delta + margin (what ends its fine approach), and the
    // truth within that plus the sim detector's bias there: up to ~4 mm (the standing body's 1-2 deg of tilt through
    // the lidar's 0.45 m above the posts, and beam quantization; the detector takes base_link as level)
    let v = vendor();
    let delta = [
        v.delta_x as f64 / 1000.0,
        v.delta_y as f64 / 1000.0,
        v.delta_yaw as f64 / 100.0,
    ];
    let reach: Vec<f64> = (0..3).map(|k| delta[k] + seq.fine_margin[k]).collect();
    let measured = fine.1.fine_point.expect("no fine point measured");
    for k in 0..3 {
        assert!(
            (measured[k] as f64).abs() < reach[k],
            "measured fine point {measured:?}"
        );
    }
    const SIM_DETECTOR_BIAS_M: f64 = 0.004;
    assert!(
        e[0].abs() <= reach[0] + SIM_DETECTOR_BIAS_M,
        "fine x {:+.4}",
        e[0]
    );
    assert!(
        e[1].abs() <= reach[1] + SIM_DETECTOR_BIAS_M,
        "fine y {:+.4}",
        e[1]
    );
    assert!(e[2].to_radians().abs() <= reach[2], "fine yaw {:+.2}", e[2]);
    assert_eq!(last.1.phase, Phase::Idle);
    assert!(!last.1.error, "{}", last.1.result);
    assert_eq!(last.1.result, "undocked");
    // back_way = auto: back to where docking started, inside ext_delta
    let ext = [v.ext_delta_x as f64 / 1000.0, v.ext_delta_y as f64 / 1000.0];
    let out = last.2[0] - result.docked_x - start[0] as f64;
    assert!(
        out.abs() <= ext[0] + SIM_DETECTOR_BIAS_M,
        "undock {out:+.3} m off its goal"
    );
    assert!((last.2[1] - start[1] as f64).abs() <= ext[1] + SIM_DETECTOR_BIAS_M);
    // the gait docking started from (0x3001 in these runs) restored
    assert_eq!(
        result.controller.current_runner(),
        Some(cfg.runner_for_gait(0x3001))
    );
}

/// ~2 m in front of the dock with a lateral offset and a heading error: dock, charge, undock.
#[test]
fn dock_charge_undock_offset() {
    dock_and_undock(DockScene {
        post_x: 2.6,
        post_y: 0.15,
        post_yaw: 8f64.to_radians(),
    });
}

/// The other side: offset right, dock turned the other way.
#[test]
fn dock_charge_undock_offset_other_side() {
    dock_and_undock(DockScene {
        post_x: 2.5,
        post_y: -0.2,
        post_yaw: -6f64.to_radians(),
    });
}

/// The charge MOS never comes on: lie down, stand, back to the staging point and retry (adjust_posture_times), then
/// NoChargePlug_way = goBack: back off to where docking started.
#[test]
fn dock_without_charge_retries_then_backs_off() {
    let v = vendor();
    let Some(result) = dock_run(DockRun {
        scene: DockScene {
            post_x: 2.6,
            post_y: 0.05,
            post_yaw: 0.0,
        },
        contact: None,
        seconds: 300.0,
    }) else {
        return;
    };
    no_fall(&result);
    let lie_downs = result
        .statuses
        .windows(2)
        .filter(|w| w[0].1.phase != Phase::ConfirmCharge && w[1].1.phase == Phase::ConfirmCharge)
        .count();
    let last = result.statuses.last().unwrap();
    println!("{lie_downs} tries; {:?}", last.1);
    assert_eq!(lie_downs as i32, v.adjust_posture_times + 1);
    assert!(entered(&result, Phase::StandForRetry).is_some());
    assert!(entered(&result, Phase::Charging).is_none());
    assert!(
        last.1.error && last.1.result.starts_with("no charge"),
        "{}",
        last.1.result
    );
    let start = last.1.start_point.expect("no start point");
    let out = last.2[0] - result.docked_x - start[0] as f64;
    assert!(
        out.abs() <= v.ext_delta_x as f64 / 1000.0 + 0.004,
        "backed off {out:+.3} m off its goal"
    );
}
