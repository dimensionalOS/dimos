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

//! m20_shadow: the controller in dry-run beside rl_deploy. It reads rl_deploy's joint frames from the ecan_tap.so
//! shim's live stream, the IMU and commands off DDS, and rl_deploy's journal, and logs what it would send next to
//! what rl_deploy sent, every cycle. Built without the "ecan" feature: no motor-bus transport is compiled in, and a
//! seccomp filter makes every send / write on its sockets fail.
//!
//!   m20_shadow --config m20_controller.toml --shadow shadow.toml [--policies DIR]          (live, on AOS)
//!   m20_shadow --config m20_controller.toml --shadow shadow.toml --session DIR --log DIR  (a recorded session)

#![cfg_attr(
    not(test),
    deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

use std::path::PathBuf;
#[cfg(target_os = "linux")]
use std::sync::atomic::{AtomicBool, Ordering};
#[cfg(target_os = "linux")]
use std::sync::mpsc;
#[cfg(target_os = "linux")]
use std::time::Duration;

use anyhow::{bail, Context, Result};
use serde_json::json;

use m20_controller::config::Config;
use m20_controller::frames::{decode_command, decode_state, Mapping};
use m20_controller::policy::RunnerSet;
use m20_controller::shadow::config::ShadowConfig;
use m20_controller::shadow::engine::{Engine, STATES};
use m20_controller::shadow::log::{Logger, RECORD_SIZE};
use m20_controller::shadow::session::{self, Input};
use m20_controller::shadow::sniff::{Parser, Sample};

#[cfg(target_os = "linux")]
static STOP: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "linux")]
extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

fn realtime_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

#[cfg(target_os = "linux")]
/// Pin every thread (they inherit it) to the configured CPUs and lower the priority.
fn place(scfg: &ShadowConfig) -> Result<()> {
    #[cfg(target_os = "linux")]
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        for &c in &scfg.cpus {
            libc::CPU_SET(c, &mut set);
        }
        if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
            bail!("sched_setaffinity: {}", std::io::Error::last_os_error());
        }
        if libc::setpriority(libc::PRIO_PROCESS, 0, scfg.nice) != 0 {
            bail!("setpriority: {}", std::io::Error::last_os_error());
        }
    }
    #[cfg(not(target_os = "linux"))]
    let _ = scfg;
    Ok(())
}

#[cfg(target_os = "linux")]
fn cpu_seconds() -> f64 {
    let mut u: libc::rusage = unsafe { std::mem::zeroed() };
    unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut u) };
    let tv = |t: libc::timeval| t.tv_sec as f64 + t.tv_usec as f64 * 1e-6;
    tv(u.ru_utime) + tv(u.ru_stime)
}

struct Args {
    config: PathBuf,
    shadow: PathBuf,
    policies: PathBuf,
    session: Option<PathBuf>,
    log: Option<PathBuf>,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        config: PathBuf::from("m20_controller.toml"),
        shadow: PathBuf::from("shadow.toml"),
        policies: PathBuf::from(std::env::var("M20_POLICY_DIR").unwrap_or_else(|_| {
            format!(
                "{}/.cache/dimos/m20_policies",
                std::env::var("HOME").unwrap_or_default()
            )
        })),
        session: None,
        log: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--config" => a.config = value()?.into(),
            "--shadow" => a.shadow = value()?.into(),
            "--policies" => a.policies = value()?.into(),
            "--session" => a.session = Some(value()?.into()),
            "--log" => a.log = Some(value()?.into()),
            "-h" | "--help" => {
                println!("see the header of src/bin/m20_shadow.rs");
                std::process::exit(0);
            }
            other => bail!("unknown argument {other}"),
        }
    }
    Ok(a)
}

fn main() -> Result<()> {
    let args = parse_args()?;
    let cfg = Config::load(&args.config)?;
    let scfg = ShadowConfig::load(&args.shadow)?;
    let map = Mapping::new(&cfg.ecan)?;
    let header = json!({
        "record_size": RECORD_SIZE,
        "joints": cfg.robot.joint_names,
        "runners": Engine::runner_names(&cfg),
        "states": STATES,
        "started": realtime_now(),
        "source": args.session.as_ref().map(|p| p.display().to_string()).unwrap_or_else(|| "live".into()),
    });
    let log_dir = args
        .log
        .clone()
        .unwrap_or_else(|| PathBuf::from(&scfg.log_dir));
    let logger = Logger::new(&log_dir, &header, scfg.file_mb, scfg.max_total_mb)?;
    let runners = RunnerSet::load(&cfg, &args.policies)?;
    let mut engine = Engine::new(cfg.clone(), scfg.clone(), runners, logger);
    if let Some(dir) = &args.session {
        let mut parser = Parser::new(&scfg)?;
        let inputs = session::load(dir, &mut parser)?;
        eprintln!("{} inputs from {}", inputs.len(), dir.display());
        for input in inputs {
            let now = input.time();
            feed(&mut engine, &map, &cfg, input);
            engine.drain(now, false)?;
        }
        engine.drain(f64::INFINITY, true)?;
        engine.flush()?;
        let c = &engine.counters;
        eprintln!(
            "cycles {} ours {} policy steps {} (errors {}) no imu {}",
            c.cycles, c.ours, c.policy_steps, c.policy_errors, c.no_imu
        );
        return Ok(());
    }
    live(engine, map, cfg, scfg)
}

fn feed(engine: &mut Engine, map: &Mapping, cfg: &Config, input: Input) {
    match input {
        Input::Frame(t_ns, sent, frame) => {
            if sent {
                if let Ok(Some(c)) = decode_command(&frame, cfg.ecan.ethertype, map) {
                    engine.tx(t_ns, c);
                }
            } else if let Ok(Some(s)) = decode_state(&frame, cfg.ecan.ethertype, map) {
                engine.rx(s);
            }
        }
        Input::Sample(Sample::Imu(i)) => engine.imu(i),
        Input::Sample(Sample::Steer(s)) => engine.steer(s),
        Input::Event(t, e) => engine.event(t, e),
    }
}

#[cfg(not(target_os = "linux"))]
fn live(_: Engine, _: Mapping, _: Config, _: ShadowConfig) -> Result<()> {
    bail!("live shadow mode runs on the robot (Linux); use --session here")
}

#[cfg(target_os = "linux")]
fn live(mut engine: Engine, map: Mapping, cfg: Config, scfg: ShadowConfig) -> Result<()> {
    use m20_controller::shadow::sniff::{self, socket::Sniffer};
    use m20_controller::shadow::{journal, seccomp, tap::TapReader};

    place(&scfg)?;
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
    // every input is opened (and journalctl started) before the lock-down; nothing can be opened after
    let mut tap = TapReader::bind(
        &scfg.tap_socket,
        Duration::from_millis(scfg.tap_idle_timeout_ms.max(1)),
    )?;
    let mut sniffers = Vec::new();
    for name in &scfg.sniff_interfaces {
        match Sniffer::open(name, scfg.dds_port_range[0], scfg.dds_port_range[1]) {
            Ok(s) => sniffers.push(s),
            Err(e) => eprintln!("sniffer on {name}: {e:#}"),
        }
    }
    let (etx, erx) = mpsc::channel();
    let mut journal_child =
        journal::follow(&scfg.journal_unit, scfg.journal_history_lines, etx.clone())?;
    // the day's log file in rl_deploy's local time, the next day's from its midnight
    let log_follower = journal::follow_log(
        &scfg.rl_log,
        scfg.utc_offset_s,
        scfg.rl_log_history_lines,
        etx,
    )?;
    let mut fds = vec![tap.fd()];
    fds.extend(sniffers.iter().map(|s| s.fd));
    seccomp::lock_down(&fds)?;
    eprintln!(
        "shadow: inputs open and locked down: {}",
        seccomp::verify(&fds)?
    );
    let (stx, srx) = mpsc::channel();
    let parser = Parser::new(&scfg)?;
    std::thread::Builder::new()
        .name("shadow-sniff".into())
        .spawn(move || sniff::run(sniffers, parser, stx, realtime_now))?;
    let mut last_status = 0u64;
    let started_cpu = cpu_seconds();
    let started = realtime_now();
    let mut dropped = (0u32, 0u32);
    while !STOP.load(Ordering::Relaxed) {
        for r in tap.recv() {
            dropped = (r.ring_dropped, r.send_dropped);
            let input = Input::Frame(r.timestamp_ns, r.sent, r.frame);
            feed(&mut engine, &map, &cfg, input);
        }
        while let Ok(s) = srx.try_recv() {
            feed(&mut engine, &map, &cfg, Input::Sample(s));
        }
        while let Ok((t, e)) = erx.try_recv() {
            engine.event(t, e);
        }
        engine.drain(realtime_now(), false)?;
        let c = &engine.counters;
        if c.cycles >= last_status + scfg.status_every {
            last_status = c.cycles;
            let wall = realtime_now() - started;
            eprintln!(
                "shadow: cycles {} ours {} policy steps {} (errors {}) no imu {} dropped {} pending {} cpu {:.1}% shim dropped {}/{}",
                c.cycles,
                c.ours,
                c.policy_steps,
                c.policy_errors,
                c.no_imu,
                c.dropped,
                engine.pending(),
                100.0 * (cpu_seconds() - started_cpu) / wall.max(1e-9),
                dropped.0,
                dropped.1
            );
            engine.flush()?;
        }
    }
    engine.drain(f64::INFINITY, true)?;
    engine.flush()?;
    let _ = journal_child.kill();
    let _ = journal_child.wait();
    log_follower.stop();
    eprintln!("shadow: stopped");
    Ok(())
}
