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

//! m20_controller: the rl_deploy replacement. Never run it on the robot without the vendor controller stopped and
//! an operator's go; the ECAN backend is implemented but untested on hardware.
//!
//!   m20_controller --config m20_controller.toml --policies DIR --backend mujoco [--sim 127.0.0.1:43921]
//!                  [--script commands.json] [--seconds N] [--pacing realtime|free] [--inline-policy]
//!   m20_controller ... --backend ecan        (eth1 motor bus; needs CAP_NET_RAW and an IMU source)

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;

use anyhow::{bail, Context, Result};

use m20_controller::commands::{CommandSource, ScriptedCommands, UdpCommands};
use m20_controller::config::Config;
use m20_controller::hardware::ecan::{EcanBus, NoImu};
use m20_controller::hardware::mujoco::MujocoSim;
use m20_controller::hardware::Hardware;
use m20_controller::policy::{InlinePolicy, PolicyService, RunnerSet, ThreadedPolicy};
use m20_controller::rt::{self, Pacing};
use m20_controller::state_machine::{Controller, State};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

struct Args {
    config: PathBuf,
    policies: PathBuf,
    backend: String,
    sim: String,
    script: Option<PathBuf>,
    seconds: Option<f64>,
    pacing: Pacing,
    inline_policy: bool,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        config: PathBuf::from("m20_controller.toml"),
        policies: PathBuf::from(std::env::var("HOME").unwrap_or_default())
            .join(".cache/dimos/m20_policies"),
        backend: "mujoco".into(),
        sim: "127.0.0.1:43921".into(),
        script: None,
        seconds: None,
        pacing: Pacing::Realtime,
        inline_policy: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--config" => a.config = value()?.into(),
            "--policies" => a.policies = value()?.into(),
            "--backend" => a.backend = value()?,
            "--sim" => a.sim = value()?,
            "--script" => a.script = Some(value()?.into()),
            "--seconds" => a.seconds = Some(value()?.parse()?),
            "--pacing" => {
                a.pacing = match value()?.as_str() {
                    "realtime" => Pacing::Realtime,
                    "free" => Pacing::Free,
                    other => bail!("unknown pacing {other}"),
                }
            }
            "--inline-policy" => a.inline_policy = true,
            "-h" | "--help" => {
                println!("see the header of src/bin/m20_controller.rs");
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
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
    let runners = RunnerSet::load(&cfg, &args.policies)?;
    let mut policy: Box<dyn PolicyService> = if args.inline_policy {
        Box::new(InlinePolicy::new(runners))
    } else {
        let rt_cfg = cfg.realtime.clone();
        Box::new(ThreadedPolicy::spawn(runners, move || {
            if let Err(e) =
                rt::apply_realtime(rt_cfg.policy_cpu, rt_cfg.policy_fifo_priority, false)
            {
                eprintln!("policy thread realtime setup: {e:#}");
            }
        }))
    };
    let mut hardware: Box<dyn Hardware> = match args.backend.as_str() {
        "mujoco" => {
            let mut sim = MujocoSim::connect(&args.sim)?;
            sim.reset(&serde_json::json!({ "pose": "lie" }))?;
            Box::new(sim)
        }
        "ecan" => Box::new(EcanBus::open(&cfg.ecan, Box::new(NoImu))?),
        other => bail!("unknown backend {other}"),
    };
    let mut commands: Box<dyn CommandSource> = match &args.script {
        Some(path) => Box::new(ScriptedCommands::load(path)?),
        None => Box::new(UdpCommands::bind(&cfg.command_input.udp_bind)?),
    };
    rt::control_realtime(&cfg.realtime)?;
    // log lines leave the 1 kHz thread through a channel; a plain thread prints them
    let (log_tx, log_rx) = mpsc::channel::<String>();
    let printer = std::thread::spawn(move || {
        for line in log_rx {
            eprintln!("{line}");
        }
    });
    let mut controller = Controller::new(cfg.clone());
    let mut last_state = State::Passive;
    let mut last_fault: Option<String> = None;
    let seconds = args.seconds;
    let stats = rt::run(
        &cfg,
        hardware.as_mut(),
        &mut controller,
        policy.as_mut(),
        commands.as_mut(),
        args.pacing,
        &STOP,
        |_, tick| {
            let c = tick.controller;
            if c.state != last_state {
                let _ = log_tx.send(format!(
                    "{:.3} {:?} -> {:?} runner {:?}",
                    c.time(),
                    last_state,
                    c.state,
                    c.current_runner()
                ));
                last_state = c.state;
            }
            if c.fault != last_fault {
                let _ = log_tx.send(format!("{:.3} fault {:?}", c.time(), c.fault));
                last_fault = c.fault.clone();
            }
            if tick.stats.ticks % 1000 == 0 {
                let k = &c.counters;
                let _ = log_tx.send(format!(
                    "{:.0} s: policy steps {} refused {} infer {:.2} ms (max {:.2}) overruns {} hw errors {} max tick {:.2} ms",
                    c.time(),
                    k.policy_steps,
                    k.policy_offers_refused,
                    k.last_inference_s * 1e3,
                    k.max_inference_s * 1e3,
                    tick.stats.overruns,
                    tick.stats.hardware_errors,
                    tick.stats.max_tick_s * 1e3
                ));
            }
            if seconds.is_some_and(|s| c.time() >= s) {
                STOP.store(true, Ordering::Relaxed);
            }
        },
    )?;
    drop(log_tx);
    let _ = printer.join();
    eprintln!("done: {stats:?}");
    Ok(())
}
