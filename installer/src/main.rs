//! `dimos`: parse, hand anything the DimOS Python CLI owns straight to it, then probe this machine
//! once, run one plan, and record what happened.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use anyhow::{Context, Result};
use clap::Parser;

use dimos_installer::cli::{
    hardware_argv, Cli, Command, HardwareTarget, RobotAction, ServiceAction,
};
use dimos_installer::pkgs::Platforms;
use dimos_installer::plan::{Action, Plan, Stage};
use dimos_installer::probe::Probes;
use dimos_installer::run::{self, Ctx};
use dimos_installer::say;
use dimos_installer::setup::g1;
use dimos_installer::state::{self, LastRun};
use dimos_installer::{forward, hardware, robot, service, setup, uninstall, update};

const SETUP_FIRST: &str = "no DimOS install recorded: run `dimos setup` first";

fn main() -> ExitCode {
    let code = match run(&Cli::parse()) {
        Ok(code) => code,
        Err(e) => {
            say::fail(&format!("{e:#}"));
            exit_code(&e)
        }
    };
    ExitCode::from(code as u8)
}

fn run(cli: &Cli) -> Result<i32> {
    let home = dirs::home_dir().context("no home directory: set $HOME and re-run")?;
    match forwarded(&cli.command) {
        Some(argv) => forward(&argv, &home),
        None => installer(cli, &home),
    }
}

/// The verbs this binary implements: probe once, run one plan, record the result.
fn installer(cli: &Cli, home: &Path) -> Result<i32> {
    let cfg = Platforms::load();
    let mut ctx = Ctx::new(cli, home)?;
    let probes = Probes::detect(&sysctl_keys(&cfg), home)?;
    let code = dispatch(cli, &mut ctx, &probes, &cfg, home)?;
    if ctx.dry_run {
        return Ok(code);
    }
    if let Some(verb) = recorded_as(&cli.command) {
        record(home, verb, code)?;
    }
    if code == 0 {
        clear_backup(&mut ctx, home, &cli.command)?;
    }
    Ok(code)
}

fn dispatch(
    cli: &Cli,
    ctx: &mut Ctx,
    probes: &Probes,
    cfg: &Platforms,
    home: &Path,
) -> Result<i32> {
    match &cli.command {
        Command::Setup(args) => setup::run(args, ctx, probes, cfg, home),
        Command::Update(args) => update::run(args, ctx, probes, cfg, home),
        Command::Service { action } => service_run(action, ctx, probes, home),
        Command::Hardware { target } => hardware_run(target, ctx, probes, cfg, home),
        Command::Robot {
            action: RobotAction::Scan(args),
        } => robot::run(args, ctx, probes),
        Command::Uninstall => uninstall::run(ctx, home),
        Command::External(argv) => forward(argv, home),
    }
}

/// Detection costs `ldd`, `dpkg -l` and `sw_vers`; a verb the Python CLI owns must not pay for them.
fn forwarded(command: &Command) -> Option<Vec<OsString>> {
    match command {
        Command::External(argv) => Some(argv.clone()),
        Command::Hardware { target } if hardware::owned(target).is_none() => {
            Some(hardware_argv(target))
        }
        _ => None,
    }
}

/// Replace this process with the venv's `dimos`; a return means there was no venv to replace it.
fn forward(argv: &[OsString], home: &Path) -> Result<i32> {
    let cwd = std::env::current_dir().context("no working directory: cd somewhere and re-run")?;
    let venv_env = std::env::var_os("VIRTUAL_ENV").map(PathBuf::from);
    let Some(path) = forward::venv_dimos(state::load(home)?.as_ref(), venv_env, &cwd) else {
        say::fail(forward::NO_VENV_HINT);
        return Ok(2);
    };
    match forward::exec(&path, argv)? {}
}

/// `g1 setup` and `jetson setup` are ours; every other `hardware ...` verb is the Python CLI's.
fn hardware_run(
    target: &HardwareTarget,
    ctx: &mut Ctx,
    probes: &Probes,
    cfg: &Platforms,
    home: &Path,
) -> Result<i32> {
    match hardware::owned(target) {
        Some((robot, args)) => hardware::run(robot, args, ctx, probes, cfg, home),
        None => forward(&hardware_argv(target), home),
    }
}

/// The only command planned here, because a systemd unit has no second caller.
fn service_run(action: &ServiceAction, ctx: &mut Ctx, probes: &Probes, home: &Path) -> Result<i32> {
    let Some(installed) = probes.installed.as_ref() else {
        say::fail(SETUP_FIRST);
        return Ok(2);
    };
    let steps = service::plan(action, installed, &probes.platform, cdds_home(home))?;
    let report = run::run(&steps, ctx)?;
    report.print(ctx);
    Ok(report.exit_code())
}

/// The unit exports CYCLONEDDS_HOME only when `hardware g1 setup` actually built it.
fn cdds_home(home: &Path) -> Option<PathBuf> {
    let path = g1::cyclonedds_home(home);
    path.is_dir().then_some(path)
}

/// installer.json keeps the last result; there is nothing to write until `setup` has created it.
fn record(home: &Path, command: &str, code: i32) -> Result<()> {
    let Some(mut installed) = state::load(home)? else {
        return Ok(());
    };
    installed.last = Some(LastRun {
        command: command.to_string(),
        exit_code: code,
        at: state::now_iso(),
    });
    state::save(home, &installed)
}

/// The `update` that swapped the binary is still the old process image, so only a later clean run
/// of another verb proves the new one; the removal goes through the executor like every mutation.
fn clear_backup(ctx: &mut Ctx, home: &Path, command: &Command) -> Result<()> {
    let bak = state::backup_bin(home);
    if matches!(command, Command::Update(_)) || !bak.exists() {
        return Ok(());
    }
    let steps = Plan {
        command: "clear-backup".to_string(),
        stages: vec![Stage::new("clear-backup", false).push(Action::Remove {
            path: bak,
            sudo: false,
        })],
        notes: Vec::new(),
    };
    run::run(&steps, ctx).map(|_| ())
}

/// The verb installer.json's `last` records; a scan saves nothing, and a forward never returns.
fn recorded_as(command: &Command) -> Option<&'static str> {
    match command {
        Command::Setup(_) => Some("setup"),
        Command::Update(_) => Some("update"),
        Command::Service { .. } => Some("service"),
        Command::Hardware { .. } => Some("hardware"),
        Command::Uninstall => Some("uninstall"),
        Command::Robot { .. } | Command::External(_) => None,
    }
}

fn sysctl_keys(cfg: &Platforms) -> Vec<&str> {
    cfg.linux.sysctl.keys().map(String::as_str).collect()
}

/// `plan::Report` owns 0 ok / 1 failed / 2 needs a human; an error that never reached a report is 1,
/// or 130 when a prompt was interrupted.
fn exit_code(err: &anyhow::Error) -> i32 {
    if err.chain().any(interrupted) {
        130
    } else {
        1
    }
}

fn interrupted(cause: &(dyn std::error::Error + 'static)) -> bool {
    cause
        .downcast_ref::<std::io::Error>()
        .is_some_and(|io| io.kind() == std::io::ErrorKind::Interrupted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    fn command(argv: &[&str]) -> Command {
        Cli::try_parse_from(argv)
            .unwrap_or_else(|e| panic!("{argv:?}: {e}"))
            .command
    }

    fn argv(parts: &[&str]) -> Option<Vec<OsString>> {
        Some(parts.iter().map(OsString::from).collect())
    }

    #[test]
    fn an_interrupted_prompt_exits_130_and_every_other_failure_exits_1() {
        let stopped = anyhow::Error::new(std::io::Error::new(
            std::io::ErrorKind::Interrupted,
            "read interrupted",
        ))
        .context("confirm system packages");
        assert_eq!(exit_code(&stopped), 130);
        assert_eq!(exit_code(&anyhow::anyhow!("apt-get exited 100")), 1);
    }

    #[test]
    fn only_a_verb_without_an_installer_wizard_is_handed_to_python_before_probing() {
        assert_eq!(forwarded(&command(&["dimos", "setup"])), None);
        assert_eq!(
            forwarded(&command(&["dimos", "hardware", "g1", "setup"])),
            None
        );
        assert_eq!(
            forwarded(&command(&["dimos", "run", "unitree-g1"])),
            argv(&["run", "unitree-g1"])
        );
        assert_eq!(
            forwarded(&command(&["dimos", "hardware", "a1z", "doctor"])),
            argv(&["hardware", "a1z", "doctor"])
        );
    }

    #[test]
    fn a_scan_is_never_written_into_installer_json() {
        assert_eq!(recorded_as(&command(&["dimos", "robot", "scan"])), None);
        assert_eq!(recorded_as(&command(&["dimos", "setup"])), Some("setup"));
    }

    /// The binary and the wheel ship on one tag, so `--version` is pyproject's, not the crate's.
    #[test]
    fn version_is_the_dimos_version() {
        let cmd = Cli::command();
        assert_eq!(cmd.get_name(), "dimos");
        assert_eq!(
            cmd.get_version(),
            Some(dimos_installer::pkgs::DIMOS_VERSION)
        );
    }
}
