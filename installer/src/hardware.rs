//! `dimos hardware g1|jetson setup`: the on-robot bring-up plans, composed from the same stages
//! `setup` and `update` use, ending in the critical verify. Every other `hardware ...` verb belongs
//! to the DimOS Python CLI, so `owned` returns None and main forwards it.

use std::path::Path;

use anyhow::{bail, Result};

use crate::cli::{HardwareSetupArgs, HardwareTarget, HardwareVerb};
use crate::pkgs::{self, Platforms};
use crate::plan::{self, say, Ctx, Plan, Report, Stage};
use crate::probe::{Arch, Os, Probes};
use crate::setup::{deps, g1, jetson, sysconfig, verify};
use crate::state::{self, HardwareRun, Installed};

const UNITREE_EXTRA: &str = "unitree";

/// A hardware target this binary brings up; `key` is also its `installer.json` hardware key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Robot {
    G1,
    Jetson,
}

impl Robot {
    pub fn key(self) -> &'static str {
        match self {
            Robot::G1 => "g1",
            Robot::Jetson => "jetson",
        }
    }

    fn command(self) -> String {
        format!("hardware {} setup", self.key())
    }
}

/// The two `hardware` verbs this binary owns; None means the DimOS Python CLI runs it.
pub fn owned(target: &HardwareTarget) -> Option<(Robot, &HardwareSetupArgs)> {
    match target {
        HardwareTarget::G1 {
            verb: HardwareVerb::Setup(args),
        } => Some((Robot::G1, args)),
        HardwareTarget::Jetson {
            verb: HardwareVerb::Setup(args),
        } => Some((Robot::Jetson, args)),
        _ => None,
    }
}

/// `dimos hardware <target> setup`, as a list of calls.
pub fn run(
    robot: Robot,
    args: &HardwareSetupArgs,
    ctx: &mut Ctx,
    probes: &Probes,
    cfg: &Platforms,
    home: &Path,
) -> Result<i32> {
    let Some(installed) = probes.installed.clone() else {
        say::fail(setup_first(robot));
        return Ok(2);
    };
    preflight(robot, args, probes, cfg, &installed)?;
    let steps = match robot {
        Robot::G1 => g1_setup(args, probes, cfg, &installed, home),
        Robot::Jetson => jetson_plan(args, probes, cfg, &installed),
    };
    let report = plan::run(&steps, ctx)?;
    report.print(ctx);
    if !ctx.dry_run {
        state::save(home, &recorded(robot, args, &installed, &report))?;
    }
    Ok(report.exit_code())
}

/// The refusals that must happen before a plan exists, each naming what the operator does next.
pub fn preflight(
    robot: Robot,
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
) -> Result<()> {
    pkgs::validate_extras(&installed.extras, probes.platform.arch, cfg)?;
    ready(
        robot,
        args,
        probes,
        installed,
        installed.venv_python().is_file(),
    )
}

/// Pure over `venv`, the one read `preflight` does, so every refusal is fixture-testable.
fn ready(
    robot: Robot,
    args: &HardwareSetupArgs,
    probes: &Probes,
    installed: &Installed,
    venv: bool,
) -> Result<()> {
    if !venv {
        bail!(
            "no venv at {}: {}",
            installed.venv_python().display(),
            setup_first(robot)
        );
    }
    match robot {
        Robot::G1 => g1_ready(args, probes, installed),
        Robot::Jetson => jetson_ready(probes),
    }
}

fn g1_ready(args: &HardwareSetupArgs, probes: &Probes, installed: &Installed) -> Result<()> {
    let platform = &probes.platform;
    if !matches!(platform.os, Os::Linux { .. }) || platform.arch != Arch::Aarch64 {
        bail!(
            "`hardware g1 setup` runs on the robot's Jetson (aarch64 Linux), not this {} host",
            platform.arch.name()
        );
    }
    if !installed.extras.iter().any(|e| e == UNITREE_EXTRA) {
        bail!(
            "this install has extras [{}], not `{UNITREE_EXTRA}`: {}",
            installed.extras.join(", "),
            setup_first(Robot::G1)
        );
    }
    interface_ready(&args.interface, probes)
}

/// The verify stage reads DDS on this NIC, so a name this machine lacks is refused up front.
fn interface_ready(interface: &str, probes: &Probes) -> Result<()> {
    if probes.ifaces.iter().any(|(name, _)| name == interface) {
        return Ok(());
    }
    let names: Vec<&str> = probes
        .ifaces
        .iter()
        .map(|(name, _)| name.as_str())
        .collect();
    bail!(
        "no interface {interface} on this machine, got [{}]: pass --interface <one of them>",
        names.join(", ")
    )
}

fn jetson_ready(probes: &Probes) -> Result<()> {
    if probes.platform.is_jetson() {
        return Ok(());
    }
    bail!("not a Jetson: /etc/nv_tegra_release is missing; a plain Linux host needs `dimos setup`")
}

fn setup_first(robot: Robot) -> &'static str {
    match robot {
        Robot::G1 => "run `dimos setup --mode dev --extras unitree` on the robot first",
        Robot::Jetson => "run `dimos setup` first",
    }
}

/// The one I/O step `g1_plan` cannot do: read the robot for what is already built.
fn g1_setup(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
    home: &Path,
) -> Plan {
    let (sdk, obs) = g1::detect(home, installed, args.sdk_path.clone());
    g1_plan(args, probes, cfg, installed, &obs, &sdk)
}

pub fn g1_plan(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
    obs: &g1::G1Observed,
    sdk: &Path,
) -> Plan {
    let mut stages = g1_stages(args, probes, cfg, installed, obs, sdk, false);
    stages.extend(checks(
        g1_target(&probes.platform.home, args),
        installed,
        args,
    ));
    Plan {
        command: Robot::G1.command(),
        stages,
        notes: notes(probes),
    }
}

/// The G1 bring-up (brief decision 13), shared with `update`. The numpy pin follows the SDK
/// install because every `uv pip`/`uv sync` before it re-applies pyproject's numpy>=2 override.
pub fn g1_stages(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
    obs: &g1::G1Observed,
    sdk: &Path,
    venv_changed: bool,
) -> Vec<Stage> {
    let home = &probes.platform.home;
    let uv = deps::uv_bin(&probes.tools, home);
    let python = installed.venv_python();
    let sdk_install = g1::sdk_install_stage(&uv, &python, sdk, &g1::cyclonedds_home(home), obs);
    let reinstalled = venv_changed || !sdk_install.actions.is_empty();
    let mut stages = deps::packages_stages(&installed.extras, probes, cfg);
    stages.extend([
        g1::cyclonedds_stage(home, obs),
        g1::sdk_clone_stage(sdk, obs),
        sdk_install,
        g1::numpy_pin_stage(&uv, &python, obs.numpy_major, reinstalled),
        g1::rc_stage(&probes.rc),
        g1::git_https_stage(obs),
        jetson::stage(&probes.platform, &probes.kernel),
        sysconfig::stage(&probes.platform, &probes.kernel, cfg),
        g1::dotenv_stage(&installed.dir, obs, args),
    ]);
    stages
}

/// What the G1 verify needs: where libddsc lives and which NIC the robot answers on.
pub fn g1_target(home: &Path, args: &HardwareSetupArgs) -> verify::Target {
    verify::Target::G1 {
        cyclonedds_home: g1::cyclonedds_home(home),
        interface: args.interface.clone(),
    }
}

/// The standalone Orin (brief decision 14): performance mode and machine config, then verify.
pub fn jetson_plan(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
) -> Plan {
    let mut stages = vec![
        jetson::stage(&probes.platform, &probes.kernel),
        sysconfig::stage(&probes.platform, &probes.kernel, cfg),
    ];
    stages.extend(checks(verify::Target::Jetson, installed, args));
    Plan {
        command: Robot::Jetson.command(),
        stages,
        notes: notes(probes),
    }
}

fn checks(target: verify::Target, installed: &Installed, args: &HardwareSetupArgs) -> Vec<Stage> {
    verify::stages(
        &target,
        &installed.venv(),
        &installed.dir,
        args.blueprint.as_deref(),
    )
}

/// Warnings the operator still needs when every stage reports `already`.
fn notes(probes: &Probes) -> Vec<String> {
    [
        jetson::static_tls_note(&probes.platform),
        jetson::thermal_note(&probes.kernel),
        sysconfig::no_systemd_note(&probes.platform),
    ]
    .into_iter()
    .flatten()
    .collect()
}

/// The `installer.json` hardware record `dimos update` reads to re-run this target's stages.
fn recorded(
    robot: Robot,
    args: &HardwareSetupArgs,
    installed: &Installed,
    report: &Report,
) -> Installed {
    let g1 = robot == Robot::G1;
    let mut out = installed.clone();
    out.hardware.insert(
        robot.key().to_string(),
        HardwareRun {
            at: state::now_iso(),
            result: outcome(report).to_string(),
            robot_ip: g1.then(|| args.robot_ip.clone()),
            interface: g1.then(|| args.interface.clone()),
        },
    );
    out
}

/// One word, read off the same exit code the shell sees.
fn outcome(report: &Report) -> &'static str {
    match report.exit_code() {
        0 => "ok",
        1 => "failed",
        _ => "needs_human",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::BTreeMap;
    use std::net::Ipv4Addr;
    use std::path::PathBuf;

    use clap::Parser;

    use crate::cli::{Cli, Command, InstallMode};
    use crate::plan::{Action, Mode, Outcome, Stage};
    use crate::probe::{Gpu, Jetson, Kernel, PkgManager, Platform, RcFile, Tools};
    use crate::state::{ActionLog, PlatformSummary, TmpDir, SCHEMA};
    use crate::sudo::Sudo;

    const HOME: &str = "/home/unitree";
    const DIR: &str = "/home/unitree/dimos";
    const PYTHON: &str = "/home/unitree/dimos/.venv/bin/python";
    const SDK: &str = "/home/unitree/unitree_sdk2_python";

    fn home() -> PathBuf {
        PathBuf::from(HOME)
    }

    fn cfg() -> Platforms {
        Platforms::load()
    }

    fn platform(arch: Arch, jetson: bool, glibc: (u32, u32)) -> Platform {
        Platform {
            os: Os::Linux {
                id: "ubuntu".into(),
                version: "20.04".into(),
            },
            arch,
            glibc: Some(glibc),
            jetson: jetson.then(|| Jetson {
                l4t: "R35.3.1".into(),
                jetpack: Some("5.1.1"),
                model: "NVIDIA Orin NX".into(),
            }),
            gpu: Gpu::Tegra,
            pkg: PkgManager::Apt,
            systemd: true,
            home: home(),
            user: "unitree".into(),
            shell: PathBuf::from("/bin/bash"),
        }
    }

    fn orin() -> Platform {
        platform(Arch::Aarch64, true, (2, 31))
    }

    fn installed(extras: &[&str]) -> Installed {
        Installed {
            schema: SCHEMA,
            installer_version: "0.0.14b1".into(),
            dimos_version: "git:aaryan/installer@1fa7cabf".into(),
            mode: InstallMode::Dev,
            dir: PathBuf::from(DIR),
            branch: Some("aaryan/installer".into()),
            extras: extras.iter().map(|e| (*e).to_string()).collect(),
            platform: PlatformSummary {
                os: "linux".into(),
                distro: "ubuntu".into(),
                version: "20.04".into(),
                arch: "aarch64".into(),
                glibc: Some("2.31".into()),
                jetson: Some("R35.3.1".into()),
            },
            hardware: BTreeMap::new(),
            last: None,
        }
    }

    fn rc(text: &str) -> Vec<RcFile> {
        vec![RcFile {
            path: home().join(".profile"),
            text: text.to_string(),
        }]
    }

    fn probes(platform: Platform, kernel: Kernel, dpkg_status: &str, rc: Vec<RcFile>) -> Probes {
        Probes {
            platform,
            kernel,
            tools: Tools {
                uv: Some(PathBuf::from("/home/unitree/.local/bin/uv")),
                dpkg_status: dpkg_status.to_string(),
                ..Default::default()
            },
            installed: Some(installed(&[UNITREE_EXTRA])),
            rc,
            ifaces: vec![("eth0".to_string(), Ipv4Addr::new(192, 168, 123, 164))],
            current_exe: PathBuf::from("/home/unitree/.local/bin/dimos"),
        }
    }

    fn fresh_g1() -> Probes {
        probes(orin(), Kernel::default(), "", rc(""))
    }

    fn args() -> HardwareSetupArgs {
        HardwareSetupArgs {
            robot_ip: "192.168.123.161".into(),
            interface: "eth0".into(),
            transport: None,
            sdk_path: None,
            blueprint: None,
        }
    }

    fn fresh_obs() -> g1::G1Observed {
        g1::G1Observed {
            nproc: 8,
            ..g1::G1Observed::default()
        }
    }

    fn built_obs() -> g1::G1Observed {
        g1::G1Observed {
            cyclonedds_lib: true,
            cyclonedds_clone: true,
            sdk_present: true,
            sdk_imports: true,
            cyclonedds_py_version: Some("0.10.2".into()),
            numpy_major: Some(1),
            git_insteadof_set: true,
            dotenv: "ROBOT_IP=192.168.123.161\nROBOT_INTERFACE=eth0\n".into(),
            nproc: 8,
        }
    }

    fn configured_kernel(cfg: &Platforms) -> Kernel {
        Kernel {
            sysctl: cfg.linux.sysctl.clone(),
            lo_multicast: true,
            multicast_route: true,
            memlock_conf_bytes: Some(cfg.linux.memlock_bytes),
            nvpmodel_maxn: Some(true),
            sysctl_conf: Some(sysconfig::render_sysctl_conf(&cfg.linux.sysctl)),
            enabled_units: vec![
                format!("{}.service", state::MULTICAST_UNIT),
                format!("{}.service", state::JETSON_CLOCKS_UNIT),
            ],
        }
    }

    fn every_package_installed(cfg: &Platforms) -> String {
        pkgs::system_packages(&[UNITREE_EXTRA.to_string()], PkgManager::Apt, cfg)
            .iter()
            .map(|p| format!("{p} install ok installed\n"))
            .collect()
    }

    fn configured_g1(cfg: &Platforms) -> Probes {
        let lines = ["export CYCLONEDDS_HOME=\"$HOME/cyclonedds/install\"".to_string()];
        probes(
            orin(),
            configured_kernel(cfg),
            &every_package_installed(cfg),
            rc(&plan::ensure_block("", g1::CDDS_MARKER, &lines).0),
        )
    }

    fn g1_plan_of(probes: &Probes, obs: &g1::G1Observed, cfg: &Platforms) -> Plan {
        g1_plan(
            &args(),
            probes,
            cfg,
            &installed(&[UNITREE_EXTRA]),
            obs,
            Path::new(SDK),
        )
    }

    fn planned(plan: &Plan) -> Vec<&str> {
        plan.stages
            .iter()
            .filter(|s| !s.actions.is_empty())
            .map(|s| s.name)
            .collect()
    }

    fn scripts(stage: &Stage) -> Vec<String> {
        stage
            .actions
            .iter()
            .map(|a| match a {
                Action::Run { argv, .. } => argv.join(" "),
                other => panic!("verify plans only Run actions, got {other:?}"),
            })
            .collect()
    }

    fn hardware_target(argv: &[&str]) -> HardwareTarget {
        let Command::Hardware { target } = Cli::try_parse_from(argv).expect("parses").command
        else {
            panic!("{argv:?} is not a hardware command")
        };
        target
    }

    #[test]
    fn only_g1_and_jetson_setup_are_ours_every_other_hardware_verb_is_the_python_clis() {
        assert!(matches!(
            owned(&hardware_target(&["dimos", "hardware", "g1", "setup"])),
            Some((Robot::G1, _))
        ));
        assert!(matches!(
            owned(&hardware_target(&["dimos", "hardware", "jetson", "setup"])),
            Some((Robot::Jetson, _))
        ));
        assert!(owned(&hardware_target(&["dimos", "hardware", "g1", "calibrate"])).is_none());
        assert!(owned(&hardware_target(&["dimos", "hardware", "go2", "setup"])).is_none());
    }

    #[test]
    fn g1_preflight_refuses_without_unitree_extra_or_venv_naming_the_setup_command() {
        let probes = fresh_g1();
        let no_extra = ready(Robot::G1, &args(), &probes, &installed(&["base"]), true).unwrap_err();
        assert!(
            no_extra.to_string().contains("--extras unitree"),
            "{no_extra}"
        );
        let no_venv = ready(
            Robot::G1,
            &args(),
            &probes,
            &installed(&[UNITREE_EXTRA]),
            false,
        )
        .unwrap_err();
        assert!(no_venv.to_string().contains(PYTHON), "{no_venv}");
        assert!(no_venv.to_string().contains("dimos setup"), "{no_venv}");
    }

    #[test]
    fn g1_preflight_refuses_a_workstation_because_the_bring_up_runs_on_the_robot() {
        let laptop = probes(
            platform(Arch::X86_64, false, (2, 39)),
            Kernel::default(),
            "",
            rc(""),
        );
        let err = ready(
            Robot::G1,
            &args(),
            &laptop,
            &installed(&[UNITREE_EXTRA]),
            true,
        )
        .unwrap_err();
        assert!(err.to_string().contains("aarch64 Linux"), "{err}");
    }

    #[test]
    fn g1_preflight_refuses_an_interface_this_machine_does_not_have() {
        let wrong = HardwareSetupArgs {
            interface: "enp0s3".into(),
            ..args()
        };
        let err = ready(
            Robot::G1,
            &wrong,
            &fresh_g1(),
            &installed(&[UNITREE_EXTRA]),
            true,
        )
        .unwrap_err();
        assert!(err.to_string().contains("no interface enp0s3"), "{err}");
        assert!(err.to_string().contains("[eth0]"), "{err}");
        assert!(ready(
            Robot::G1,
            &args(),
            &fresh_g1(),
            &installed(&[UNITREE_EXTRA]),
            true
        )
        .is_ok());
    }

    #[test]
    fn g1_preflight_accepts_an_aarch64_linux_robot_with_the_unitree_extra() {
        assert!(preflight(
            Robot::G1,
            &args(),
            &fresh_g1(),
            &cfg(),
            &installed(&[UNITREE_EXTRA])
        )
        .is_err_and(|e| e.to_string().contains("no venv at")));
        assert!(ready(
            Robot::G1,
            &args(),
            &fresh_g1(),
            &installed(&[UNITREE_EXTRA]),
            true
        )
        .is_ok());
    }

    #[test]
    fn jetson_preflight_refuses_non_jetson_and_cuda_extra() {
        let host = probes(
            platform(Arch::X86_64, false, (2, 39)),
            Kernel::default(),
            "",
            rc(""),
        );
        let err = ready(Robot::Jetson, &args(), &host, &installed(&["base"]), true).unwrap_err();
        assert!(err.to_string().contains("/etc/nv_tegra_release"), "{err}");

        let cuda = preflight(
            Robot::Jetson,
            &args(),
            &fresh_g1(),
            &cfg(),
            &installed(&["cuda"]),
        )
        .unwrap_err();
        assert!(cuda.to_string().contains("platform_machine"), "{cuda}");
    }

    #[test]
    fn no_installer_json_is_exit_2_naming_the_setup_command_per_target() {
        let tmp = TmpDir::new("hardware-bare");
        let mut ctx = Ctx {
            mode: Mode::NonInteractive,
            dry_run: true,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(tmp.path()).expect("open the action log"),
            run_id: "test".to_string(),
        };
        let mut bare = fresh_g1();
        bare.installed = None;
        for robot in [Robot::G1, Robot::Jetson] {
            let code = run(robot, &args(), &mut ctx, &bare, &cfg(), tmp.path()).expect("exits");
            assert_eq!(code, 2, "{robot:?}");
        }
        assert!(setup_first(Robot::G1).contains("--extras unitree"));
    }

    #[test]
    fn a_fresh_g1_plans_every_bring_up_stage_in_the_documented_order() {
        let cfg = cfg();
        assert_eq!(
            planned(&g1_plan_of(&fresh_g1(), &fresh_obs(), &cfg)),
            vec![
                "apt update",
                "packages",
                "cyclonedds build",
                "unitree sdk clone",
                "unitree sdk install",
                "numpy pin",
                "cyclonedds env",
                "git https rewrite",
                "jetson perf",
                "sysconfig",
                "robot .env",
                "verify-shell",
                "verify-torch",
                "verify",
            ]
        );
    }

    #[test]
    fn a_configured_g1_plans_nothing_but_the_checks() {
        let cfg = cfg();
        assert_eq!(
            planned(&g1_plan_of(&configured_g1(&cfg), &built_obs(), &cfg)),
            vec!["verify-shell", "verify-torch", "verify"]
        );
    }

    #[test]
    fn the_numpy_pin_follows_the_sdk_install_so_no_uv_pip_can_undo_it() {
        let cfg = cfg();
        let plan = g1_plan_of(&fresh_g1(), &fresh_obs(), &cfg);
        let names = planned(&plan);
        let at = |name: &str| names.iter().position(|n| *n == name).expect(name);
        assert!(at("numpy pin") > at("unitree sdk install"));
        assert!(at("numpy pin") > at("cyclonedds build"));
    }

    #[test]
    fn a_venv_change_replans_the_numpy_pin_on_a_configured_g1() {
        let cfg = cfg();
        let probes = configured_g1(&cfg);
        let inst = installed(&[UNITREE_EXTRA]);
        let quiet = g1_stages(
            &args(),
            &probes,
            &cfg,
            &inst,
            &built_obs(),
            Path::new(SDK),
            false,
        );
        let changed = g1_stages(
            &args(),
            &probes,
            &cfg,
            &inst,
            &built_obs(),
            Path::new(SDK),
            true,
        );
        let pin = |stages: &[Stage]| {
            stages
                .iter()
                .find(|s| s.name == "numpy pin")
                .expect("numpy pin")
                .actions
                .len()
        };
        assert_eq!(pin(&quiet), 0);
        assert_eq!(pin(&changed), 1);
    }

    #[test]
    fn g1_plan_ends_with_the_critical_verify_carrying_cyclonedds_home_and_the_interface() {
        let cfg = cfg();
        let plan = g1_plan_of(&fresh_g1(), &fresh_obs(), &cfg);
        let last = plan.stages.last().expect("verify is last");
        assert_eq!(last.name, "verify");
        assert!(last.critical);
        for action in &last.actions {
            let Action::Run { env, .. } = action else {
                panic!("verify plans only Run actions")
            };
            assert_eq!(
                env,
                &[(
                    "CYCLONEDDS_HOME".to_string(),
                    "/home/unitree/cyclonedds/install".to_string()
                )]
            );
        }
        let scripts = scripts(last);
        assert!(
            scripts.last().expect("a live check").contains("\"eth0\""),
            "{scripts:?}"
        );
    }

    #[test]
    fn verify_runs_the_recorded_venv_python_by_absolute_path_not_by_name() {
        let cfg = cfg();
        let plan = g1_plan_of(&fresh_g1(), &fresh_obs(), &cfg);
        let scripts = scripts(plan.stages.last().expect("verify is last"));
        assert!(scripts.iter().any(|s| s.contains(PYTHON)), "{scripts:?}");
        assert!(
            scripts.iter().all(|s| !s.contains(" python3 ")),
            "{scripts:?}"
        );
    }

    #[test]
    fn an_explicit_blueprint_reaches_the_verify_stage() {
        let cfg = cfg();
        let plan = jetson_plan(
            &HardwareSetupArgs {
                blueprint: Some("unitree-g1-basic".into()),
                ..args()
            },
            &fresh_g1(),
            &cfg,
            &installed(&["base"]),
        );
        let scripts = scripts(plan.stages.last().expect("verify is last"));
        assert!(
            scripts.iter().any(|s| s.contains("unitree-g1-basic")),
            "{scripts:?}"
        );
    }

    #[test]
    fn jetson_plan_is_perf_then_machine_config_then_the_checks() {
        let cfg = cfg();
        let plan = jetson_plan(&args(), &fresh_g1(), &cfg, &installed(&["base"]));
        assert_eq!(plan.command, "hardware jetson setup");
        assert_eq!(
            planned(&plan),
            vec!["jetson perf", "sysconfig", "verify-torch", "verify"]
        );
        assert!(plan.stages.last().expect("verify is last").critical);
    }

    #[test]
    fn jetson_plan_never_touches_the_dds_stack_or_system_packages() {
        let cfg = cfg();
        let plan = jetson_plan(&args(), &fresh_g1(), &cfg, &installed(&["base"]));
        let text: String = plan
            .stages
            .iter()
            .flat_map(|s| s.actions.iter().map(Action::display))
            .collect();
        assert!(!text.contains("cyclonedds"), "{text}");
        assert!(!text.contains("apt-get"), "{text}");
    }

    #[test]
    fn jetson_plan_static_tls_note_on_glibc_2_31_and_absent_on_2_35() {
        let cfg = cfg();
        let at_risk = jetson_plan(&args(), &fresh_g1(), &cfg, &installed(&["base"]));
        assert!(
            at_risk.notes.iter().any(|n| n.contains("LD_PRELOAD")),
            "{:?}",
            at_risk.notes
        );
        let safe = probes(
            platform(Arch::Aarch64, true, (2, 35)),
            Kernel::default(),
            "",
            rc(""),
        );
        let plan = jetson_plan(&args(), &safe, &cfg, &installed(&["base"]));
        assert!(
            !plan.notes.iter().any(|n| n.contains("LD_PRELOAD")),
            "{:?}",
            plan.notes
        );
    }

    #[test]
    fn a_robot_off_maxn_is_warned_that_it_runs_hotter() {
        let cfg = cfg();
        let hot = probes(
            orin(),
            Kernel {
                nvpmodel_maxn: Some(false),
                ..Kernel::default()
            },
            "",
            rc(""),
        );
        let plan = jetson_plan(&args(), &hot, &cfg, &installed(&["base"]));
        assert!(
            plan.notes.iter().any(|n| n.contains("charger")),
            "{:?}",
            plan.notes
        );
    }

    #[test]
    fn the_hardware_record_is_keyed_by_target_and_only_the_g1_keeps_a_robot_address() {
        let ok = Report {
            command: "hardware g1 setup".to_string(),
            stages: vec![("verify".to_string(), Outcome::Applied)],
        };
        let g1 = recorded(Robot::G1, &args(), &installed(&[UNITREE_EXTRA]), &ok);
        let record = &g1.hardware["g1"];
        assert_eq!(record.result, "ok");
        assert_eq!(record.robot_ip.as_deref(), Some("192.168.123.161"));
        assert_eq!(record.interface.as_deref(), Some("eth0"));

        let jetson = recorded(Robot::Jetson, &args(), &installed(&["base"]), &ok);
        assert_eq!(jetson.hardware["jetson"].robot_ip, None);
        assert_eq!(jetson.hardware["jetson"].interface, None);
    }

    #[test]
    fn the_record_carries_the_verdict_the_shell_saw() {
        let verdicts = [
            (Outcome::Already, "ok"),
            (Outcome::Failed("boom".into()), "failed"),
            (Outcome::NeedsHuman("sudo".into()), "needs_human"),
        ];
        for (stage, want) in verdicts {
            let report = Report {
                command: "hardware g1 setup".to_string(),
                stages: vec![("verify".to_string(), stage.clone())],
            };
            assert_eq!(outcome(&report), want, "{stage:?}");
        }
    }

    #[test]
    fn a_second_hardware_run_keeps_the_records_of_every_other_target() {
        let ok = Report {
            command: "hardware jetson setup".to_string(),
            stages: vec![("verify".to_string(), Outcome::Already)],
        };
        let first = recorded(Robot::G1, &args(), &installed(&[UNITREE_EXTRA]), &ok);
        let second = recorded(Robot::Jetson, &args(), &first, &ok);
        assert_eq!(second.hardware.keys().collect::<Vec<_>>(), ["g1", "jetson"]);
    }

    #[test]
    fn no_hardware_plan_asks_sudo_to_carry_an_environment_it_would_drop() {
        let cfg = cfg();
        for plan in [
            g1_plan_of(&fresh_g1(), &fresh_obs(), &cfg),
            jetson_plan(&args(), &fresh_g1(), &cfg, &installed(&["base"])),
        ] {
            assert!(plan::sudo_env_violations(&plan).is_empty(), "{plan:?}");
        }
    }
}
