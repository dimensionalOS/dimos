//! `dimos update` is doctor and update in one (brief decision 19): swap the binary, update DimOS in
//! the recorded directory, re-apply the machine config, then verify. `--dry-run` is the read-only
//! report — every probe runs, nothing is applied. `observe` is the only I/O outside `run::run`.

use std::path::{Path, PathBuf};

use anyhow::Result;

use crate::action::{text, Action};
use crate::cli::{HardwareSetupArgs, InstallMode, UpdateArgs};
use crate::install_record::{self, Installed};
use crate::plan::{Outcome, Plan, Stage};
use crate::platforms::{self, Platforms, DIMOS_VERSION};
use crate::probe::{capture, Probes};
use crate::run::{self, Report};
use crate::run_context::Ctx;
use crate::say;
use crate::self_update::{rollback, self_update_or_note};
use crate::setup::{dimos_venv, system_config, system_packages, verify};
use crate::version::{normalize, parse_latest_tag, parse_ls_remote, RELEASES};
use crate::wizards::nvidia::jetson;
use crate::wizards::unitree::g1::{self, G1Observed};
use crate::wizards::Robot;

/// The same override `scripts/install.sh` honors, so a LAN test serves both from one directory.
const INSTALLER_URL_ENV: &str = "DIMOS_INSTALLER_URL";
const NOT_VERIFIED: &str = "verify not run in dry-run: a real `dimos update` runs the checks";

const GIT_TIMEOUT_S: u64 = 600;
const PIP_TIMEOUT_S: u64 = 1800;
/// curl's own `--max-time 15`, plus process start-up.
const LATEST_PROBE_TIMEOUT_S: u64 = 20;
const GIT_PROBE_TIMEOUT_S: u64 = 10;
/// A half-open network (the G1's wifi lease moved) hangs `ls-remote` until TCP gives up.
const LS_REMOTE_TIMEOUT_S: u64 = 30;

/// What the probes saw; `--dry-run` fills it exactly as a real run does.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct Observed {
    /// The release to install, or None when GitHub was unreachable or a URL override is set.
    pub latest: Option<String>,
    pub head: Option<String>,
    pub remote: Option<String>,
    /// The robot's state, and the SDK path it was found at, when installer.json records a G1.
    pub g1: Option<(PathBuf, G1Observed)>,
}

/// The only I/O in this file, and all of it read-only, so `--dry-run` runs every probe.
fn observe(
    installed: &Installed,
    args: &UpdateArgs,
    override_url: Option<&str>,
    home: &Path,
) -> Observed {
    let dev = installed.mode == InstallMode::Dev;
    let dir = text(&installed.dir);
    Observed {
        latest: candidate(args, override_url),
        head: dev
            .then(|| git(&dir, &["rev-parse", "HEAD"], GIT_PROBE_TIMEOUT_S))
            .flatten(),
        remote: dev
            .then(|| remote_head(&dir, installed.branch.as_deref()))
            .flatten(),
        g1: g1_record(installed).map(|_| {
            g1::detect(
                home,
                installed,
                std::env::var_os("SDK2_PATH").map(PathBuf::from),
            )
        }),
    }
}

/// `--version` wins; a URL override is a plain directory with no `/latest` to follow.
fn candidate(args: &UpdateArgs, override_url: Option<&str>) -> Option<String> {
    match (&args.version, override_url) {
        (Some(tag), _) => Some(normalize(tag)),
        (None, Some(_)) => None,
        (None, None) => latest_release(),
    }
}

/// GitHub redirects `/releases/latest` to `/releases/tag/v<version>`; the effective URL is the answer.
fn latest_release() -> Option<String> {
    let url = format!("{RELEASES}/latest");
    let args = [
        "-fsSLI",
        "-o",
        "/dev/null",
        "-w",
        "%{url_effective}",
        "--max-time",
        "15",
        &url,
    ];
    parse_latest_tag(&capture("curl", &args, &[], LATEST_PROBE_TIMEOUT_S)?)
}

fn remote_head(dir: &str, branch: Option<&str>) -> Option<String> {
    let refname = format!("refs/heads/{}", branch?);
    parse_ls_remote(&git(
        dir,
        &["ls-remote", "origin", &refname],
        LS_REMOTE_TIMEOUT_S,
    )?)
}

/// GIT_TERMINAL_PROMPT=0 so a private remote fails instead of blocking on a credential prompt.
fn git(dir: &str, args: &[&str], timeout_s: u64) -> Option<String> {
    let mut argv = vec!["-C", dir];
    argv.extend_from_slice(args);
    capture("git", &argv, &[("GIT_TERMINAL_PROMPT", "0")], timeout_s)
}

/// The G1 record with the address and NIC its stages are rebuilt from; None for an older record.
fn g1_record(installed: &Installed) -> Option<HardwareSetupArgs> {
    let rec = installed.hardware.get(Robot::G1.key())?;
    Some(HardwareSetupArgs {
        robot_ip: rec.robot_ip.clone()?,
        interface: rec.interface.clone()?,
        transport: None,
        sdk_path: None,
        blueprint: None,
    })
}

/// Library re-pins from PyPI, dev fast-forwards and re-syncs; empty when the probe says it is there.
fn dimos_stage(installed: &Installed, uv: &Path, obs: &Observed, force: bool) -> Stage {
    // non-critical: a checkout with local commits must not stop the machine-config repair.
    let stage = Stage::new("dimos", false);
    if !force && at_wanted_version(installed, obs) {
        return stage;
    }
    match installed.mode {
        InstallMode::Library => stage.push(pip_action(installed, uv)),
        InstallMode::Dev => stage
            .push(pull_action(installed))
            .push(dimos_venv::sync_action(
                &installed.dir,
                &installed.extras,
                false,
                uv,
                None,
            )),
    }
}

/// Library pins to this binary's own version; dev compares the checkout to its remote branch.
fn at_wanted_version(installed: &Installed, obs: &Observed) -> bool {
    match installed.mode {
        InstallMode::Library => installed.dimos_version == DIMOS_VERSION,
        InstallMode::Dev => obs.head.is_some() && obs.head == obs.remote,
    }
}

fn pip_action(installed: &Installed, uv: &Path) -> Action {
    let argv = vec![
        text(uv),
        "pip".to_string(),
        "install".to_string(),
        "--python".to_string(),
        text(&installed.venv_python()),
        platforms::pip_spec(&installed.extras),
    ];
    Action::run_owned(argv, false, Some(&installed.dir), &[], PIP_TIMEOUT_S)
}

/// `--ff-only` so a checkout carrying local commits stops instead of being merged under its owner.
fn pull_action(installed: &Installed) -> Action {
    let argv = vec![
        "git".to_string(),
        "-C".to_string(),
        text(&installed.dir),
        "pull".to_string(),
        "--ff-only".to_string(),
    ];
    Action::run_owned(
        argv,
        false,
        Some(&installed.dir),
        &[("GIT_LFS_SKIP_SMUDGE", "1")],
        GIT_TIMEOUT_S,
    )
}

/// A recorded G1 gets its whole bring-up re-asserted (D2); any other machine, the host config.
fn machine_stages(
    installed: &Installed,
    probes: &Probes,
    cfg: &Platforms,
    obs: &Observed,
    dimos_runs: bool,
) -> Vec<Stage> {
    if let (Some(args), Some((sdk, g1_obs))) = (g1_record(installed), &obs.g1) {
        return g1::stages(&args, probes, cfg, installed, g1_obs, sdk, dimos_runs);
    }
    let mut stages = system_packages::packages_stages(&installed.extras, probes, cfg);
    stages.extend([
        system_config::stage(&probes.platform, &probes.kernel, cfg),
        jetson::stage(&probes.platform, &probes.kernel),
    ]);
    stages
}

/// A recorded G1 is verified as a G1, so a broken DDS stack fails the run instead of passing quietly.
fn target(installed: &Installed, probes: &Probes, home: &Path) -> verify::Target {
    match g1_record(installed) {
        Some(args) => g1::target(home, &args),
        None if probes.platform.is_jetson() => verify::Target::Jetson,
        None => verify::Target::Host,
    }
}

/// A G1 recorded before the address fields existed cannot be rebuilt from installer.json.
fn g1_note(installed: &Installed) -> Option<String> {
    (installed.hardware.contains_key(Robot::G1.key()) && g1_record(installed).is_none()).then(
        || {
            "the g1 record has no address: `dimos hardware g1 setup --robot-ip <ip>` repairs it"
                .to_string()
        },
    )
}

fn notes(installed: &Installed, probes: &Probes, skipped: Option<String>) -> Vec<String> {
    [
        skipped,
        g1_note(installed),
        jetson::static_tls_note(&probes.platform),
        jetson::thermal_note(&probes.kernel),
        system_config::no_systemd_note(&probes.platform),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn plan(
    installed: &Installed,
    args: &UpdateArgs,
    probes: &Probes,
    cfg: &Platforms,
    obs: &Observed,
    home: &Path,
) -> Plan {
    let uv = system_packages::uv_bin(&probes.tools, home);
    let (self_update, skipped) = self_update_or_note(args, obs, probes, home);
    let dimos = dimos_stage(installed, &uv, obs, args.force);
    let dimos_runs = !dimos.actions.is_empty();
    let mut stages = vec![self_update, dimos];
    stages.extend(machine_stages(installed, probes, cfg, obs, dimos_runs));
    stages.extend(verify::stages(
        &target(installed, probes, home),
        &installed.venv(),
        &installed.dir,
        None,
    ));
    Plan {
        command: "update".to_string(),
        stages,
        notes: notes(installed, probes, skipped),
    }
}

/// Exit 0 only when every stage but the checks is already there; the checks did not run.
fn dry_run_exit(report: &Report) -> i32 {
    let pending = report
        .stages
        .iter()
        .any(|(name, o)| !verify::is_check(name) && *o != Outcome::Already);
    i32::from(pending)
}

/// installer.json says what the venv holds now, so the next run's probe reads the truth.
fn record(installed: &Installed, obs: &Observed, report: &Report, home: &Path) -> Result<()> {
    let landed = report
        .stages
        .iter()
        .any(|(name, o)| name == "dimos" && *o == Outcome::Applied);
    if !landed {
        return Ok(());
    }
    let mut out = installed.clone();
    out.dimos_version = dimos_venv::dimos_version_string(
        out.mode,
        out.branch.as_deref().unwrap_or_default(),
        obs.remote.as_deref(),
    );
    install_record::save(home, &out)
}

/// `dimos update`, as a list of calls.
pub fn run(
    args: &UpdateArgs,
    ctx: &mut Ctx,
    probes: &Probes,
    cfg: &Platforms,
    home: &Path,
) -> Result<i32> {
    if args.rollback {
        return rollback(ctx, home);
    }
    let Some(installed) = install_record::load(home)? else {
        say::fail("no DimOS install recorded: run `dimos setup` first");
        return Ok(2);
    };
    let obs = observe(&installed, args, override_url().as_deref(), home);
    let steps = plan(&installed, args, probes, cfg, &obs, home);
    let report = run::run(&steps, ctx)?;
    report.print(ctx);
    if ctx.dry_run {
        say::warn(NOT_VERIFIED);
        return Ok(dry_run_exit(&report));
    }
    record(&installed, &obs, &report, home)?;
    rollback_hint(&report, home);
    Ok(report.exit_code())
}

fn rollback_hint(report: &Report, home: &Path) {
    if report.exit_code() != 0 && install_record::backup_bin(home).exists() {
        say::info("rollback: dimos update --rollback");
    }
}

pub(crate) fn override_url() -> Option<String> {
    std::env::var(INSTALLER_URL_ENV)
        .ok()
        .filter(|s| !s.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action_log::ActionLog;
    use crate::cli::InstallMode;
    use crate::install_record::{HardwareRun, PlatformSummary, TmpDir};
    use crate::probe::{Arch, Gpu, Kernel, Os, PkgManager, Platform, RcFile, Tools};
    use crate::sudo::Sudo;
    use std::collections::BTreeMap;
    use std::net::Ipv4Addr;

    fn installed(mode: InstallMode, home: &Path) -> Installed {
        Installed {
            schema: install_record::SCHEMA,
            installer_version: DIMOS_VERSION.to_string(),
            dimos_version: DIMOS_VERSION.to_string(),
            mode,
            dir: home.join("dimos-app"),
            branch: Some("main".to_string()),
            extras: vec!["base".to_string()],
            platform: PlatformSummary {
                os: "linux".to_string(),
                distro: "ubuntu".to_string(),
                version: "22.04".to_string(),
                arch: "x86_64".to_string(),
                glibc: Some("2.35".to_string()),
                jetson: None,
            },
            hardware: BTreeMap::new(),
            last: None,
        }
    }

    fn with_g1(mut installed: Installed) -> Installed {
        installed.extras = vec!["unitree".to_string()];
        installed.hardware.insert(
            Robot::G1.key().to_string(),
            HardwareRun {
                at: "2026-09-01T00:00:00Z".to_string(),
                result: "applied".to_string(),
                robot_ip: Some("192.168.123.161".to_string()),
                interface: Some("eth0".to_string()),
            },
        );
        installed
    }

    fn g1_observed(home: &Path) -> Observed {
        let obs = G1Observed {
            cyclonedds_lib: true,
            cyclonedds_clone: true,
            sdk_present: true,
            sdk_imports: true,
            cyclonedds_py_version: Some("0.10.2".into()),
            numpy_major: Some(1),
            git_insteadof_set: true,
            dotenv: "ROBOT_IP=192.168.123.161\nROBOT_INTERFACE=eth0\n".into(),
            nproc: 8,
        };
        Observed {
            g1: Some((home.join("unitree_sdk2_python"), obs)),
            ..Observed::default()
        }
    }

    fn probes(home: &Path, current_exe: PathBuf) -> Probes {
        Probes {
            platform: Platform {
                os: Os::Linux {
                    id: "ubuntu".to_string(),
                    version: "22.04".to_string(),
                },
                arch: Arch::X86_64,
                glibc: Some((2, 35)),
                jetson: None,
                gpu: Gpu::None,
                pkg: PkgManager::Apt,
                systemd: false,
                home: home.to_path_buf(),
                user: "tester".to_string(),
                shell: PathBuf::from("/bin/bash"),
            },
            kernel: Kernel::default(),
            tools: Tools::default(),
            installed: None,
            rc: vec![RcFile {
                path: home.join(".profile"),
                text: String::new(),
            }],
            ifaces: vec![("eth0".to_string(), Ipv4Addr::new(192, 168, 123, 164))],
            current_exe,
        }
    }

    /// Every apt package present, so the machine-config stages of a host plan are empty.
    fn configured(home: &Path) -> Probes {
        let mut probes = probes(home, install_record::installed_bin(home));
        probes.tools.dpkg_status =
            platforms::system_packages(&["base".to_string()], PkgManager::Apt, &Platforms::load())
                .iter()
                .map(|p| format!("{p} install ok installed\n"))
                .collect();
        probes
    }

    fn args() -> UpdateArgs {
        UpdateArgs {
            version: None,
            force: false,
            rollback: false,
        }
    }

    fn ctx(home: &Path, dry_run: bool) -> Ctx {
        Ctx {
            mode: crate::run_context::Mode::NonInteractive,
            dry_run,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(home).expect("open the action log"),
            run_id: "test".to_string(),
        }
    }

    fn argv_of(stage: &Stage) -> Vec<Vec<String>> {
        stage
            .actions
            .iter()
            .filter_map(|a| match a {
                Action::Run { argv, .. } => Some(argv.clone()),
                _ => None,
            })
            .collect()
    }

    fn stage_named<'a>(plan: &'a Plan, name: &str) -> &'a Stage {
        plan.stages
            .iter()
            .find(|s| s.name == name)
            .expect("stage in plan")
    }

    fn names(plan: &Plan) -> Vec<&str> {
        plan.stages.iter().map(|s| s.name).collect()
    }

    #[test]
    fn self_update_is_a_note_not_a_stage_when_this_is_not_the_installed_binary() {
        let home = TmpDir::new("update-exe");
        let probes = probes(home.path(), PathBuf::from("/repo/target/release/dimos"));
        let obs = Observed {
            latest: Some("9.9.9".to_string()),
            ..Observed::default()
        };
        let (stage, note) = self_update_or_note(&args(), &obs, &probes, home.path());
        assert!(stage.actions.is_empty());
        assert!(note.unwrap_or_default().contains("installed copy"));
    }

    #[test]
    fn self_update_is_empty_when_the_latest_release_is_not_newer() {
        let home = TmpDir::new("update-same");
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        let obs = Observed {
            latest: Some(DIMOS_VERSION.to_string()),
            ..Observed::default()
        };
        let (stage, note) = self_update_or_note(&args(), &obs, &probes, home.path());
        assert!(stage.actions.is_empty() && note.is_none());
    }

    #[test]
    fn library_dimos_stage_is_empty_when_the_recorded_version_is_this_binarys() {
        let home = TmpDir::new("update-lib");
        let inst = installed(InstallMode::Library, home.path());
        let stage = dimos_stage(&inst, Path::new("/uv"), &Observed::default(), false);
        assert!(stage.actions.is_empty());
    }

    #[test]
    fn library_dimos_stage_pins_the_recorded_extras_to_this_binarys_version() {
        let home = TmpDir::new("update-lib-old");
        let mut inst = installed(InstallMode::Library, home.path());
        inst.dimos_version = "0.0.1".to_string();
        let stage = dimos_stage(&inst, Path::new("/uv"), &Observed::default(), false);
        let argv = argv_of(&stage);
        assert_eq!(argv.len(), 1);
        assert_eq!(argv[0].last().unwrap(), &platforms::pip_spec(&inst.extras));
    }

    #[test]
    fn dev_dimos_stage_is_empty_when_head_matches_the_remote() {
        let home = TmpDir::new("update-dev-same");
        let inst = installed(InstallMode::Dev, home.path());
        let obs = Observed {
            head: Some("abc".to_string()),
            remote: Some("abc".to_string()),
            ..Observed::default()
        };
        assert!(dimos_stage(&inst, Path::new("/uv"), &obs, false)
            .actions
            .is_empty());
    }

    #[test]
    fn dev_dimos_stage_pulls_ff_only_then_syncs_when_the_remote_moved() {
        let home = TmpDir::new("update-dev-moved");
        let inst = installed(InstallMode::Dev, home.path());
        let obs = Observed {
            head: Some("abc".to_string()),
            remote: Some("def".to_string()),
            ..Observed::default()
        };
        let argv = argv_of(&dimos_stage(&inst, Path::new("/uv"), &obs, false));
        assert_eq!(argv.len(), 2);
        assert!(argv[0].contains(&"--ff-only".to_string()));
        assert_eq!(argv[1][1], "sync");
    }

    #[test]
    fn force_replans_the_dimos_stage_the_probe_would_skip() {
        let home = TmpDir::new("update-force");
        let inst = installed(InstallMode::Library, home.path());
        let stage = dimos_stage(&inst, Path::new("/uv"), &Observed::default(), true);
        assert_eq!(argv_of(&stage).len(), 1);
    }

    #[test]
    fn plan_ends_with_a_critical_verify_stage() {
        let home = TmpDir::new("update-plan");
        let inst = installed(InstallMode::Library, home.path());
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &probes,
            &cfg,
            &Observed::default(),
            home.path(),
        );
        let last = steps.stages.last().expect("stages");
        assert_eq!(last.name, "verify");
        assert!(last.critical);
    }

    #[test]
    fn a_recorded_g1_rebuilds_every_g1_stage_from_its_record() {
        let home = TmpDir::new("update-g1-stages");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &probes,
            &cfg,
            &g1_observed(home.path()),
            home.path(),
        );
        let got = names(&steps);
        for name in [
            "cyclonedds build",
            "unitree sdk install",
            "numpy pin",
            "cyclonedds env",
            "git https rewrite",
            "robot .env",
            "verify-shell",
        ] {
            assert!(got.contains(&name), "{name} missing from {got:?}");
        }
        assert!(steps.notes.iter().all(|n| !n.contains("g1 record")));
    }

    #[test]
    fn plan_orders_the_numpy_pin_after_the_dimos_stage_that_breaks_it() {
        let home = TmpDir::new("update-order");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        let cfg = Platforms::load();
        let obs = Observed {
            head: Some("abc".to_string()),
            remote: Some("def".to_string()),
            ..g1_observed(home.path())
        };
        let steps = plan(&inst, &args(), &probes, &cfg, &obs, home.path());
        let names = names(&steps);
        let dimos_at = names.iter().position(|n| *n == "dimos").expect("dimos");
        let numpy_at = names.iter().position(|n| *n == "numpy pin").expect("numpy");
        assert!(dimos_at < numpy_at);
        assert_eq!(stage_named(&steps, "numpy pin").actions.len(), 1);
        assert_eq!(stage_named(&steps, "self-update").actions.len(), 0);
    }

    #[test]
    fn a_g1_record_without_an_address_gets_the_repair_note_not_the_stages() {
        let home = TmpDir::new("update-g1-old");
        let mut inst = with_g1(installed(InstallMode::Dev, home.path()));
        inst.hardware.get_mut("g1").expect("g1").robot_ip = None;
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &probes,
            &cfg,
            &Observed::default(),
            home.path(),
        );
        assert!(!names(&steps).contains(&"numpy pin"));
        assert!(steps.notes.iter().any(|n| n.contains("--robot-ip")));
    }

    #[test]
    fn a_recorded_g1_is_verified_as_a_g1_on_its_recorded_interface() {
        let home = TmpDir::new("update-g1-target");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        assert_eq!(
            target(&inst, &probes, home.path()),
            verify::Target::G1 {
                cyclonedds_home: g1::cyclonedds_home(home.path()),
                interface: "eth0".to_string(),
            }
        );
    }

    #[test]
    fn dry_run_exit_is_0_when_every_stage_but_the_checks_is_already() {
        let report = |stages: Vec<(&str, Outcome)>| Report {
            command: "update".to_string(),
            stages: stages
                .into_iter()
                .map(|(n, o)| (n.to_string(), o))
                .collect(),
        };
        let clean = report(vec![
            ("dimos", Outcome::Already),
            ("verify", Outcome::DryRun),
        ]);
        let pending = report(vec![
            ("dimos", Outcome::Already),
            ("sysconfig", Outcome::DryRun),
            ("verify", Outcome::DryRun),
        ]);
        assert_eq!(dry_run_exit(&clean), 0);
        assert_eq!(dry_run_exit(&pending), 1);
    }

    #[test]
    fn a_configured_install_dry_runs_the_real_plan_to_exit_0() {
        let home = TmpDir::new("update-dry-clean");
        let inst = installed(InstallMode::Library, home.path());
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &configured(home.path()),
            &cfg,
            &Observed::default(),
            home.path(),
        );
        let report = run::run(&steps, &mut ctx(home.path(), true)).expect("dry run");
        assert_eq!(dry_run_exit(&report), 0, "{:?}", report.stages);
    }

    #[test]
    fn a_dimos_stage_that_landed_writes_the_new_version_into_installer_json() {
        let home = TmpDir::new("update-record");
        let mut inst = installed(InstallMode::Dev, home.path());
        inst.dimos_version = "git:main@old".to_string();
        install_record::save(home.path(), &inst).expect("save the prior record");
        let obs = Observed {
            remote: Some("9f8b2c1d".to_string()),
            ..Observed::default()
        };
        let report = |o: Outcome| Report {
            command: "update".to_string(),
            stages: vec![("dimos".to_string(), o)],
        };
        record(&inst, &obs, &report(Outcome::Already), home.path()).expect("no write");
        let untouched = install_record::load(home.path())
            .expect("load")
            .expect("recorded");
        assert_eq!(untouched.dimos_version, "git:main@old");
        record(&inst, &obs, &report(Outcome::Applied), home.path()).expect("write");
        let after = install_record::load(home.path())
            .expect("load")
            .expect("recorded");
        assert_eq!(after.dimos_version, "git:main@9f8b2c1d");
    }

    #[test]
    fn a_library_update_records_this_binarys_version() {
        let home = TmpDir::new("update-record-lib");
        let mut inst = installed(InstallMode::Library, home.path());
        inst.dimos_version = "0.0.1".to_string();
        let report = Report {
            command: "update".to_string(),
            stages: vec![("dimos".to_string(), Outcome::Applied)],
        };
        record(&inst, &Observed::default(), &report, home.path()).expect("write");
        let after = install_record::load(home.path())
            .expect("load")
            .expect("recorded");
        assert_eq!(after.dimos_version, DIMOS_VERSION);
    }

    #[test]
    fn no_sudo_action_in_the_update_plan_carries_an_environment() {
        let home = TmpDir::new("update-sudo-env");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), install_record::installed_bin(home.path()));
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &probes,
            &cfg,
            &g1_observed(home.path()),
            home.path(),
        );
        assert!(run::sudo_env_violations(&steps).is_empty());
    }
}
