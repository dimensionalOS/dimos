//! `dimos setup` as a list of calls: resolve what to install, refuse a machine that cannot hold it,
//! build the ordered stage list, run it, and record what landed.

pub mod dimos_venv;
pub mod self_install;
pub mod system_config;
pub mod system_packages;
pub mod verify;

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};

use crate::cli::{InstallMode, SetupArgs};
use crate::file_actions;
use crate::install_record::{self, Installed};
use crate::plan::{text, Plan, Stage};
use crate::platforms::{self, Platforms, DIMOS_VERSION, EXTRAS};
use crate::probe::{self, Arch, Os, PkgManager, Platform, Probes};
use crate::run;
use crate::run_context::Ctx;
use crate::say;
use crate::wizards::nvidia::jetson;

const GIB: u64 = 1024 * 1024 * 1024;
/// A dev install with the base extras unpacks ~10 GiB of wheels, torch included.
pub const MIN_DISK_BYTES: u64 = 12 * GIB;
const DF_TIMEOUT_S: u64 = 20;

const DEFAULT_DIR: &str = "dimos-app";
const DEFAULT_BRANCH: &str = "main";
/// The extra whose SDK and DDS build only `dimos hardware g1 setup` can finish.
const UNITREE_EXTRA: &str = "unitree";

/// What this run installs, resolved once from the flags, the prior install and the prompts.
#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    pub mode: InstallMode,
    pub extras: Vec<String>,
    pub branch: String,
    pub dir: PathBuf,
    pub with_nix: bool,
    pub blueprint: Option<String>,
}

/// Flags win, then installer.json, then a prompt; a non-interactive prompt takes its default.
pub fn resolve_target(
    args: &SetupArgs,
    prior: Option<&Installed>,
    cwd: &Path,
    arch: Arch,
    cfg: &Platforms,
    ctx: &Ctx,
) -> Result<Target> {
    Ok(Target {
        mode: mode_of(args, prior, ctx)?,
        extras: extras_of(args, prior, ctx, arch, cfg)?,
        branch: branch_of(args, prior),
        dir: dir_of(args, prior, cwd, ctx)?,
        with_nix: args.with_nix,
        blueprint: args.blueprint.clone(),
    })
}

fn mode_of(args: &SetupArgs, prior: Option<&Installed>, ctx: &Ctx) -> Result<InstallMode> {
    if let Some(mode) = args.mode.or_else(|| prior.map(|p| p.mode)) {
        return Ok(mode);
    }
    let options = ["library: dimos from PyPI", "dev: a git clone you can edit"];
    Ok(match ctx.choose("install mode", &options, 0)? {
        0 => InstallMode::Library,
        _ => InstallMode::Dev,
    })
}

fn extras_of(
    args: &SetupArgs,
    prior: Option<&Installed>,
    ctx: &Ctx,
    arch: Arch,
    cfg: &Platforms,
) -> Result<Vec<String>> {
    let asked = (!args.extras.is_empty())
        .then(|| args.extras.clone())
        .or_else(|| prior.map(|p| p.extras.clone()));
    let wanted = match asked {
        Some(extras) => extras,
        None => ctx
            .input(&format!("extras ({})", EXTRAS.join(", ")), "base")?
            .split(',')
            .map(str::to_string)
            .collect(),
    };
    platforms::validate_extras(&wanted, arch, cfg)
}

fn branch_of(args: &SetupArgs, prior: Option<&Installed>) -> String {
    args.branch
        .clone()
        .or_else(|| prior.and_then(|p| p.branch.clone()))
        .unwrap_or_else(|| DEFAULT_BRANCH.to_string())
}

/// `cwd.join` keeps an absolute answer and anchors a relative one, so installer.json is absolute.
fn dir_of(args: &SetupArgs, prior: Option<&Installed>, cwd: &Path, ctx: &Ctx) -> Result<PathBuf> {
    if let Some(dir) = args.dir.clone().or_else(|| prior.map(|p| p.dir.clone())) {
        return Ok(cwd.join(dir));
    }
    let default = cwd.join(DEFAULT_DIR);
    let answer = ctx.input("install directory", &default.to_string_lossy())?;
    Ok(cwd.join(answer))
}

/// The refusals that must happen before a plan exists: no build for this platform, or no room.
fn preflight(target: &Target, probes: &Probes) -> Result<()> {
    probes.platform.target()?;
    disk_gate(
        free_bytes(&target.dir),
        probes.installed.as_ref(),
        &target.dir,
    )
}

/// Unknown free space never blocks, and a recorded install means the wheels are already here.
fn disk_gate(free: Option<u64>, prior: Option<&Installed>, dir: &Path) -> Result<()> {
    let Some(free) = free.filter(|_| prior.is_none()) else {
        return Ok(());
    };
    if free < MIN_DISK_BYTES {
        bail!(
            "{} has {} GiB free, DimOS needs {} GiB: free space, or point --dir at a bigger disk",
            dir.display(),
            free / GIB,
            MIN_DISK_BYTES / GIB
        );
    }
    Ok(())
}

/// The one probe in this file, and read-only: `df` on the nearest parent of `dir` that exists.
fn free_bytes(dir: &Path) -> Option<u64> {
    let mount = dir.ancestors().find(|p| p.exists())?;
    let out = probe::capture("df", &["-Pk", &text(mount)], &[], DF_TIMEOUT_S)?;
    parse_df_kib(&out).map(|kib| kib * 1024)
}

/// `df -P` guarantees one line per filesystem, with Available as the fourth field, in KiB.
fn parse_df_kib(text: &str) -> Option<u64> {
    text.lines().nth(1)?.split_whitespace().nth(3)?.parse().ok()
}

/// The whole run as one plan; a stage whose probe says the machine is ready comes back empty.
pub fn plan(target: &Target, probes: &Probes, cfg: &Platforms, home: &Path) -> Result<Plan> {
    let dir_state = dimos_venv::dir_state(&target.dir);
    Ok(Plan {
        command: "setup".to_string(),
        stages: stages(target, probes, cfg, home, &dir_state)?,
        notes: notes(target, probes, &dir_state),
    })
}

/// self-install leads so `dimos` resolves in the next shell even when a later stage fails.
fn stages(
    target: &Target,
    probes: &Probes,
    cfg: &Platforms,
    home: &Path,
    dir_state: &dimos_venv::DirState,
) -> Result<Vec<Stage>> {
    let uv = system_packages::uv_bin(&probes.tools, home);
    let mut out = vec![self_install_stage(probes, home)];
    out.extend(system_packages::packages_stages(
        &target.extras,
        probes,
        cfg,
    ));
    out.extend([
        system_packages::uv_stage(&probes.tools, home),
        system_packages::nix_stage(&probes.tools, home, target.with_nix),
        install_stage(target, &uv, dir_state, probes.installed.as_ref())?,
        system_config::stage(&probes.platform, &probes.kernel, cfg),
        jetson::stage(&probes.platform, &probes.kernel),
    ]);
    out.extend(verify::stages(
        &checks(probes),
        &install_record::venv(&target.dir),
        &target.dir,
        target.blueprint.as_deref(),
    ));
    Ok(out)
}

/// An unreadable binary hashes to nothing, so the copy is planned rather than silently skipped.
fn self_install_stage(probes: &Probes, home: &Path) -> Stage {
    let own = file_actions::sha256_hex(&probes.current_exe).unwrap_or_default();
    let installed = file_actions::sha256_hex(&install_record::installed_bin(home)).ok();
    self_install::stage(
        &probes.current_exe,
        home,
        &probes.tools,
        &probes.rc,
        installed.as_deref(),
        &own,
    )
}

fn install_stage(
    target: &Target,
    uv: &Path,
    dir_state: &dimos_venv::DirState,
    prior: Option<&Installed>,
) -> Result<Stage> {
    if install_is_current(target, dir_state, prior) {
        return Ok(Stage::new("dimos", true));
    }
    dimos_venv::dimos_stage(
        target.mode,
        &target.dir,
        &target.extras,
        &target.branch,
        target.with_nix,
        uv,
        dir_state,
    )
}

fn install_is_current(
    target: &Target,
    dir_state: &dimos_venv::DirState,
    prior: Option<&Installed>,
) -> bool {
    let Some(prior) = prior else {
        return false;
    };
    let branch_matches = match target.mode {
        InstallMode::Library => prior.branch.is_none(),
        InstallMode::Dev => {
            prior.branch.as_deref() == Some(&target.branch)
                && matches!(
                    dir_state,
                    dimos_venv::DirState::Clone { branch: Some(branch) } if branch == &target.branch
                )
        }
    };
    prior.mode == target.mode
        && prior.dir == target.dir
        && prior.extras == target.extras
        && branch_matches
        && install_record::venv_python(&target.dir).is_file()
        && install_record::venv(&target.dir)
            .join("bin/dimos")
            .is_file()
}

/// `setup` verifies the host it runs on; the G1's DDS stack is `dimos hardware g1 setup`.
fn checks(probes: &Probes) -> verify::Target {
    if probes.platform.is_jetson() {
        verify::Target::Jetson
    } else {
        verify::Target::Host
    }
}

fn notes(target: &Target, probes: &Probes, dir_state: &dimos_venv::DirState) -> Vec<String> {
    [
        (target.mode == InstallMode::Dev)
            .then(|| dimos_venv::branch_note(dir_state, &target.branch, &target.dir))
            .flatten(),
        unmanaged_note(&probes.platform),
        jetson::static_tls_note(&probes.platform),
        jetson::thermal_note(&probes.kernel),
        system_config::no_systemd_note(&probes.platform),
    ]
    .into_iter()
    .flatten()
    .collect()
}

/// Neither apt nor brew: the packages stage can install nothing, so the note names the one fix.
fn unmanaged_note(platform: &Platform) -> Option<String> {
    let fix = match platform.os {
        Os::MacOs { .. } => "no Homebrew: install it from https://brew.sh and re-run `dimos setup`",
        Os::Linux { .. } => "no apt: install installer/platforms.toml's packages by hand",
    };
    (platform.pkg == PkgManager::None).then(|| fix.to_string())
}

/// `hardware` and `last` are other commands' writes, so a re-run carries them over untouched.
fn record(target: &Target, probes: &Probes, prior: Option<&Installed>) -> Installed {
    Installed {
        schema: install_record::SCHEMA,
        installer_version: DIMOS_VERSION.to_string(),
        dimos_version: dimos_venv::dimos_version_string(target.mode, &target.branch, None),
        mode: target.mode,
        dir: target.dir.clone(),
        branch: (target.mode == InstallMode::Dev).then(|| target.branch.clone()),
        extras: target.extras.clone(),
        platform: probes.platform.summary(),
        hardware: prior.map(|p| p.hardware.clone()).unwrap_or_default(),
        last: prior.and_then(|p| p.last.clone()),
    }
}

/// The one command a G1 still needs: `setup` installs no CycloneDDS and no Unitree SDK.
fn next_steps(target: &Target, probes: &Probes) {
    if probes.platform.is_jetson() && target.extras.iter().any(|e| e == UNITREE_EXTRA) {
        say::info("next: dimos hardware g1 setup --robot-ip <ip>");
    }
}

/// `dimos setup`, as a list of calls.
pub fn run(
    args: &SetupArgs,
    ctx: &mut Ctx,
    probes: &Probes,
    cfg: &Platforms,
    home: &Path,
) -> Result<i32> {
    let cwd = std::env::current_dir().context("no working directory: cd somewhere and re-run")?;
    let prior = probes.installed.as_ref();
    let target = resolve_target(args, prior, &cwd, probes.platform.arch, cfg, ctx)?;
    preflight(&target, probes)?;
    let steps = plan(&target, probes, cfg, home)?;
    let report = run::run(&steps, ctx)?;
    report.print(ctx);
    if !ctx.dry_run {
        install_record::save(home, &record(&target, probes, prior))?;
    }
    if report.exit_code() == 0 {
        next_steps(&target, probes);
    }
    Ok(report.exit_code())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action_log::ActionLog;
    use crate::install_record::{PlatformSummary, TmpDir};
    use crate::probe::{Gpu, Kernel, Tools};
    use crate::run_context::Mode;
    use crate::sudo::Sudo;

    fn ctx(home: &Path) -> Ctx {
        Ctx {
            mode: Mode::NonInteractive,
            dry_run: true,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(home).expect("open the action log"),
            run_id: "test".to_string(),
        }
    }

    fn args() -> SetupArgs {
        SetupArgs {
            mode: None,
            extras: Vec::new(),
            branch: None,
            dir: None,
            blueprint: None,
            with_nix: false,
        }
    }

    fn probes(home: &Path) -> Probes {
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
                systemd: true,
                home: home.to_path_buf(),
                user: "tester".to_string(),
                shell: PathBuf::from("/bin/bash"),
            },
            kernel: Kernel::default(),
            tools: Tools::default(),
            installed: None,
            rc: Vec::new(),
            ifaces: Vec::new(),
            current_exe: PathBuf::from("/tmp/dimos"),
        }
    }

    fn stage_named<'a>(plan: &'a Plan, name: &str) -> &'a Stage {
        plan.stages
            .iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("{name} is in the plan"))
    }

    fn installed(dir: &Path) -> Installed {
        Installed {
            schema: install_record::SCHEMA,
            installer_version: DIMOS_VERSION.to_string(),
            dimos_version: DIMOS_VERSION.to_string(),
            mode: InstallMode::Dev,
            dir: dir.to_path_buf(),
            branch: Some("aaryan/installer".to_string()),
            extras: vec!["base".to_string(), "unitree".to_string()],
            platform: PlatformSummary {
                os: "linux".to_string(),
                distro: "ubuntu".to_string(),
                version: "20.04".to_string(),
                arch: "aarch64".to_string(),
                glibc: Some("2.31".to_string()),
                jetson: Some("R35.3.1".to_string()),
            },
            hardware: std::collections::BTreeMap::new(),
            last: None,
        }
    }

    fn target(home: &Path) -> Target {
        Target {
            mode: InstallMode::Dev,
            extras: vec!["base".to_string()],
            branch: DEFAULT_BRANCH.to_string(),
            dir: home.join(DEFAULT_DIR),
            with_nix: false,
            blueprint: None,
        }
    }

    /// Every package the plan wants, reported by dpkg as already installed.
    fn configured(home: &Path, cfg: &Platforms) -> Probes {
        let mut probes = probes(home);
        probes.current_exe = install_record::installed_bin(home);
        probes.tools = Tools {
            uv: Some(home.join(".local/bin/uv")),
            login_path_has_local_bin: true,
            dpkg_status: platforms::system_packages(&["base".to_string()], PkgManager::Apt, cfg)
                .iter()
                .map(|p| format!("{p} install ok installed\n"))
                .collect(),
            ..Tools::default()
        };
        probes.kernel = Kernel {
            sysctl: cfg.linux.sysctl.clone(),
            lo_multicast: true,
            multicast_route: true,
            memlock_conf_bytes: Some(cfg.linux.memlock_bytes),
            sysctl_conf: Some(system_config::render_sysctl_conf(&cfg.linux.sysctl)),
            enabled_units: vec![format!("{}.service", install_record::MULTICAST_UNIT)],
            nvpmodel_maxn: None,
        };
        probes
    }

    fn names(plan: &Plan) -> Vec<&str> {
        plan.stages.iter().map(|s| s.name).collect()
    }

    #[test]
    fn bare_non_interactive_target_is_library_base_main_dimos_app() {
        let home = TmpDir::new("setup-bare");
        let got = resolve_target(
            &args(),
            None,
            Path::new("/work"),
            Arch::X86_64,
            &Platforms::load(),
            &ctx(home.path()),
        )
        .expect("a bare run resolves to defaults");
        assert_eq!(got.mode, InstallMode::Library);
        assert_eq!(got.extras, ["base"]);
        assert_eq!(got.branch, "main");
        assert_eq!(got.dir, Path::new("/work/dimos-app"));
        assert!(!got.with_nix);
    }

    #[test]
    fn prior_state_supplies_mode_dir_extras_on_rerun() {
        let home = TmpDir::new("setup-prior");
        let prior = installed(Path::new("/srv/dimos"));
        let got = resolve_target(
            &args(),
            Some(&prior),
            Path::new("/work"),
            Arch::X86_64,
            &Platforms::load(),
            &ctx(home.path()),
        )
        .expect("a re-run reuses installer.json");
        assert_eq!(got.mode, InstallMode::Dev);
        assert_eq!(got.dir, Path::new("/srv/dimos"));
        assert_eq!(got.extras, ["base", "unitree"]);
        assert_eq!(got.branch, "aaryan/installer");
    }

    #[test]
    fn an_explicit_flag_beats_the_prior_install() {
        let home = TmpDir::new("setup-flags");
        let mut args = args();
        args.mode = Some(InstallMode::Library);
        args.extras = vec!["base".to_string()];
        args.dir = Some(PathBuf::from("elsewhere"));
        let got = resolve_target(
            &args,
            Some(&installed(Path::new("/srv/dimos"))),
            Path::new("/work"),
            Arch::X86_64,
            &Platforms::load(),
            &ctx(home.path()),
        )
        .expect("flags win over installer.json");
        assert_eq!(got.mode, InstallMode::Library);
        assert_eq!(got.extras, ["base"]);
        assert_eq!(got.dir, Path::new("/work/elsewhere"));
    }

    #[test]
    fn plan_order_is_fixed_for_linux_fixture_and_macos_has_no_sysconfig() {
        let home = TmpDir::new("setup-order");
        let cfg = Platforms::load();
        let linux = plan(
            &target(home.path()),
            &probes(home.path()),
            &cfg,
            home.path(),
        )
        .expect("the linux fixture plans");
        assert_eq!(
            names(&linux),
            [
                "self-install",
                "apt update",
                "packages",
                "uv",
                "nix",
                "dimos",
                "sysconfig",
                "jetson perf",
                "verify"
            ]
        );
        let mut mac = probes(home.path());
        mac.platform.os = Os::MacOs {
            version: "15.0".to_string(),
        };
        mac.platform.arch = Arch::Aarch64;
        mac.platform.pkg = PkgManager::Brew;
        mac.platform.systemd = false;
        let macos =
            plan(&target(home.path()), &mac, &cfg, home.path()).expect("the macos fixture plans");
        assert_eq!(names(&macos), names(&linux));
        assert!(stage_named(&macos, "sysconfig").actions.is_empty());
        assert!(stage_named(&macos, "apt update").actions.is_empty());
    }

    #[test]
    fn unsupported_platform_errors_before_any_stage() {
        let home = TmpDir::new("setup-intel-mac");
        let mut probes = probes(home.path());
        probes.platform.os = Os::MacOs {
            version: "13.6".to_string(),
        };
        let err = preflight(&target(home.path()), &probes).expect_err("Intel macOS has no build");
        assert!(format!("{err:#}").contains("Intel macOS"), "{err:#}");
    }

    #[test]
    fn disk_below_12gib_without_prior_install_errors() {
        let dir = Path::new("/work/dimos-app");
        let err = disk_gate(Some(MIN_DISK_BYTES - GIB), None, dir).expect_err("11 GiB is too few");
        let text = format!("{err:#}");
        assert!(
            text.contains("11 GiB free") && text.contains("needs 12 GiB"),
            "{text}"
        );
        assert!(disk_gate(Some(MIN_DISK_BYTES), None, dir).is_ok());
        assert!(disk_gate(None, None, dir).is_ok());
        assert!(disk_gate(Some(0), Some(&installed(dir)), dir).is_ok());
    }

    #[test]
    fn df_available_is_the_fourth_posix_field_in_kib() {
        let text = "Filesystem 1024-blocks      Used Available Capacity Mounted on\n\
                    /dev/nvme0n1p1 460043484 121276168 315283924      28% /\n";
        assert_eq!(parse_df_kib(text), Some(315_283_924));
        assert_eq!(parse_df_kib("Filesystem 1024-blocks\n"), None);
    }

    #[test]
    fn installed_fixture_second_run_plans_only_the_verify() {
        let home = TmpDir::new("setup-second");
        let cfg = Platforms::load();
        let dir = target(home.path()).dir;
        std::fs::create_dir_all(dir.join(".git")).unwrap();
        std::fs::write(dir.join(".git/HEAD"), "ref: refs/heads/main\n").unwrap();
        std::fs::create_dir_all(install_record::venv(&dir).join("bin")).unwrap();
        std::fs::write(install_record::venv_python(&dir), "").unwrap();
        std::fs::write(install_record::venv(&dir).join("bin/dimos"), "").unwrap();
        let mut probes = configured(home.path(), &cfg);
        probes.installed = Some(Installed {
            mode: InstallMode::Dev,
            dir: dir.clone(),
            branch: Some("main".to_string()),
            extras: vec!["base".to_string()],
            ..installed(&dir)
        });
        let steps = plan(&target(home.path()), &probes, &cfg, home.path())
            .expect("a configured machine plans");
        let busy: Vec<&str> = steps
            .stages
            .iter()
            .filter(|s| !s.actions.is_empty())
            .map(|s| s.name)
            .collect();
        assert_eq!(busy, ["verify"]);
    }

    #[test]
    fn a_machine_with_no_package_manager_gets_the_manual_note() {
        let home = TmpDir::new("setup-unmanaged");
        let cfg = Platforms::load();
        let mut probes = probes(home.path());
        probes.platform.pkg = PkgManager::None;
        let steps = plan(&target(home.path()), &probes, &cfg, home.path()).expect("plans anyway");
        assert!(
            steps.notes.iter().any(|n| n.contains("no apt")),
            "{:?}",
            steps.notes
        );
        assert!(stage_named(&steps, "packages").actions.is_empty());
    }

    #[test]
    fn a_dev_record_keeps_the_branch_and_the_prior_hardware_entry() {
        let home = TmpDir::new("setup-record");
        let mut prior = installed(&home.path().join(DEFAULT_DIR));
        prior.hardware.insert(
            "g1".to_string(),
            crate::install_record::HardwareRun {
                at: "2026-09-01T00:00:00Z".to_string(),
                result: "applied".to_string(),
                robot_ip: None,
                interface: None,
            },
        );
        let got = record(&target(home.path()), &probes(home.path()), Some(&prior));
        assert_eq!(got.branch.as_deref(), Some("main"));
        assert_eq!(got.dimos_version, "git:main");
        assert!(got.hardware.contains_key("g1"));
    }
}
