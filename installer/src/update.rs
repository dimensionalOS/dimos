//! `dimos update` is doctor and update in one (brief decision 19): swap the binary, update DimOS in
//! the recorded directory, re-apply the machine config, then verify. `--dry-run` is the read-only
//! report — every probe runs, nothing is applied. `observe` is the only I/O outside `plan::run`.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use anyhow::{bail, Result};

use crate::cli::{InstallMode, UpdateArgs};
use crate::hardware::Robot;
use crate::pkgs::{self, Platforms, DIMOS_VERSION};
use crate::plan::{self, say, Action, Ctx, Outcome, Plan, Report, Stage};
use crate::probe::Probes;
use crate::setup::{deps, g1, jetson, sysconfig, verify};
use crate::state::{self, Installed};

pub const RELEASES: &str = "https://github.com/dimensionalOS/dimos/releases";
/// The same override `scripts/install.sh` honors, so a LAN test serves both from one directory.
const INSTALLER_URL_ENV: &str = "DIMOS_INSTALLER_URL";
const NUMPY_CODE: &str = "import numpy; print(numpy.__version__)";
/// A pre-release ranks below every marker, so a final release takes the highest rank.
const FINAL: u8 = 3;

const DOWNLOAD_TIMEOUT_S: u64 = 600;
const SHA_TIMEOUT_S: u64 = 120;
const SWAP_TIMEOUT_S: u64 = 30;
const GIT_TIMEOUT_S: u64 = 600;
/// A cold `uv sync` on an Orin NX builds wheels from source; the same budget `setup` gives it.
const SYNC_TIMEOUT_S: u64 = 3600;
const PIP_TIMEOUT_S: u64 = 1800;

/// What the probes saw; `--dry-run` fills it exactly as a real run does.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Observed {
    /// The release to install, or None when GitHub was unreachable or a URL override is set.
    pub latest: Option<String>,
    pub head: Option<String>,
    pub remote: Option<String>,
    pub numpy_major: Option<u32>,
}

/// The only I/O in this file, and all of it read-only, so `--dry-run` runs every probe.
pub fn observe(installed: &Installed, args: &UpdateArgs, override_url: Option<&str>) -> Observed {
    let dev = installed.mode == InstallMode::Dev;
    let dir = text(&installed.dir);
    Observed {
        latest: candidate(args, override_url),
        head: dev.then(|| git(&dir, &["rev-parse", "HEAD"])).flatten(),
        remote: dev
            .then(|| remote_head(&dir, installed.branch.as_deref()))
            .flatten(),
        numpy_major: installed
            .hardware
            .contains_key(Robot::G1.key())
            .then(|| numpy_major(installed))
            .flatten(),
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
    parse_latest_tag(&capture("curl", &args, &[])?)
}

fn remote_head(dir: &str, branch: Option<&str>) -> Option<String> {
    let refname = format!("refs/heads/{}", branch?);
    parse_ls_remote(&git(dir, &["ls-remote", "origin", &refname])?)
}

/// GIT_TERMINAL_PROMPT=0 so a private remote fails instead of blocking on a credential prompt.
fn git(dir: &str, args: &[&str]) -> Option<String> {
    let mut argv = vec!["-C", dir];
    argv.extend_from_slice(args);
    capture("git", &argv, &[("GIT_TERMINAL_PROMPT", "0")])
}

fn numpy_major(installed: &Installed) -> Option<u32> {
    let python = text(&installed.venv_python());
    parse_major(&capture(&python, &["-c", NUMPY_CODE], &[])?)
}

fn capture(program: &str, args: &[&str], env: &[(&str, &str)]) -> Option<String> {
    let mut cmd = Command::new(program);
    cmd.args(args).stdin(Stdio::null());
    for (key, value) in env {
        cmd.env(key, value);
    }
    let out = cmd.output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

/// `https://github.com/x/y/releases/tag/v0.0.15` -> `0.0.15`.
pub fn parse_latest_tag(url: &str) -> Option<String> {
    let (_, tag) = url.trim().rsplit_once("/tag/")?;
    let version = normalize(tag);
    (!version.is_empty()).then_some(version)
}

/// The 40-hex object id `git ls-remote` prints before the ref name.
pub fn parse_ls_remote(text: &str) -> Option<String> {
    let sha = text.split_whitespace().next()?;
    (sha.len() == 40 && sha.chars().all(|c| c.is_ascii_hexdigit())).then(|| sha.to_string())
}

/// `1.26.4` -> 1.
pub fn parse_major(version: &str) -> Option<u32> {
    version.trim().split('.').next()?.parse().ok()
}

fn normalize(tag: &str) -> String {
    tag.trim().trim_start_matches('v').to_string()
}

/// PEP 440-lite: the release numbers, then a rank that sorts `0.0.14b1` below `0.0.14`.
pub fn version_key(v: &str) -> (Vec<u32>, (u8, u32)) {
    let stripped = normalize(v);
    let (release, pre) = split_pre(&stripped);
    let numbers = release.split('.').map(|p| p.parse().unwrap_or(0)).collect();
    (numbers, pre)
}

/// `rc` first: it carries a `c`, so a later `a`/`b` split would cut a release candidate in half.
fn split_pre(v: &str) -> (&str, (u8, u32)) {
    for (marker, rank) in [("rc", 2u8), ("a", 0), ("b", 1)] {
        if let Some((release, n)) = v.split_once(marker) {
            return (release, (rank, n.parse().unwrap_or(0)));
        }
    }
    (v, (FINAL, 0))
}

pub fn newer(current: &str, candidate: &str) -> bool {
    version_key(candidate) > version_key(current)
}

pub fn release_base(version: Option<&str>, override_url: Option<&str>) -> String {
    match (override_url, version) {
        (Some(url), _) => url.trim_end_matches('/').to_string(),
        (None, Some(v)) => format!("{RELEASES}/download/v{}", normalize(v)),
        (None, None) => format!("{RELEASES}/latest/download"),
    }
}

pub fn artifact(target: &str) -> String {
    format!("dimos-{target}")
}

/// The swap renames a sibling file over the binary, so it only works on the installed copy.
pub fn require_installed_exe(home: &Path, current_exe: &Path) -> Result<PathBuf> {
    let installed = state::installed_bin(home);
    if current_exe != installed {
        bail!(
            "running {}, not the installed copy: `{} update` updates the binary",
            current_exe.display(),
            installed.display()
        );
    }
    Ok(installed)
}

/// Downloads beside the binary (one filesystem, so the swap is an atomic rename) and swaps only
/// after the sha256 and a `--version` run on the new file both pass.
pub fn self_update_stage(base: &str, target: &str, bin: &Path, bak: &Path) -> Stage {
    let (new, sums) = (bin.with_extension("new"), bin.with_extension("new.sha256"));
    fetch_actions(base, &artifact(target), &new, &sums)
        .into_iter()
        .chain(swap_actions(bin, bak, new, sums))
        .fold(Stage::new("self-update", false), Stage::push)
}

fn fetch_actions(base: &str, asset: &str, new: &Path, sums: &Path) -> Vec<Action> {
    let (new_s, sums_s) = (text(new), text(sums));
    let (bin_url, sums_url) = (format!("{base}/{asset}"), format!("{base}/{asset}.sha256"));
    vec![
        Action::run(
            &["curl", "-fsSL", "--retry", "3", &bin_url, "-o", &new_s],
            DOWNLOAD_TIMEOUT_S,
        ),
        Action::run(
            &["curl", "-fsSL", "--retry", "3", &sums_url, "-o", &sums_s],
            SHA_TIMEOUT_S,
        ),
        Action::VerifySha256 {
            file: new.to_path_buf(),
            sums_file: sums.to_path_buf(),
        },
        Action::run(&["chmod", "0755", &new_s], SWAP_TIMEOUT_S),
        Action::run(&[&new_s, "--version"], SWAP_TIMEOUT_S),
    ]
}

/// Copy, not rename: the binary is never absent, so a failed swap still leaves a working `dimos`.
fn swap_actions(bin: &Path, bak: &Path, new: PathBuf, sums: PathBuf) -> Vec<Action> {
    vec![
        Action::Copy {
            from: bin.to_path_buf(),
            to: bak.to_path_buf(),
            mode: 0o755,
        },
        Action::Rename {
            from: new,
            to: bin.to_path_buf(),
        },
        Action::Remove {
            path: sums,
            sudo: false,
        },
    ]
}

/// Skipped with a note rather than fatal: the rest of `update` is a repair and should still run.
fn self_update_or_note(
    args: &UpdateArgs,
    obs: &Observed,
    probes: &Probes,
    home: &Path,
) -> (Stage, Option<String>) {
    let empty = Stage::new("self-update", false);
    let bin = match require_installed_exe(home, &probes.current_exe) {
        Ok(bin) => bin,
        Err(e) => return (empty, Some(format!("{e:#}"))),
    };
    let target = match probes.platform.target() {
        Ok(target) => target,
        Err(e) => return (empty, Some(format!("{e:#}"))),
    };
    let Some(candidate) = obs.latest.as_deref() else {
        return (empty, None);
    };
    if !args.force && !newer(DIMOS_VERSION, candidate) {
        return (empty, None);
    }
    let base = release_base(Some(candidate), override_url().as_deref());
    (
        self_update_stage(&base, target, &bin, &state::backup_bin(home)),
        None,
    )
}

/// Library re-pins from PyPI, dev fast-forwards and re-syncs; empty when the probe says it is there.
pub fn dimos_stage(installed: &Installed, uv: &Path, obs: &Observed, force: bool) -> Stage {
    // non-critical: a checkout with local commits must not stop the machine-config repair.
    let stage = Stage::new("dimos", false);
    if !force && at_wanted_version(installed, obs) {
        return stage;
    }
    match installed.mode {
        InstallMode::Library => stage.push(pip_action(installed, uv)),
        InstallMode::Dev => stage
            .push(pull_action(installed))
            .push(sync_action(installed, uv)),
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
        pkgs::pip_spec(&installed.extras),
    ];
    in_dir(argv, installed, &[], PIP_TIMEOUT_S)
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
    in_dir(
        argv,
        installed,
        &[("GIT_LFS_SKIP_SMUDGE", "1")],
        GIT_TIMEOUT_S,
    )
}

/// `--inexact` leaves packages the lockfile does not name, so a hand-installed SDK survives.
fn sync_action(installed: &Installed, uv: &Path) -> Action {
    let mut argv = vec![text(uv), "sync".to_string(), "--inexact".to_string()];
    argv.extend(pkgs::sync_args(&installed.extras));
    in_dir(
        argv,
        installed,
        &[("GIT_LFS_SKIP_SMUDGE", "1")],
        SYNC_TIMEOUT_S,
    )
}

fn in_dir(
    argv: Vec<String>,
    installed: &Installed,
    env: &[(&str, &str)],
    timeout_s: u64,
) -> Action {
    let borrowed: Vec<&str> = argv.iter().map(String::as_str).collect();
    Action::run_in(&borrowed, Some(&installed.dir), env, timeout_s)
}

/// `uv sync` re-applies pyproject's numpy>=2 override (pyproject.toml:547), which breaks the G1's
/// DDS bindings, so the pin is re-asserted whenever the DimOS stage runs or the venv has drifted.
fn numpy_stages(installed: &Installed, uv: &Path, obs: &Observed, dimos_runs: bool) -> Vec<Stage> {
    if !installed.hardware.contains_key(Robot::G1.key()) {
        return Vec::new();
    }
    if !dimos_runs && obs.numpy_major == Some(1) {
        return vec![Stage::new("numpy pin", true)];
    }
    vec![g1::numpy_pin_stage(uv, &installed.venv_python())]
}

fn machine_stages(installed: &Installed, probes: &Probes, cfg: &Platforms) -> Vec<Stage> {
    vec![
        deps::packages_stage(&installed.extras, probes, cfg),
        sysconfig::stage(&probes.platform, &probes.kernel, cfg),
        jetson::stage(&probes.platform, &probes.kernel),
    ]
}

/// A recorded G1 is verified as a G1, so a broken DDS stack fails the run instead of passing quietly.
fn target(installed: &Installed, probes: &Probes, home: &Path) -> verify::Target {
    if installed.hardware.contains_key(Robot::G1.key()) {
        return verify::Target::G1 {
            cyclonedds_home: g1::cyclonedds_home(home),
        };
    }
    if probes.platform.is_jetson() {
        verify::Target::Jetson
    } else {
        verify::Target::Host
    }
}

fn notes(installed: &Installed, probes: &Probes, skipped: Option<String>) -> Vec<String> {
    [
        skipped,
        installed.hardware.contains_key(Robot::G1.key()).then(|| {
            "cyclonedds and the Unitree SDK are `dimos hardware g1 setup`, not update".to_string()
        }),
        jetson::static_tls_note(&probes.platform),
        jetson::thermal_note(&probes.kernel),
        sysconfig::no_systemd_note(&probes.platform),
    ]
    .into_iter()
    .flatten()
    .collect()
}

pub fn plan(
    installed: &Installed,
    args: &UpdateArgs,
    probes: &Probes,
    cfg: &Platforms,
    obs: &Observed,
    home: &Path,
) -> Plan {
    let uv = deps::uv_bin(&probes.tools, home);
    let (self_update, skipped) = self_update_or_note(args, obs, probes, home);
    let dimos = dimos_stage(installed, &uv, obs, args.force);
    let dimos_runs = !dimos.actions.is_empty();
    let mut stages = vec![self_update, dimos];
    stages.extend(numpy_stages(installed, &uv, obs, dimos_runs));
    stages.extend(machine_stages(installed, probes, cfg));
    let checks = target(installed, probes, home);
    stages.extend(verify::stages(
        &checks,
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

/// A dry run exits 0 only when the machine is already where it should be; a planned stage means no.
pub fn dry_run_exit(report: &Report) -> i32 {
    i32::from(!report.stages.iter().all(|(_, o)| *o == Outcome::Already))
}

/// Puts the kept `.bak` back through the same executor, so `--dry-run --rollback` prints the swap.
pub fn rollback(ctx: &mut Ctx, home: &Path) -> Result<i32> {
    let (bin, bak) = (state::installed_bin(home), state::backup_bin(home));
    if !bak.exists() {
        bail!("nothing to roll back: {} does not exist", bak.display());
    }
    let steps = Plan {
        command: "update --rollback".to_string(),
        stages: vec![Stage::new("rollback", true).push(Action::Rename { from: bak, to: bin })],
        notes: Vec::new(),
    };
    let report = plan::run(&steps, ctx)?;
    report.print(ctx);
    Ok(report.exit_code())
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
    let Some(installed) = state::load(home)? else {
        say::fail("no DimOS install recorded: run `dimos setup` first");
        return Ok(2);
    };
    let obs = observe(&installed, args, override_url().as_deref());
    let steps = plan(&installed, args, probes, cfg, &obs, home);
    let report = plan::run(&steps, ctx)?;
    report.print(ctx);
    if ctx.dry_run {
        return Ok(dry_run_exit(&report));
    }
    rollback_hint(&report, home);
    Ok(report.exit_code())
}

fn rollback_hint(report: &Report, home: &Path) {
    if report.exit_code() != 0 && state::backup_bin(home).exists() {
        say::info("rollback: dimos update --rollback");
    }
}

fn override_url() -> Option<String> {
    std::env::var(INSTALLER_URL_ENV)
        .ok()
        .filter(|s| !s.is_empty())
}

fn text(path: &Path) -> String {
    path.display().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::InstallMode;
    use crate::probe::{Arch, Gpu, Kernel, Os, PkgManager, Platform, Tools};
    use crate::state::{ActionLog, HardwareRun, PlatformSummary, TmpDir};
    use crate::sudo::Sudo;
    use std::collections::BTreeMap;

    fn installed(mode: InstallMode, home: &Path) -> Installed {
        Installed {
            schema: state::SCHEMA,
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
        installed.hardware.insert(
            Robot::G1.key().to_string(),
            HardwareRun {
                at: "2026-09-01T00:00:00Z".to_string(),
                result: "applied".to_string(),
                robot_ip: None,
                interface: None,
            },
        );
        installed
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
            rc: Vec::new(),
            ifaces: Vec::new(),
            dotenv: None,
            current_exe,
        }
    }

    fn args() -> UpdateArgs {
        UpdateArgs {
            version: None,
            force: false,
            rollback: false,
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

    #[test]
    fn artifact_is_dimos_dash_target() {
        assert_eq!(artifact("aarch64-linux-musl"), "dimos-aarch64-linux-musl");
    }

    #[test]
    fn release_base_prefers_the_url_override_then_the_pinned_tag_then_latest() {
        assert_eq!(
            release_base(Some("0.0.15"), Some("http://10.0.0.5:8000/")),
            "http://10.0.0.5:8000"
        );
        assert_eq!(
            release_base(Some("v0.0.15"), None),
            format!("{RELEASES}/download/v0.0.15")
        );
        assert_eq!(
            release_base(None, None),
            format!("{RELEASES}/latest/download")
        );
    }

    #[test]
    fn version_key_sorts_a_prerelease_below_its_final_release() {
        assert!(version_key("0.0.14b1") < version_key("0.0.14"));
        assert!(version_key("0.0.14") < version_key("0.0.15"));
        assert!(version_key("0.0.14a2") < version_key("0.0.14b1"));
        assert!(version_key("0.0.14b1") < version_key("0.0.14rc1"));
    }

    #[test]
    fn newer_is_false_for_the_same_version() {
        assert!(!newer("0.0.14", "0.0.14"));
        assert!(newer("0.0.14", "0.0.15"));
        assert!(!newer("0.0.15", "0.0.14b1"));
    }

    #[test]
    fn parse_latest_tag_reads_the_version_out_of_the_redirect_url() {
        assert_eq!(
            parse_latest_tag("https://github.com/dimensionalOS/dimos/releases/tag/v0.0.15\n"),
            Some("0.0.15".to_string())
        );
        assert_eq!(parse_latest_tag("https://github.com/x/y/releases"), None);
    }

    #[test]
    fn parse_ls_remote_takes_the_object_id_not_the_ref_name() {
        let line = "9f8b2c1d0e4a6b7c8d9e0f1a2b3c4d5e6f708192\trefs/heads/main\n";
        assert_eq!(
            parse_ls_remote(line),
            Some("9f8b2c1d0e4a6b7c8d9e0f1a2b3c4d5e6f708192".to_string())
        );
        assert_eq!(parse_ls_remote("fatal: could not read"), None);
    }

    #[test]
    fn parse_major_reads_the_first_component() {
        assert_eq!(parse_major("1.26.4\n"), Some(1));
        assert_eq!(parse_major("2.1.0"), Some(2));
        assert_eq!(parse_major(""), None);
    }

    #[test]
    fn self_update_swaps_only_after_the_sha256_and_the_version_run_pass() {
        let bin = PathBuf::from("/home/u/.local/bin/dimos");
        let bak = PathBuf::from("/home/u/.local/bin/dimos.bak");
        let stage = self_update_stage("http://host/d", "x86_64-linux-musl", &bin, &bak);
        let verify_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::VerifySha256 { .. }))
            .expect("a sha256 check");
        let rename_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::Rename { .. }))
            .expect("a swap");
        assert!(verify_at < rename_at);
        assert!(argv_of(&stage)
            .iter()
            .any(|argv| argv == &["/home/u/.local/bin/dimos.new", "--version"]));
    }

    #[test]
    fn self_update_copies_the_old_binary_before_the_rename_so_it_is_never_absent() {
        let bin = PathBuf::from("/home/u/.local/bin/dimos");
        let bak = bin.with_extension("bak");
        let stage = self_update_stage("http://host/d", "x86_64-linux-musl", &bin, &bak);
        let copy_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::Copy { to, .. } if *to == bak))
            .expect("a backup copy");
        let rename_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::Rename { to, .. } if *to == bin))
            .expect("a swap");
        assert!(copy_at < rename_at);
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
        let probes = probes(home.path(), state::installed_bin(home.path()));
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
        assert_eq!(argv[0].last().unwrap(), &pkgs::pip_spec(&inst.extras));
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
    fn numpy_pin_is_planned_whenever_the_dimos_stage_runs_on_a_recorded_g1() {
        let home = TmpDir::new("update-numpy-run");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let obs = Observed {
            numpy_major: Some(1),
            ..Observed::default()
        };
        let stages = numpy_stages(&inst, Path::new("/uv"), &obs, true);
        assert_eq!(argv_of(&stages[0]).len(), 1);
    }

    #[test]
    fn numpy_pin_is_skipped_when_the_g1_venv_already_holds_numpy_1_and_nothing_syncs() {
        let home = TmpDir::new("update-numpy-ok");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let obs = Observed {
            numpy_major: Some(1),
            ..Observed::default()
        };
        let stages = numpy_stages(&inst, Path::new("/uv"), &obs, false);
        assert!(stages[0].actions.is_empty());
    }

    #[test]
    fn numpy_pin_is_absent_off_a_recorded_g1() {
        let home = TmpDir::new("update-numpy-host");
        let inst = installed(InstallMode::Dev, home.path());
        let obs = Observed {
            numpy_major: None,
            ..Observed::default()
        };
        assert!(numpy_stages(&inst, Path::new("/uv"), &obs, true).is_empty());
    }

    #[test]
    fn plan_ends_with_a_critical_verify_stage() {
        let home = TmpDir::new("update-plan");
        let inst = installed(InstallMode::Library, home.path());
        let probes = probes(home.path(), state::installed_bin(home.path()));
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
    fn plan_orders_the_numpy_pin_after_the_dimos_stage_that_breaks_it() {
        let home = TmpDir::new("update-order");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), state::installed_bin(home.path()));
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &probes,
            &cfg,
            &Observed::default(),
            home.path(),
        );
        let names: Vec<&str> = steps.stages.iter().map(|s| s.name).collect();
        let dimos_at = names.iter().position(|n| *n == "dimos").expect("dimos");
        let numpy_at = names.iter().position(|n| *n == "numpy pin").expect("numpy");
        assert!(dimos_at < numpy_at);
        assert_eq!(stage_named(&steps, "self-update").actions.len(), 0);
    }

    #[test]
    fn a_recorded_g1_is_verified_as_a_g1() {
        let home = TmpDir::new("update-g1-target");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), state::installed_bin(home.path()));
        assert_eq!(
            target(&inst, &probes, home.path()),
            verify::Target::G1 {
                cyclonedds_home: g1::cyclonedds_home(home.path())
            }
        );
    }

    #[test]
    fn dry_run_exit_is_0_only_when_every_stage_is_already() {
        let clean = Report {
            command: "update".to_string(),
            stages: vec![("dimos".to_string(), Outcome::Already)],
        };
        let pending = Report {
            command: "update".to_string(),
            stages: vec![
                ("dimos".to_string(), Outcome::Already),
                ("sysconfig".to_string(), Outcome::DryRun),
            ],
        };
        assert_eq!(dry_run_exit(&clean), 0);
        assert_eq!(dry_run_exit(&pending), 1);
    }

    #[test]
    fn rollback_without_a_backup_names_the_missing_file() {
        let home = TmpDir::new("update-rollback");
        let mut ctx = Ctx {
            mode: crate::plan::Mode::NonInteractive,
            dry_run: true,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(home.path()).unwrap(),
            run_id: "test".to_string(),
        };
        let err = rollback(&mut ctx, home.path()).unwrap_err();
        assert!(format!("{err:#}").contains("dimos.bak"));
    }

    #[test]
    fn no_sudo_action_in_the_update_plan_carries_an_environment() {
        let home = TmpDir::new("update-sudo-env");
        let inst = with_g1(installed(InstallMode::Dev, home.path()));
        let probes = probes(home.path(), state::installed_bin(home.path()));
        let cfg = Platforms::load();
        let steps = plan(
            &inst,
            &args(),
            &probes,
            &cfg,
            &Observed::default(),
            home.path(),
        );
        assert!(plan::sudo_env_violations(&steps).is_empty());
    }
}
