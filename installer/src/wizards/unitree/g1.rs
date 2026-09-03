//! The Unitree G1 wizard (brief decision 13): CycloneDDS built from source, the Unitree SDK into the
//! venv, and the shell/git/.env edits it needs. `observe` is the only I/O; every stage is pure over
//! its result, so a configured robot plans nothing.

use std::path::{Path, PathBuf};

use anyhow::{bail, Result};

use crate::action::{self, text, Action};
use crate::cli::HardwareSetupArgs;
use crate::install_record::Installed;
use crate::plan::{Plan, Stage};
use crate::platforms::Platforms;
use crate::probe::{capture, Arch, Os, Probes, RcFile};
use crate::setup::{system_config, system_packages, verify};
use crate::wizards::nvidia::jetson;
use crate::wizards::{checks, interface_ready, notes, setup_first, Robot};

pub(crate) const UNITREE_EXTRA: &str = "unitree";
const CYCLONEDDS_REPO: &str = "https://github.com/eclipse-cyclonedds/cyclonedds";
/// unitree_sdk2py's bindings only build against the 0.10 line; main is ABI-incompatible.
const CYCLONEDDS_BRANCH: &str = "releases/0.10.x";
const CYCLONEDDS_PIP: &str = "cyclonedds==0.10.2";
const NUMPY_PIN: &str = "numpy<2,>=1.26";
const SDK_REPO: &str = "https://github.com/unitreerobotics/unitree_sdk2_python.git";
pub(crate) const CDDS_MARKER: &str = "cyclonedds";

const CDDS_HOME_REL: &str = "cyclonedds/install";
const OPT_SDK: &str = "/opt/unitree_sdk2_python";
const GIT_INSTEADOF_KEY: &str = "url.https://github.com/.insteadOf";
const GIT_SSH_URL: &str = "ssh://git@github.com/";
/// Reads the dist metadata, not the extension, so it answers before CYCLONEDDS_HOME is set.
const CDDS_VERSION_CODE: &str = "import importlib.metadata as m; print(m.version('cyclonedds'))";
const NUMPY_CODE: &str = "import numpy; print(numpy.__version__)";
const CLONE_TIMEOUT_S: u64 = 900;
const CONFIGURE_TIMEOUT_S: u64 = 600;
/// Both the C library and the cyclonedds wheel compile C on an Orin NX: minutes, not seconds.
const BUILD_TIMEOUT_S: u64 = 1800;
const PIP_TIMEOUT_S: u64 = 600;
const GIT_CONFIG_TIMEOUT_S: u64 = 30;
const PROBE_TIMEOUT_S: u64 = 20;
/// A cold import of the SDK or numpy on an Orin NX reads a lot of shared objects off eMMC.
const IMPORT_TIMEOUT_S: u64 = 120;

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub(crate) struct G1Observed {
    pub cyclonedds_lib: bool,
    pub cyclonedds_clone: bool,
    pub sdk_present: bool,
    pub sdk_imports: bool,
    pub cyclonedds_py_version: Option<String>,
    pub numpy_major: Option<u32>,
    pub git_insteadof_set: bool,
    pub dotenv: String,
    pub nproc: usize,
}

/// The only I/O in this file, all of it read-only through `capture`, so `--dry-run` may call it.
fn observe(home: &Path, venv_python: &Path, sdk: &Path, dir: &Path) -> G1Observed {
    let cdds = text(&cyclonedds_home(home));
    let python = text(venv_python);
    G1Observed {
        cyclonedds_lib: cyclonedds_home(home).join("lib/libddsc.so").exists(),
        cyclonedds_clone: home.join("cyclonedds/.git").exists(),
        sdk_present: sdk.exists(),
        sdk_imports: capture(
            &python,
            &["-c", "import unitree_sdk2py"],
            &[("CYCLONEDDS_HOME", &cdds)],
            IMPORT_TIMEOUT_S,
        )
        .is_some(),
        cyclonedds_py_version: capture(&python, &["-c", CDDS_VERSION_CODE], &[], PROBE_TIMEOUT_S)
            .filter(|v| !v.is_empty()),
        numpy_major: numpy_major(venv_python),
        git_insteadof_set: git_insteadof(),
        dotenv: std::fs::read_to_string(dir.join(".env")).unwrap_or_default(),
        nproc: std::thread::available_parallelism().map_or(4, |n| n.get()),
    }
}

/// Where the SDK is or goes, then what the robot already has; the one call `hardware` and `update` share.
pub(crate) fn detect(
    home: &Path,
    installed: &Installed,
    sdk_override: Option<PathBuf>,
) -> (PathBuf, G1Observed) {
    let sdk = sdk_path(sdk_override, Path::new(OPT_SDK).exists(), home);
    let obs = observe(home, &installed.venv_python(), &sdk, &installed.dir);
    (sdk, obs)
}

fn numpy_major(venv_python: &Path) -> Option<u32> {
    parse_major(&capture(
        &text(venv_python),
        &["-c", NUMPY_CODE],
        &[],
        IMPORT_TIMEOUT_S,
    )?)
}

/// `1.26.4` -> 1.
fn parse_major(version: &str) -> Option<u32> {
    version.trim().split('.').next()?.parse().ok()
}

pub fn cyclonedds_home(home: &Path) -> PathBuf {
    home.join(CDDS_HOME_REL)
}

/// `$SDK2_PATH`, else an existing `/opt` checkout, else `~` — never a sudo clone into `/opt`.
fn sdk_path(env_sdk2: Option<PathBuf>, opt_exists: bool, home: &Path) -> PathBuf {
    match env_sdk2 {
        Some(path) => path,
        None if opt_exists => PathBuf::from(OPT_SDK),
        None => home.join("unitree_sdk2_python"),
    }
}

fn cyclonedds_stage(home: &Path, obs: &G1Observed) -> Stage {
    let mut stage = Stage::new("cyclonedds build", true);
    if obs.cyclonedds_lib {
        return stage;
    }
    let dir = home.join("cyclonedds");
    let (src, build) = (text(&dir), text(&dir.join("build")));
    let prefix = format!("-DCMAKE_INSTALL_PREFIX={}", text(&cyclonedds_home(home)));
    let jobs = format!("-j{}", obs.nproc);
    if !obs.cyclonedds_clone {
        stage = stage.push(clone(CYCLONEDDS_REPO, &src, Some(CYCLONEDDS_BRANCH)));
    }
    stage
        .run(
            &["cmake", "-S", &src, "-B", &build, &prefix],
            CONFIGURE_TIMEOUT_S,
        )
        .run(
            &["cmake", "--build", &build, "--target", "install", &jobs],
            BUILD_TIMEOUT_S,
        )
}

fn sdk_clone_stage(sdk: &Path, obs: &G1Observed) -> Stage {
    let stage = Stage::new("unitree sdk clone", false);
    if obs.sdk_present {
        return stage;
    }
    stage.push(clone(SDK_REPO, &text(sdk), None))
}

/// Shallow: the robot pulls over wifi and neither checkout is ever used as a git history.
fn clone(repo: &str, into: &str, branch: Option<&str>) -> Action {
    let mut argv = vec!["git", "clone", "--depth", "1"];
    argv.extend(branch.map(|b| ["-b", b]).into_iter().flatten());
    argv.extend([repo, into]);
    Action::run(&argv, CLONE_TIMEOUT_S)
}

fn sdk_install_stage(
    uv: &Path,
    venv_python: &Path,
    sdk: &Path,
    cdds_home: &Path,
    obs: &G1Observed,
) -> Stage {
    let mut stage = Stage::new("unitree sdk install", true);
    let cdds = text(cdds_home);
    let env = [("CYCLONEDDS_HOME", cdds.as_str())];
    let (uv, python, sdk) = (text(uv), text(venv_python), text(sdk));
    if obs.cyclonedds_py_version.as_deref() != Some(pinned_version(CYCLONEDDS_PIP)) {
        stage = stage.push(uv_pip(
            &uv,
            &python,
            &[CYCLONEDDS_PIP],
            &env,
            BUILD_TIMEOUT_S,
        ));
    }
    if !obs.sdk_imports {
        let editable = ["--no-deps", "-e", &sdk];
        stage = stage.push(uv_pip(&uv, &python, &editable, &env, PIP_TIMEOUT_S));
    }
    stage
}

fn uv_pip(uv: &str, python: &str, rest: &[&str], env: &[(&str, &str)], timeout_s: u64) -> Action {
    let mut argv = vec![uv, "pip", "install", "--python", python];
    argv.extend_from_slice(rest);
    Action::run_in(&argv, None, env, timeout_s)
}

/// `uv sync` and `uv pip` re-apply pyproject's `numpy>=2` override (pyproject.toml:547), so the pin
/// is re-asserted after every install into the venv, and whenever the probe finds numpy 2.
fn numpy_pin_stage(
    uv: &Path,
    venv_python: &Path,
    numpy_major: Option<u32>,
    venv_changed: bool,
) -> Stage {
    let stage = Stage::new("numpy pin", true);
    if !venv_changed && numpy_major == Some(1) {
        return stage;
    }
    stage.push(uv_pip(
        &text(uv),
        &text(venv_python),
        &[NUMPY_PIN],
        &[],
        PIP_TIMEOUT_S,
    ))
}

/// Empty once every rc file the login shell reads already holds the block.
fn rc_stage(rc: &[RcFile]) -> Stage {
    let lines = vec![format!("export CYCLONEDDS_HOME=\"$HOME/{CDDS_HOME_REL}\"")];
    rc.iter()
        .filter(|file| action::ensure_block(&file.text, CDDS_MARKER, &lines).1)
        .fold(Stage::new("cyclonedds env", false), |stage, file| {
            stage.push(Action::EnsureBlock {
                file: file.path.clone(),
                marker: CDDS_MARKER.to_string(),
                lines: lines.clone(),
            })
        })
}

fn git_https_stage(obs: &G1Observed) -> Stage {
    let stage = Stage::new("git https rewrite", false);
    if obs.git_insteadof_set {
        return stage;
    }
    stage.run(
        &["git", "config", "--global", GIT_INSTEADOF_KEY, GIT_SSH_URL],
        GIT_CONFIG_TIMEOUT_S,
    )
}

/// Empty when `.env` already says exactly this, so a re-run plans nothing.
fn dotenv_stage(dir: &Path, obs: &G1Observed, args: &HardwareSetupArgs) -> Stage {
    let mut kv = vec![
        ("ROBOT_IP", args.robot_ip.as_str()),
        ("ROBOT_INTERFACE", args.interface.as_str()),
    ];
    if let Some(transport) = args.transport {
        kv.push(("DIMOS_TRANSPORT", transport.name()));
    }
    let stage = Stage::new("robot .env", false);
    let contents = merge_dotenv(&obs.dotenv, &kv);
    if contents == obs.dotenv {
        return stage;
    }
    stage.push(Action::WriteFile {
        path: dir.join(".env"),
        mode: 0o600,
        contents,
        sudo: false,
    })
}

/// Rewrites each `KEY=` line where it stands, appends the missing ones, keeps every other line.
fn merge_dotenv(existing: &str, kv: &[(&str, &str)]) -> String {
    let mut seen = vec![false; kv.len()];
    let mut out = String::new();
    for line in existing.lines() {
        match kv.iter().position(|(key, _)| assigns(line, key)) {
            Some(i) => {
                seen[i] = true;
                out.push_str(&format!("{}={}\n", kv[i].0, kv[i].1));
            }
            None => {
                out.push_str(line);
                out.push('\n');
            }
        }
    }
    for (i, (key, value)) in kv.iter().enumerate() {
        if !seen[i] {
            out.push_str(&format!("{key}={value}\n"));
        }
    }
    out
}

fn assigns(line: &str, key: &str) -> bool {
    line.strip_prefix(key)
        .is_some_and(|rest| rest.starts_with('='))
}

fn pinned_version(spec: &str) -> &str {
    spec.split_once("==").map_or(spec, |(_, version)| version)
}

fn git_insteadof() -> bool {
    capture(
        "git",
        &["config", "--global", "--get", GIT_INSTEADOF_KEY],
        &[],
        PROBE_TIMEOUT_S,
    )
    .is_some_and(|value| value == GIT_SSH_URL)
}

pub(crate) fn ready(
    args: &HardwareSetupArgs,
    probes: &Probes,
    installed: &Installed,
) -> Result<()> {
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

/// The one I/O step `plan` cannot do: read the robot for what is already built.
pub(crate) fn setup(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
    home: &Path,
) -> Plan {
    let (sdk, obs) = detect(home, installed, args.sdk_path.clone());
    plan(args, probes, cfg, installed, &obs, &sdk)
}

pub(crate) fn plan(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
    obs: &G1Observed,
    sdk: &Path,
) -> Plan {
    let mut stages = stages(args, probes, cfg, installed, obs, sdk, false);
    stages.extend(checks(target(&probes.platform.home, args), installed, args));
    Plan {
        command: Robot::G1.command(),
        stages,
        notes: notes(probes),
    }
}

/// The G1 bring-up (brief decision 13), shared with `update`. The numpy pin follows the SDK
/// install because every `uv pip`/`uv sync` before it re-applies pyproject's numpy>=2 override.
pub(crate) fn stages(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
    obs: &G1Observed,
    sdk: &Path,
    venv_changed: bool,
) -> Vec<Stage> {
    let home = &probes.platform.home;
    let uv = system_packages::uv_bin(&probes.tools, home);
    let python = installed.venv_python();
    let sdk_install = sdk_install_stage(&uv, &python, sdk, &cyclonedds_home(home), obs);
    let reinstalled = venv_changed || !sdk_install.actions.is_empty();
    let mut stages = system_packages::packages_stages(&installed.extras, probes, cfg);
    stages.extend([
        cyclonedds_stage(home, obs),
        sdk_clone_stage(sdk, obs),
        sdk_install,
        numpy_pin_stage(&uv, &python, obs.numpy_major, reinstalled),
        rc_stage(&probes.rc),
        git_https_stage(obs),
        jetson::stage(&probes.platform, &probes.kernel),
        system_config::stage(&probes.platform, &probes.kernel, cfg),
        dotenv_stage(&installed.dir, obs, args),
    ]);
    stages
}

/// What the G1 verify needs: where libddsc lives and which NIC the robot answers on.
pub(crate) fn target(home: &Path, args: &HardwareSetupArgs) -> verify::Target {
    verify::Target::G1 {
        cyclonedds_home: cyclonedds_home(home),
        interface: args.interface.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::Transport;

    const HOME: &str = "/home/unitree";
    const DIR: &str = "/home/unitree/dimos";
    const UV: &str = "/home/unitree/.local/bin/uv";
    const PYTHON: &str = "/home/unitree/dimos/.venv/bin/python";
    const SDK: &str = "/home/unitree/unitree_sdk2_python";

    fn home() -> PathBuf {
        PathBuf::from(HOME)
    }

    fn fresh() -> G1Observed {
        G1Observed {
            nproc: 8,
            ..G1Observed::default()
        }
    }

    fn installed() -> G1Observed {
        G1Observed {
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

    fn args() -> HardwareSetupArgs {
        HardwareSetupArgs {
            robot_ip: "192.168.123.161".into(),
            interface: "eth0".into(),
            transport: None,
            sdk_path: None,
            blueprint: None,
        }
    }

    fn rc(text: &str) -> Vec<RcFile> {
        vec![RcFile {
            path: home().join(".profile"),
            text: text.to_string(),
        }]
    }

    fn rc_with_block() -> Vec<RcFile> {
        let lines = [format!("export CYCLONEDDS_HOME=\"$HOME/{CDDS_HOME_REL}\"")];
        rc(&action::ensure_block("", CDDS_MARKER, &lines).0)
    }

    fn argvs(stage: &Stage) -> Vec<Vec<String>> {
        stage
            .actions
            .iter()
            .filter_map(|a| match a {
                Action::Run { argv, .. } => Some(argv.clone()),
                _ => None,
            })
            .collect()
    }

    fn every_stage(obs: &G1Observed, rc: &[RcFile]) -> Vec<Stage> {
        let cdds = cyclonedds_home(&home());
        let sdk_install =
            sdk_install_stage(Path::new(UV), Path::new(PYTHON), Path::new(SDK), &cdds, obs);
        let reinstalled = !sdk_install.actions.is_empty();
        vec![
            cyclonedds_stage(&home(), obs),
            sdk_clone_stage(Path::new(SDK), obs),
            sdk_install,
            numpy_pin_stage(
                Path::new(UV),
                Path::new(PYTHON),
                obs.numpy_major,
                reinstalled,
            ),
            rc_stage(rc),
            git_https_stage(obs),
            dotenv_stage(Path::new(DIR), obs, &args()),
        ]
    }

    fn planned(stages: &[Stage]) -> Vec<&str> {
        stages
            .iter()
            .filter(|s| !s.actions.is_empty())
            .map(|s| s.name)
            .collect()
    }

    #[test]
    fn a_configured_robot_plans_nothing() {
        let stages = every_stage(&installed(), &rc_with_block());
        assert_eq!(planned(&stages), Vec::<&str>::new());
    }

    #[test]
    fn a_fresh_robot_plans_every_build_stage() {
        assert_eq!(
            planned(&every_stage(&fresh(), &rc(""))),
            vec![
                "cyclonedds build",
                "unitree sdk clone",
                "unitree sdk install",
                "numpy pin",
                "cyclonedds env",
                "git https rewrite",
                "robot .env",
            ]
        );
    }

    #[test]
    fn cyclonedds_skips_the_clone_when_the_checkout_exists_but_still_builds_when_the_lib_is_missing(
    ) {
        let obs = G1Observed {
            cyclonedds_clone: true,
            ..fresh()
        };
        let planned = argvs(&cyclonedds_stage(&home(), &obs));
        assert_eq!(planned.len(), 2);
        assert_eq!(planned[0][0], "cmake");
        assert_eq!(planned[1].last().unwrap(), "-j8");
    }

    #[test]
    fn cyclonedds_clones_the_zero_ten_branch_into_home_and_installs_beside_it() {
        let planned = argvs(&cyclonedds_stage(&home(), &fresh()));
        assert_eq!(
            planned[0],
            vec![
                "git",
                "clone",
                "--depth",
                "1",
                "-b",
                "releases/0.10.x",
                CYCLONEDDS_REPO,
                "/home/unitree/cyclonedds",
            ]
        );
        assert!(planned[1]
            .contains(&"-DCMAKE_INSTALL_PREFIX=/home/unitree/cyclonedds/install".to_string()));
    }

    #[test]
    fn cyclonedds_stage_is_critical_and_empty_once_the_library_is_present() {
        let stage = cyclonedds_stage(&home(), &installed());
        assert!(stage.critical);
        assert!(stage.actions.is_empty());
    }

    #[test]
    fn sdk_path_prefers_env_then_an_existing_opt_then_home_and_never_plans_sudo() {
        let chosen = PathBuf::from("/srv/sdk");
        assert_eq!(sdk_path(Some(chosen.clone()), true, &home()), chosen);
        assert_eq!(sdk_path(None, true, &home()), PathBuf::from(OPT_SDK));
        assert_eq!(
            sdk_path(None, false, &home()),
            home().join("unitree_sdk2_python")
        );
    }

    #[test]
    fn no_g1_stage_asks_for_sudo() {
        assert!(every_stage(&fresh(), &rc(""))
            .iter()
            .all(|s| !s.needs_sudo()));
    }

    #[test]
    fn sdk_install_pins_cyclonedds_and_installs_the_sdk_no_deps_editable_with_cyclonedds_home() {
        let cdds = cyclonedds_home(&home());
        let stage = sdk_install_stage(
            Path::new(UV),
            Path::new(PYTHON),
            Path::new(SDK),
            &cdds,
            &fresh(),
        );
        assert!(stage.critical);
        let planned = argvs(&stage);
        assert_eq!(
            planned[0],
            vec![UV, "pip", "install", "--python", PYTHON, CYCLONEDDS_PIP]
        );
        assert_eq!(
            planned[1],
            vec![
                UV,
                "pip",
                "install",
                "--python",
                PYTHON,
                "--no-deps",
                "-e",
                SDK
            ]
        );
        for action in &stage.actions {
            let Action::Run { env, .. } = action else {
                unreachable!()
            };
            assert_eq!(
                env,
                &vec![(
                    "CYCLONEDDS_HOME".to_string(),
                    "/home/unitree/cyclonedds/install".to_string()
                )]
            );
        }
    }

    #[test]
    fn sdk_install_skips_the_binding_build_when_the_pinned_version_is_already_there() {
        let obs = G1Observed {
            cyclonedds_py_version: Some("0.10.2".into()),
            ..fresh()
        };
        let cdds = cyclonedds_home(&home());
        let planned = argvs(&sdk_install_stage(
            Path::new(UV),
            Path::new(PYTHON),
            Path::new(SDK),
            &cdds,
            &obs,
        ));
        assert_eq!(planned.len(), 1);
        assert!(planned[0].contains(&"--no-deps".to_string()));
    }

    #[test]
    fn numpy_pin_is_planned_after_a_venv_change_or_at_numpy_2_and_skipped_at_numpy_1() {
        let pin = |major, changed| {
            argvs(&numpy_pin_stage(
                Path::new(UV),
                Path::new(PYTHON),
                major,
                changed,
            ))
        };
        assert_eq!(pin(Some(1), false), Vec::<Vec<String>>::new());
        assert_eq!(
            pin(Some(1), true),
            vec![vec![
                UV,
                "pip",
                "install",
                "--python",
                PYTHON,
                "numpy<2,>=1.26"
            ]]
        );
        assert_eq!(pin(Some(2), false).len(), 1);
        assert_eq!(pin(None, false).len(), 1);
    }

    #[test]
    fn parse_major_reads_the_first_component() {
        assert_eq!(parse_major("1.26.4\n"), Some(1));
        assert_eq!(parse_major("2.1.0"), Some(2));
        assert_eq!(parse_major(""), None);
    }

    #[test]
    fn no_action_mentions_opencv_python_or_the_unitree_sdk2_main_checkout() {
        let text: String = every_stage(&fresh(), &rc(""))
            .iter()
            .flat_map(|s| s.actions.iter().map(Action::display))
            .collect();
        assert!(!text.contains("opencv-python"));
        assert!(!text.contains("unitree_sdk2-main"));
    }

    #[test]
    fn rc_block_exports_cyclonedds_home_through_the_shell_variable_not_a_baked_path() {
        let stage = rc_stage(&rc(""));
        let Action::EnsureBlock {
            file,
            marker,
            lines,
        } = &stage.actions[0]
        else {
            unreachable!()
        };
        assert_eq!(file, &home().join(".profile"));
        assert_eq!(marker, CDDS_MARKER);
        assert_eq!(
            lines,
            &vec!["export CYCLONEDDS_HOME=\"$HOME/cyclonedds/install\"".to_string()]
        );
    }

    #[test]
    fn rc_stage_is_empty_when_every_rc_file_already_holds_the_block() {
        assert!(rc_stage(&rc_with_block()).actions.is_empty());
        assert_eq!(rc_stage(&rc("export A=1\n")).actions.len(), 1);
    }

    #[test]
    fn git_https_stage_is_empty_when_the_rewrite_is_already_configured() {
        assert!(git_https_stage(&installed()).actions.is_empty());
        assert_eq!(
            argvs(&git_https_stage(&fresh()))[0],
            vec!["git", "config", "--global", GIT_INSTEADOF_KEY, GIT_SSH_URL]
        );
    }

    #[test]
    fn merge_dotenv_replaces_a_key_in_place_keeps_the_rest_and_appends_what_is_missing() {
        let existing = "# notes\nROBOT_IP=10.0.0.1\nOTHER=keep\n";
        let merged = merge_dotenv(existing, &[("ROBOT_IP", "192.168.123.161"), ("NEW", "1")]);
        assert_eq!(
            merged,
            "# notes\nROBOT_IP=192.168.123.161\nOTHER=keep\nNEW=1\n"
        );
    }

    #[test]
    fn merge_dotenv_on_an_empty_file_writes_only_the_pairs() {
        assert_eq!(
            merge_dotenv("", &[("ROBOT_IP", "1.2.3.4")]),
            "ROBOT_IP=1.2.3.4\n"
        );
    }

    #[test]
    fn dotenv_stage_writes_transport_only_when_the_flag_was_given() {
        let without = dotenv_stage(Path::new(DIR), &fresh(), &args());
        let with = dotenv_stage(
            Path::new(DIR),
            &fresh(),
            &HardwareSetupArgs {
                transport: Some(Transport::Zenoh),
                ..args()
            },
        );
        let text = |stage: &Stage| match &stage.actions[0] {
            Action::WriteFile { contents, .. } => contents.clone(),
            _ => unreachable!(),
        };
        assert_eq!(
            text(&without),
            "ROBOT_IP=192.168.123.161\nROBOT_INTERFACE=eth0\n"
        );
        assert_eq!(
            text(&with),
            "ROBOT_IP=192.168.123.161\nROBOT_INTERFACE=eth0\nDIMOS_TRANSPORT=zenoh\n"
        );
    }

    #[test]
    fn dotenv_stage_is_empty_when_the_file_already_says_exactly_this() {
        assert!(dotenv_stage(Path::new(DIR), &installed(), &args())
            .actions
            .is_empty());
        let moved = HardwareSetupArgs {
            robot_ip: "192.168.123.99".into(),
            ..args()
        };
        assert_eq!(
            dotenv_stage(Path::new(DIR), &installed(), &moved)
                .actions
                .len(),
            1
        );
    }

    #[test]
    fn dotenv_is_written_owner_only_because_it_carries_robot_addresses() {
        let Action::WriteFile { path, mode, .. } =
            &dotenv_stage(Path::new(DIR), &fresh(), &args()).actions[0]
        else {
            unreachable!()
        };
        assert_eq!(path, &PathBuf::from("/home/unitree/dimos/.env"));
        assert_eq!(*mode, 0o600);
    }

    #[test]
    fn the_probed_binding_version_is_read_off_the_one_pip_spec() {
        assert_eq!(pinned_version(CYCLONEDDS_PIP), "0.10.2");
    }
}
