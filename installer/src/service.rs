//! `dimos service`: one root systemd unit per blueprint. Rendering and every plan are pure over
//! their arguments; the only I/O is reading /etc/systemd/system back to list what is installed.

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};

use crate::cli::ServiceAction;
use crate::install_record::{self, Installed};
use crate::plan::{Action, Plan, Stage};
use crate::probe::Platform;

/// systemctl returns as soon as the job is queued; the budget only bounds a hung sudo.
const SYSTEMCTL_TIMEOUT_S: u64 = 60;
const UNIT_PREFIX: &str = "dimos-";

pub fn unit_name(blueprint: &str) -> String {
    format!("{UNIT_PREFIX}{blueprint}")
}

/// A blueprint name is lowercase letters, digits and dashes, the same shape `dimos list` prints.
pub fn validate_blueprint(name: &str) -> Result<()> {
    let ok = !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-');
    if !ok {
        bail!("a blueprint name is lowercase letters, digits and '-', got {name:?}");
    }
    Ok(())
}

/// Split each `--env K=V` into its pair.
pub fn parse_env(pairs: &[String]) -> Result<Vec<(String, String)>> {
    pairs
        .iter()
        .map(|pair| {
            let (key, value) = pair
                .split_once('=')
                .filter(|(key, _)| !key.is_empty())
                .with_context(|| format!("--env wants K=V with a non-empty K, got {pair:?}"))?;
            Ok((key.to_string(), value.to_string()))
        })
        .collect()
}

/// Escape a value for a quoted `Environment=` line.
pub fn escape_env(value: &str) -> Result<String> {
    if value.contains('\n') {
        bail!("env value contains a newline");
    }
    let mut escaped = String::with_capacity(value.len());
    for c in value.chars() {
        match c {
            // systemd reads the quoted value C-style, and expands a lone `%` as a unit specifier.
            '\\' | '"' => escaped.push('\\'),
            '%' => escaped.push('%'),
            _ => {}
        }
        escaped.push(c);
    }
    Ok(escaped)
}

/// Render the unit that runs one blueprint as the installing user.
pub fn render_unit(
    blueprint: &str,
    user: &str,
    home: &Path,
    dir: &Path,
    venv_bin: &Path,
    cdds_home: Option<&Path>,
    env: &[(String, String)],
) -> Result<String> {
    validate_blueprint(blueprint)?;
    let mut vars = vec![("PATH".to_string(), unit_path_var(home, venv_bin))];
    if let Some(cdds) = cdds_home {
        vars.push(("CYCLONEDDS_HOME".to_string(), cdds.display().to_string()));
    }
    vars.extend_from_slice(env);
    let environment = render_environment(&vars)?;
    let multicast = install_record::MULTICAST_UNIT;
    Ok(format!(
        "[Unit]\n\
         Description=DimOS blueprint {blueprint}\n\
         After=network-online.target {multicast}.service\n\
         Wants=network-online.target\n\
         \n\
         [Service]\n\
         Type=simple\n\
         User={user}\n\
         WorkingDirectory={dir}\n\
         {environment}\
         ExecStart={venv}/dimos --rerun-open none run {blueprint}\n\
         Restart=on-failure\n\
         RestartSec=5\n\
         \n\
         [Install]\n\
         WantedBy=multi-user.target\n",
        dir = dir.display(),
        venv = venv_bin.display(),
    ))
}

fn render_environment(vars: &[(String, String)]) -> Result<String> {
    vars.iter()
        .map(|(key, value)| {
            let escaped = escape_env(value).with_context(|| format!("--env {key}"))?;
            Ok(format!("Environment=\"{key}={escaped}\"\n"))
        })
        .collect()
}

/// systemd expands no `~` and sources no login shell, so the unit spells its PATH out.
fn unit_path_var(home: &Path, venv_bin: &Path) -> String {
    format!(
        "{}:{}/.local/bin:/usr/local/bin:/usr/bin:/bin",
        venv_bin.display(),
        home.display()
    )
}

/// One stage per verb, so `--dry-run` shows exactly the systemctl calls it would make.
pub fn plan(
    action: &ServiceAction,
    installed: &Installed,
    platform: &Platform,
    cdds_home: Option<PathBuf>,
) -> Result<Plan> {
    let (verb, blueprint) = verb_and_blueprint(action);
    validate_blueprint(blueprint)?;
    require_systemd(platform)?;
    let stage = stage_for(action, blueprint, installed, platform, cdds_home.as_deref())?;
    Ok(Plan {
        command: format!("service {verb} {blueprint}"),
        stages: vec![stage],
        notes: Vec::new(),
    })
}

fn stage_for(
    action: &ServiceAction,
    blueprint: &str,
    installed: &Installed,
    platform: &Platform,
    cdds_home: Option<&Path>,
) -> Result<Stage> {
    let unit = format!("{}.service", unit_name(blueprint));
    Ok(match action {
        ServiceAction::Setup { env, .. } => {
            setup_stage(&unit, blueprint, installed, platform, cdds_home, env)?
        }
        ServiceAction::Start { .. } => control_stage("service-start", &unit, "start", true),
        ServiceAction::Stop { .. } => control_stage("service-stop", &unit, "stop", false),
        ServiceAction::Restart { .. } => control_stage("service-restart", &unit, "restart", true),
        ServiceAction::Status { .. } => status_stage(&unit),
        ServiceAction::Remove { .. } => {
            let path = install_record::unit_path(&unit_name(blueprint));
            let present = path.exists();
            remove_stage(&unit, &path, present)
        }
    })
}

fn verb_and_blueprint(action: &ServiceAction) -> (&'static str, &str) {
    match action {
        ServiceAction::Setup { blueprint, .. } => ("setup", blueprint),
        ServiceAction::Start { blueprint } => ("start", blueprint),
        ServiceAction::Stop { blueprint } => ("stop", blueprint),
        ServiceAction::Restart { blueprint } => ("restart", blueprint),
        ServiceAction::Status { blueprint } => ("status", blueprint),
        ServiceAction::Remove { blueprint } => ("remove", blueprint),
    }
}

/// Every verb here is a systemctl call; without systemd there is nothing to talk to.
fn require_systemd(platform: &Platform) -> Result<()> {
    if platform.systemd {
        return Ok(());
    }
    let os = platform.summary();
    bail!(
        "a service needs Linux with systemd, got {} {}: run the blueprint directly with `dimos run <blueprint>`",
        os.distro,
        os.version
    );
}

fn setup_stage(
    unit: &str,
    blueprint: &str,
    installed: &Installed,
    platform: &Platform,
    cdds_home: Option<&Path>,
    env: &[String],
) -> Result<Stage> {
    let venv_bin = installed.venv().join("bin");
    let contents = render_unit(
        blueprint,
        &platform.user,
        &platform.home,
        &installed.dir,
        &venv_bin,
        cdds_home,
        &parse_env(env)?,
    )?;
    Ok(Stage::new("service-setup", true)
        .push(Action::WriteFile {
            path: install_record::unit_path(&unit_name(blueprint)),
            mode: 0o644,
            contents,
            sudo: true,
        })
        .sudo(&["systemctl", "daemon-reload"], SYSTEMCTL_TIMEOUT_S)
        .sudo(&["systemctl", "enable", unit], SYSTEMCTL_TIMEOUT_S)
        .post(&["systemctl", "is-enabled", unit]))
}

fn control_stage(name: &'static str, unit: &str, verb: &str, active_after: bool) -> Stage {
    let stage = Stage::new(name, true).sudo(&["systemctl", verb, unit], SYSTEMCTL_TIMEOUT_S);
    if active_after {
        stage.post(&["systemctl", "is-active", unit])
    } else {
        stage
    }
}

/// Non-critical and unprivileged: `systemctl status` exits 3 for a stopped unit, which is an answer.
fn status_stage(unit: &str) -> Stage {
    Stage::new("service-status", false)
        .run(
            &["systemctl", "status", "--no-pager", unit],
            SYSTEMCTL_TIMEOUT_S,
        )
        .check()
        .warn_only()
}

/// An absent unit plans nothing, so a second `service remove` reports `ok already`.
fn remove_stage(unit: &str, path: &Path, present: bool) -> Stage {
    let stage = Stage::new("service-remove", true);
    if !present {
        return stage;
    }
    stage
        .sudo(
            &["systemctl", "disable", "--now", unit],
            SYSTEMCTL_TIMEOUT_S,
        )
        .push(Action::Remove {
            path: path.to_path_buf(),
            sudo: true,
        })
        .sudo(&["systemctl", "daemon-reload"], SYSTEMCTL_TIMEOUT_S)
}

/// Every unit the installer may have written, for `uninstall` to disable and remove.
pub fn installed_units() -> Vec<PathBuf> {
    let mut units: Vec<PathBuf> = std::fs::read_dir(install_record::UNIT_DIR)
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| is_dimos_unit(path))
        .collect();
    units.sort();
    units
}

fn is_dimos_unit(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| name.starts_with(UNIT_PREFIX) && name.ends_with(".service"))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::cli::InstallMode;
    use crate::install_record::PlatformSummary;
    use crate::probe::{Arch, Gpu, Os, PkgManager};

    const DIR: &str = "/home/unitree/dimos";
    const HOME: &str = "/home/unitree";

    fn installed() -> Installed {
        Installed {
            schema: install_record::SCHEMA,
            installer_version: "0.1.0".to_string(),
            dimos_version: "0.1.0".to_string(),
            mode: InstallMode::Dev,
            dir: PathBuf::from(DIR),
            branch: Some("main".to_string()),
            extras: vec!["unitree".to_string()],
            platform: PlatformSummary {
                os: "linux".to_string(),
                distro: "ubuntu".to_string(),
                version: "20.04".to_string(),
                arch: "aarch64".to_string(),
                glibc: Some("2.31".to_string()),
                jetson: Some("35.3.1".to_string()),
            },
            hardware: Default::default(),
            last: None,
        }
    }

    fn platform(systemd: bool) -> Platform {
        Platform {
            os: Os::Linux {
                id: "ubuntu".to_string(),
                version: "20.04".to_string(),
            },
            arch: Arch::Aarch64,
            glibc: Some((2, 31)),
            jetson: None,
            gpu: Gpu::Tegra,
            pkg: PkgManager::Apt,
            systemd,
            home: PathBuf::from(HOME),
            user: "unitree".to_string(),
            shell: PathBuf::from("/bin/bash"),
        }
    }

    fn macos() -> Platform {
        Platform {
            os: Os::MacOs {
                version: "15.5".to_string(),
            },
            systemd: false,
            ..platform(false)
        }
    }

    fn unit() -> String {
        render_unit(
            "unitree-g1",
            "unitree",
            Path::new(HOME),
            Path::new(DIR),
            Path::new("/home/unitree/dimos/.venv/bin"),
            Some(Path::new("/home/unitree/cyclonedds/install")),
            &[("ROBOT_IP".to_string(), "192.168.123.161".to_string())],
        )
        .expect("fixture renders")
    }

    fn displays(stage: &Stage) -> Vec<String> {
        stage.actions.iter().map(Action::display).collect()
    }

    fn argv(words: &[&str]) -> Vec<String> {
        words.iter().map(|w| (*w).to_string()).collect()
    }

    #[test]
    fn unit_renders_expected_lines_for_fixture() {
        let text = unit();
        for line in [
            "[Unit]",
            "[Service]",
            "[Install]",
            "Type=simple",
            "Description=DimOS blueprint unitree-g1",
            "After=network-online.target dimos-multicast.service",
            "User=unitree",
            "WorkingDirectory=/home/unitree/dimos",
            "ExecStart=/home/unitree/dimos/.venv/bin/dimos --rerun-open none run unitree-g1",
            "Restart=on-failure",
            "WantedBy=multi-user.target",
            r#"Environment="CYCLONEDDS_HOME=/home/unitree/cyclonedds/install""#,
            r#"Environment="ROBOT_IP=192.168.123.161""#,
        ] {
            assert!(text.lines().any(|l| l == line), "missing {line}\n{text}");
        }
    }

    #[test]
    fn the_unit_path_is_absolute_because_systemd_expands_no_tilde_and_reads_no_rc_file() {
        let text = unit();
        let path = text
            .lines()
            .find(|l| l.starts_with(r#"Environment="PATH="#))
            .expect("a PATH line");
        assert_eq!(
            path,
            r#"Environment="PATH=/home/unitree/dimos/.venv/bin:/home/unitree/.local/bin:/usr/local/bin:/usr/bin:/bin""#
        );
        assert!(!text.contains('~'), "{text}");
    }

    #[test]
    fn environment_value_with_quote_and_backslash_is_escaped() {
        assert_eq!(
            escape_env(r#"say "hi" C:\x"#).expect("escapes"),
            r#"say \"hi\" C:\\x"#
        );
    }

    #[test]
    fn a_percent_in_an_env_value_is_doubled_so_systemd_expands_no_specifier() {
        assert_eq!(escape_env("100%h").expect("escapes"), "100%%h");
    }

    #[test]
    fn newline_in_env_value_is_an_error() {
        let err = escape_env("one\ntwo").expect_err("a newline ends the line early");
        assert!(err.to_string().contains("newline"), "{err}");
    }

    #[test]
    fn an_env_pair_without_a_key_or_an_equals_is_an_error_naming_the_pair() {
        assert_eq!(
            parse_env(&["ROBOT_IP=10.0.0.1".to_string()]).expect("splits"),
            vec![("ROBOT_IP".to_string(), "10.0.0.1".to_string())]
        );
        for bad in ["ROBOT_IP", "=10.0.0.1"] {
            let err = parse_env(&[bad.to_string()]).expect_err("K=V required");
            assert!(err.to_string().contains(bad), "{err}");
        }
    }

    #[test]
    fn an_env_value_may_hold_an_equals_sign() {
        assert_eq!(
            parse_env(&["FLAGS=a=b".to_string()]).expect("splits at the first ="),
            vec![("FLAGS".to_string(), "a=b".to_string())]
        );
    }

    #[test]
    fn blueprint_name_with_slash_or_space_is_rejected() {
        assert!(validate_blueprint("unitree-g1").is_ok());
        for bad in ["../etc/passwd", "unitree g1", "Unitree", "", "g1;reboot"] {
            assert!(validate_blueprint(bad).is_err(), "accepted {bad:?}");
        }
    }

    #[test]
    fn setup_plan_writes_unit_then_reloads_then_enables() {
        let action = ServiceAction::Setup {
            blueprint: "unitree-g1".to_string(),
            env: vec!["ROBOT_IP=192.168.123.161".to_string()],
        };
        let plan = plan(&action, &installed(), &platform(true), None).expect("plans");
        assert_eq!(plan.command, "service setup unitree-g1");
        let stage = &plan.stages[0];
        assert!(stage.critical);
        assert!(stage.needs_sudo());
        let shown = displays(stage);
        assert!(
            shown[0].starts_with("sudo write /etc/systemd/system/dimos-unitree-g1.service"),
            "{:?}",
            shown[0]
        );
        assert!(shown[0].ends_with("mode 644)"), "{:?}", shown[0]);
        assert_eq!(
            shown[1..],
            argv(&[
                "sudo systemctl daemon-reload",
                "sudo systemctl enable dimos-unitree-g1.service",
            ])[..]
        );
        assert_eq!(
            stage.post,
            Some(argv(&[
                "systemctl",
                "is-enabled",
                "dimos-unitree-g1.service"
            ]))
        );
    }

    #[test]
    fn start_and_restart_assert_the_unit_is_active_afterwards_and_stop_does_not() {
        for (action, verb) in [
            (
                ServiceAction::Start {
                    blueprint: "unitree-g1".to_string(),
                },
                "start",
            ),
            (
                ServiceAction::Restart {
                    blueprint: "unitree-g1".to_string(),
                },
                "restart",
            ),
        ] {
            let plan = plan(&action, &installed(), &platform(true), None).expect("plans");
            let stage = &plan.stages[0];
            assert_eq!(
                displays(stage),
                [format!("sudo systemctl {verb} dimos-unitree-g1.service")]
            );
            assert_eq!(
                stage.post,
                Some(argv(&[
                    "systemctl",
                    "is-active",
                    "dimos-unitree-g1.service"
                ]))
            );
        }
        let stop = ServiceAction::Stop {
            blueprint: "unitree-g1".to_string(),
        };
        let plan = plan(&stop, &installed(), &platform(true), None).expect("plans");
        assert_eq!(plan.stages[0].post, None);
    }

    #[test]
    fn status_is_unprivileged_and_not_critical_because_a_stopped_unit_exits_3() {
        let action = ServiceAction::Status {
            blueprint: "unitree-g1".to_string(),
        };
        let plan = plan(&action, &installed(), &platform(true), None).expect("plans");
        let stage = &plan.stages[0];
        assert!(!stage.critical);
        assert!(stage.check && stage.warn_only);
        assert!(!stage.needs_sudo());
        assert_eq!(
            displays(stage),
            ["systemctl status --no-pager dimos-unitree-g1.service"]
        );
    }

    #[test]
    fn removing_an_absent_unit_plans_nothing_so_a_second_remove_is_already() {
        let path = install_record::unit_path(&unit_name("unitree-g1"));
        assert_eq!(
            remove_stage("dimos-unitree-g1.service", &path, false).actions,
            []
        );
    }

    #[test]
    fn removing_an_installed_unit_disables_it_then_deletes_it_then_reloads() {
        let path = install_record::unit_path(&unit_name("unitree-g1"));
        assert_eq!(
            displays(&remove_stage("dimos-unitree-g1.service", &path, true)),
            argv(&[
                "sudo systemctl disable --now dimos-unitree-g1.service",
                "sudo remove /etc/systemd/system/dimos-unitree-g1.service",
                "sudo systemctl daemon-reload",
            ])[..]
        );
    }

    #[test]
    fn plan_refuses_without_systemd_naming_the_os() {
        let action = ServiceAction::Start {
            blueprint: "unitree-g1".to_string(),
        };
        let err = plan(&action, &installed(), &macos(), None).expect_err("no systemd on macOS");
        let text = err.to_string();
        assert!(text.contains("macos 15.5"), "{text}");
        assert!(text.contains("dimos run <blueprint>"), "{text}");
    }

    #[test]
    fn only_dimos_service_files_are_listed_for_uninstall() {
        assert!(is_dimos_unit(&install_record::unit_path(
            install_record::MULTICAST_UNIT
        )));
        assert!(is_dimos_unit(&install_record::unit_path(&unit_name(
            "unitree-g1"
        ))));
        for other in [
            "/etc/systemd/system/ssh.service",
            "/etc/systemd/system/dimos-g1.timer",
        ] {
            assert!(!is_dimos_unit(Path::new(other)), "{other}");
        }
    }
}
