//! The verify stages. Every check is one `Action::Run` of `bash -lc`, so what it proves is what a
//! user's own login shell gets: the PATH and CYCLONEDDS_HOME rc blocks included.

use std::path::{Path, PathBuf};

use crate::plan::{Action, Stage};

/// A cold `import dimos` on an Orin NX is minutes, not seconds.
const CHECK_TIMEOUT_S: u64 = 300;

#[derive(Debug, Clone, PartialEq)]
pub enum Target {
    Host,
    /// The G1 is a Jetson, so its checks are the Jetson ones plus the DDS stack.
    G1 {
        cyclonedds_home: PathBuf,
    },
    Jetson,
}

/// What `dimos list` must show when `--blueprint` is absent.
pub fn default_blueprint(target: &Target) -> Option<&'static str> {
    match target {
        Target::G1 { .. } => Some("unitree-g1"),
        Target::Host | Target::Jetson => None,
    }
}

/// The verify stages for a target; the critical one is always last.
pub fn stages(target: &Target, venv: &Path, dir: &Path, blueprint: Option<&str>) -> Vec<Stage> {
    let mut out = Vec::new();
    if let Target::G1 { cyclonedds_home } = target {
        out.push(login_shell_stage(cyclonedds_home));
    }
    out.push(verify_stage(target, venv, dir, blueprint));
    out
}

fn verify_stage(target: &Target, venv: &Path, dir: &Path, blueprint: Option<&str>) -> Stage {
    let cdds = cyclonedds_home(target);
    let env: Vec<(&str, &str)> = cdds
        .iter()
        .map(|home| ("CYCLONEDDS_HOME", home.as_str()))
        .collect();
    let blueprint = blueprint.or_else(|| default_blueprint(target));
    checks(target, venv, dir, blueprint)
        .iter()
        .fold(Stage::new("verify", true), |stage, script| {
            stage.push(Action::run_in(
                &["bash", "-lc", script],
                None,
                &env,
                CHECK_TIMEOUT_S,
            ))
        })
}

/// Non-critical: a login shell that missed the rc block earns a `!!`, not a dead install.
fn login_shell_stage(cyclonedds_home: &Path) -> Stage {
    let script = format!(
        "test \"$CYCLONEDDS_HOME\" = {}",
        shq(&cyclonedds_home.to_string_lossy())
    );
    Stage::new("verify-shell", false).push(Action::run(&["bash", "-lc", &script], CHECK_TIMEOUT_S))
}

fn checks(target: &Target, venv: &Path, dir: &Path, blueprint: Option<&str>) -> Vec<String> {
    let mut out = vec![py(venv, "import dimos"), blueprint_check(venv, blueprint)];
    match target {
        Target::Host => {}
        Target::Jetson => out.extend(jetson_checks(venv)),
        Target::G1 { .. } => {
            out.extend(jetson_checks(venv));
            out.extend(g1_checks(venv, dir));
        }
    }
    out
}

/// `grep -qx` on the two-space list line (info.py:40), so `unitree-g1-basic` cannot pass for
/// `unitree-g1`; pipefail so a dead `dimos` is a failure and not an empty grep.
fn blueprint_check(venv: &Path, blueprint: Option<&str>) -> String {
    let list = format!("{} list", bin(venv, "dimos"));
    match blueprint {
        Some(name) => format!(
            "set -o pipefail; {list} | grep -qx {}",
            shq(&format!("  {name}"))
        ),
        None => format!("{list} > /dev/null"),
    }
}

fn jetson_checks(venv: &Path) -> Vec<String> {
    vec![
        // probe::parse_nv_tegra_release owns the real parse; this only re-asserts the shape.
        "grep -q REVISION: /etc/nv_tegra_release".to_string(),
        format!(
            "{} || echo 'torch: not importable (absent, or the static TLS error named in the plan notes)'",
            py(venv, "import torch")
        ),
    ]
}

fn g1_checks(venv: &Path, dir: &Path) -> Vec<String> {
    vec![
        py(venv, "import cyclonedds, unitree_sdk2py"),
        py(
            venv,
            "import numpy, sys; sys.exit(int(numpy.__version__.split(\".\")[0]) >= 2)",
        ),
        format!(
            "grep -q '^ROBOT_IP=' {}",
            shq(&dir.join(".env").to_string_lossy())
        ),
    ]
}

/// Belt for the rc block's braces: `import cyclonedds` needs CYCLONEDDS_HOME to find libddsc.
fn cyclonedds_home(target: &Target) -> Option<String> {
    match target {
        Target::G1 { cyclonedds_home } => Some(cyclonedds_home.to_string_lossy().into_owned()),
        Target::Host | Target::Jetson => None,
    }
}

fn py(venv: &Path, code: &str) -> String {
    format!("{} -c {}", bin(venv, "python"), shq(code))
}

fn bin(venv: &Path, name: &str) -> String {
    shq(&venv.join("bin").join(name).to_string_lossy())
}

/// POSIX single-quoting: a `'` closes the quote, escapes itself, and reopens it.
fn shq(text: &str) -> String {
    format!("'{}'", text.replace('\'', r"'\''"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn venv() -> PathBuf {
        PathBuf::from("/home/unitree/dimos/.venv")
    }

    fn dir() -> PathBuf {
        PathBuf::from("/home/unitree/dimos")
    }

    fn g1() -> Target {
        Target::G1 {
            cyclonedds_home: PathBuf::from("/home/unitree/cyclonedds/install"),
        }
    }

    fn scripts(stage: &Stage) -> Vec<String> {
        stage
            .actions
            .iter()
            .map(|a| match a {
                Action::Run { argv, .. } => argv[2].clone(),
                other => panic!("verify plans only Run actions, got {other:?}"),
            })
            .collect()
    }

    #[test]
    fn host_verify_is_one_critical_stage_that_imports_dimos_and_runs_the_console_script() {
        let stages = stages(&Target::Host, &venv(), &dir(), None);
        assert_eq!(stages.len(), 1);
        assert!(stages[0].critical);
        assert_eq!(
            scripts(&stages[0]),
            vec![
                "'/home/unitree/dimos/.venv/bin/python' -c 'import dimos'".to_string(),
                "'/home/unitree/dimos/.venv/bin/dimos' list > /dev/null".to_string(),
            ]
        );
    }

    #[test]
    fn g1_snippets_render_the_exact_python_one_liners() {
        let stages = stages(&g1(), &venv(), &dir(), None);
        assert_eq!(
            scripts(&stages[1]),
            vec![
                "'/home/unitree/dimos/.venv/bin/python' -c 'import dimos'",
                "set -o pipefail; '/home/unitree/dimos/.venv/bin/dimos' list | grep -qx '  unitree-g1'",
                "grep -q REVISION: /etc/nv_tegra_release",
                "'/home/unitree/dimos/.venv/bin/python' -c 'import torch' || echo 'torch: not importable (absent, or the static TLS error named in the plan notes)'",
                "'/home/unitree/dimos/.venv/bin/python' -c 'import cyclonedds, unitree_sdk2py'",
                "'/home/unitree/dimos/.venv/bin/python' -c 'import numpy, sys; sys.exit(int(numpy.__version__.split(\".\")[0]) >= 2)'",
                "grep -q '^ROBOT_IP=' '/home/unitree/dimos/.env'",
            ]
        );
    }

    #[test]
    fn g1_adds_a_non_critical_login_shell_stage_before_the_critical_verify() {
        let stages = stages(&g1(), &venv(), &dir(), None);
        assert_eq!(stages.len(), 2);
        assert_eq!(stages[0].name, "verify-shell");
        assert!(!stages[0].critical);
        assert_eq!(
            scripts(&stages[0]),
            vec!["test \"$CYCLONEDDS_HOME\" = '/home/unitree/cyclonedds/install'".to_string()]
        );
        assert_eq!(stages[1].name, "verify");
        assert!(stages[1].critical);
    }

    #[test]
    fn every_g1_check_carries_cyclonedds_home_so_the_rc_block_is_only_a_belt() {
        let stages = stages(&g1(), &venv(), &dir(), None);
        for action in &stages[1].actions {
            let Action::Run { env, .. } = action else {
                panic!("verify plans only Run actions");
            };
            assert_eq!(
                env,
                &[(
                    "CYCLONEDDS_HOME".to_string(),
                    "/home/unitree/cyclonedds/install".to_string()
                )]
            );
        }
    }

    #[test]
    fn host_and_jetson_checks_carry_no_env() {
        for target in [Target::Host, Target::Jetson] {
            for action in &stages(&target, &venv(), &dir(), None)[0].actions {
                let Action::Run { env, .. } = action else {
                    panic!("verify plans only Run actions");
                };
                assert!(env.is_empty());
            }
        }
    }

    #[test]
    fn every_check_runs_under_a_fresh_login_shell_without_sudo() {
        for target in [Target::Host, Target::Jetson, g1()] {
            for stage in stages(&target, &venv(), &dir(), None) {
                for action in &stage.actions {
                    let Action::Run { argv, sudo, .. } = action else {
                        panic!("verify plans only Run actions");
                    };
                    assert_eq!(&argv[..2], ["bash".to_string(), "-lc".to_string()]);
                    assert!(!sudo);
                }
            }
        }
    }

    #[test]
    fn default_blueprint_is_unitree_g1_only_for_the_g1() {
        assert_eq!(default_blueprint(&g1()), Some("unitree-g1"));
        assert_eq!(default_blueprint(&Target::Host), None);
        assert_eq!(default_blueprint(&Target::Jetson), None);
    }

    #[test]
    fn an_explicit_blueprint_overrides_the_target_default() {
        let stages = stages(&g1(), &venv(), &dir(), Some("unitree-g1-basic"));
        assert!(scripts(&stages[1])[1].ends_with("| grep -qx '  unitree-g1-basic'"));
    }

    #[test]
    fn blueprint_check_matches_a_whole_line_so_a_longer_name_cannot_satisfy_it() {
        let script = blueprint_check(&venv(), Some("unitree-g1"));
        assert!(script.contains("grep -qx '  unitree-g1'"));
        assert!(script.starts_with("set -o pipefail; "));
    }

    #[test]
    fn the_torch_check_swallows_its_own_failure_so_static_tls_never_fails_verify() {
        let torch = &jetson_checks(&venv())[1];
        assert!(torch.contains("-c 'import torch' || echo "));
    }

    #[test]
    fn jetson_target_checks_nv_tegra_release_and_torch_but_not_the_dds_stack() {
        let scripts = scripts(&stages(&Target::Jetson, &venv(), &dir(), None)[0]);
        assert_eq!(scripts.len(), 4);
        assert!(scripts.iter().any(|s| s.contains("/etc/nv_tegra_release")));
        assert!(!scripts.iter().any(|s| s.contains("cyclonedds")));
    }

    #[test]
    fn a_quote_in_a_path_stays_literal_inside_the_login_shell_script() {
        assert_eq!(shq("/home/o'brien/dimos"), r"'/home/o'\''brien/dimos'");
    }

    #[test]
    fn no_check_calls_a_python_outside_the_recorded_venv() {
        for stage in stages(&g1(), &venv(), &dir(), None) {
            for script in scripts(&stage) {
                assert!(!script.contains(" python") && !script.contains("python3"));
            }
        }
    }
}
