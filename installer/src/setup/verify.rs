//! The verify stages. Every check is one `Action::Run` of `bash -lc`, so what it proves is what a
//! user's own login shell gets: the PATH and CYCLONEDDS_HOME rc blocks included.

use std::path::{Path, PathBuf};

use crate::action::Action;
use crate::plan::Stage;
use crate::wizards::nvidia::jetson::LD_PRELOAD_FIX;

/// A cold `import dimos` on an Orin NX is minutes, not seconds.
const CHECK_TIMEOUT_S: u64 = 300;
/// A live G1 publishes LowState every 2 ms; five silent seconds is a dead link or a wrong NIC.
const LOWSTATE_WAIT_S: u32 = 5;
const STAGE_NAMES: [&str; 3] = ["verify-shell", "verify-torch", "verify"];

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Target {
    Host,
    /// The G1 is a Jetson, so its checks are the Jetson ones plus the DDS stack and a live read.
    G1 {
        cyclonedds_home: PathBuf,
        interface: String,
    },
    Jetson,
}

/// What `dimos list` must show when `--blueprint` is absent.
fn default_blueprint(target: &Target) -> Option<&'static str> {
    match target {
        Target::G1 { .. } => Some("unitree-g1"),
        Target::Host | Target::Jetson => None,
    }
}

/// A stage this file built; `update --dry-run` reports them as not run rather than as pending.
pub(crate) fn is_check(name: &str) -> bool {
    STAGE_NAMES.contains(&name)
}

/// The verify stages for a target; the critical one is always last.
pub(crate) fn stages(
    target: &Target,
    venv: &Path,
    dir: &Path,
    blueprint: Option<&str>,
) -> Vec<Stage> {
    let mut out = Vec::new();
    if let Target::G1 {
        cyclonedds_home, ..
    } = target
    {
        out.push(login_shell_stage(cyclonedds_home));
    }
    if !matches!(target, Target::Host) {
        out.push(torch_stage(venv));
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
        .check()
}

/// Non-critical: a login shell that missed the rc block earns a `!!`, not a dead install.
fn login_shell_stage(cyclonedds_home: &Path) -> Stage {
    let script = format!(
        "test \"$CYCLONEDDS_HOME\" = {}",
        shq(&cyclonedds_home.to_string_lossy())
    );
    Stage::new("verify-shell", false)
        .push(Action::run(&["bash", "-lc", &script], CHECK_TIMEOUT_S))
        .check()
}

/// Warn-only: torch absent, or broken by static TLS, is one `!!` line with the fix, never a failure.
fn torch_stage(venv: &Path) -> Stage {
    Stage::new("verify-torch", false)
        .push(Action::run(
            &["bash", "-lc", &py(venv, &torch_code())],
            CHECK_TIMEOUT_S,
        ))
        .check()
        .warn_only()
}

/// The last line python prints is what the runner shows, so the fix goes there.
fn torch_code() -> String {
    format!(
        "import importlib.util, sys\n\
         if importlib.util.find_spec(\"torch\") is None:\n\
         \x20   sys.exit(\"torch: not installed, skipped\")\n\
         try:\n\
         \x20   import torch\n\
         except OSError as e:\n\
         \x20   fix = \"; fix: {LD_PRELOAD_FIX}\" if \"static TLS\" in str(e) else \"\"\n\
         \x20   sys.exit(f\"torch: {{e}}{{fix}}\")"
    )
}

fn checks(target: &Target, venv: &Path, dir: &Path, blueprint: Option<&str>) -> Vec<String> {
    let mut out = vec![py(venv, "import dimos"), blueprint_check(venv, blueprint)];
    match target {
        Target::Host => {}
        Target::Jetson => out.push(tegra_check()),
        Target::G1 { interface, .. } => {
            out.push(tegra_check());
            out.extend(g1_checks(venv, dir, interface));
        }
    }
    out
}

/// `grep -qx` on the two-space list line (info.py:40), so `unitree-g1-basic` cannot pass for
/// `unitree-g1`; the list is captured first, so grep's early exit never breaks the writer's pipe.
fn blueprint_check(venv: &Path, blueprint: Option<&str>) -> String {
    let list = format!("{} list", bin(venv, "dimos"));
    match blueprint {
        Some(name) => format!(
            "out=$({list}) && grep -qx {} <<<\"$out\"",
            shq(&format!("  {name}"))
        ),
        None => format!("{list} > /dev/null"),
    }
}

/// probe_parse::parse_nv_tegra_release owns the real parse; this only re-asserts the shape.
fn tegra_check() -> String {
    "grep -q REVISION: /etc/nv_tegra_release".to_string()
}

fn g1_checks(venv: &Path, dir: &Path, interface: &str) -> Vec<String> {
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
        py(venv, &lowstate_code(interface)),
    ]
}

/// wholebody_connection.py's subscriber held for one sample: DDS, the NIC and a powered robot at once.
fn lowstate_code(interface: &str) -> String {
    format!(
        "import sys; \
         from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber; \
         from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_; \
         ChannelFactoryInitialize(0, \"{interface}\"); \
         s = ChannelSubscriber(\"rt/lowstate\", LowState_); s.Init(None, 0); \
         sys.exit(0 if s.Read({LOWSTATE_WAIT_S}) else \
         \"no rt/lowstate on {interface} in {LOWSTATE_WAIT_S} s: robot off, or wrong --interface\")"
    )
}

/// Belt for the rc block's braces: `import cyclonedds` needs CYCLONEDDS_HOME to find libddsc.
fn cyclonedds_home(target: &Target) -> Option<String> {
    match target {
        Target::G1 {
            cyclonedds_home, ..
        } => Some(cyclonedds_home.to_string_lossy().into_owned()),
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
            interface: "eth0".to_string(),
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

    fn names(stages: &[Stage]) -> Vec<&str> {
        stages.iter().map(|s| s.name).collect()
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
            scripts(&stages[2]),
            vec![
                "'/home/unitree/dimos/.venv/bin/python' -c 'import dimos'",
                "out=$('/home/unitree/dimos/.venv/bin/dimos' list) && grep -qx '  unitree-g1' <<<\"$out\"",
                "grep -q REVISION: /etc/nv_tegra_release",
                "'/home/unitree/dimos/.venv/bin/python' -c 'import cyclonedds, unitree_sdk2py'",
                "'/home/unitree/dimos/.venv/bin/python' -c 'import numpy, sys; sys.exit(int(numpy.__version__.split(\".\")[0]) >= 2)'",
                "grep -q '^ROBOT_IP=' '/home/unitree/dimos/.env'",
                &py(&venv(), &lowstate_code("eth0")),
            ]
        );
    }

    #[test]
    fn g1_stages_are_shell_then_torch_then_the_critical_verify() {
        let stages = stages(&g1(), &venv(), &dir(), None);
        assert_eq!(names(&stages), ["verify-shell", "verify-torch", "verify"]);
        assert!(!stages[0].critical && !stages[1].critical && stages[2].critical);
        assert_eq!(
            scripts(&stages[0]),
            vec!["test \"$CYCLONEDDS_HOME\" = '/home/unitree/cyclonedds/install'".to_string()]
        );
    }

    #[test]
    fn a_g1_verify_ends_with_a_live_lowstate_read_on_the_given_interface() {
        let last = scripts(&stages(&g1(), &venv(), &dir(), None)[2])
            .pop()
            .expect("a last check");
        assert!(
            last.contains("ChannelFactoryInitialize(0, \"eth0\")"),
            "{last}"
        );
        assert!(last.contains("\"rt/lowstate\""), "{last}");
        assert!(last.contains("s.Read(5)"), "{last}");
        assert!(last.contains("wrong --interface"), "{last}");
    }

    #[test]
    fn every_g1_check_carries_cyclonedds_home_so_the_rc_block_is_only_a_belt() {
        let stages = stages(&g1(), &venv(), &dir(), None);
        for action in &stages[2].actions {
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
            for stage in stages(&target, &venv(), &dir(), None) {
                for action in &stage.actions {
                    let Action::Run { env, .. } = action else {
                        panic!("verify plans only Run actions");
                    };
                    assert!(env.is_empty());
                }
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
        assert!(scripts(&stages[2])[1].contains("grep -qx '  unitree-g1-basic' <<<"));
    }

    #[test]
    fn blueprint_check_captures_the_list_before_grep_so_no_writer_sees_a_closed_pipe() {
        let script = blueprint_check(&venv(), Some("unitree-g1"));
        assert!(script.starts_with("out=$("), "{script}");
        assert!(
            script.contains(") && grep -qx '  unitree-g1' <<<\"$out\""),
            "{script}"
        );
        assert!(!script.contains('|'), "{script}");
    }

    #[test]
    fn torch_is_its_own_warn_only_stage_whose_last_line_is_the_ld_preload_fix() {
        let stages = stages(&Target::Jetson, &venv(), &dir(), None);
        let torch = &stages[0];
        assert_eq!(torch.name, "verify-torch");
        assert!(torch.warn_only && !torch.critical);
        let script = &scripts(torch)[0];
        assert!(script.contains("find_spec(\"torch\")"), "{script}");
        assert!(script.contains("\"static TLS\" in str(e)"), "{script}");
        assert!(script.contains(LD_PRELOAD_FIX), "{script}");
        assert!(script.contains("not installed, skipped"), "{script}");
    }

    #[test]
    fn jetson_target_checks_nv_tegra_release_but_not_the_dds_stack() {
        let stages = stages(&Target::Jetson, &venv(), &dir(), None);
        assert_eq!(names(&stages), ["verify-torch", "verify"]);
        let scripts = scripts(&stages[1]);
        assert_eq!(scripts.len(), 3);
        assert!(scripts.iter().any(|s| s.contains("/etc/nv_tegra_release")));
        assert!(!scripts.iter().any(|s| s.contains("cyclonedds")));
    }

    #[test]
    fn is_check_names_exactly_the_stages_this_file_builds() {
        for stage in stages(&g1(), &venv(), &dir(), None) {
            assert!(is_check(stage.name), "{}", stage.name);
        }
        assert!(!is_check("dimos") && !is_check("packages"));
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
