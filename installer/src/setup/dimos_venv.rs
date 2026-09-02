//! The DimOS install stage: a version-pinned PyPI install (library) or a git checkout plus
//! `uv sync` (dev). Pure over `DirState`, which is the one probe this file performs.

use std::fs;
use std::path::Path;

use anyhow::{bail, Result};

use crate::cli::InstallMode;
use crate::install_record;
use crate::plan::{text, Action, Stage};
use crate::platforms::{self, DIMOS_VERSION};

const REPO_URL: &str = "https://github.com/dimensionalOS/dimos";
const PYTHON_VERSION: &str = "3.12";
const MKDIR_TIMEOUT_S: u64 = 30;
const CLONE_TIMEOUT_S: u64 = 900;
const VENV_TIMEOUT_S: u64 = 300;
/// A cold sync on a Jetson builds wheels from source; an hour is the observed worst case.
const INSTALL_TIMEOUT_S: u64 = 3600;

#[derive(Debug, Clone, PartialEq)]
pub enum DirState {
    Absent,
    Clone { branch: Option<String> },
    NotAClone,
}

/// The only I/O in this file: `.git/HEAD` decides clone vs not; an empty directory still clones.
pub fn dir_state(dir: &Path) -> DirState {
    match fs::read_to_string(dir.join(".git/HEAD")) {
        Ok(head) => DirState::Clone {
            branch: head_branch(&head),
        },
        Err(_) if is_free(dir) => DirState::Absent,
        Err(_) => DirState::NotAClone,
    }
}

fn head_branch(head: &str) -> Option<String> {
    head.trim()
        .strip_prefix("ref: refs/heads/")
        .map(str::to_string)
}

fn is_free(dir: &Path) -> bool {
    match fs::read_dir(dir) {
        Ok(mut entries) => entries.next().is_none(),
        Err(_) => true, // absent, or unreadable: git clone reports the real reason
    }
}

/// A clone on another branch is left alone; the operator is told, never switched under them.
pub fn branch_note(state: &DirState, wanted: &str, dir: &Path) -> Option<String> {
    let DirState::Clone { branch: Some(on) } = state else {
        return None;
    };
    if on == wanted {
        return None;
    }
    let path = text(dir);
    Some(format!(
        "{path} is on branch {on}, not {wanted}: left as is; \
         `git -C {path} checkout {wanted}` switches it"
    ))
}

/// What installer.json records: a PyPI pin, or the branch and commit a dev checkout sits on.
pub fn dimos_version_string(mode: InstallMode, branch: &str, git_rev: Option<&str>) -> String {
    match (mode, git_rev) {
        (InstallMode::Library, _) => DIMOS_VERSION.to_string(),
        (InstallMode::Dev, Some(rev)) => format!("git:{branch}@{rev}"),
        (InstallMode::Dev, None) => format!("git:{branch}"),
    }
}

/// The critical install stage; the verify stage, with its own budget, proves the import.
pub fn dimos_stage(
    mode: InstallMode,
    dir: &Path,
    extras: &[String],
    branch: &str,
    with_nix: bool,
    uv: &Path,
    state: &DirState,
) -> Result<Stage> {
    let actions = match mode {
        InstallMode::Library => library_actions(dir, extras, with_nix, uv),
        InstallMode::Dev => dev_actions(dir, extras, branch, with_nix, uv, state)?,
    };
    Ok(actions
        .into_iter()
        .fold(Stage::new("dimos", true), Stage::push))
}

fn library_actions(dir: &Path, extras: &[String], with_nix: bool, uv: &Path) -> Vec<Action> {
    let install = nix_wrap(
        with_nix,
        dir,
        vec![
            text(uv),
            "pip".into(),
            "install".into(),
            "--python".into(),
            text(&install_record::venv_python(dir)),
            platforms::pip_spec(extras),
        ],
    );
    vec![
        Action::run(&["mkdir", "-p", &text(dir)], MKDIR_TIMEOUT_S),
        Action::run_owned(venv_argv(uv, dir), false, None, &[], VENV_TIMEOUT_S),
        Action::run_owned(install, false, Some(dir), &[], INSTALL_TIMEOUT_S),
    ]
}

/// `--allow-existing`: a bare `uv venv` deletes the venv, taking a hand-built cyclonedds with it.
fn venv_argv(uv: &Path, dir: &Path) -> Vec<String> {
    vec![
        text(uv),
        "venv".into(),
        "--python".into(),
        PYTHON_VERSION.into(),
        "--allow-existing".into(),
        text(&install_record::venv(dir)),
    ]
}

fn dev_actions(
    dir: &Path,
    extras: &[String],
    branch: &str,
    with_nix: bool,
    uv: &Path,
    state: &DirState,
) -> Result<Vec<Action>> {
    if *state == DirState::NotAClone {
        bail!(
            "{} exists and is not a git checkout: pass --dir <new path>, or move it aside",
            text(dir)
        );
    }
    let mut actions = Vec::new();
    if *state == DirState::Absent {
        actions.push(clone_action(branch, dir));
    }
    actions.push(sync_action(dir, extras, with_nix, uv));
    Ok(actions)
}

fn clone_action(branch: &str, dir: &Path) -> Action {
    let argv = vec![
        "git".into(),
        "-c".into(),
        "filter.lfs.process=".into(),
        "-c".into(),
        "filter.lfs.smudge=".into(),
        "-c".into(),
        "filter.lfs.required=false".into(),
        "clone".into(),
        "-b".into(),
        branch.into(),
        "--single-branch".into(),
        REPO_URL.into(),
        text(dir),
    ];
    Action::run_owned(argv, false, None, &[], CLONE_TIMEOUT_S)
}

/// `--inexact` leaves packages the lockfile does not name, so a hand-installed SDK survives.
fn sync_action(dir: &Path, extras: &[String], with_nix: bool, uv: &Path) -> Action {
    let mut argv = vec![text(uv), "sync".into(), "--inexact".into()];
    argv.extend(platforms::sync_args(extras));
    Action::run_owned(
        nix_wrap(with_nix, dir, argv),
        false,
        Some(dir),
        &[("GIT_LFS_SKIP_SMUDGE", "1"), ("UV_PYTHON", PYTHON_VERSION)],
        INSTALL_TIMEOUT_S,
    )
}

/// `nix develop` supplies the system libraries (libGL, mesa) the wheels link against.
fn nix_wrap(with_nix: bool, dir: &Path, argv: Vec<String>) -> Vec<String> {
    if !with_nix {
        return argv;
    }
    let mut wrapped = vec![
        "nix".into(),
        "develop".into(),
        text(dir),
        "--command".into(),
    ];
    wrapped.extend(argv);
    wrapped
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::install_record::TmpDir;
    use std::path::PathBuf;

    fn extras() -> Vec<String> {
        vec!["base".to_string()]
    }

    fn uv() -> PathBuf {
        PathBuf::from("/home/u/.local/bin/uv")
    }

    fn stage(mode: InstallMode, state: &DirState, with_nix: bool) -> Stage {
        dimos_stage(
            mode,
            Path::new("/home/u/dimos-app"),
            &extras(),
            "main",
            with_nix,
            &uv(),
            state,
        )
        .expect("fixture plans build")
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

    #[test]
    fn library_plan_pins_dimos_version_from_pyproject() {
        let argv = argvs(&stage(InstallMode::Library, &DirState::Absent, false));
        assert_eq!(
            argv[2].last().map(String::as_str),
            Some(format!("dimos[base]=={DIMOS_VERSION}").as_str())
        );
        assert!(argv[2].contains(&"--python".to_string()));
    }

    #[test]
    fn library_venv_allows_existing_so_a_rerun_keeps_a_hand_built_venv() {
        let argv = argvs(&stage(InstallMode::Library, &DirState::Absent, false));
        assert_eq!(
            argv[1],
            [
                "/home/u/.local/bin/uv",
                "venv",
                "--python",
                "3.12",
                "--allow-existing",
                "/home/u/dimos-app/.venv"
            ]
        );
    }

    #[test]
    fn dev_fresh_clone_uses_branch_single_branch_without_needing_git_lfs() {
        let stage = dimos_stage(
            InstallMode::Dev,
            Path::new("/home/u/dimos"),
            &extras(),
            "aaryan/installer",
            false,
            &uv(),
            &DirState::Absent,
        )
        .unwrap();
        let Action::Run { argv, env, .. } = &stage.actions[0] else {
            panic!("the first dev action is the clone");
        };
        assert_eq!(
            argv,
            &[
                "git",
                "-c",
                "filter.lfs.process=",
                "-c",
                "filter.lfs.smudge=",
                "-c",
                "filter.lfs.required=false",
                "clone",
                "-b",
                "aaryan/installer",
                "--single-branch",
                REPO_URL,
                "/home/u/dimos"
            ]
        );
        assert!(env.is_empty());
    }

    #[test]
    fn dev_existing_clone_is_not_touched_only_synced() {
        let state = DirState::Clone {
            branch: Some("main".into()),
        };
        let argv = argvs(&stage(InstallMode::Dev, &state, false));
        assert_eq!(argv.len(), 1);
        assert!(!argv[0].contains(&"clone".to_string()));
    }

    #[test]
    fn dev_sync_uses_inexact_so_g1_sdk_survives_rerun() {
        let argv = argvs(&stage(InstallMode::Dev, &DirState::Absent, false));
        assert_eq!(
            argv[1],
            [
                "/home/u/.local/bin/uv",
                "sync",
                "--inexact",
                "--extra",
                "base"
            ]
        );
    }

    #[test]
    fn non_clone_dir_is_an_error_naming_the_path() {
        let err = dimos_stage(
            InstallMode::Dev,
            Path::new("/home/u/dimos-app"),
            &extras(),
            "main",
            false,
            &uv(),
            &DirState::NotAClone,
        )
        .expect_err("a populated non-checkout must stop the plan");
        assert!(err.to_string().starts_with("/home/u/dimos-app exists"));
        assert!(err.to_string().contains("--dir"));
    }

    #[test]
    fn nix_wraps_only_the_uv_step_and_leaves_the_clone_bare() {
        let argv = argvs(&stage(InstallMode::Dev, &DirState::Absent, true));
        assert_eq!(argv[0][0], "git");
        assert_eq!(
            argv[1][..4],
            ["nix", "develop", "/home/u/dimos-app", "--command"]
        );
    }

    #[test]
    fn the_stage_is_critical_and_leaves_the_import_check_to_the_verify_stage() {
        let stage = stage(InstallMode::Library, &DirState::Absent, false);
        assert!(stage.critical);
        assert_eq!(stage.post, None);
    }

    #[test]
    fn dir_state_reads_the_branch_from_head_and_calls_an_empty_dir_absent() {
        let tmp = TmpDir::new("install-dirstate");
        let clone = tmp.path().join("clone");
        fs::create_dir_all(clone.join(".git")).unwrap();
        fs::write(
            clone.join(".git/HEAD"),
            "ref: refs/heads/aaryan/installer\n",
        )
        .unwrap();
        assert_eq!(
            dir_state(&clone),
            DirState::Clone {
                branch: Some("aaryan/installer".into())
            }
        );

        let empty = tmp.path().join("empty");
        fs::create_dir_all(&empty).unwrap();
        assert_eq!(dir_state(&empty), DirState::Absent);
        assert_eq!(dir_state(&tmp.path().join("nothing")), DirState::Absent);
    }

    #[test]
    fn a_populated_directory_without_git_is_not_a_clone() {
        let tmp = TmpDir::new("install-notaclone");
        let dir = tmp.path().join("stuff");
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("notes.txt"), "hi").unwrap();
        assert_eq!(dir_state(&dir), DirState::NotAClone);
    }

    #[test]
    fn a_detached_head_reports_no_branch_so_no_mismatch_note_fires() {
        assert_eq!(head_branch("9f2c1ab0\n"), None);
        let detached = DirState::Clone { branch: None };
        assert_eq!(branch_note(&detached, "main", Path::new("/d")), None);
    }

    #[test]
    fn branch_note_names_the_checkout_command_when_the_clone_is_elsewhere() {
        let state = DirState::Clone {
            branch: Some("main".into()),
        };
        let note = branch_note(&state, "aaryan/installer", Path::new("/home/u/dimos")).unwrap();
        assert_eq!(
            note,
            "/home/u/dimos is on branch main, not aaryan/installer: left as is; \
             `git -C /home/u/dimos checkout aaryan/installer` switches it"
        );
        assert_eq!(
            branch_note(&state, "main", Path::new("/home/u/dimos")),
            None
        );
    }

    #[test]
    fn dev_version_string_carries_branch_and_rev_while_library_carries_the_pin() {
        assert_eq!(
            dimos_version_string(InstallMode::Library, "main", Some("9f2c1ab")),
            DIMOS_VERSION
        );
        assert_eq!(
            dimos_version_string(InstallMode::Dev, "aaryan/installer", Some("9f2c1ab")),
            "git:aaryan/installer@9f2c1ab"
        );
        assert_eq!(
            dimos_version_string(InstallMode::Dev, "main", None),
            "git:main"
        );
    }
}
