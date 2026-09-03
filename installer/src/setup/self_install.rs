//! Stage: put this binary at `~/.local/bin/dimos` and make a fresh login shell able to find it.

use std::path::Path;

use crate::action::{text, Action};
use crate::install_record;
use crate::plan::Stage;
use crate::probe::{RcFile, Tools};

pub(crate) const PATH_MARKER: &str = "path";

/// The line a login shell needs before `dimos` resolves in a new terminal.
fn path_lines() -> Vec<String> {
    vec!["export PATH=\"$HOME/.local/bin:$PATH\"".to_string()]
}

/// Install this binary and put `~/.local/bin` on PATH; empty when both already hold.
pub(crate) fn stage(
    current_exe: &Path,
    home: &Path,
    tools: &Tools,
    rc: &[RcFile],
    installed_sha: Option<&str>,
    own_sha: &str,
) -> Stage {
    let dest = install_record::installed_bin(home);
    let mut stage = Stage::new("self-install", false);
    for action in install_actions(current_exe, &dest, installed_sha, own_sha) {
        stage = stage.push(action);
    }
    if !tools.login_path_has_local_bin {
        for file in rc {
            stage = stage.push(Action::EnsureBlock {
                file: file.path.clone(),
                marker: PATH_MARKER.to_string(),
                lines: path_lines(),
            });
        }
    }
    stage.post(&[&text(&dest), "--version"])
}

/// Empty when the destination already holds this exact binary.
fn install_actions(
    current_exe: &Path,
    dest: &Path,
    installed_sha: Option<&str>,
    own_sha: &str,
) -> Vec<Action> {
    if current_exe == dest || installed_sha == Some(own_sha) {
        return Vec::new();
    }
    vec![Action::Copy {
        from: current_exe.to_path_buf(),
        to: dest.to_path_buf(),
        mode: 0o755,
    }]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    const HOME: &str = "/home/u";

    fn home() -> &'static Path {
        Path::new(HOME)
    }

    fn tools(login_path_has_local_bin: bool) -> Tools {
        Tools {
            login_path_has_local_bin,
            ..Tools::default()
        }
    }

    fn rc(name: &str) -> Vec<RcFile> {
        vec![RcFile {
            path: home().join(name),
            text: String::new(),
        }]
    }

    #[test]
    fn same_binary_already_installed_is_empty_stage() {
        let dest = install_record::installed_bin(home());
        let by_path = stage(
            &dest,
            home(),
            &tools(true),
            &rc(".bashrc"),
            Some("aa"),
            "aa",
        );
        let by_sha = stage(
            Path::new("/tmp/dimos"),
            home(),
            &tools(true),
            &rc(".bashrc"),
            Some("aa"),
            "aa",
        );
        assert!(by_path.actions.is_empty());
        assert!(by_sha.actions.is_empty());
    }

    #[test]
    fn different_sha_copies_with_mode_755() {
        let s = stage(
            Path::new("/tmp/dimos"),
            home(),
            &tools(true),
            &rc(".bashrc"),
            Some("old"),
            "new",
        );
        assert_eq!(
            s.actions.last(),
            Some(&Action::Copy {
                from: PathBuf::from("/tmp/dimos"),
                to: install_record::installed_bin(home()),
                mode: 0o755,
            })
        );
    }

    #[test]
    fn path_block_only_when_login_path_lacks_local_bin() {
        let installed = install_record::installed_bin(home());
        let on_path = stage(
            &installed,
            home(),
            &tools(true),
            &rc(".bashrc"),
            Some("aa"),
            "aa",
        );
        let off_path = stage(
            &installed,
            home(),
            &tools(false),
            &rc(".bashrc"),
            Some("aa"),
            "aa",
        );
        assert!(on_path.actions.is_empty());
        assert_eq!(
            off_path.actions,
            vec![Action::EnsureBlock {
                file: home().join(".bashrc"),
                marker: PATH_MARKER.to_string(),
                lines: path_lines(),
            }]
        );
    }

    #[test]
    fn actions_never_touch_etc() {
        let s = stage(
            Path::new("/tmp/dimos"),
            home(),
            &tools(false),
            &rc(".zprofile"),
            None,
            "new",
        );
        assert!(!s.needs_sudo());
        for action in &s.actions {
            let written = match action {
                Action::EnsureBlock { file, .. } => file.clone(),
                Action::Copy { to, .. } => to.clone(),
                other => panic!("unexpected action {}", other.display()),
            };
            assert!(written.starts_with(home()), "{}", written.display());
        }
    }

    #[test]
    fn post_condition_runs_the_installed_binary() {
        let s = stage(
            Path::new("/tmp/dimos"),
            home(),
            &tools(true),
            &rc(".bashrc"),
            None,
            "new",
        );
        assert_eq!(
            s.post,
            Some(vec![
                install_record::installed_bin(home()).display().to_string(),
                "--version".to_string(),
            ])
        );
    }
}
