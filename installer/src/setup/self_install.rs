//! Stage: put this binary at `~/.local/bin/dimos` and make a fresh login shell able to find it.

use std::path::Path;

use crate::plan::{Action, Stage};
use crate::probe::Tools;
use crate::state;

pub const PATH_MARKER: &str = "path";

/// The line a login shell needs before `dimos` resolves in a new terminal.
pub fn path_lines() -> Vec<String> {
    vec!["export PATH=\"$HOME/.local/bin:$PATH\"".to_string()]
}

/// Install this binary and put `~/.local/bin` on PATH; empty when both already hold.
pub fn stage(
    current_exe: &Path,
    home: &Path,
    tools: &Tools,
    installed_sha: Option<&str>,
    own_sha: &str,
) -> Stage {
    let dest = state::installed_bin(home);
    let mut stage = Stage::new("self-install", false);
    for action in install_actions(current_exe, &dest, installed_sha, own_sha) {
        stage = stage.push(action);
    }
    if !tools.login_path_has_local_bin {
        for file in state::rc_files(home) {
            stage = stage.push(Action::EnsureBlock {
                file,
                marker: PATH_MARKER.to_string(),
                lines: path_lines(),
            });
        }
    }
    stage.post(&[&dest.to_string_lossy(), "--version"])
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
    use crate::state::TmpDir;
    use std::fs;
    use std::path::PathBuf;

    fn tools(login_path_has_local_bin: bool) -> Tools {
        Tools {
            login_path_has_local_bin,
            ..Tools::default()
        }
    }

    #[test]
    fn same_binary_already_installed_is_empty_stage() {
        let tmp = TmpDir::new("self-same");
        let home = tmp.path();
        let dest = state::installed_bin(home);
        let by_path = stage(&dest, home, &tools(true), Some("aa"), "aa");
        let by_sha = stage(
            Path::new("/tmp/dimos"),
            home,
            &tools(true),
            Some("aa"),
            "aa",
        );
        assert!(by_path.actions.is_empty());
        assert!(by_sha.actions.is_empty());
    }

    #[test]
    fn different_sha_copies_with_mode_755() {
        let tmp = TmpDir::new("self-copy");
        let home = tmp.path();
        let s = stage(
            Path::new("/tmp/dimos"),
            home,
            &tools(true),
            Some("old"),
            "new",
        );
        assert_eq!(
            s.actions.last(),
            Some(&Action::Copy {
                from: PathBuf::from("/tmp/dimos"),
                to: state::installed_bin(home),
                mode: 0o755,
            })
        );
    }

    #[test]
    fn path_block_only_when_login_path_lacks_local_bin() {
        let tmp = TmpDir::new("self-path");
        let home = tmp.path();
        fs::write(home.join(".bashrc"), "").expect("write rc fixture");
        let installed = state::installed_bin(home);
        let on_path = stage(&installed, home, &tools(true), Some("aa"), "aa");
        let off_path = stage(&installed, home, &tools(false), Some("aa"), "aa");
        assert!(on_path.actions.is_empty());
        assert_eq!(
            off_path.actions,
            vec![Action::EnsureBlock {
                file: home.join(".bashrc"),
                marker: PATH_MARKER.to_string(),
                lines: path_lines(),
            }]
        );
    }

    #[test]
    fn actions_never_touch_etc() {
        let tmp = TmpDir::new("self-etc");
        let home = tmp.path();
        fs::write(home.join(".zshrc"), "").expect("write rc fixture");
        let s = stage(Path::new("/tmp/dimos"), home, &tools(false), None, "new");
        assert!(!s.needs_sudo());
        for action in &s.actions {
            let written = match action {
                Action::EnsureBlock { file, .. } => file.clone(),
                Action::Copy { to, .. } => to.clone(),
                other => panic!("unexpected action {}", other.display()),
            };
            assert!(written.starts_with(home), "{}", written.display());
        }
    }

    #[test]
    fn post_condition_runs_the_installed_binary() {
        let tmp = TmpDir::new("self-post");
        let home = tmp.path();
        let s = stage(Path::new("/tmp/dimos"), home, &tools(true), None, "new");
        assert_eq!(
            s.post,
            Some(vec![
                state::installed_bin(home).display().to_string(),
                "--version".to_string(),
            ])
        );
    }
}
