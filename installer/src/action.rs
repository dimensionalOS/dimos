//! `Action` — the one thing a stage can do — and the text it renders as in a plan and a log.

use std::path::{Path, PathBuf};

use crate::action_log::ActionView;

#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// argv spawned directly, never a shell; `sudo` routes through `Ctx.sudo`; killed at timeout_s.
    Run {
        argv: Vec<String>,
        sudo: bool,
        cwd: Option<PathBuf>,
        env: Vec<(String, String)>,
        timeout_s: u64,
    },
    /// Noop when the file already holds `contents`; with sudo, write $TMPDIR then `install -m`.
    WriteFile {
        path: PathBuf,
        mode: u32,
        contents: String,
        sudo: bool,
    },
    /// A `# DIMOS-ADDED: <marker>` block in a per-user rc file; empty `lines` removes it.
    EnsureBlock {
        file: PathBuf,
        marker: String,
        lines: Vec<String>,
    },
    Copy {
        from: PathBuf,
        to: PathBuf,
        mode: u32,
    },
    Rename {
        from: PathBuf,
        to: PathBuf,
    },
    Remove {
        path: PathBuf,
        sudo: bool,
    },
    VerifySha256 {
        file: PathBuf,
        sums_file: PathBuf,
    },
}

impl Action {
    /// The one Run constructor; `run`, `sudo` and `run_in` are its borrowed spellings.
    pub(crate) fn run_owned(
        argv: Vec<String>,
        sudo: bool,
        cwd: Option<&Path>,
        env: &[(&str, &str)],
        timeout_s: u64,
    ) -> Action {
        Action::Run {
            argv,
            sudo,
            cwd: cwd.map(Path::to_path_buf),
            env: env
                .iter()
                .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
                .collect(),
            timeout_s,
        }
    }

    pub(crate) fn run(argv: &[&str], timeout_s: u64) -> Action {
        Action::run_owned(owned(argv), false, None, &[], timeout_s)
    }

    pub(crate) fn sudo(argv: &[&str], timeout_s: u64) -> Action {
        Action::run_owned(owned(argv), true, None, &[], timeout_s)
    }

    /// A Run with a working directory and environment; never sudo, which would drop the env.
    pub(crate) fn run_in(
        argv: &[&str],
        cwd: Option<&Path>,
        env: &[(&str, &str)],
        timeout_s: u64,
    ) -> Action {
        Action::run_owned(owned(argv), false, cwd, env, timeout_s)
    }

    /// The redacted shape written to the action log.
    pub(crate) fn view(&self) -> ActionView<'_> {
        match self {
            Action::Run {
                argv,
                sudo,
                cwd,
                env,
                timeout_s,
            } => view_run(argv, *sudo, cwd.as_deref(), env, *timeout_s),
            Action::WriteFile {
                path,
                mode,
                contents,
                sudo,
            } => view_write(path, *mode, contents.len(), *sudo),
            Action::EnsureBlock {
                file,
                marker,
                lines,
            } => view_block(file, marker, !lines.is_empty()),
            Action::Copy { from, to, .. } => ActionView::Copy { from, to },
            Action::Rename { from, to } => ActionView::Rename { from, to },
            Action::Remove { path, sudo } => ActionView::Remove { path, sudo: *sudo },
            Action::VerifySha256 { file, .. } => ActionView::VerifySha256 { file },
        }
    }

    /// One line for `--dry-run` and the consent prompt; a sudo password is added at exec, not here.
    pub(crate) fn display(&self) -> String {
        match self {
            Action::Run {
                argv,
                sudo,
                cwd,
                env,
                ..
            } => display_run(argv, *sudo, cwd.as_deref(), env),
            Action::WriteFile {
                path,
                mode,
                contents,
                sudo,
            } => display_write(path, *mode, contents.len(), *sudo),
            Action::EnsureBlock {
                file,
                marker,
                lines,
            } => display_block(file, marker, lines.is_empty()),
            Action::Copy { from, to, .. } => display_move("copy", from, to),
            Action::Rename { from, to } => display_move("rename", from, to),
            Action::Remove { path, sudo } => {
                format!("{}remove {}", sudo_word(*sudo), path.display())
            }
            Action::VerifySha256 { file, sums_file } => {
                format!("sha256 {} against {}", file.display(), sums_file.display())
            }
        }
    }
}

fn view_run<'a>(
    argv: &'a [String],
    sudo: bool,
    cwd: Option<&'a Path>,
    env: &'a [(String, String)],
    timeout_s: u64,
) -> ActionView<'a> {
    ActionView::Run {
        argv,
        sudo,
        cwd,
        env_keys: env.iter().map(|(k, _)| k.as_str()).collect(),
        timeout_s,
    }
}

fn view_write(path: &Path, mode: u32, bytes: usize, sudo: bool) -> ActionView<'_> {
    ActionView::WriteFile {
        path,
        mode,
        bytes,
        sudo,
    }
}

fn view_block<'a>(file: &'a Path, marker: &'a str, present: bool) -> ActionView<'a> {
    ActionView::EnsureBlock {
        file,
        marker,
        present,
    }
}

fn display_write(path: &Path, mode: u32, bytes: usize, sudo: bool) -> String {
    format!(
        "{}write {} ({bytes} bytes, mode {mode:o})",
        sudo_word(sudo),
        path.display()
    )
}

fn display_block(file: &Path, marker: &str, removing: bool) -> String {
    if removing {
        format!("remove the {marker} block from {}", file.display())
    } else {
        format!("keep the {marker} block in {}", file.display())
    }
}

fn display_move(verb: &str, from: &Path, to: &Path) -> String {
    format!("{verb} {} -> {}", from.display(), to.display())
}

fn sudo_word(sudo: bool) -> &'static str {
    if sudo {
        "sudo "
    } else {
        ""
    }
}

fn display_run(
    argv: &[String],
    sudo: bool,
    cwd: Option<&Path>,
    env: &[(String, String)],
) -> String {
    let prefix = if sudo { "sudo " } else { "" };
    let vars: String = env.iter().map(|(k, v)| format!("{k}={v} ")).collect();
    let dir = cwd.map_or(String::new(), |d| format!("   (in {})", d.display()));
    format!("{prefix}{vars}{}{dir}", display_argv(argv))
}

pub(crate) fn display_argv(argv: &[String]) -> String {
    argv.iter()
        .map(|a| {
            if a.contains(' ') {
                format!("'{a}'")
            } else {
                a.clone()
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

pub(crate) fn owned(argv: &[&str]) -> Vec<String> {
    argv.iter().map(|a| (*a).to_string()).collect()
}

/// The one spelling of a path as an argv word.
pub(crate) fn text(path: &Path) -> String {
    path.display().to_string()
}

/// Replace a stale block in place, append when absent, remove when `lines` is empty.
pub(crate) fn ensure_block(text: &str, marker: &str, lines: &[String]) -> (String, bool) {
    let start = format!("# DIMOS-ADDED: {marker} start");
    let end = format!("# DIMOS-ADDED: {marker} end");
    let block = if lines.is_empty() {
        String::new()
    } else {
        format!("{start}\n{}\n{end}\n", lines.join("\n"))
    };
    let out = match block_span(text, &start, &end) {
        Some(span) => format!("{}{block}{}", &text[..span.start], &text[span.end..]),
        None if block.is_empty() => return (text.to_string(), false),
        None if text.is_empty() || text.ends_with('\n') => format!("{text}{block}"),
        None => format!("{text}\n{block}"),
    };
    let changed = out != text;
    (out, changed)
}

fn block_span(text: &str, start: &str, end: &str) -> Option<std::ops::Range<usize>> {
    let from = text.find(start)?;
    let after_end = text[from..].find(end).map(|i| from + i + end.len())?;
    let to = text[after_end..]
        .find('\n')
        .map_or(text.len(), |i| after_end + i + 1);
    Some(from..to)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sudo::Sudo;

    #[test]
    fn ensure_block_appends_once_replaces_stale_in_place_and_removes_when_empty() {
        let line = |s: &str| vec![s.to_string()];
        let (once, changed) = ensure_block("export A=1\n", "path", &line("export B=2"));
        assert!(changed);
        assert_eq!(
            once,
            "export A=1\n# DIMOS-ADDED: path start\nexport B=2\n# DIMOS-ADDED: path end\n"
        );
        assert!(!ensure_block(&once, "path", &line("export B=2")).1);
        let (fresh, changed) = ensure_block(&once, "path", &line("export B=3"));
        assert!(changed && fresh.matches("DIMOS-ADDED").count() == 2);
        assert!(fresh.starts_with("export A=1\n") && fresh.contains("export B=3"));
        let (gone, changed) = ensure_block(&fresh, "path", &[]);
        assert!(changed);
        assert_eq!(gone, "export A=1\n");
    }

    #[test]
    fn ensure_block_leaves_a_file_without_the_block_alone_when_removing() {
        assert_eq!(
            ensure_block("export A=1\n", "path", &[]),
            ("export A=1\n".to_string(), false)
        );
    }

    #[test]
    fn display_never_shows_stdin_password() {
        let action = Action::sudo(&["apt-get", "install", "-y", "git"], 60);
        let line = action.display();
        assert_eq!(line, "sudo apt-get install -y git");
        let (argv, stdin) =
            Sudo::Stdin(crate::sudo::Secret::new("hunter2".into())).wrap(&["apt-get".to_string()]);
        assert!(!line.contains("hunter2") && !argv.join(" ").contains("hunter2"));
        assert!(stdin.is_some());
    }
}
