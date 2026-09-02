//! The plan vocabulary: `Action`, `Stage`, `Plan`, and the `Outcome` a stage reports.

use std::path::{Path, PathBuf};

use serde::Serialize;

use crate::action_log::ActionView;

/// The floor for a stage's post-condition; a stage with a slower action lends it that budget.
const POST_TIMEOUT_S: u64 = 120;

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
    pub fn run_owned(
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

    pub fn run(argv: &[&str], timeout_s: u64) -> Action {
        Action::run_owned(owned(argv), false, None, &[], timeout_s)
    }

    pub fn sudo(argv: &[&str], timeout_s: u64) -> Action {
        Action::run_owned(owned(argv), true, None, &[], timeout_s)
    }

    /// A Run with a working directory and environment; never sudo, which would drop the env.
    pub fn run_in(
        argv: &[&str],
        cwd: Option<&Path>,
        env: &[(&str, &str)],
        timeout_s: u64,
    ) -> Action {
        Action::run_owned(owned(argv), false, cwd, env, timeout_s)
    }

    /// The redacted shape written to the action log.
    pub fn view(&self) -> ActionView<'_> {
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
    pub fn display(&self) -> String {
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

pub fn owned(argv: &[&str]) -> Vec<String> {
    argv.iter().map(|a| (*a).to_string()).collect()
}

/// The one spelling of a path as an argv word.
pub fn text(path: &Path) -> String {
    path.display().to_string()
}

#[derive(Debug, Clone, PartialEq)]
pub struct Stage {
    pub name: &'static str,
    pub critical: bool,
    /// Needs `--yes` or one interactive confirm; a sudo action does not imply it, a package install does.
    pub consent: bool,
    /// Empty means the machine is already in the wanted state, reported as `Outcome::Already`.
    pub actions: Vec<Action>,
    /// argv that must exit 0 after the actions; the post-condition, not the verify stage.
    pub post: Option<Vec<String>>,
    /// A failure is one `!!` line carrying the child's last output line: never a stop, never a human.
    pub warn_only: bool,
    /// Successful actions proved health but changed nothing, so the log says `checked`, not `applied`.
    pub check: bool,
}

impl Stage {
    pub fn new(name: &'static str, critical: bool) -> Stage {
        Stage {
            name,
            critical,
            consent: false,
            actions: Vec::new(),
            post: None,
            warn_only: false,
            check: false,
        }
    }

    pub fn push(mut self, action: Action) -> Stage {
        self.actions.push(action);
        self
    }

    pub fn run(self, argv: &[&str], timeout_s: u64) -> Stage {
        self.push(Action::run(argv, timeout_s))
    }

    pub fn sudo(self, argv: &[&str], timeout_s: u64) -> Stage {
        self.push(Action::sudo(argv, timeout_s))
    }

    pub fn post(mut self, argv: &[&str]) -> Stage {
        self.post = Some(owned(argv));
        self
    }

    pub fn consent(mut self) -> Stage {
        self.consent = true;
        self
    }

    pub fn warn_only(mut self) -> Stage {
        self.warn_only = true;
        self
    }

    pub fn check(mut self) -> Stage {
        self.check = true;
        self
    }

    /// The slowest action's budget, so a post-condition is never killed before its own build.
    pub(crate) fn post_timeout_s(&self) -> u64 {
        self.actions
            .iter()
            .filter_map(|a| match a {
                Action::Run { timeout_s, .. } => Some(*timeout_s),
                _ => None,
            })
            .max()
            .map_or(POST_TIMEOUT_S, |slowest| slowest.max(POST_TIMEOUT_S))
    }

    pub fn needs_sudo(&self) -> bool {
        self.actions.iter().any(|a| {
            matches!(
                a,
                Action::Run { sudo: true, .. }
                    | Action::WriteFile { sudo: true, .. }
                    | Action::Remove { sudo: true, .. }
            )
        })
    }
}

#[derive(Debug, Default)]
pub struct Plan {
    pub command: String,
    pub stages: Vec<Stage>,
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    Applied,
    Checked,
    Already,
    DryRun,
    Skipped(String),
    NeedsHuman(String),
    Failed(String),
}

impl Outcome {
    /// The machine is where the stage wanted it, or a dry run said what would get it there.
    pub fn done(&self) -> bool {
        matches!(
            self,
            Outcome::Applied | Outcome::Checked | Outcome::Already | Outcome::DryRun
        )
    }

    pub fn label(&self) -> &'static str {
        match self {
            Outcome::Applied => "applied",
            Outcome::Checked => "checked",
            Outcome::Already => "already",
            Outcome::DryRun => "dry_run",
            Outcome::Skipped(_) => "skipped",
            Outcome::NeedsHuman(_) => "needs_human",
            Outcome::Failed(_) => "failed",
        }
    }
}

/// Replace a stale block in place, append when absent, remove when `lines` is empty.
pub fn ensure_block(text: &str, marker: &str, lines: &[String]) -> (String, bool) {
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
    fn applied_checked_already_and_dry_run_count_as_done() {
        assert!(
            Outcome::Applied.done()
                && Outcome::Checked.done()
                && Outcome::Already.done()
                && Outcome::DryRun.done()
        );
        assert!(!Outcome::Skipped("s".into()).done());
        assert!(!Outcome::NeedsHuman("h".into()).done());
        assert!(!Outcome::Failed("f".into()).done());
    }

    #[test]
    fn a_post_condition_borrows_the_slowest_actions_budget_or_the_floor() {
        assert_eq!(Stage::new("s", true).post_timeout_s(), POST_TIMEOUT_S);
        assert_eq!(
            Stage::new("s", true).run(&["true"], 10).post_timeout_s(),
            POST_TIMEOUT_S
        );
        assert_eq!(
            Stage::new("s", true).run(&["true"], 3600).post_timeout_s(),
            3600
        );
    }

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
