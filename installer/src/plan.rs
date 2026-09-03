//! The shape of a run: `Stage`, the `Plan` that lists them, and the `Outcome` a stage reports.

use serde::Serialize;

use crate::action::{owned, Action};

/// The floor for a stage's post-condition; a stage with a slower action lends it that budget.
const POST_TIMEOUT_S: u64 = 120;

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

    pub(crate) fn run(self, argv: &[&str], timeout_s: u64) -> Stage {
        self.push(Action::run(argv, timeout_s))
    }

    pub(crate) fn sudo(self, argv: &[&str], timeout_s: u64) -> Stage {
        self.push(Action::sudo(argv, timeout_s))
    }

    pub(crate) fn post(mut self, argv: &[&str]) -> Stage {
        self.post = Some(owned(argv));
        self
    }

    pub(crate) fn consent(mut self) -> Stage {
        self.consent = true;
        self
    }

    pub(crate) fn warn_only(mut self) -> Stage {
        self.warn_only = true;
        self
    }

    pub(crate) fn check(mut self) -> Stage {
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

    pub(crate) fn needs_sudo(&self) -> bool {
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
    pub(crate) fn done(&self) -> bool {
        matches!(
            self,
            Outcome::Applied | Outcome::Checked | Outcome::Already | Outcome::DryRun
        )
    }

    pub(crate) fn label(&self) -> &'static str {
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
