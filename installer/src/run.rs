//! The executor: `run` walks a plan's stages, gates each one, and reports what the machine did.

use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use serde::Serialize;

use crate::action::{display_argv, Action};
use crate::action_log::{ActionRecord, ActionView};
use crate::file_actions::exec_action;
use crate::install_record;
use crate::plan::{Outcome, Plan, Stage};
use crate::run_context::{open_shell, Ctx, Mode};
use crate::say;
use crate::spawn::run_argv;

const HELP_URL: &str = "https://github.com/dimensionalOS/dimos/issues";

#[derive(Debug, Serialize)]
pub struct Report {
    pub command: String,
    pub stages: Vec<(String, Outcome)>,
}

impl Report {
    /// 1 when a critical stage failed; else 2 when anything needs a human; else 0.
    pub fn exit_code(&self) -> i32 {
        if self
            .stages
            .iter()
            .any(|(_, o)| matches!(o, Outcome::Failed(_)))
        {
            return 1;
        }
        if !self.needs_human().is_empty() {
            return 2;
        }
        0
    }

    pub fn needs_human(&self) -> Vec<String> {
        self.stages
            .iter()
            .filter_map(|(_, o)| match o {
                Outcome::NeedsHuman(why) => Some(why.clone()),
                _ => None,
            })
            .collect()
    }

    pub fn print(&self, ctx: &Ctx) {
        if ctx.mode == Mode::Agent {
            println!("{}", serde_json::to_string(self).unwrap_or_default());
            return;
        }
        for item in self.needs_human() {
            say::warn(&item);
        }
        match self.exit_code() {
            0 => say::ok(&format!("{} done", self.command)),
            1 => say::fail(&format!("{} failed", self.command)),
            _ => say::warn(&format!("{} needs a human", self.command)),
        }
    }
}

/// sudo's env_reset drops an Action's env, so a sudo Run must carry its pairs in argv (`env K=V ...`).
pub fn sudo_env_violations(plan: &Plan) -> Vec<String> {
    plan.stages
        .iter()
        .flat_map(|stage| stage.actions.iter().map(move |a| (stage.name, a)))
        .filter_map(|(name, action)| match action {
            Action::Run {
                argv,
                sudo: true,
                env,
                ..
            } if !env.is_empty() => Some(format!("{name}: {}", display_argv(argv))),
            _ => None,
        })
        .collect()
}

pub fn run(plan: &Plan, ctx: &mut Ctx) -> Result<Report> {
    let leaky = sudo_env_violations(plan);
    if !leaky.is_empty() {
        bail!(
            "sudo resets the environment, so these steps would lose theirs: {}\n  \
             put the pairs in argv instead: [\"env\", \"K=V\", \"<cmd>\", ...]",
            leaky.join(", ")
        );
    }
    say::print_plan(plan, ctx);
    if !ctx.dry_run && plan.stages.iter().any(Stage::needs_sudo) {
        ctx.sudo.refresh_or_demote();
    }
    let mut stages = Vec::new();
    for stage in &plan.stages {
        let outcome = exec_stage(stage, ctx, &plan.command);
        let stop = stage.critical && !outcome.done();
        stages.push((stage.name.to_string(), outcome));
        if stop {
            break;
        }
    }
    Ok(Report {
        command: plan.command.clone(),
        stages,
    })
}

fn exec_stage(stage: &Stage, ctx: &mut Ctx, command: &str) -> Outcome {
    let started = Instant::now();
    let outcome = decide(stage, ctx, command);
    log(
        ctx,
        command,
        stage.name,
        None,
        &outcome,
        None,
        started.elapsed(),
    );
    say::report_stage(stage.name, &outcome);
    outcome
}

fn decide(stage: &Stage, ctx: &mut Ctx, command: &str) -> Outcome {
    if stage.actions.is_empty() {
        return Outcome::Already;
    }
    if ctx.dry_run {
        return dry_run_stage(stage, ctx, command);
    }
    if let Some(stopped) = sudo_gate(stage, ctx).or_else(|| consent_gate(stage, ctx)) {
        return stopped;
    }
    match apply(stage, ctx, command) {
        Ok(()) if stage.check => Outcome::Checked,
        Ok(()) => Outcome::Applied,
        Err(e) if stage.warn_only => Outcome::Skipped(format!("{}: {}", stage.name, last_line(&e))),
        Err(e) if stage.critical => Outcome::Failed(format!("{}: {e:#}", stage.name)),
        Err(e) => recover(stage.name, e, ctx),
    }
}

/// The child's last output line, which a warn-only check writes as the operator's fix.
fn last_line(err: &anyhow::Error) -> String {
    let text = format!("{err:#}");
    text.lines()
        .rev()
        .map(str::trim)
        .find(|l| !l.is_empty())
        .unwrap_or_default()
        .to_string()
}

/// Render every action, touch nothing; a consent-gated stage says what consent would cover.
fn dry_run_stage(stage: &Stage, ctx: &Ctx, command: &str) -> Outcome {
    for action in &stage.actions {
        log(
            ctx,
            command,
            stage.name,
            Some(action.view()),
            &Outcome::DryRun,
            None,
            Duration::ZERO,
        );
    }
    if stage.consent && !ctx.yes {
        say::info(&format!(
            "{}: would need --yes or an interactive confirm",
            stage.name
        ));
    }
    Outcome::DryRun
}

fn sudo_gate(stage: &Stage, ctx: &Ctx) -> Option<Outcome> {
    (stage.needs_sudo() && !ctx.sudo.available()).then(|| {
        Outcome::NeedsHuman(format!(
            "{}: {}\n    {}",
            stage.name,
            ctx.sudo.human_fix(),
            stage
                .actions
                .iter()
                .map(Action::display)
                .collect::<Vec<_>>()
                .join("\n    ")
        ))
    })
}

fn consent_gate(stage: &Stage, ctx: &Ctx) -> Option<Outcome> {
    if !stage.consent || ctx.yes {
        return None;
    }
    let what = stage
        .actions
        .iter()
        .map(Action::display)
        .collect::<Vec<_>>()
        .join("\n    ");
    if ctx.mode != Mode::Interactive {
        return Some(Outcome::NeedsHuman(format!(
            "{}: needs --yes for:\n    {what}",
            stage.name
        )));
    }
    say::info(&format!("{} will run:\n    {what}", stage.name));
    match ctx.confirm("proceed?", true) {
        Ok(true) => None,
        Ok(false) => Some(Outcome::Skipped(format!("{}: declined", stage.name))),
        Err(e) => Some(Outcome::NeedsHuman(format!("{}: {e:#}", stage.name))),
    }
}

fn apply(stage: &Stage, ctx: &Ctx, command: &str) -> Result<()> {
    for action in &stage.actions {
        let started = Instant::now();
        let result = exec_action(action, ctx);
        let outcome = match &result {
            Ok(_) if stage.check => Outcome::Checked,
            Ok(_) => Outcome::Applied,
            Err(e) => Outcome::Failed(format!("{e:#}")),
        };
        let exit = result.as_ref().ok().copied().flatten();
        log(
            ctx,
            command,
            stage.name,
            Some(action.view()),
            &outcome,
            exit,
            started.elapsed(),
        );
        result?;
    }
    match &stage.post {
        Some(argv) => run_argv(argv, false, None, &[], stage.post_timeout_s(), ctx).map(|_| ()),
        None => Ok(()),
    }
}

fn recover(stage: &'static str, err: anyhow::Error, ctx: &Ctx) -> Outcome {
    let why = format!("{stage}: {err:#}");
    say::fail(&why);
    if ctx.mode != Mode::Interactive {
        return Outcome::NeedsHuman(why);
    }
    match ctx.input("[c]ontinue / [s]hell / [h]elp", "c") {
        Ok(answer) if answer.starts_with('s') => {
            open_shell();
            Outcome::Skipped(format!("{stage}: fixed by hand"))
        }
        Ok(answer) if answer.starts_with('h') => {
            say::info(&format!(
                "help: {HELP_URL} — log: {}",
                ctx.log.path().display()
            ));
            Outcome::NeedsHuman(why)
        }
        _ => Outcome::Skipped(why),
    }
}

fn log<'a>(
    ctx: &'a Ctx,
    command: &'a str,
    stage: &'a str,
    action: Option<ActionView<'a>>,
    outcome: &Outcome,
    exit: Option<i32>,
    took: Duration,
) {
    let record = ActionRecord {
        ts: install_record::now_iso(),
        run: &ctx.run_id,
        command,
        stage,
        action,
        outcome: outcome.label(),
        exit,
        duration_ms: took.as_millis() as u64,
    };
    if let Err(e) = ctx.log.append(&record) {
        say::warn(&format!("action log: {e:#}"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action_log::ActionLog;
    use crate::install_record::TmpDir;
    use crate::sudo::Sudo;
    use std::fs;
    use std::path::Path;

    fn ctx(home: &Path, dry_run: bool) -> Ctx {
        Ctx {
            mode: Mode::NonInteractive,
            dry_run,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(home).unwrap(),
            run_id: "test".to_string(),
        }
    }

    fn plan_of(command: &str, stages: Vec<Stage>) -> Plan {
        Plan {
            command: command.to_string(),
            stages,
            notes: Vec::new(),
        }
    }

    #[test]
    fn dry_run_writes_nothing_outside_the_action_log() {
        let home = TmpDir::new("plan-dry");
        let marked = home.path().join("touched");
        let written = home.path().join("written.conf");
        let plan = plan_of(
            "setup",
            vec![Stage::new("s", false)
                .run(&["touch", &marked.display().to_string()], 10)
                .push(Action::WriteFile {
                    path: written.clone(),
                    mode: 0o644,
                    contents: "x".to_string(),
                    sudo: false,
                })],
        );
        let mut c = ctx(home.path(), true);
        let report = run(&plan, &mut c).unwrap();
        assert_eq!(report.exit_code(), 0);
        assert!(!marked.exists() && !written.exists());
        assert!(!install_record::installer_json(home.path()).exists());
        let log = fs::read_to_string(c.log.path()).unwrap();
        assert!(log.lines().all(|l| l.contains("\"dry_run\"")), "{log}");
    }

    #[test]
    fn dry_run_without_yes_is_exit_0_and_lists_what_consent_would_cover() {
        let home = TmpDir::new("plan-dry-consent");
        let plan = plan_of(
            "setup",
            vec![Stage::new("packages", true)
                .sudo(&["apt-get", "install", "-y", "git"], 60)
                .consent()],
        );
        let mut c = ctx(home.path(), true);
        c.yes = false;
        c.sudo = Sudo::Unavailable("no root".to_string());
        let report = run(&plan, &mut c).unwrap();
        assert_eq!(report.stages[0].1, Outcome::DryRun);
        assert_eq!(report.exit_code(), 0);
    }

    #[test]
    fn empty_stage_is_already_not_applied() {
        let home = TmpDir::new("plan-empty");
        let plan = plan_of("setup", vec![Stage::new("sysconfig", true)]);
        let report = run(&plan, &mut ctx(home.path(), false)).unwrap();
        assert_eq!(report.stages, [("sysconfig".to_string(), Outcome::Already)]);
        assert_eq!(report.exit_code(), 0);
    }

    #[test]
    fn critical_failure_stops_the_plan_and_exit_code_is_1() {
        let home = TmpDir::new("plan-critical");
        let plan = plan_of(
            "setup",
            vec![
                Stage::new("first", true).run(&["false"], 10),
                Stage::new("second", false).run(&["true"], 10),
            ],
        );
        let report = run(&plan, &mut ctx(home.path(), false)).unwrap();
        assert_eq!(report.stages.len(), 1);
        assert_eq!(report.exit_code(), 1);
    }

    #[test]
    fn critical_needs_human_stops_the_plan() {
        let home = TmpDir::new("plan-critical-human");
        let plan = plan_of(
            "setup",
            vec![
                Stage::new("first", true).sudo(&["true"], 10),
                Stage::new("second", false).run(&["true"], 10),
            ],
        );
        let mut c = ctx(home.path(), false);
        c.sudo = Sudo::Unavailable("no tty".to_string());
        let report = run(&plan, &mut c).unwrap();
        assert_eq!(report.stages.len(), 1);
        assert!(matches!(report.stages[0].1, Outcome::NeedsHuman(_)));
        assert_eq!(report.exit_code(), 2);
    }

    #[test]
    fn a_warn_only_failure_is_one_line_carrying_the_childs_last_output_and_exit_0() {
        let home = TmpDir::new("plan-warn");
        let script = "echo 'torch: static TLS'; echo 'fix: export LD_PRELOAD=x' >&2; exit 1";
        let plan = plan_of(
            "setup",
            vec![Stage::new("verify-torch", false)
                .run(&["sh", "-c", script], 10)
                .warn_only()],
        );
        let report = run(&plan, &mut ctx(home.path(), false)).unwrap();
        assert_eq!(
            report.stages[0].1,
            Outcome::Skipped("verify-torch: fix: export LD_PRELOAD=x".to_string())
        );
        assert_eq!(report.exit_code(), 0);
    }

    #[test]
    fn a_check_runs_but_never_logs_applied() {
        let home = TmpDir::new("plan-check");
        let plan = plan_of(
            "setup",
            vec![Stage::new("verify", true).run(&["true"], 10).check()],
        );
        let mut c = ctx(home.path(), false);
        let report = run(&plan, &mut c).unwrap();
        assert_eq!(report.stages[0].1, Outcome::Checked);
        let log = fs::read_to_string(c.log.path()).unwrap();
        assert!(
            log.lines().all(|line| line.contains("\"checked\"")),
            "{log}"
        );
    }

    #[test]
    fn non_critical_failure_non_interactive_is_needs_human_exit_2() {
        let home = TmpDir::new("plan-noncritical");
        let plan = plan_of(
            "setup",
            vec![Stage::new("extra", false).run(&["false"], 10)],
        );
        let report = run(&plan, &mut ctx(home.path(), false)).unwrap();
        assert_eq!(report.exit_code(), 2);
        assert_eq!(report.needs_human().len(), 1);
    }

    #[test]
    fn sudo_stage_with_unavailable_sudo_never_spawns_and_is_needs_human() {
        let home = TmpDir::new("plan-nosudo");
        let marked = home.path().join("touched");
        let mut stage = Stage::new("sysconfig", false);
        stage = stage.push(Action::Run {
            argv: vec!["touch".to_string(), marked.display().to_string()],
            sudo: true,
            cwd: None,
            env: Vec::new(),
            timeout_s: 10,
        });
        let mut c = ctx(home.path(), false);
        c.sudo = Sudo::Unavailable("no tty".to_string());
        let report = run(&plan_of("setup", vec![stage]), &mut c).unwrap();
        assert!(!marked.exists());
        assert_eq!(report.exit_code(), 2);
    }

    #[test]
    fn consent_stage_without_yes_non_interactive_lists_its_commands() {
        let home = TmpDir::new("plan-consent");
        let plan = plan_of(
            "setup",
            vec![Stage::new("packages", false).run(&["true"], 10).consent()],
        );
        let mut c = ctx(home.path(), false);
        c.yes = false;
        let report = run(&plan, &mut c).unwrap();
        assert_eq!(report.exit_code(), 2);
        assert!(report.needs_human()[0].contains("true"));
    }

    #[test]
    fn a_sudo_run_carrying_env_is_refused_because_sudo_resets_the_environment() {
        let plan = plan_of(
            "setup",
            vec![Stage::new("packages", true).push(Action::Run {
                argv: vec!["apt-get".to_string(), "install".to_string()],
                sudo: true,
                cwd: None,
                env: vec![("DEBIAN_FRONTEND".to_string(), "noninteractive".to_string())],
                timeout_s: 60,
            })],
        );
        assert_eq!(sudo_env_violations(&plan), ["packages: apt-get install"]);
        let home = TmpDir::new("plan-sudoenv");
        let err = run(&plan, &mut ctx(home.path(), false)).unwrap_err();
        assert!(format!("{err:#}").contains("env"), "{err:#}");
    }

    #[test]
    fn run_kills_child_at_timeout() {
        let home = TmpDir::new("plan-timeout");
        let plan = plan_of(
            "setup",
            vec![Stage::new("slow", true).run(&["sleep", "5"], 1)],
        );
        let report = run(&plan, &mut ctx(home.path(), false)).unwrap();
        let Outcome::Failed(why) = &report.stages[0].1 else {
            panic!("want Failed, got {:?}", report.stages[0].1)
        };
        assert!(why.contains("1 s deadline"), "{why}");
    }

    #[test]
    fn exit_code_mapping_table() {
        let table = [
            (Outcome::Already, 0),
            (Outcome::Applied, 0),
            (Outcome::Checked, 0),
            (Outcome::DryRun, 0),
            (Outcome::Skipped("s".into()), 0),
            (Outcome::NeedsHuman("h".into()), 2),
            (Outcome::Failed("f".into()), 1),
        ];
        for (outcome, want) in table {
            let report = Report {
                command: "setup".to_string(),
                stages: vec![("s".to_string(), outcome.clone())],
            };
            assert_eq!(report.exit_code(), want, "{outcome:?}");
        }
    }
}
