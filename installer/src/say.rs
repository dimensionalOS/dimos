//! Everything the terminal shows: four fixed prefixes, and the plan and stage lines built on them.

use std::io::IsTerminal;

use serde::Serialize;

use crate::action_log::ActionView;
use crate::plan::{Action, Outcome, Plan, Stage};
use crate::run_context::{Ctx, Mode};

pub fn info(msg: &str) {
    line("->", "\x1b[34m", msg);
}

pub fn ok(msg: &str) {
    line("ok", "\x1b[32m", msg);
}

pub fn warn(msg: &str) {
    line("!!", "\x1b[33m", msg);
}

pub fn fail(msg: &str) {
    line("xx", "\x1b[31m", msg);
}

fn line(prefix: &str, colour: &str, msg: &str) {
    if std::io::stderr().is_terminal() && std::env::var_os("NO_COLOR").is_none() {
        eprintln!("{colour}{prefix}\x1b[0m {msg}");
    } else {
        eprintln!("{prefix} {msg}");
    }
}

#[derive(Serialize)]
struct StageView<'a> {
    name: &'a str,
    critical: bool,
    consent: bool,
    check: bool,
    actions: Vec<ActionView<'a>>,
    post: Option<&'a [String]>,
}

#[derive(Serialize)]
struct PlanView<'a> {
    command: &'a str,
    stages: Vec<StageView<'a>>,
    notes: &'a [String],
}

impl<'a> PlanView<'a> {
    fn of(plan: &'a Plan) -> PlanView<'a> {
        PlanView {
            command: &plan.command,
            stages: plan
                .stages
                .iter()
                .map(|s| StageView {
                    name: s.name,
                    critical: s.critical,
                    consent: s.consent,
                    check: s.check,
                    actions: s.actions.iter().map(Action::view).collect(),
                    post: s.post.as_deref(),
                })
                .collect(),
            notes: &plan.notes,
        }
    }
}

pub fn print_plan(plan: &Plan, ctx: &Ctx) {
    if ctx.mode == Mode::Agent {
        println!(
            "{}",
            serde_json::to_string(&PlanView::of(plan)).unwrap_or_default()
        );
        return;
    }
    info(&format!(
        "{}: {} stage(s){}",
        plan.command,
        plan.stages.len(),
        if ctx.dry_run { " (dry run)" } else { "" }
    ));
    for stage in &plan.stages {
        print_stage(stage, ctx);
    }
    for note in &plan.notes {
        warn(note);
    }
}

fn print_stage(stage: &Stage, ctx: &Ctx) {
    if stage.actions.is_empty() {
        return;
    }
    info(&format!("{}: {} step(s)", stage.name, stage.actions.len()));
    if !ctx.dry_run && !ctx.verbose {
        return;
    }
    for action in &stage.actions {
        eprintln!("     {}", action.display());
    }
}

pub(crate) fn report_stage(name: &str, outcome: &Outcome) {
    match outcome {
        Outcome::Applied | Outcome::Checked => ok(name),
        Outcome::Already => ok(&format!("{name} already")),
        Outcome::DryRun => info(&format!("{name} (dry run)")),
        Outcome::Skipped(why) | Outcome::NeedsHuman(why) => warn(why),
        Outcome::Failed(why) => fail(why),
    }
}
