//! The executor: `run` is the only function in the crate that mutates the machine or reads stdin.

use std::collections::VecDeque;
use std::fs;
use std::io::{BufRead, BufReader, IsTerminal, Read, Write};
use std::os::unix::fs::PermissionsExt;
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::cli::Cli;
use crate::plan::{display_argv, ensure_block, Action, Outcome, Plan, Stage};
use crate::say;
use crate::state::{self, ActionLog, ActionRecord, ActionView};
use crate::sudo::Sudo;

const TAIL_LINES: usize = 20;
const HELP_URL: &str = "https://github.com/dimensionalOS/dimos/issues";

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    Interactive,
    NonInteractive,
    Agent,
}

impl Mode {
    pub fn from_flags(non_interactive: bool, agent: bool, stdin_is_tty: bool) -> Mode {
        match (agent, non_interactive || !stdin_is_tty) {
            (true, _) => Mode::Agent,
            (false, true) => Mode::NonInteractive,
            (false, false) => Mode::Interactive,
        }
    }
}

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

pub struct Ctx {
    pub mode: Mode,
    pub dry_run: bool,
    pub verbose: bool,
    pub yes: bool,
    pub sudo: Sudo,
    pub log: ActionLog,
    pub run_id: String,
}

impl Ctx {
    pub fn new(cli: &Cli, home: &Path) -> Result<Ctx> {
        let mode = Mode::from_flags(cli.non_interactive, cli.agent, stdin_is_tty());
        Ok(Ctx {
            mode,
            dry_run: cli.dry_run,
            verbose: cli.verbose,
            yes: cli.yes,
            sudo: Sudo::resolve(mode),
            log: ActionLog::open(home)?,
            run_id: state::run_id(),
        })
    }

    pub fn confirm(&self, question: &str, default: bool) -> Result<bool> {
        if self.yes {
            return Ok(true);
        }
        if self.mode != Mode::Interactive {
            return Ok(default);
        }
        let hint = if default { "[Y/n]" } else { "[y/N]" };
        let answer = ask(&format!("{question} {hint}: "))?;
        Ok(match answer.trim().to_ascii_lowercase().as_str() {
            "" => default,
            other => other.starts_with('y'),
        })
    }

    pub fn choose(&self, question: &str, options: &[&str], default: usize) -> Result<usize> {
        if self.mode != Mode::Interactive {
            return Ok(default);
        }
        say::info(question);
        for (i, option) in options.iter().enumerate() {
            eprintln!("   {}) {option}", i + 1);
        }
        let answer = ask(&format!("choice [{}]: ", default + 1))?;
        Ok(answer
            .trim()
            .parse::<usize>()
            .ok()
            .filter(|n| (1..=options.len()).contains(n))
            .map_or(default, |n| n - 1))
    }

    pub fn input(&self, question: &str, default: &str) -> Result<String> {
        if self.mode != Mode::Interactive {
            return Ok(default.to_string());
        }
        let answer = ask(&format!("{question} [{default}]: "))?;
        let trimmed = answer.trim();
        Ok(if trimmed.is_empty() {
            default.to_string()
        } else {
            trimmed.to_string()
        })
    }
}

/// The one stdin read in the crate; the prompt goes to stderr so `--agent` stdout stays JSON.
fn ask(prompt: &str) -> Result<String> {
    eprint!("{prompt}");
    std::io::stderr().flush()?;
    let mut line = String::new();
    std::io::stdin().read_line(&mut line)?;
    Ok(line)
}

pub fn stdin_is_tty() -> bool {
    std::io::stdin().is_terminal()
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
    say::info(&format!(
        "{}: {} stage(s){}",
        plan.command,
        plan.stages.len(),
        if ctx.dry_run { " (dry run)" } else { "" }
    ));
    for stage in &plan.stages {
        print_stage(stage, ctx);
    }
    for note in &plan.notes {
        say::warn(note);
    }
}

fn print_stage(stage: &Stage, ctx: &Ctx) {
    if stage.actions.is_empty() {
        return;
    }
    say::info(&format!("{}: {} step(s)", stage.name, stage.actions.len()));
    if !ctx.dry_run && !ctx.verbose {
        return;
    }
    for action in &stage.actions {
        eprintln!("     {}", action.display());
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
    print_plan(plan, ctx);
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
    report_stage(stage.name, &outcome);
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

fn exec_action(action: &Action, ctx: &Ctx) -> Result<Option<i32>> {
    match action {
        Action::Run {
            argv,
            sudo,
            cwd,
            env,
            timeout_s,
        } => run_argv(argv, *sudo, cwd.as_deref(), env, *timeout_s, ctx).map(Some),
        Action::WriteFile {
            path,
            mode,
            contents,
            sudo,
        } => write_file(path, *mode, contents, *sudo, ctx).map(|_| None),
        Action::EnsureBlock {
            file,
            marker,
            lines,
        } => ensure_block_file(file, marker, lines).map(|_| None),
        Action::Copy { from, to, mode } => copy_file(from, to, *mode).map(|_| None),
        Action::Rename { from, to } => fs::rename(from, to)
            .with_context(|| format!("rename {} -> {}", from.display(), to.display()))
            .map(|_| None),
        Action::Remove { path, sudo } => remove_file(path, *sudo, ctx).map(|_| None),
        Action::VerifySha256 { file, sums_file } => verify_sha256(file, sums_file).map(|_| None),
    }
}

fn write_file(path: &Path, mode: u32, contents: &str, sudo: bool, ctx: &Ctx) -> Result<()> {
    if fs::read_to_string(path).is_ok_and(|t| t == contents) {
        return Ok(());
    }
    if !sudo {
        return write_atomic(path, mode, contents);
    }
    let name = path.file_name().unwrap_or_default().to_string_lossy();
    let tmp = std::env::temp_dir().join(format!("dimos-{}-{name}", ctx.run_id));
    write_atomic(&tmp, mode, contents)?;
    let argv = [
        "install".to_string(),
        "-m".to_string(),
        format!("{mode:o}"),
        tmp.display().to_string(),
        path.display().to_string(),
    ];
    let status = run_argv(&argv, true, None, &[], 60, ctx);
    let _ = fs::remove_file(&tmp);
    status.map(|_| ())
}

/// The rename lands on the real file, so a symlinked rc file (stow, chezmoi) keeps its link.
fn write_atomic(path: &Path, mode: u32, contents: &str) -> Result<()> {
    let path = fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    ensure_parent(&path)?;
    let tmp = path.with_extension("dimos-tmp");
    fs::write(&tmp, contents).with_context(|| format!("write {}", tmp.display()))?;
    fs::set_permissions(&tmp, fs::Permissions::from_mode(mode))
        .with_context(|| format!("chmod {mode:o} {}", tmp.display()))?;
    fs::rename(&tmp, &path)
        .with_context(|| format!("rename {} -> {}", tmp.display(), path.display()))
}

fn ensure_block_file(file: &Path, marker: &str, lines: &[String]) -> Result<()> {
    let before = fs::read_to_string(file).unwrap_or_default();
    let (after, changed) = ensure_block(&before, marker, lines);
    if !changed {
        return Ok(());
    }
    let mode = fs::metadata(file).map_or(0o644, |m| m.permissions().mode() & 0o777);
    write_atomic(file, mode, &after)
}

fn copy_file(from: &Path, to: &Path, mode: u32) -> Result<()> {
    ensure_parent(to)?;
    fs::copy(from, to).with_context(|| format!("copy {} -> {}", from.display(), to.display()))?;
    fs::set_permissions(to, fs::Permissions::from_mode(mode))
        .with_context(|| format!("chmod {mode:o} {}", to.display()))
}

fn ensure_parent(path: &Path) -> Result<()> {
    match path.parent() {
        Some(dir) => fs::create_dir_all(dir).with_context(|| format!("create {}", dir.display())),
        None => Ok(()),
    }
}

fn remove_file(path: &Path, sudo: bool, ctx: &Ctx) -> Result<()> {
    if sudo {
        let argv = [
            "rm".to_string(),
            "-f".to_string(),
            path.display().to_string(),
        ];
        return run_argv(&argv, true, None, &[], 30, ctx).map(|_| ());
    }
    match fs::remove_file(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        other => other.with_context(|| format!("remove {}", path.display())),
    }
}

fn verify_sha256(file: &Path, sums_file: &Path) -> Result<()> {
    let text =
        fs::read_to_string(sums_file).with_context(|| format!("read {}", sums_file.display()))?;
    let want = parse_sha_file(&text)?;
    let got = sha256_hex(file)?;
    if got != want {
        bail!("sha256 of {} is {got}, want {want}", file.display());
    }
    Ok(())
}

fn run_argv(
    argv: &[String],
    sudo: bool,
    cwd: Option<&Path>,
    env: &[(String, String)],
    timeout_s: u64,
    ctx: &Ctx,
) -> Result<i32> {
    let (full, password) = if sudo {
        ctx.sudo.refresh()?; // the human types before the deadline starts, not inside it
        ctx.sudo.wrap(argv)
    } else {
        (argv.to_vec(), None)
    };
    say::info(&format!("running: {}", display_argv(&full)));
    let started = Instant::now();
    let mut child = spawn(&full, cwd, env, password.is_some())?;
    if let (Some(bytes), Some(mut pipe)) = (password, child.stdin.take()) {
        let _ = pipe.write_all(&bytes);
    }
    let tail = Tail::attach(&mut child, ctx.verbose);
    let status = wait_until(&mut child, Duration::from_secs(timeout_s))?;
    finish(
        status,
        &full,
        timeout_s,
        started.elapsed().as_secs(),
        tail.join(),
    )
}

fn finish(
    status: Option<ExitStatus>,
    argv: &[String],
    timeout_s: u64,
    took_s: u64,
    lines: String,
) -> Result<i32> {
    match status {
        Some(s) if s.success() => Ok(s.code().unwrap_or(0)),
        Some(s) => bail!(
            "command failed (exit {} after {took_s} s): {}\n{lines}",
            s.code().unwrap_or(-1),
            display_argv(argv)
        ),
        None => bail!(
            "command hit its {timeout_s} s deadline and was killed: {}\n{lines}",
            display_argv(argv)
        ),
    }
}

fn spawn(
    argv: &[String],
    cwd: Option<&Path>,
    env: &[(String, String)],
    feed_stdin: bool,
) -> Result<Child> {
    let (program, args) = argv
        .split_first()
        .context("an Action::Run has empty argv")?;
    let mut cmd = Command::new(program);
    cmd.args(args)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .stdin(if feed_stdin {
            Stdio::piped()
        } else {
            Stdio::null()
        });
    if let Some(dir) = cwd {
        cmd.current_dir(dir);
    }
    for (key, value) in env {
        cmd.env(key, value);
    }
    cmd.spawn()
        .with_context(|| format!("spawn {program}: not on PATH?"))
}

/// Poll instead of blocking so a hung child is killed at its deadline instead of hanging the run.
pub(crate) fn wait_until(child: &mut Child, deadline: Duration) -> Result<Option<ExitStatus>> {
    let started = Instant::now();
    loop {
        match child.try_wait().context("wait for the child process")? {
            Some(status) => return Ok(Some(status)),
            None if started.elapsed() < deadline => std::thread::sleep(Duration::from_millis(50)),
            None => {
                let _ = child.kill();
                let _ = child.wait();
                return Ok(None);
            }
        }
    }
}

/// Drains both pipes so a chatty child never blocks on a full buffer, keeping the last lines.
struct Tail {
    stdout: Arc<Mutex<VecDeque<String>>>,
    stderr: Arc<Mutex<VecDeque<String>>>,
    threads: Vec<JoinHandle<()>>,
}

impl Tail {
    fn attach(child: &mut Child, verbose: bool) -> Tail {
        let stdout = Arc::new(Mutex::new(VecDeque::with_capacity(TAIL_LINES)));
        let stderr = Arc::new(Mutex::new(VecDeque::with_capacity(TAIL_LINES)));
        let mut threads = Vec::new();
        if let Some(out) = child.stdout.take() {
            threads.push(drain(out, Arc::clone(&stdout), verbose));
        }
        if let Some(err) = child.stderr.take() {
            threads.push(drain(err, Arc::clone(&stderr), verbose));
        }
        Tail {
            stdout,
            stderr,
            threads,
        }
    }

    fn join(self) -> String {
        for thread in self.threads {
            let _ = thread.join();
        }
        let stdout = self.stdout.lock().unwrap_or_else(|e| e.into_inner());
        let stderr = self.stderr.lock().unwrap_or_else(|e| e.into_inner());
        // Diagnostics conventionally use stderr, so its final line must win the warning summary.
        let lines: Vec<&String> = stdout.iter().chain(stderr.iter()).collect();
        lines[lines.len().saturating_sub(TAIL_LINES)..]
            .iter()
            .map(|l| format!("    {l}"))
            .collect::<Vec<_>>()
            .join("\n")
    }
}

fn drain(
    pipe: impl Read + Send + 'static,
    sink: Arc<Mutex<VecDeque<String>>>,
    verbose: bool,
) -> JoinHandle<()> {
    std::thread::spawn(move || {
        for line in BufReader::new(pipe).lines().map_while(Result::ok) {
            if verbose {
                eprintln!("     {line}");
            }
            let mut kept = sink.lock().unwrap_or_else(|e| e.into_inner());
            if kept.len() == TAIL_LINES {
                kept.pop_front();
            }
            kept.push_back(line);
        }
    })
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

fn open_shell() {
    let shell = std::env::var("SHELL").unwrap_or_else(|_| "/bin/sh".to_string());
    say::info("type 'exit' when you have finished");
    let _ = Command::new(shell).status();
}

fn report_stage(name: &str, outcome: &Outcome) {
    match outcome {
        Outcome::Applied | Outcome::Checked => say::ok(name),
        Outcome::Already => say::ok(&format!("{name} already")),
        Outcome::DryRun => say::info(&format!("{name} (dry run)")),
        Outcome::Skipped(why) | Outcome::NeedsHuman(why) => say::warn(why),
        Outcome::Failed(why) => say::fail(why),
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
        ts: state::now_iso(),
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

pub fn sha256_hex(path: &Path) -> Result<String> {
    let mut file = fs::File::open(path).with_context(|| format!("read {}", path.display()))?;
    let mut hasher = Sha256::new();
    std::io::copy(&mut file, &mut hasher).with_context(|| format!("hash {}", path.display()))?;
    Ok(hasher
        .finalize()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect())
}

/// The first 64-hex token, so `sha256sum` and `shasum -a 256` files both parse.
pub fn parse_sha_file(text: &str) -> Result<String> {
    text.split_whitespace()
        .find(|t| t.len() == 64 && t.chars().all(|c| c.is_ascii_hexdigit()))
        .map(|t| t.to_ascii_lowercase())
        .context("no 64-hex sha256 token in the .sha256 file")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::TmpDir;

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
        assert!(!state::installer_json(home.path()).exists());
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
    fn a_symlinked_rc_file_keeps_its_link_when_a_block_is_written() {
        let home = TmpDir::new("plan-symlink");
        let real = home.path().join("dotfiles/zshrc");
        fs::create_dir_all(real.parent().unwrap()).unwrap();
        fs::write(&real, "export A=1\n").unwrap();
        let link = home.path().join(".zshrc");
        std::os::unix::fs::symlink(&real, &link).unwrap();
        ensure_block_file(&link, "path", &["export B=2".to_string()]).unwrap();
        assert!(fs::symlink_metadata(&link)
            .unwrap()
            .file_type()
            .is_symlink());
        assert!(fs::read_to_string(&real).unwrap().contains("export B=2"));
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
    fn write_file_equal_contents_is_noop() {
        let home = TmpDir::new("plan-write");
        let path = home.path().join("f.conf");
        let c = ctx(home.path(), false);
        write_file(&path, 0o644, "same\n", false, &c).unwrap();
        let first = fs::metadata(&path).unwrap().modified().unwrap();
        write_file(&path, 0o644, "same\n", false, &c).unwrap();
        assert_eq!(fs::metadata(&path).unwrap().modified().unwrap(), first);
        assert_eq!(fs::read_to_string(&path).unwrap(), "same\n");
    }

    #[test]
    fn parse_sha_file_accepts_sha256sum_and_shasum_formats() {
        let hex = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
        assert_eq!(parse_sha_file(&format!("{hex}  dimos\n")).unwrap(), hex);
        assert_eq!(
            parse_sha_file(&format!("SHA256 (dimos) = {}\n", hex.to_uppercase())).unwrap(),
            hex
        );
        assert!(parse_sha_file("not a sum\n").is_err());
    }

    #[test]
    fn verify_sha256_error_is_got_vs_want() {
        let home = TmpDir::new("plan-sha");
        let file = home.path().join("dimos");
        let sums = home.path().join("dimos.sha256");
        fs::write(&file, b"payload").unwrap();
        let wrong = "0".repeat(64);
        fs::write(&sums, format!("{wrong}  dimos\n")).unwrap();
        let err = format!("{:#}", verify_sha256(&file, &sums).unwrap_err());
        assert!(err.contains("is ") && err.contains("want 0000"), "{err}");
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

    #[test]
    fn mode_agent_beats_non_interactive_and_a_pipe_is_non_interactive() {
        assert_eq!(Mode::from_flags(false, true, true), Mode::Agent);
        assert_eq!(Mode::from_flags(true, false, true), Mode::NonInteractive);
        assert_eq!(Mode::from_flags(false, false, false), Mode::NonInteractive);
        assert_eq!(Mode::from_flags(false, false, true), Mode::Interactive);
    }

    fn sources() -> Vec<(String, String)> {
        fn walk(dir: &Path, out: &mut Vec<(String, String)>) {
            for entry in fs::read_dir(dir).expect("read src/").flatten() {
                let path = entry.path();
                if path.is_dir() {
                    walk(&path, out);
                } else if path.extension().is_some_and(|e| e == "rs") {
                    let name = path.file_name().unwrap().to_string_lossy().into_owned();
                    out.push((name, fs::read_to_string(&path).expect("read source")));
                }
            }
        }
        let mut out = Vec::new();
        walk(&Path::new(env!("CARGO_MANIFEST_DIR")).join("src"), &mut out);
        out
    }

    #[test]
    fn prompts_live_only_in_run_rs() {
        let elsewhere: Vec<String> = sources()
            .iter()
            .filter(|(name, text)| name != "run.rs" && text.contains("stdin()"))
            .map(|(name, _)| name.clone())
            .collect();
        assert!(
            elsewhere.is_empty(),
            "stdin read outside run.rs: {elsewhere:?}"
        );
    }

    /// The text before `#[cfg(test)]`: a fixture may write into a TmpDir, the binary may not.
    fn runtime(text: &str) -> &str {
        text.split("#[cfg(test)]").next().unwrap_or(text)
    }

    #[test]
    fn the_filesystem_is_mutated_only_from_run_rs_and_state_rs() {
        const WRITERS: [&str; 6] = [
            "fs::write",
            "fs::create_dir",
            "fs::remove_",
            "fs::rename",
            "fs::copy",
            "fs::set_permissions",
        ];
        let offenders: Vec<String> = sources()
            .iter()
            .filter(|(name, _)| name != "run.rs" && name != "state.rs")
            .filter(|(_, text)| WRITERS.iter().any(|w| runtime(text).contains(w)))
            .map(|(name, _)| name.clone())
            .collect();
        assert!(offenders.is_empty(), "writes outside run.rs: {offenders:?}");
    }
}
