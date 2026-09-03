//! One program under a deadline, keeping the tail of its output for the failure message.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Read, Write};
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

use crate::plan::display_argv;
use crate::run_context::Ctx;
use crate::say;

const TAIL_LINES: usize = 20;

pub(crate) fn run_argv(
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
